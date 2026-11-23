#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

namespace {

constexpr const char * kBlackboardParamPrefix = "gp_";

// 參數 gp_<key> 寫入 blackboard 的 <key>；型別沿用參數的型別（bool / int / double / string）。
// 樹的 port 型別必須一致：double 的 port 要寫成 1.0 而不是 1。不支援的型別或型別衝突時回傳錯誤訊息
std::optional<std::string> sync_to_blackboard(
  const BT::Blackboard::Ptr & blackboard, const rclcpp::Parameter & param)
{
  const auto key = param.get_name().substr(std::char_traits<char>::length(kBlackboardParamPrefix));
  try {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        blackboard->set<bool>(key, param.as_bool());
        return std::nullopt;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        blackboard->set<int>(key, static_cast<int>(param.as_int()));
        return std::nullopt;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        blackboard->set<double>(key, param.as_double());
        return std::nullopt;
      case rclcpp::ParameterType::PARAMETER_STRING:
        blackboard->set<std::string>(key, param.as_string());
        return std::nullopt;
      default:
        return "unsupported type " + std::string(param.get_type_name());
    }
  } catch (const std::exception & e) {
    return std::string(e.what());
  }
}

bool is_blackboard_param(const std::string & name) {
  return name.rfind(kBlackboardParamPrefix, 0) == 0 &&
         name.size() > std::char_traits<char>::length(kBlackboardParamPrefix);
}

// 載入 behavior tree、以固定週期 tick 的執行器。
// 所有 gp_* 參數（設定檔中的、以及下面宣告的預設值）與 topic /is_ok、/wait_seconds 會同步寫入
// blackboard（gp_ 之後的名稱為 key），讓外部（rqt、命令列）能在執行中調整樹的行為；
// 新的樹只要在設定檔加上 gp_* 參數即可，不需要修改這個程式。
int run(const rclcpp::Node::SharedPtr & node) {
  const auto logger = node->get_logger();

  // 設定檔中的參數已由 NodeOptions 自動宣告（型別取自設定檔），這裡只補上沒有設定時的預設值
  const auto declare_default = [&node](const std::string & name, const auto & value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, value);
      }
    };
  declare_default("tree_file", std::string{});
  declare_default("plugins", std::vector<std::string>{"graphen_bt_nodes"});
  declare_default("tick_period_ms", 100);
  declare_default("groot.enabled", true);
  declare_default("groot.max_msg_per_second", 25);
  declare_default("groot.publisher_port", 1666);
  declare_default("groot.server_port", 1667);

  // 既有樹使用的 blackboard 參數的預設值；其他 gp_* 由設定檔提供
  declare_default("gp_is_ok", true);
  declare_default("gp_wait_seconds", 3);
  declare_default("gp_rate", 100.0);
  declare_default("gp_sys_count", 0);
  declare_default("gp_enable_walk", false);
  declare_default("gp_enable_reciprocate", true);

  const auto tree_file = node->get_parameter("tree_file").as_string();
  if (tree_file.empty()) {
    RCLCPP_ERROR(logger, "Parameter 'tree_file' is empty.");
    return 1;
  }
  const auto tick_period_ms = node->get_parameter("tick_period_ms").as_int();
  if (tick_period_ms <= 0) {
    RCLCPP_ERROR(logger, "Parameter 'tick_period_ms' must be greater than zero.");
    return 1;
  }

  // 先複製到區域變數：as_string_array() 回傳的是暫存 Parameter 內部的參考，
  // 直接放進 range-for 會在迴圈開始前就失效
  const auto plugins = node->get_parameter("plugins").as_string_array();

  BT::BehaviorTreeFactory factory;
  for (const auto & plugin : plugins) {
    // 以函式庫名稱載入（例如 graphen_bt_nodes -> libgraphen_bt_nodes.so），透過 LD_LIBRARY_PATH 尋找
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    RCLCPP_INFO(logger, "Loaded BT plugin [ %s ]", plugin.c_str());
  }

  // 建樹前先放入 ROS node，需要 ROS 的節點會在建構時從 blackboard 取得
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  const auto parameter_names = node->list_parameters({}, 0).names;
  for (const auto & name : parameter_names) {
    if (!is_blackboard_param(name)) {
      continue;
    }
    if (const auto error = sync_to_blackboard(blackboard, node->get_parameter(name))) {
      RCLCPP_ERROR(logger, "Parameter '%s' cannot be written to the blackboard: %s",
        name.c_str(), error->c_str());
      return 1;
    }
  }
  // 計數一律從 0 開始（gp_sys_count 是計數的鏡像，由 IncrementSysCount 等節點回寫）
  blackboard->set<int>("sys_count", 0);

  auto tree = factory.createTreeFromFile(tree_file, blackboard);
  RCLCPP_INFO(logger, "Loaded tree from [ %s ]", tree_file.c_str());

  // rqt / ros2 param set 修改參數時同步寫入 blackboard
  auto param_callback_handle = node->add_on_set_parameters_callback(
    [blackboard, logger](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        if (!is_blackboard_param(param.get_name())) {
          continue;
        }
        if (const auto error = sync_to_blackboard(blackboard, param)) {
          // 型別和樹的 port 不一致時拒絕修改，不讓樹在 tick 時才失敗
          result.successful = false;
          result.reason = param.get_name() + ": " + *error;
          return result;
        }
        RCLCPP_INFO(logger, "Parameter changed: %s -> %s",
          param.get_name().substr(std::char_traits<char>::length(kBlackboardParamPrefix)).c_str(),
          param.value_to_string().c_str());
      }
      return result;
    });

  auto wait_seconds_sub = node->create_subscription<std_msgs::msg::Int32>(
    "/wait_seconds", 10,
    [blackboard, logger](const std_msgs::msg::Int32::SharedPtr msg) {
      blackboard->set<int>("wait_seconds", msg->data);
      RCLCPP_INFO(logger, "wait_seconds changed to %d", msg->data);
    });

  auto is_ok_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/is_ok", 10,
    [blackboard, logger](const std_msgs::msg::Bool::SharedPtr msg) {
      blackboard->set<bool>("is_ok", msg->data);
      RCLCPP_INFO(logger, "is_ok changed to %s", msg->data ? "true" : "false");
    });

  // v3 的 ZMQ publisher 對應 Groot（v1）
  std::optional<BT::PublisherZMQ> groot_publisher;
  if (node->get_parameter("groot.enabled").as_bool()) {
    groot_publisher.emplace(
      tree,
      static_cast<unsigned>(node->get_parameter("groot.max_msg_per_second").as_int()),
      static_cast<unsigned>(node->get_parameter("groot.publisher_port").as_int()),
      static_cast<unsigned>(node->get_parameter("groot.server_port").as_int()));
    RCLCPP_INFO(logger, "Groot (v1) monitor enabled.");
  }

  // ROS callback 在背景 thread 處理，主 thread 專心 tick
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  RCLCPP_INFO(logger, "Behavior tree started (tick period %ld ms).", tick_period_ms);

  const auto tick_period = std::chrono::milliseconds(tick_period_ms);
  while (rclcpp::ok()) {
    tree.tickRoot();
    std::this_thread::sleep_for(tick_period);
  }

  // 先停止 tree（讓 RateControl 等背景 thread 結束），再停止 executor
  tree.haltTree();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  // 設定檔中的 gp_* 不需要逐一在程式中宣告：自動宣告設定檔中的所有參數
  auto node = rclcpp::Node::make_shared(
    "bt_runner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  int ret = 1;
  try {
    ret = run(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Behavior tree runner failed: %s", e.what());
  }

  rclcpp::shutdown();
  return ret;
}
