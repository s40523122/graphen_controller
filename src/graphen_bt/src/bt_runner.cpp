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

// 載入 behavior tree、以固定週期 tick 的執行器。
// ROS 參數 gp_* 與 topic /is_ok、/wait_seconds 會同步寫入 blackboard，
// 讓外部（rqt、命令列）能在執行中調整樹的行為。
int run(const rclcpp::Node::SharedPtr & node) {
  const auto logger = node->get_logger();

  node->declare_parameter<std::string>("tree_file", "");
  node->declare_parameter<std::vector<std::string>>("plugins", {"graphen_bt_nodes"});
  node->declare_parameter<int>("tick_period_ms", 100);
  node->declare_parameter<bool>("groot.enabled", true);
  node->declare_parameter<int>("groot.max_msg_per_second", 25);
  node->declare_parameter<int>("groot.publisher_port", 1666);
  node->declare_parameter<int>("groot.server_port", 1667);

  // 對外調整 blackboard 用的參數
  node->declare_parameter<bool>("gp_is_ok", true);
  node->declare_parameter<int>("gp_wait_seconds", 3);
  node->declare_parameter<double>("gp_rate", 100.0);
  node->declare_parameter<int>("gp_sys_count", 0);
  node->declare_parameter<bool>("gp_enable_walk", false);
  node->declare_parameter<bool>("gp_enable_reciprocate", true);

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
  blackboard->set<bool>("is_ok", node->get_parameter("gp_is_ok").as_bool());
  blackboard->set<int>(
    "wait_seconds", static_cast<int>(node->get_parameter("gp_wait_seconds").as_int()));
  blackboard->set<double>("rate", node->get_parameter("gp_rate").as_double());
  blackboard->set<int>("sys_count", 0);
  blackboard->set<bool>("enable_walk", node->get_parameter("gp_enable_walk").as_bool());
  blackboard->set<bool>(
    "enable_reciprocate", node->get_parameter("gp_enable_reciprocate").as_bool());

  auto tree = factory.createTreeFromFile(tree_file, blackboard);
  RCLCPP_INFO(logger, "Loaded tree from [ %s ]", tree_file.c_str());

  // rqt / ros2 param set 修改參數時同步寫入 blackboard
  auto param_callback_handle = node->add_on_set_parameters_callback(
    [blackboard, logger](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        if (param.get_name() == "gp_is_ok") {
          blackboard->set<bool>("is_ok", param.as_bool());
          RCLCPP_INFO(logger, "Parameter changed: is_ok -> %s", param.as_bool() ? "true" : "false");
        } else if (param.get_name() == "gp_wait_seconds") {
          blackboard->set<int>("wait_seconds", static_cast<int>(param.as_int()));
          RCLCPP_INFO(logger, "Parameter changed: wait_seconds -> %ld", param.as_int());
        } else if (param.get_name() == "gp_rate") {
          blackboard->set<double>("rate", param.as_double());
          RCLCPP_INFO(logger, "Parameter changed: rate -> %f", param.as_double());
        } else if (param.get_name() == "gp_enable_walk") {
          blackboard->set<bool>("enable_walk", param.as_bool());
          RCLCPP_INFO(logger, "Parameter changed: enable_walk -> %s",
            param.as_bool() ? "true" : "false");
        } else if (param.get_name() == "gp_enable_reciprocate") {
          blackboard->set<bool>("enable_reciprocate", param.as_bool());
          RCLCPP_INFO(logger, "Parameter changed: enable_reciprocate -> %s",
            param.as_bool() ? "true" : "false");
        }
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
  auto node = rclcpp::Node::make_shared("bt_runner");

  int ret = 1;
  try {
    ret = run(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Behavior tree runner failed: %s", e.what());
  }

  rclcpp::shutdown();
  return ret;
}
