#include "graphen_bt_nodes/get_pixel_error.hpp"

#include <algorithm>
#include <cmath>

#include "graphen_bt_nodes/ros_node.hpp"

namespace graphen::bt {

namespace {

constexpr const char * kDefaultTopic = "/face_tracker/target";
constexpr int kDefaultTimeoutMs = 500;
// 從未收到資料時的 lost_s：大於任何合理的門檻
constexpr double kNeverReceived = 1.0e9;

double sign_of(double value) {
  return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0);
}

}  // namespace

GetPixelError::GetPixelError(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {
  const auto node = get_ros_node(config, name);

  std::string topic = kDefaultTopic;
  getInput("topic", topic);

  subscription_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
    topic, rclcpp::SensorDataQoS(),
    [latest = latest_](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(latest->mutex);
      latest->point = msg->point;
      latest->received = std::chrono::steady_clock::now();
    });
}

BT::PortsList GetPixelError::providedPorts() {
  return {
    BT::InputPort<std::string>("topic", kDefaultTopic, "geometry_msgs/PointStamped, pixels"),
    BT::InputPort<double>("image_width", "Image width (px)"),
    BT::InputPort<double>("image_height", "Image height (px)"),
    BT::InputPort<int>("timeout_ms", kDefaultTimeoutMs, "Target is lost after this long"),
    BT::OutputPort<double>("err_x", "Normalized error, right is positive"),
    BT::OutputPort<double>("err_y", "Normalized error, down is positive"),
    BT::OutputPort<double>("abs_x", "|err_x|"),
    BT::OutputPort<double>("abs_y", "|err_y|"),
    BT::OutputPort<double>("dir_x", "Sign of err_x (-1, 0, 1)"),
    BT::OutputPort<double>("dir_y", "Sign of err_y (-1, 0, 1)"),
    BT::OutputPort<double>("lost_s", "Seconds since the last detection"),
  };
}

BT::NodeStatus GetPixelError::tick() {
  double width = 0.0;
  double height = 0.0;
  if (!getInput("image_width", width) || !(width > 0.0)) {
    throw BT::RuntimeError("GetPixelError: [image_width] must be greater than zero");
  }
  if (!getInput("image_height", height) || !(height > 0.0)) {
    throw BT::RuntimeError("GetPixelError: [image_height] must be greater than zero");
  }
  int timeout_ms = kDefaultTimeoutMs;
  getInput("timeout_ms", timeout_ms);

  std::optional<geometry_msgs::msg::Point> point;
  std::chrono::steady_clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(latest_->mutex);
    point = latest_->point;
    received = latest_->received;
  }

  if (!point) {
    setOutput("lost_s", kNeverReceived);
    return BT::NodeStatus::FAILURE;
  }
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count();
  setOutput("lost_s", age);
  if (age * 1000.0 > timeout_ms) {
    return BT::NodeStatus::FAILURE;
  }

  // 以畫面中心為原點、半寬 / 半高為 1；偵測框邊緣可能略超出畫面，截在 [-1, 1]
  const double err_x = std::clamp((point->x - width / 2.0) / (width / 2.0), -1.0, 1.0);
  const double err_y = std::clamp((point->y - height / 2.0) / (height / 2.0), -1.0, 1.0);
  setOutput("err_x", err_x);
  setOutput("err_y", err_y);
  setOutput("abs_x", std::abs(err_x));
  setOutput("abs_y", std::abs(err_y));
  setOutput("dir_x", sign_of(err_x));
  setOutput("dir_y", sign_of(err_y));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace graphen::bt
