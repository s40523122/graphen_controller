#include "graphen_bt_nodes/joint_velocity.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "graphen_bt_nodes/ros_node.hpp"

namespace graphen::bt {

namespace {

constexpr const char * kPlayChoreoAction = "/switch_group_controller/play_choreo";
constexpr const char * kDefaultStatesTopic = "/switch_group_controller/states";
constexpr double kDefaultHorizon = 1.0;
constexpr double kDegToRad = M_PI / 180.0;
// 移動距離換算的時間短於這個值時視為已在邊界，改成 hold（編舞的 keyframe 時間必須 > 0）
constexpr double kMinSegment = 0.01;

std::vector<std::string> split_names(const std::string & text) {
  std::vector<std::string> names;
  for (const auto & part : BT::splitString(text, ';')) {
    std::string name(part.data(), part.size());
    name.erase(0, name.find_first_not_of(" \t"));
    name.erase(name.find_last_not_of(" \t") + 1);
    if (!name.empty()) {
      names.push_back(std::move(name));
    }
  }
  return names;
}

std::vector<double> required_doubles(
  const BT::TreeNode & node, const std::string & port, size_t size)
{
  std::string text;
  if (!node.getInput(port, text)) {
    throw BT::RuntimeError("JointVelocity: missing input [" + port + "]");
  }
  std::vector<double> values;
  try {
    values = BT::convertFromString<std::vector<double>>(text);
  } catch (const std::exception & e) {
    throw BT::RuntimeError("JointVelocity: [" + port + "] is not a list of numbers: " + text);
  }
  if (values.size() != size) {
    throw BT::RuntimeError(
            "JointVelocity: [" + port + "] must contain " + std::to_string(size) + " values, got " +
            std::to_string(values.size()));
  }
  if (std::any_of(values.begin(), values.end(), [](double v) {return !std::isfinite(v);})) {
    throw BT::RuntimeError("JointVelocity: [" + port + "] must be finite");
  }
  return values;
}

}  // namespace

JointVelocity::JointVelocity(const std::string & name, const BT::NodeConfiguration & config)
: PlaybackActionNode(name, config, "JointVelocity", kPlayChoreoAction),
  node_logger_(rclcpp::get_logger("BT")) {
  const auto node = get_ros_node(config, name);
  node_logger_ = node->get_logger();

  std::string topic = kDefaultStatesTopic;
  getInput("states_topic", topic);
  // 與 StateInterfacePublisher 的發布端 QoS 相同
  subscription_ = node->create_subscription<control_msgs::msg::DynamicJointState>(
    topic, rclcpp::SystemDefaultsQoS(),
    [latest = latest_](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(latest->mutex);
      latest->msg = std::move(msg);
    });
}

BT::PortsList JointVelocity::providedPorts() {
  auto ports = common_ports(kPlayChoreoAction);
  // 續送的內容只有目標位置不同，但到達邊界後可能完全相同：一律重新開始，讓有效時間重新計算
  ports.erase("on_same");
  ports.insert(BT::InputPort<std::string>("on_same", "restart",
    "When this is already playing: ignore, restart or restart_if_finished"));
  ports.insert(BT::InputPort<std::string>("joints", "Joint names separated by ;"));
  // 字串型別：通常和 SendOnChange / Switch 的命令共用同一個 blackboard key（v3 要求同一個 key 的型別一致）
  ports.insert(BT::InputPort<std::string>("velocities",
    "Velocity of each joint in deg/s, separated by ;"));
  ports.insert(BT::InputPort<std::string>("lower", "Lower bound of each joint (rad), separated by ;"));
  ports.insert(BT::InputPort<std::string>("upper", "Upper bound of each joint (rad), separated by ;"));
  ports.insert(BT::InputPort<double>("horizon", kDefaultHorizon,
    "How long one command keeps moving (s); resend before it expires"));
  ports.insert(BT::InputPort<std::string>("states_topic", kDefaultStatesTopic,
    "DynamicJointState topic for the current positions"));
  return ports;
}

bool JointVelocity::read_positions(
  const control_msgs::msg::DynamicJointState & msg, const std::vector<std::string> & joints,
  std::vector<double> & positions)
{
  positions.assign(joints.size(), 0.0);
  for (size_t j = 0; j < joints.size(); ++j) {
    bool found = false;
    for (size_t i = 0; i < msg.joint_names.size() && i < msg.interface_values.size() && !found; ++i) {
      if (msg.joint_names[i] != joints[j]) {
        continue;
      }
      const auto & values = msg.interface_values[i];
      for (size_t k = 0; k < values.interface_names.size() && k < values.values.size(); ++k) {
        if (values.interface_names[k] == "position") {
          positions[j] = values.values[k];
          found = true;
          break;
        }
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

BT::NodeStatus JointVelocity::onStart() {
  std::string joints_text;
  if (!getInput("joints", joints_text)) {
    throw BT::RuntimeError("JointVelocity: missing input [joints]");
  }
  joints_ = split_names(joints_text);
  if (joints_.empty()) {
    throw BT::RuntimeError("JointVelocity: [joints] is empty");
  }

  control_msgs::msg::DynamicJointState::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(latest_->mutex);
    msg = latest_->msg;
  }
  // 狀態還沒到（controller 尚未啟動）是執行中的情況，不是設定錯誤：回傳 FAILURE 讓樹重試
  if (!msg || !read_positions(*msg, joints_, positions_)) {
    RCLCPP_WARN_THROTTLE(node_logger_, steady_clock_, 2000,
      "JointVelocity: no position state for all of [ %s ] yet", joints_text.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return PlaybackActionNode::onStart();
}

std::string JointVelocity::fill_goal(Goal & goal) {
  const auto velocities = required_doubles(*this, "velocities", joints_.size());
  const auto lower = required_doubles(*this, "lower", joints_.size());
  const auto upper = required_doubles(*this, "upper", joints_.size());
  double horizon = kDefaultHorizon;
  getInput("horizon", horizon);
  if (!(horizon > kMinSegment) || !std::isfinite(horizon)) {
    throw BT::RuntimeError("JointVelocity: [horizon] must be greater than " +
            std::to_string(kMinSegment));
  }

  std::ostringstream yaml;
  yaml << std::setprecision(9);
  yaml << "version: 1\ndefault_interp: linear\ntracks:\n";
  std::ostringstream label;
  label << std::fixed << std::setprecision(1);
  for (size_t j = 0; j < joints_.size(); ++j) {
    if (!(lower[j] < upper[j])) {
      throw BT::RuntimeError("JointVelocity: [lower] must be below [upper] for " + joints_[j]);
    }
    const double velocity = velocities[j] * kDegToRad;
    const double target =
      std::clamp(positions_[j] + velocity * horizon, lower[j], upper[j]);
    const double duration =
      velocity != 0.0 ? std::abs(target - positions_[j]) / std::abs(velocity) : 0.0;

    // 一條軌道一個 joint：被邊界截短的 joint 提早到達，不影響其他 joint 的速度
    yaml << "  " << joints_[j] << ":\n"
         << "    joints: [" << joints_[j] << "]\n"
         << "    keyframes:\n";
    if (duration >= kMinSegment) {
      yaml << "      - {t: " << duration << ", positions: {" << joints_[j] << ": " << target <<
        "}}\n";
      label << (j == 0 ? "" : " ") << joints_[j] << "=" << velocities[j] << "deg/s";
    } else {
      // 速度為 0 或已在邊界：停在切換當下的命令位置
      yaml << "      - {t: " << horizon << ", hold: true}\n";
      label << (j == 0 ? "" : " ") << joints_[j] << "=hold";
    }
  }
  goal.content = yaml.str();
  return "velocity " + label.str();
}

}  // namespace graphen::bt
