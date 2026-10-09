#include "graphen_bt_nodes/get_state_value.hpp"

#include "graphen_bt_nodes/ros_node.hpp"

namespace graphen::bt {

namespace {

constexpr const char * kDefaultTopic = "/switch_group_controller/states";

}  // namespace

GetStateValue::GetStateValue(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {
  const auto node = get_ros_node(config, name);

  std::string topic = kDefaultTopic;
  getInput("topic", topic);

  // 與 StateInterfacePublisher 的發布端 QoS 相同
  subscription_ = node->create_subscription<control_msgs::msg::DynamicJointState>(
    topic, rclcpp::SystemDefaultsQoS(),
    [latest = latest_](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(latest->mutex);
      latest->msg = std::move(msg);
    });
}

BT::PortsList GetStateValue::providedPorts() {
  return {
    BT::InputPort<std::string>("topic", kDefaultTopic, "DynamicJointState topic"),
    BT::InputPort<std::string>("joint", "Joint name"),
    BT::InputPort<std::string>("interface", "Interface name"),
    BT::OutputPort<double>("value", "Latest interface value"),
  };
}

BT::NodeStatus GetStateValue::tick() {
  std::string joint;
  std::string interface;
  if (!getInput("joint", joint)) {
    throw BT::RuntimeError("GetStateValue: missing input [joint]");
  }
  if (!getInput("interface", interface)) {
    throw BT::RuntimeError("GetStateValue: missing input [interface]");
  }

  control_msgs::msg::DynamicJointState::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(latest_->mutex);
    msg = latest_->msg;
  }
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  for (size_t i = 0; i < msg->joint_names.size() && i < msg->interface_values.size(); ++i) {
    if (msg->joint_names[i] != joint) {
      continue;
    }
    const auto & values = msg->interface_values[i];
    for (size_t j = 0; j < values.interface_names.size() && j < values.values.size(); ++j) {
      if (values.interface_names[j] == interface) {
        setOutput("value", values.values[j]);
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace graphen::bt
