#include "graphen_bt_nodes/sys_count_nodes.hpp"

namespace graphen::bt {

namespace {

constexpr const char * kCountKey = "sys_count";
constexpr const char * kCountParam = "gp_sys_count";

// 從 blackboard 取得執行器放入的 ROS node
rclcpp::Node::SharedPtr get_ros_node(const BT::NodeConfiguration & config, const std::string & name) {
  rclcpp::Node::SharedPtr node;
  if (!config.blackboard || !config.blackboard->get("node", node) || !node) {
    throw BT::RuntimeError(name, ": missing ROS node in blackboard key [node]");
  }
  return node;
}

int get_count(const BT::NodeConfiguration & config) {
  int count = 0;
  if (!config.blackboard->get(kCountKey, count)) {
    count = 0;
  }
  return count;
}

void set_count(const BT::NodeConfiguration & config, const rclcpp::Node::SharedPtr & node, int count) {
  config.blackboard->set(kCountKey, count);
  node->set_parameter(rclcpp::Parameter(kCountParam, count));
}

}  // namespace

IncrementSysCount::IncrementSysCount(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config), ros_node_(get_ros_node(config, name)) {}

BT::NodeStatus IncrementSysCount::tick() {
  set_count(config(), ros_node_, get_count(config()) + 1);
  return BT::NodeStatus::SUCCESS;
}

CheckSysCount::CheckSysCount(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}

BT::PortsList CheckSysCount::providedPorts() {
  return {BT::InputPort<int>("target")};
}

BT::NodeStatus CheckSysCount::tick() {
  int target = 0;
  if (!getInput("target", target)) {
    throw BT::RuntimeError("CheckSysCount: missing input [target]");
  }
  return get_count(config()) >= target ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ClearSysCount::ClearSysCount(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config), ros_node_(get_ros_node(config, name)) {}

BT::NodeStatus ClearSysCount::tick() {
  set_count(config(), ros_node_, 0);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace graphen::bt
