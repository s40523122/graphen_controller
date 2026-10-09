#ifndef GRAPHEN_BT_NODES__ROS_NODE_HPP_
#define GRAPHEN_BT_NODES__ROS_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/tree_node.h"
#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

// 從 blackboard 的 "node" key 取得執行器放入的 ROS node（Nav2 慣例）。
// 取不到時丟出 BT::RuntimeError，讓建樹階段就發現設定錯誤。
inline rclcpp::Node::SharedPtr get_ros_node(
  const BT::NodeConfiguration & config, const std::string & name) {
  rclcpp::Node::SharedPtr node;
  if (!config.blackboard || !config.blackboard->get("node", node) || !node) {
    throw BT::RuntimeError(name, ": missing ROS node in blackboard key [node]");
  }
  return node;
}

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__ROS_NODE_HPP_
