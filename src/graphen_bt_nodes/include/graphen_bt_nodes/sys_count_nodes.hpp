#ifndef GRAPHEN_BT_NODES__SYS_COUNT_NODES_HPP_
#define GRAPHEN_BT_NODES__SYS_COUNT_NODES_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

// sys_count 系列節點共用 blackboard 的 "sys_count"（int），
// Increment / Clear 同時把值同步到 ROS 參數 gp_sys_count 供外部觀察。
//
// Increment / Clear 需要 ROS node：從 blackboard 的 "node" key 取得
// （由執行器在建立 tree 前放入），取不到時建構會丟出 BT::RuntimeError。

// sys_count + 1
class IncrementSysCount : public BT::SyncActionNode {
public:
  IncrementSysCount(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
};

// sys_count >= target 時回傳 SUCCESS
//
// Ports：
//   target  (int)
class CheckSysCount : public BT::ConditionNode {
public:
  CheckSysCount(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

// sys_count 歸零
class ClearSysCount : public BT::SyncActionNode {
public:
  ClearSysCount(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__SYS_COUNT_NODES_HPP_
