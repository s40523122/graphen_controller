#ifndef GRAPHEN_BT_NODES__GET_STATE_VALUE_HPP_
#define GRAPHEN_BT_NODES__GET_STATE_VALUE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

// 從 control_msgs/DynamicJointState topic 取出指定 joint 的某個 interface 值。
//
// 建構時訂閱 topic，callback 在 bt_runner 的 executor thread 中只保存最新的 msg；
// tick 時才查詢 joint / interface，所以這兩個 port 可以在執行中改變。
// 還沒收到資料，或 msg 中沒有指定的 joint / interface 時，回傳 FAILURE。
//
// Ports：
//   topic      (string) 只在建構時讀取，預設 /switch_group_controller/states
//   joint      (string) joint 名稱，例如 joint_arm1
//   interface  (string) interface 名稱，例如 position
//   value      (double, output)
//
// 需要 ROS node：從 blackboard 的 "node" key 取得。
class GetStateValue : public BT::SyncActionNode {
public:
  GetStateValue(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // callback 與 tick 共用；以 shared_ptr 持有，callback 不依賴本節點的生命週期
  struct Latest {
    std::mutex mutex;
    control_msgs::msg::DynamicJointState::ConstSharedPtr msg;
  };

  std::shared_ptr<Latest> latest_{std::make_shared<Latest>()};
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__GET_STATE_VALUE_HPP_
