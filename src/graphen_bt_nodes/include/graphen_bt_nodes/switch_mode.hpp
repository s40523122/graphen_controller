#ifndef GRAPHEN_BT_NODES__SWITCH_MODE_HPP_
#define GRAPHEN_BT_NODES__SWITCH_MODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "graphen_interfaces/action/switch_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace graphen::bt {

// 透過 graphen_interfaces/action/SwitchMode 要求 controller 切換模式。
//
// 非阻塞：goal 以 async 送出，回應由 bt_runner 的 executor thread 處理，
// 期間回傳 RUNNING。goal 被 reject、server 回報失敗，或 timeout_ms 內
// （含等待 server 出現）沒有完成時，回傳 FAILURE。
//
// 每次執行都會送出 goal（不快取上一次的模式），應放在只有換向時才會走到的分支中。
//
// Ports：
//   action_name  (string) 只在建構時讀取，預設 /switch_group_controller/switch_mode
//   mode         (string) 模式名稱，必須列在 controller 的 modes 中
//   timeout_ms   (int)    預設 1000
//
// 需要 ROS node：從 blackboard 的 "node" key 取得。
class SwitchMode : public BT::StatefulActionNode {
public:
  using ActionT = graphen_interfaces::action::SwitchMode;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  SwitchMode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // 一次 goal 請求的狀態；callback 只寫入自己所屬的 Request，
  // halt 後才回來的舊 callback 不會影響新的請求
  struct Request {
    enum class Status {Pending, Succeeded, Failed};

    std::mutex mutex;
    Status status{Status::Pending};
    std::string message;
    GoalHandle::SharedPtr goal_handle;
  };

  // server 已就緒時送出 goal；否則在 timeout 前回傳 RUNNING 持續等待
  BT::NodeStatus try_send_goal();
  void cancel_request();

  rclcpp::Logger logger_;
  std::string action_name_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;

  std::shared_ptr<Request> request_;
  std::string pending_mode_;
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__SWITCH_MODE_HPP_
