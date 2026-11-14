#ifndef GRAPHEN_BT_NODES__SWITCH_MODE_HPP_
#define GRAPHEN_BT_NODES__SWITCH_MODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
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
// wait_finished 為 true 時，等 controller 回報這個模式的動作播完才回傳 SUCCESS，期間回傳 RUNNING；
// 被其他模式取代、動作失敗或超過 finish_timeout_ms 時回傳 FAILURE。
// 被 halt（或逾時）時會 cancel goal，但 controller 只是不再回報，**動作會繼續執行**：
// 要停下來必須另外切到別的模式（例如 hold）。
// 目前的模式不可中斷（<mode>.interruptible: false）而被拒絕時回傳 FAILURE。
//
// Ports：
//   action_name       (string) 只在建構時讀取，預設 /switch_group_controller/switch_mode
//   mode              (string) 模式名稱，必須列在 controller 的 modes 中
//   timeout_ms        (int)    預設 1000。等待 server 出現並完成切換的上限；
//                              wait_finished 時只算到 goal 被接受為止
//   wait_finished     (bool)   預設 false。true 時等動作播完才回傳 SUCCESS
//   on_same_mode      (string) 已經在這個模式時的處理方式，預設 ignore：
//                              ignore（不重新開始）、restart（從頭重播）、
//                              restart_if_finished（目前這次播放已結束才重播，播放中重複送出的指令被吃掉）
//   finish_timeout_ms (int)    預設 0。wait_finished 時，goal 被接受之後等待播完的上限；0 表示不限
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
    // controller 回報的進入模式後經過的時間（秒）
    double elapsed{0.0};
  };

  // server 已就緒時送出 goal；否則在 timeout 前回傳 RUNNING 持續等待
  BT::NodeStatus try_send_goal();
  void cancel_request();

  rclcpp::Logger logger_;
  std::string action_name_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;

  std::shared_ptr<Request> request_;
  std::string pending_mode_;
  bool wait_finished_{false};
  int8_t on_same_mode_{ActionT::Goal::ON_SAME_MODE_IGNORE};
  // 等待 server 與切換完成的期限（wait_finished 時只算到 goal 被接受）
  std::chrono::steady_clock::time_point deadline_;
  // wait_finished 時等待播完的上限；goal 被接受時才開始計時（finish_deadline_）
  std::optional<std::chrono::milliseconds> finish_timeout_;
  std::optional<std::chrono::steady_clock::time_point> finish_deadline_;
  bool accepted_{false};
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__SWITCH_MODE_HPP_
