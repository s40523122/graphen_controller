#ifndef GRAPHEN_BT_NODES__PLAYBACK_ACTION_NODE_HPP_
#define GRAPHEN_BT_NODES__PLAYBACK_ACTION_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/action_node.h"
#include "graphen_bt_nodes/ros_node.hpp"
#include "graphen_interfaces/msg/playback_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace graphen::bt {

// 播放類 action（SwitchMode / PlayChoreo / PlayBvh / RunPolicy）的 BT 節點共用的部分。
// 這些 action 的 goal 都有 PlaybackOptions、result 都是 PlaybackResult、feedback 都是 PlaybackFeedback，
// 所以送出、等待與逾時的處理完全相同；子類別只定義自己的 port 與 goal 的專用欄位（fill_goal()）。
//
// 非阻塞：goal 以 async 送出，回應由 bt_runner 的 executor thread 處理，期間回傳 RUNNING。
// goal 被 reject、controller 回報失敗（內容有誤、目前在播的不可中斷…），或 timeout_ms 內
// （含等待 server 出現）沒有完成時，回傳 FAILURE。
//
// 每次執行都會送出 goal（不快取上一次的結果），應放在只有需要時才會走到的分支中。
//
// wait_finished 為 true 時，等 controller 回報動作播完才回傳 SUCCESS，期間回傳 RUNNING；
// 被其他播放取代、動作失敗或超過 finish_timeout_ms 時回傳 FAILURE。
// 被 halt（或逾時）時會 cancel goal，但 controller 只是不再回報，**動作會繼續執行**：
// 要停下來必須另外切到停止類的模式（SwitchMode 到 hold）。
//
// 共通的 port：
//   action_name       (string) 只在建構時讀取，預設值由子類別提供
//   timeout_ms        (int)    預設 1000。等待 server 出現並完成切換的上限（含載入內容的時間）；
//                              wait_finished 時只算到 goal 被接受為止
//   wait_finished     (bool)   預設 false。true 時等動作播完才回傳 SUCCESS
//   on_same           (string) 目前在播的就是同一個時的處理方式，預設 ignore：
//                              ignore（不重新開始）、restart（從頭重播）、
//                              restart_if_finished（目前這次播放已結束才重播，播放中重複送出的指令被吃掉）
//   finish_timeout_ms (int)    預設 0。wait_finished 時，goal 被接受之後等待播完的上限；0 表示不限
//
// 需要 ROS node：從 blackboard 的 "node" key 取得。
template<typename ActionT>
class PlaybackActionNode : public BT::StatefulActionNode {
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  BT::NodeStatus onStart() override {
    Goal goal;
    // 子類別讀取自己的 port；label 是 log 中用來稱呼這次播放的文字
    label_ = fill_goal(goal);

    int timeout_ms = kDefaultTimeoutMs;
    getInput("timeout_ms", timeout_ms);
    deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    wait_finished_ = false;
    getInput("wait_finished", wait_finished_);
    std::string on_same = kDefaultOnSame;
    getInput("on_same", on_same);
    const auto parsed = parse_on_same(on_same);
    if (!parsed) {
      throw BT::RuntimeError(
              type_ + ": [on_same] must be ignore, restart or restart_if_finished, got '" +
              on_same + "'");
    }
    int finish_timeout_ms = kDefaultFinishTimeoutMs;
    getInput("finish_timeout_ms", finish_timeout_ms);
    if (finish_timeout_ms < 0) {
      throw BT::RuntimeError(type_ + ": [finish_timeout_ms] must not be negative");
    }
    finish_timeout_.reset();
    if (finish_timeout_ms > 0) {
      finish_timeout_ = std::chrono::milliseconds(finish_timeout_ms);
    }
    finish_deadline_.reset();
    accepted_ = false;

    goal.options.on_same = *parsed;
    goal.options.wait_until_finished = wait_finished_;
    goal_ = std::move(goal);
    request_.reset();
    return try_send_goal();
  }

  BT::NodeStatus onRunning() override {
    if (!request_) {
      return try_send_goal();
    }

    typename Request::Status status;
    std::string message;
    bool accepted = false;
    double elapsed = 0.0;
    {
      std::lock_guard<std::mutex> lock(request_->mutex);
      status = request_->status;
      message = request_->message;
      accepted = request_->goal_handle != nullptr;
      elapsed = request_->elapsed;
    }

    switch (status) {
      case Request::Status::Succeeded:
        request_.reset();
        RCLCPP_INFO(logger_, "%s: %s [ %s ]", type_.c_str(),
          wait_finished_ ? "finished" : "switched to", label_.c_str());
        return BT::NodeStatus::SUCCESS;
      case Request::Status::Failed:
        request_.reset();
        RCLCPP_WARN(logger_, "%s: [ %s ] failed: %s", type_.c_str(), label_.c_str(),
          message.c_str());
        return BT::NodeStatus::FAILURE;
      case Request::Status::Pending:
        break;
    }

    const auto now = std::chrono::steady_clock::now();
    if (wait_finished_ && accepted) {
      // goal 已被接受，之後只受 finish_timeout_ms 限制
      if (!accepted_) {
        accepted_ = true;
        if (finish_timeout_) {
          finish_deadline_ = now + *finish_timeout_;
        }
      }
      if (finish_deadline_ && now > *finish_deadline_) {
        RCLCPP_WARN(logger_,
          "%s: [ %s ] did not finish in time (%.2f s elapsed); it keeps running",
          type_.c_str(), label_.c_str(), elapsed);
        cancel_request();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    if (now > deadline_) {
      RCLCPP_WARN(logger_, "%s: [ %s ] timed out", type_.c_str(), label_.c_str());
      cancel_request();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {cancel_request();}

protected:
  // type：節點類型的名稱（log 用）；default_action_name：action_name port 沒有設定時使用
  PlaybackActionNode(
    const std::string & name, const BT::NodeConfiguration & config, std::string type,
    const std::string & default_action_name)
  : BT::StatefulActionNode(name, config), logger_(rclcpp::get_logger("BT")),
    type_(std::move(type))
  {
    const auto node = get_ros_node(config, name);
    logger_ = node->get_logger();

    action_name_ = default_action_name;
    getInput("action_name", action_name_);
    client_ = rclcpp_action::create_client<ActionT>(node, action_name_);
  }

  // 所有播放類節點共通的 port；子類別的 providedPorts() 以它為基礎再加上自己的
  static BT::PortsList common_ports(const std::string & default_action_name) {
    return {
      BT::InputPort<std::string>("action_name", default_action_name, "Action name"),
      BT::InputPort<int>("timeout_ms", kDefaultTimeoutMs,
        "Timeout for server discovery and the switch itself (including loading)"),
      BT::InputPort<bool>("wait_finished", false,
        "Return SUCCESS only after the motion has finished"),
      BT::InputPort<std::string>("on_same", kDefaultOnSame,
        "When this is already playing: ignore, restart or restart_if_finished"),
      BT::InputPort<int>("finish_timeout_ms", kDefaultFinishTimeoutMs,
        "With wait_finished: limit on waiting for the motion to finish, 0 = no limit"),
    };
  }

  // 讀取子類別自己的 port，填入 goal 的專用欄位（options 由基類填），回傳 log 中稱呼這次播放的文字。
  // port 缺少或不合法時丟出 BT::RuntimeError
  virtual std::string fill_goal(Goal & goal) = 0;

  const std::string & node_type_name() const {return type_;}

private:
  static constexpr int kDefaultTimeoutMs = 1000;
  static constexpr int kDefaultFinishTimeoutMs = 0;
  static constexpr const char * kDefaultOnSame = "ignore";

  // 一次 goal 請求的狀態；callback 只寫入自己所屬的 Request，
  // halt 後才回來的舊 callback 不會影響新的請求
  struct Request {
    enum class Status {Pending, Succeeded, Failed};

    std::mutex mutex;
    Status status{Status::Pending};
    std::string message;
    typename GoalHandle::SharedPtr goal_handle;
    // controller 回報的開始播放後經過的時間（秒）
    double elapsed{0.0};
  };

  // on_same port 的字串 → PlaybackOptions 的常數
  static std::optional<int8_t> parse_on_same(const std::string & text) {
    using Options = graphen_interfaces::msg::PlaybackOptions;
    if (text == "ignore") {
      return Options::ON_SAME_IGNORE;
    }
    if (text == "restart") {
      return Options::ON_SAME_RESTART;
    }
    if (text == "restart_if_finished") {
      return Options::ON_SAME_RESTART_IF_FINISHED;
    }
    return std::nullopt;
  }

  // server 已就緒時送出 goal；否則在 timeout 前回傳 RUNNING 持續等待
  BT::NodeStatus try_send_goal() {
    if (!client_->action_server_is_ready()) {
      if (std::chrono::steady_clock::now() > deadline_) {
        RCLCPP_WARN(logger_, "%s: action server [ %s ] is not available", type_.c_str(),
          action_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    auto request = std::make_shared<Request>();

    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback = [request](typename GoalHandle::SharedPtr goal_handle) {
        std::lock_guard<std::mutex> lock(request->mutex);
        if (!goal_handle) {
          request->status = Request::Status::Failed;
          request->message = "goal rejected";
        } else {
          request->goal_handle = std::move(goal_handle);
        }
      };
    options.feedback_callback =
      [request](typename GoalHandle::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(request->mutex);
        request->elapsed = feedback->feedback.elapsed;
      };
    options.result_callback = [request](const typename GoalHandle::WrappedResult & wrapped) {
        std::lock_guard<std::mutex> lock(request->mutex);
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
          wrapped.result->result.success)
        {
          request->status = Request::Status::Succeeded;
        } else {
          request->status = Request::Status::Failed;
          request->message = wrapped.result && !wrapped.result->result.message.empty() ?
            wrapped.result->result.message : "no result";
        }
      };

    client_->async_send_goal(goal_, options);
    request_ = std::move(request);
    return BT::NodeStatus::RUNNING;
  }

  void cancel_request() {
    if (!request_) {
      return;
    }
    typename GoalHandle::SharedPtr goal_handle;
    {
      std::lock_guard<std::mutex> lock(request_->mutex);
      goal_handle = request_->goal_handle;
    }
    request_.reset();
    if (!goal_handle) {
      return;
    }
    try {
      client_->async_cancel_goal(goal_handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // goal 已經結束，不需要取消
    }
  }

  rclcpp::Logger logger_;
  const std::string type_;
  std::string action_name_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;

  std::shared_ptr<Request> request_;
  Goal goal_;
  std::string label_;
  bool wait_finished_{false};
  // 等待 server 與切換完成的期限（wait_finished 時只算到 goal 被接受）
  std::chrono::steady_clock::time_point deadline_;
  // wait_finished 時等待播完的上限；goal 被接受時才開始計時（finish_deadline_）
  std::optional<std::chrono::milliseconds> finish_timeout_;
  std::optional<std::chrono::steady_clock::time_point> finish_deadline_;
  bool accepted_{false};
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__PLAYBACK_ACTION_NODE_HPP_
