#ifndef GRAPHEN_CONTROLLERS__PLAYBACK_SERVER_HPP_
#define GRAPHEN_CONTROLLERS__PLAYBACK_SERVER_HPP_

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "graphen_controllers/playback_arbiter.hpp"
#include "graphen_interfaces/msg/playback_feedback.hpp"
#include "graphen_interfaces/msg/playback_options.hpp"
#include "graphen_interfaces/msg/playback_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace graphen::control {

// PlaybackOptions.on_same → OnSame；超出範圍時回傳 nullopt
inline std::optional<OnSame> to_on_same(int8_t value) {
  using Options = graphen_interfaces::msg::PlaybackOptions;
  switch (value) {
    case Options::ON_SAME_IGNORE:
      return OnSame::kIgnore;
    case Options::ON_SAME_RESTART:
      return OnSame::kRestart;
    case Options::ON_SAME_RESTART_IF_FINISHED:
      return OnSame::kRestartIfFinished;
    default:
      return std::nullopt;
  }
}

// 讓 controller 可以用同一個 timer 檢查所有播放類 action 的等待中 goal
class PlaybackServerBase {
public:
  virtual ~PlaybackServerBase() = default;

  // 檢查 wait_until_finished 的 goal：播完、失敗、被取代、取消或 controller 停用時結束它，
  // 否則發布 feedback。在 executor thread 中定時呼叫
  virtual void poll() = 0;
};

// 播放類 action（SwitchMode / PlayChoreo / PlayBvh / RunPolicy）共用的 server。
//
// ActionT 的形式必須是：goal 有 `PlaybackOptions options`，result 有 `PlaybackResult result`，
// feedback 有 `PlaybackFeedback feedback`；其餘 goal 欄位由各類型自行定義，這裡不碰。
//
// 流程（都在 executor thread，non-RT）：
//   handle_goal      controller 沒有啟用、on_same 不合法或 Hooks::validate 不通過時 REJECT
//   handle_accepted  需要載入時先發布 STAGE_LOADING，再呼叫 Hooks::start（載入內容、送交 PlaybackArbiter）。
//                    start 丟出例外（內容有誤、檔案不存在…）或被 arbiter 拒絕時 abort，message 說明原因；
//                    這時還沒有切換，機器人繼續原本的動作
//   之後             不等待的 goal 立即成功；wait_until_finished 的 goal 交給 poll()
// cancel 只會停止等待，不會停止或改變目前的動作。
//
// 鎖的順序：waiting_mutex_ → PlaybackArbiter 內部的 mutex / Hooks 中的鎖。
template<typename ActionT>
class PlaybackServer : public PlaybackServerBase {
public:
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using FeedbackMsg = graphen_interfaces::msg::PlaybackFeedback;
  using ResultMsg = graphen_interfaces::msg::PlaybackResult;

  struct Hooks {
    // controller 是否在 active 狀態
    std::function<bool()> active;
    // goal 專用欄位的基本檢查（不做載入）：回傳非空字串時 REJECT，字串為原因（只寫入 log）
    std::function<std::string(const Goal &)> validate;
    // start 是否需要載入內容；true 時先發布 STAGE_LOADING。可以為空（不需要）
    std::function<bool(const Goal &)> needs_loading;
    // 載入內容並送交 arbiter；失敗時丟出例外，what() 會成為 result 的 message
    std::function<PlaybackOutcome(const Goal &, OnSame)> start;
    // 填入 result 的 interface_names / values（state 快照）
    std::function<void(ResultMsg &)> fill_state;
    // log 中用來稱呼這個 goal 的文字（例如 "choreo:show_1"）
    std::function<std::string(const Goal &)> describe;
  };

  // group：action 的 callback group；nullptr 表示 node 預設的 group。
  // 會載入內容的 action 要用獨立的 group，和 ~/cmd_vel 等其他 callback 分開
  PlaybackServer(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const std::string & name,
    rclcpp::CallbackGroup::SharedPtr group, PlaybackArbiter * arbiter, Hooks hooks)
  : logger_(node->get_logger()), name_(name), arbiter_(arbiter), hooks_(std::move(hooks))
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node, std::string(node->get_name()) + "/" + name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Goal> goal) {
        return handle_goal(*goal);
      },
      [](std::shared_ptr<GoalHandle>) {
        // 只有 wait_until_finished 的 goal 會停留在執行階段。cancel 只停止等待，由 poll() 結束 goal
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {handle_accepted(std::move(handle));},
      rcl_action_server_get_default_options(), std::move(group));
  }

  void poll() override {
    std::lock_guard<std::mutex> lock(waiting_mutex_);
    if (waiting_goals_.empty()) {
      return;
    }
    const bool active = hooks_.active();
    const auto progress = arbiter_->progress();
    const auto current = arbiter_->current();

    // 回傳 true 表示 goal 已結束，從清單移除
    const auto finish = [&](const WaitingGoal & waiting) {
        const auto & handle = waiting.handle;
        const auto label = hooks_.describe(*handle->get_goal());
        if (handle->is_canceling()) {
          handle->canceled(make_result(false, "Stopped waiting; the motion keeps running"));
          return true;
        }
        if (!active) {
          handle->abort(make_result(false, "Controller is not active"));
          return true;
        }
        if (waiting.sequence != current.sequence) {
          handle->abort(make_result(
              false, "Preempted by [ " + playback_label(current.mode, current.item) + " ]"));
          return true;
        }
        // planner 還沒切換到這一次播放時，progress 仍是上一個 Motion 的
        const bool entered = progress.sequence == waiting.sequence;
        if (entered && progress.status == MotionStatus::kFinished) {
          RCLCPP_INFO(logger_, "[ %s ] finished (%.2f s after it was entered)", label.c_str(),
            progress.elapsed);
          handle->succeed(make_result(true, "Finished"));
          return true;
        }
        if (entered && progress.status == MotionStatus::kFailed) {
          handle->abort(make_result(false, "[ " + label + " ] failed, see the controller log"));
          return true;
        }
        publish_feedback(*handle, FeedbackMsg::STAGE_PLAYING, entered ? progress.elapsed : 0.0);
        return false;
      };
    waiting_goals_.erase(
      std::remove_if(waiting_goals_.begin(), waiting_goals_.end(), finish), waiting_goals_.end());
  }

private:
  struct WaitingGoal {
    std::shared_ptr<GoalHandle> handle;
    uint64_t sequence{0};
  };

  rclcpp_action::GoalResponse handle_goal(const Goal & goal) {
    std::string reason;
    if (!hooks_.active()) {
      reason = "controller is not active";
    } else if (!to_on_same(goal.options.on_same)) {
      reason = "invalid on_same " + std::to_string(static_cast<int>(goal.options.on_same));
    } else {
      reason = hooks_.validate(goal);
    }
    if (!reason.empty()) {
      RCLCPP_WARN(logger_, "Rejected %s goal [ %s ]: %s.", name_.c_str(),
        hooks_.describe(goal).c_str(), reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(std::shared_ptr<GoalHandle> handle) {
    const auto goal = handle->get_goal();
    if (hooks_.needs_loading && hooks_.needs_loading(*goal)) {
      publish_feedback(*handle, FeedbackMsg::STAGE_LOADING, 0.0);
    }

    PlaybackOutcome outcome;
    try {
      // handle_goal 已檢查過 on_same
      outcome = hooks_.start(*goal, *to_on_same(goal->options.on_same));
    } catch (const std::exception & e) {
      // 內容有誤、檔案不存在等：還沒有切換，原本的動作不受影響
      outcome.status = PlaybackOutcome::Status::kRejected;
      outcome.message = e.what();
    }
    if (outcome.status == PlaybackOutcome::Status::kRejected) {
      // 不可中斷的判斷必須和切換在同一個 lock 中進行，所以在這裡（而不是 handle_goal）拒絕
      RCLCPP_WARN(logger_, "%s [ %s ]: %s", name_.c_str(), hooks_.describe(*goal).c_str(),
        outcome.message.c_str());
      handle->abort(make_result(false, outcome.message));
      return;
    }
    if (!goal->options.wait_until_finished) {
      handle->succeed(make_result(true,
        outcome.status == PlaybackOutcome::Status::kUnchanged ?
        "Already playing this, not restarted" : "Switched"));
      return;
    }
    // 等動作播完：交給 poll()
    std::lock_guard<std::mutex> lock(waiting_mutex_);
    waiting_goals_.push_back({std::move(handle), outcome.sequence});
  }

  std::shared_ptr<Result> make_result(bool success, const std::string & message) const {
    auto result = std::make_shared<Result>();
    result->result.success = success;
    result->result.message = message;
    const auto current = arbiter_->current();
    result->result.mode = current.mode;
    result->result.item = current.item;
    hooks_.fill_state(result->result);
    return result;
  }

  static void publish_feedback(GoalHandle & handle, uint8_t stage, double elapsed) {
    auto feedback = std::make_shared<Feedback>();
    feedback->feedback.stage = stage;
    feedback->feedback.elapsed = elapsed;
    handle.publish_feedback(feedback);
  }

  const rclcpp::Logger logger_;
  const std::string name_;
  PlaybackArbiter * const arbiter_;
  const Hooks hooks_;

  std::mutex waiting_mutex_;
  std::vector<WaitingGoal> waiting_goals_;
  // 最後宣告：server 的 callback 會用到上面的成員
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__PLAYBACK_SERVER_HPP_
