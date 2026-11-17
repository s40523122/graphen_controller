#include "graphen_controllers/playback_arbiter.hpp"

#include <utility>

#include "rclcpp/logging.hpp"

namespace graphen::control {

std::string playback_label(const std::string & mode, const std::string & item) {
  return item.empty() ? mode : mode + ":" + item;
}

PlaybackArbiter::PlaybackArbiter(LocomotionPlanner * planner, rclcpp::Logger logger)
: planner_(planner), logger_(std::move(logger)) {}

bool PlaybackArbiter::current_ended() const {
  const auto progress = planner_->progress();
  // planner 還沒開始這一次播放時，progress 仍是上一個 Motion 的：視為還在播放
  return progress.sequence == sequence_ &&
         (progress.status == MotionStatus::kFinished || progress.status == MotionStatus::kFailed);
}

PlaybackOutcome PlaybackArbiter::request(PlaybackRequest request, OnSame on_same) {
  std::lock_guard<std::mutex> lock(mutex_);
  PlaybackOutcome outcome;
  const bool same = !mode_.empty() && identity_ == request.identity;

  // 已經在播同一個：依這一次指令的 on_same 決定是否重新開始
  if (same) {
    const bool restart = on_same == OnSame::kRestart ||
      (on_same == OnSame::kRestartIfFinished && current_ended());
    if (!restart) {
      outcome.status = PlaybackOutcome::Status::kUnchanged;
      outcome.sequence = sequence_;
      return outcome;
    }
  }

  // 目前在播的不可中斷、而且還沒結束：只有 preempt 的（停止類）可以切入。重播同一個也算中斷
  if (!mode_.empty() && !policy_.interruptible && !request.policy.preempt && !current_ended()) {
    outcome.status = PlaybackOutcome::Status::kRejected;
    outcome.message = "[ " + playback_label(mode_, item_) +
      " ] is not interruptible until it finishes; [ " +
      playback_label(request.mode, request.item) + " ] rejected.";
    return outcome;
  }

  RCLCPP_INFO(logger_, "%s [ %s ]%s%s", same ? "Restart" : "Change to",
    playback_label(request.mode, request.item).c_str(),
    request.detail.empty() ? "" : ", ", request.detail.c_str());
  mode_ = std::move(request.mode);
  item_ = std::move(request.item);
  identity_ = std::move(request.identity);
  policy_ = request.policy;
  sequence_ = planner_->set_motion(std::move(request.motion));
  outcome.status = PlaybackOutcome::Status::kSwitched;
  outcome.sequence = sequence_;
  return outcome;
}

uint64_t PlaybackArbiter::force(PlaybackRequest request) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = std::move(request.mode);
  item_ = std::move(request.item);
  identity_ = std::move(request.identity);
  policy_ = request.policy;
  sequence_ = planner_->set_motion(std::move(request.motion));
  return sequence_;
}

PlaybackArbiter::Current PlaybackArbiter::current() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {mode_, item_, sequence_};
}

}  // namespace graphen::control
