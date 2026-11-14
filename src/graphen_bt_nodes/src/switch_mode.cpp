#include "graphen_bt_nodes/switch_mode.hpp"

#include "graphen_bt_nodes/ros_node.hpp"

namespace graphen::bt {

namespace {

constexpr const char * kDefaultActionName = "/switch_group_controller/switch_mode";
constexpr int kDefaultTimeoutMs = 1000;
constexpr int kDefaultFinishTimeoutMs = 0;
constexpr const char * kDefaultOnSameMode = "ignore";

// on_same_mode port 的字串 → goal 的常數
std::optional<int8_t> parse_on_same_mode(const std::string & text) {
  using Goal = graphen_interfaces::action::SwitchMode::Goal;
  if (text == "ignore") {
    return Goal::ON_SAME_MODE_IGNORE;
  }
  if (text == "restart") {
    return Goal::ON_SAME_MODE_RESTART;
  }
  if (text == "restart_if_finished") {
    return Goal::ON_SAME_MODE_RESTART_IF_FINISHED;
  }
  return std::nullopt;
}

}  // namespace

SwitchMode::SwitchMode(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), logger_(rclcpp::get_logger("BT")) {
  const auto node = get_ros_node(config, name);
  logger_ = node->get_logger();

  action_name_ = kDefaultActionName;
  getInput("action_name", action_name_);
  client_ = rclcpp_action::create_client<ActionT>(node, action_name_);
}

BT::PortsList SwitchMode::providedPorts() {
  return {
    BT::InputPort<std::string>("action_name", kDefaultActionName, "SwitchMode action name"),
    BT::InputPort<std::string>("mode", "Mode name to switch to"),
    BT::InputPort<int>("timeout_ms", kDefaultTimeoutMs,
      "Timeout for server discovery and the switch itself"),
    BT::InputPort<bool>("wait_finished", false,
      "Return SUCCESS only after the mode's motion has finished"),
    BT::InputPort<std::string>("on_same_mode", kDefaultOnSameMode,
      "When already in this mode: ignore, restart or restart_if_finished"),
    BT::InputPort<int>("finish_timeout_ms", kDefaultFinishTimeoutMs,
      "With wait_finished: limit on waiting for the motion to finish, 0 = no limit"),
  };
}

BT::NodeStatus SwitchMode::onStart() {
  if (!getInput("mode", pending_mode_)) {
    throw BT::RuntimeError("SwitchMode: missing input [mode]");
  }
  int timeout_ms = kDefaultTimeoutMs;
  getInput("timeout_ms", timeout_ms);
  deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  wait_finished_ = false;
  getInput("wait_finished", wait_finished_);
  std::string on_same_mode = kDefaultOnSameMode;
  getInput("on_same_mode", on_same_mode);
  const auto parsed = parse_on_same_mode(on_same_mode);
  if (!parsed) {
    throw BT::RuntimeError(
            "SwitchMode: [on_same_mode] must be ignore, restart or restart_if_finished, got '" +
            on_same_mode + "'");
  }
  on_same_mode_ = *parsed;
  int finish_timeout_ms = kDefaultFinishTimeoutMs;
  getInput("finish_timeout_ms", finish_timeout_ms);
  if (finish_timeout_ms < 0) {
    throw BT::RuntimeError("SwitchMode: [finish_timeout_ms] must not be negative");
  }
  finish_timeout_.reset();
  if (finish_timeout_ms > 0) {
    finish_timeout_ = std::chrono::milliseconds(finish_timeout_ms);
  }
  finish_deadline_.reset();
  accepted_ = false;

  request_.reset();
  return try_send_goal();
}

BT::NodeStatus SwitchMode::onRunning() {
  if (!request_) {
    return try_send_goal();
  }

  Request::Status status;
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
      RCLCPP_INFO(logger_, "SwitchMode: %s mode [ %s ]",
        wait_finished_ ? "finished" : "switched to", pending_mode_.c_str());
      return BT::NodeStatus::SUCCESS;
    case Request::Status::Failed:
      request_.reset();
      RCLCPP_WARN(logger_, "SwitchMode: mode [ %s ] failed: %s",
        pending_mode_.c_str(), message.c_str());
      return BT::NodeStatus::FAILURE;
    case Request::Status::Pending:
      break;
  }

  const auto now = std::chrono::steady_clock::now();
  if (wait_finished_ && accepted) {
    // goal 已被接受（模式已切換），之後只受 finish_timeout_ms 限制
    if (!accepted_) {
      accepted_ = true;
      if (finish_timeout_) {
        finish_deadline_ = now + *finish_timeout_;
      }
    }
    if (finish_deadline_ && now > *finish_deadline_) {
      RCLCPP_WARN(logger_,
        "SwitchMode: mode [ %s ] did not finish in time (%.2f s elapsed); it keeps running",
        pending_mode_.c_str(), elapsed);
      cancel_request();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (now > deadline_) {
    RCLCPP_WARN(logger_, "SwitchMode: mode [ %s ] timed out", pending_mode_.c_str());
    cancel_request();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void SwitchMode::onHalted() {
  cancel_request();
}

BT::NodeStatus SwitchMode::try_send_goal() {
  if (!client_->action_server_is_ready()) {
    if (std::chrono::steady_clock::now() > deadline_) {
      RCLCPP_WARN(logger_, "SwitchMode: action server [ %s ] is not available",
        action_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  auto request = std::make_shared<Request>();

  rclcpp_action::Client<ActionT>::SendGoalOptions options;
  options.goal_response_callback = [request](GoalHandle::SharedPtr goal_handle) {
      std::lock_guard<std::mutex> lock(request->mutex);
      if (!goal_handle) {
        request->status = Request::Status::Failed;
        request->message = "goal rejected";
      } else {
        request->goal_handle = std::move(goal_handle);
      }
    };
  options.feedback_callback =
    [request](GoalHandle::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(request->mutex);
      request->elapsed = feedback->elapsed;
    };
  options.result_callback = [request](const GoalHandle::WrappedResult & result) {
      std::lock_guard<std::mutex> lock(request->mutex);
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        result.result->success)
      {
        request->status = Request::Status::Succeeded;
      } else {
        request->status = Request::Status::Failed;
        request->message = result.result && !result.result->message.empty() ?
          result.result->message : "no result";
      }
    };

  ActionT::Goal goal;
  goal.mode = pending_mode_;
  goal.wait_until_finished = wait_finished_;
  goal.on_same_mode = on_same_mode_;
  client_->async_send_goal(goal, options);
  request_ = std::move(request);
  return BT::NodeStatus::RUNNING;
}

void SwitchMode::cancel_request() {
  if (!request_) {
    return;
  }
  GoalHandle::SharedPtr goal_handle;
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

}  // namespace graphen::bt
