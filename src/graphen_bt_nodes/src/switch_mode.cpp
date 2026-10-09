#include "graphen_bt_nodes/switch_mode.hpp"

#include "graphen_bt_nodes/ros_node.hpp"

namespace graphen::bt {

namespace {

constexpr const char * kDefaultActionName = "/switch_group_controller/switch_mode";
constexpr int kDefaultTimeoutMs = 1000;

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
    BT::InputPort<int>("timeout_ms", kDefaultTimeoutMs, "Timeout including server discovery"),
  };
}

BT::NodeStatus SwitchMode::onStart() {
  if (!getInput("mode", pending_mode_)) {
    throw BT::RuntimeError("SwitchMode: missing input [mode]");
  }
  int timeout_ms = kDefaultTimeoutMs;
  getInput("timeout_ms", timeout_ms);
  deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  request_.reset();
  return try_send_goal();
}

BT::NodeStatus SwitchMode::onRunning() {
  if (!request_) {
    return try_send_goal();
  }

  Request::Status status;
  std::string message;
  {
    std::lock_guard<std::mutex> lock(request_->mutex);
    status = request_->status;
    message = request_->message;
  }

  switch (status) {
    case Request::Status::Succeeded:
      request_.reset();
      RCLCPP_INFO(logger_, "SwitchMode: switched to mode [ %s ]", pending_mode_.c_str());
      return BT::NodeStatus::SUCCESS;
    case Request::Status::Failed:
      request_.reset();
      RCLCPP_WARN(logger_, "SwitchMode: mode [ %s ] failed: %s",
        pending_mode_.c_str(), message.c_str());
      return BT::NodeStatus::FAILURE;
    case Request::Status::Pending:
      break;
  }

  if (std::chrono::steady_clock::now() > deadline_) {
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
  options.result_callback = [request](const GoalHandle::WrappedResult & result) {
      std::lock_guard<std::mutex> lock(request->mutex);
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        result.result->success)
      {
        request->status = Request::Status::Succeeded;
      } else {
        request->status = Request::Status::Failed;
        request->message = result.result ? result.result->message : "no result";
      }
    };

  ActionT::Goal goal;
  goal.mode = pending_mode_;
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
