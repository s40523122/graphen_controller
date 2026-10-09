#include "graphen_controllers/switch_group_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <iterator>

#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::control {

namespace {

// 模式名稱會當作參數前綴，只允許不含 '.' 等特殊字元的識別字
bool is_valid_mode_name(const std::string & name) {
  return !name.empty() && std::all_of(name.begin(), name.end(), [](unsigned char c) {
             return std::isalnum(c) || c == '_';
           });
}

}  // namespace

controller_interface::CallbackReturn SwitchGroupController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>{});
    auto_declare<std::string>("sensor", std::string{});
    auto_declare<double>("state_publish_rate", 50.0);
    auto_declare<std::vector<std::string>>("modes", std::vector<std::string>{});
    auto_declare<std::string>("default_mode", std::string{});
    auto_declare<double>("planner_rate", 100.0);
    auto_declare<std::vector<double>>("planner_max_steps", std::vector<double>{});
    auto_declare<std::vector<std::string>>("target_state_interfaces", std::vector<std::string>{});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception caught during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwitchGroupController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  const auto logger = get_node()->get_logger();

  // reconfigure 時先釋放上一次的介面，避免舊 callback 存取即將被替換的 planner
  mode_subscriber_.reset();
  switch_mode_server_.reset();
  planner_.reset();

  command_interface_names_ = get_node()->get_parameter("command_interfaces").as_string_array();
  if (command_interface_names_.empty()) {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t num_commands = command_interface_names_.size();

  // 模式以名稱識別，名稱同時是該模式參數的前綴（<mode>.targets）
  const auto modes = get_node()->get_parameter("modes").as_string_array();
  if (modes.empty()) {
    RCLCPP_ERROR(logger, "'modes' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  mode_names_.clear();
  mode_targets_.clear();
  zero_targets_.assign(num_commands, 0.0);
  for (const auto & mode : modes) {
    if (!is_valid_mode_name(mode)) {
      RCLCPP_ERROR(logger,
        "Invalid mode name [ %s ]: must be non-empty and contain only [A-Za-z0-9_].",
        mode.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (std::find(mode_names_.begin(), mode_names_.end(), mode) != mode_names_.end()) {
      RCLCPP_ERROR(logger, "Duplicate mode [ %s ] in 'modes'.", mode.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    // controller node 會自動宣告 YAML 中的參數；沒寫在 YAML 時才在這裡宣告，再由下方檢查報錯
    const std::string targets_param = mode + ".targets";
    if (!get_node()->has_parameter(targets_param)) {
      get_node()->declare_parameter<std::vector<double>>(targets_param, std::vector<double>{});
    }
    auto targets = get_node()->get_parameter(targets_param).as_double_array();
    if (targets.size() != num_commands) {
      RCLCPP_ERROR(logger,
        "'%s' must contain %zu entries (one per command interface), got %zu.",
        targets_param.c_str(), num_commands, targets.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    mode_names_.push_back(mode);
    mode_targets_.push_back(std::move(targets));
  }

  default_mode_ = get_node()->get_parameter("default_mode").as_string();
  if (targets_for_mode(default_mode_) == nullptr) {
    RCLCPP_ERROR(logger, "'default_mode' [ %s ] is not listed in 'modes'.", default_mode_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  state_interface_names_ = get_node()->get_parameter("state_interfaces").as_string_array();

  // 若有指定 sensor interface，確保它在 state_interfaces 中（沒有就附加在最後），並記錄其 index
  sensor_interface_name_ = get_node()->get_parameter("sensor").as_string();
  sensor_index_.reset();
  if (!sensor_interface_name_.empty()) {
    auto it = std::find(
      state_interface_names_.begin(), state_interface_names_.end(), sensor_interface_name_);
    if (it == state_interface_names_.end()) {
      state_interface_names_.push_back(sensor_interface_name_);
      it = std::prev(state_interface_names_.end());
    }
    sensor_index_ = static_cast<size_t>(std::distance(state_interface_names_.begin(), it));
    RCLCPP_INFO(logger,
      "Sensor interface [ %s ] at state interface index %zu",
      sensor_interface_name_.c_str(), *sensor_index_);
  }

  if (state_interface_names_.empty()) {
    RCLCPP_ERROR(logger,
      "No state interfaces configured ('state_interfaces' and 'sensor' are both empty).");
    return controller_interface::CallbackReturn::ERROR;
  }

  // target_state_interfaces 必須與 command_interfaces 一一對應，且列在 state_interfaces 中
  const auto target_state_names =
    get_node()->get_parameter("target_state_interfaces").as_string_array();
  if (target_state_names.size() != num_commands) {
    RCLCPP_ERROR(logger,
      "'target_state_interfaces' must contain %zu entries (one per command interface), got %zu.",
      num_commands, target_state_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  target_state_indices_.clear();
  for (const auto & name : target_state_names) {
    const auto it = std::find(state_interface_names_.begin(), state_interface_names_.end(), name);
    if (it == state_interface_names_.end()) {
      RCLCPP_ERROR(logger,
        "Target state interface [ %s ] is not listed in 'state_interfaces'.", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    target_state_indices_.push_back(
      static_cast<size_t>(std::distance(state_interface_names_.begin(), it)));
  }

  const double planner_rate = get_node()->get_parameter("planner_rate").as_double();
  if (planner_rate <= 0.0) {
    RCLCPP_ERROR(logger, "'planner_rate' must be greater than zero, got %f.", planner_rate);
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto max_steps = get_node()->get_parameter("planner_max_steps").as_double_array();
  if (max_steps.size() != num_commands) {
    RCLCPP_ERROR(logger,
      "'planner_max_steps' must contain %zu entries (one per command interface), got %zu.",
      num_commands, max_steps.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (std::any_of(max_steps.begin(), max_steps.end(), [](double s) {return !(s > 0.0);})) {
    RCLCPP_ERROR(logger, "Every entry of 'planner_max_steps' must be greater than zero.");
    return controller_interface::CallbackReturn::ERROR;
  }
  planner_ = std::make_unique<LocomotionPlanner>(planner_rate, max_steps);

  state_snapshot_.assign(state_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  state_publisher_ = std::make_unique<StateInterfacePublisher>(
    get_node(), "~/states", state_interface_names_,
    get_node()->get_parameter("state_publish_rate").as_double());

  // reliable + transient_local：模式切換不可遺失，且晚啟動也能收到最後一次設定的模式
  rclcpp::QoS sub_qos(1);
  sub_qos.reliable();
  sub_qos.transient_local();

  mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
    "~/mode",
    sub_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {apply_mode(msg->data);});

  using namespace std::placeholders;
  switch_mode_server_ = rclcpp_action::create_server<SwitchModeAction>(
    get_node(),
    std::string(get_node()->get_name()) + "/switch_mode",
    std::bind(&SwitchGroupController::handle_switch_mode_goal, this, _1, _2),
    std::bind(&SwitchGroupController::handle_switch_mode_cancel, this, _1),
    std::bind(&SwitchGroupController::handle_switch_mode_accepted, this, _1));

  // reconfigure 時回到預設模式（planner 未啟動前設定的目標會保留到 activate）
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    current_mode_ = default_mode_;
    planner_->set_goal(*targets_for_mode(default_mode_));
  }

  RCLCPP_INFO(logger,
    "SwitchGroupController configured with %zu command interfaces, %zu state interfaces "
    "and %zu modes (default mode [ %s ], planner %.1f Hz).",
    num_commands, state_interface_names_.size(), mode_names_.size(), default_mode_.c_str(),
    planner_rate);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwitchGroupController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  // 以目前的 state 作為 planner 起點，避免啟動瞬間輸出跳動
  std::vector<double> initial_positions;
  initial_positions.reserve(target_state_indices_.size());
  for (const auto index : target_state_indices_) {
    const double value = state_interfaces_[index].get_value();
    if (!std::isfinite(value)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Target state interface [ %s ] has no valid value, cannot start planner.",
        state_interface_names_[index].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    initial_positions.push_back(value);
  }
  planner_->start(initial_positions, &target_buffer_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwitchGroupController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  planner_->stop();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SwitchGroupController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
}

controller_interface::InterfaceConfiguration
SwitchGroupController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}

const std::vector<double> * SwitchGroupController::targets_for_mode(
  const std::string & mode) const {
  const auto it = std::find(mode_names_.begin(), mode_names_.end(), mode);
  if (it == mode_names_.end()) {
    return nullptr;
  }
  return &mode_targets_[static_cast<size_t>(std::distance(mode_names_.begin(), it))];
}

void SwitchGroupController::apply_mode(const std::string & mode) {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  const auto * targets = targets_for_mode(mode);
  // 未知模式：目標全部設為 0（沿用原本的行為）
  planner_->set_goal(targets != nullptr ? *targets : zero_targets_);
  if (current_mode_ == mode) {
    return;
  }
  current_mode_ = mode;
  if (targets == nullptr) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Unknown mode [ %s ], moving all targets to 0.0", mode.c_str());
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Change mode to [ %s ]", mode.c_str());
  }
}

rclcpp_action::GoalResponse SwitchGroupController::handle_switch_mode_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const SwitchModeAction::Goal> goal) {
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Rejected switch_mode goal [ %s ]: controller is not active.", goal->mode.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // mode 表在 configure 後即唯讀，可在 executor thread 安全查詢
  if (targets_for_mode(goal->mode) == nullptr) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Rejected switch_mode goal: unknown mode [ %s ].", goal->mode.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SwitchGroupController::handle_switch_mode_cancel(
  std::shared_ptr<SwitchModeGoalHandle> /*goal_handle*/) {
  // goal 在 accepted 時就已完成，沒有可以取消的執行階段
  return rclcpp_action::CancelResponse::REJECT;
}

void SwitchGroupController::handle_switch_mode_accepted(
  std::shared_ptr<SwitchModeGoalHandle> goal_handle) {
  const std::string mode = goal_handle->get_goal()->mode;
  apply_mode(mode);

  auto result = std::make_shared<SwitchModeAction::Result>();
  result->success = true;
  result->message = "Mode switched";
  result->mode = mode;
  result->interface_names = state_interface_names_;
  {
    std::lock_guard<std::mutex> lock(state_snapshot_mutex_);
    result->values = state_snapshot_;
  }
  goal_handle->succeed(result);
}

controller_interface::return_type SwitchGroupController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period) {

  // INDIVIDUAL 模式下 state_interfaces_ 的順序與 state_interface_names_ 相同
  state_publisher_->publish(state_interfaces_, time, period);

  if (sensor_index_ && *sensor_index_ < state_interfaces_.size()) {
    sensor_value_ = state_interfaces_[*sensor_index_].get_value();
  }

  // 更新 action 用的 state 快照；拿不到 lock 就跳過這一次，不阻塞 RT
  {
    std::unique_lock<std::mutex> lock(state_snapshot_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      for (size_t i = 0; i < state_interfaces_.size() && i < state_snapshot_.size(); ++i) {
        state_snapshot_[i] = state_interfaces_[i].get_value();
      }
    }
  }

  // 輸出 planner 的最新目標；planner 尚未寫入（長度不符）時不輸出
  const auto * targets = target_buffer_.readFromRT();
  if (targets != nullptr && targets->size() == command_interfaces_.size()) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value((*targets)[i]);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace graphen::control

PLUGINLIB_EXPORT_CLASS(
  graphen::control::SwitchGroupController,
  controller_interface::ControllerInterface)
