#include "graphen_controllers/switch_group_controller.hpp"

#include <algorithm>
#include <iterator>

#include "pluginlib/class_list_macros.hpp"

namespace graphen::control {

controller_interface::CallbackReturn SwitchGroupController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>{});
    auto_declare<std::string>("sensor", std::string{});
    auto_declare<double>("state_publish_rate", 50.0);
    auto_declare<std::vector<int64_t>>("modes", std::vector<int64_t>{});
    auto_declare<std::vector<double>>("mode_targets", std::vector<double>{});
    auto_declare<int>("default_mode", 1);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception caught during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwitchGroupController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  const auto logger = get_node()->get_logger();

  command_interface_names_ = get_node()->get_parameter("command_interfaces").as_string_array();
  if (command_interface_names_.empty()) {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t num_commands = command_interface_names_.size();

  // mode_targets 依 modes 順序攤平，每個模式佔 num_commands 個值
  const auto modes = get_node()->get_parameter("modes").as_integer_array();
  const auto flat_targets = get_node()->get_parameter("mode_targets").as_double_array();
  if (modes.empty()) {
    RCLCPP_ERROR(logger, "'modes' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (flat_targets.size() != modes.size() * num_commands) {
    RCLCPP_ERROR(logger,
      "'mode_targets' must contain %zu entries (%zu modes x %zu command interfaces), got %zu.",
      modes.size() * num_commands, modes.size(), num_commands, flat_targets.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  mode_ids_.clear();
  mode_targets_.clear();
  for (size_t m = 0; m < modes.size(); ++m) {
    const int id = static_cast<int>(modes[m]);
    if (std::find(mode_ids_.begin(), mode_ids_.end(), id) != mode_ids_.end()) {
      RCLCPP_ERROR(logger, "Duplicate mode id [ %d ] in 'modes'.", id);
      return controller_interface::CallbackReturn::ERROR;
    }
    const auto first = flat_targets.begin() + static_cast<std::ptrdiff_t>(m * num_commands);
    mode_ids_.push_back(id);
    mode_targets_.emplace_back(first, first + static_cast<std::ptrdiff_t>(num_commands));
  }

  default_mode_ = static_cast<int>(get_node()->get_parameter("default_mode").as_int());
  if (targets_for_mode(default_mode_) == nullptr) {
    RCLCPP_ERROR(logger, "'default_mode' [ %d ] is not listed in 'modes'.", default_mode_);
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

  state_publisher_ = std::make_unique<StateInterfacePublisher>(
    get_node(), "~/states", state_interface_names_,
    get_node()->get_parameter("state_publish_rate").as_double());

  // reliable + transient_local：模式切換不可遺失，且晚啟動也能收到最後一次設定的模式
  rclcpp::QoS sub_qos(1);
  sub_qos.reliable();
  sub_qos.transient_local();

  mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
    "~/mode",
    sub_qos,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      mode_buffer_.writeFromNonRT(msg->data);
    });

  // reconfigure 時回到預設模式
  mode_buffer_.writeFromNonRT(default_mode_);
  current_mode_ = default_mode_;

  RCLCPP_INFO(logger,
    "SwitchGroupController configured with %zu command interfaces, %zu state interfaces "
    "and %zu modes (default mode [ %d ]).",
    num_commands, state_interface_names_.size(), mode_ids_.size(), default_mode_);

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

const std::vector<double> * SwitchGroupController::targets_for_mode(int mode) const {
  const auto it = std::find(mode_ids_.begin(), mode_ids_.end(), mode);
  if (it == mode_ids_.end()) {
    return nullptr;
  }
  return &mode_targets_[static_cast<size_t>(std::distance(mode_ids_.begin(), it))];
}

controller_interface::return_type SwitchGroupController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period) {

  // INDIVIDUAL 模式下 state_interfaces_ 的順序與 state_interface_names_ 相同
  state_publisher_->publish(state_interfaces_, time, period);

  if (sensor_index_ && *sensor_index_ < state_interfaces_.size()) {
    sensor_value_ = state_interfaces_[*sensor_index_].get_value();
  }

  // 套用 ~/mode 收到的最新模式
  const int new_mode = *mode_buffer_.readFromRT();
  if (current_mode_ != new_mode) {
    current_mode_ = new_mode;
    if (targets_for_mode(current_mode_) == nullptr) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Unknown mode [ %d ], commanding 0.0 to all interfaces", current_mode_);
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Change mode to [ %d ]", current_mode_);
    }
  }

  // command_interfaces_ 順序與 command_interface_names_ 相同；未知模式輸出 0（沿用原行為）
  const auto * targets = targets_for_mode(current_mode_);
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(targets != nullptr ? (*targets)[i] : 0.0);
  }

  return controller_interface::return_type::OK;
}

}  // namespace graphen::control

PLUGINLIB_EXPORT_CLASS(
  graphen::control::SwitchGroupController,
  controller_interface::ControllerInterface)
