#include "graphen_controllers/sub_group_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <limits>

namespace graphen::control {

controller_interface::CallbackReturn SubGroupController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("input_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("output_interfaces", std::vector<std::string>{});
    auto_declare<double>("reference_publish_rate", 50.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception caught during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SubGroupController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {

  input_interfaces_ = get_node()->get_parameter("input_interfaces").as_string_array();
  if (input_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'input_interfaces' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  output_interfaces_ = get_node()->get_parameter("output_interfaces").as_string_array();
  if (output_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'output_interfaces' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 基類的 reference_interfaces_ 是導出介面的實際儲存空間，大小需與輸入數量一致；
  // 初始為 NaN，表示上層尚未寫入
  reference_interfaces_.assign(input_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());

  // 發布名稱與導出的 reference interface 全名一致：<node_name>/<input>
  std::vector<std::string> reference_names;
  reference_names.reserve(input_interfaces_.size());
  for (const auto & input : input_interfaces_) {
    reference_names.push_back(std::string(get_node()->get_name()) + "/" + input);
  }
  reference_publisher_ = std::make_unique<StateInterfacePublisher>(
    get_node(), "~/references", reference_names,
    get_node()->get_parameter("reference_publish_rate").as_double());

  RCLCPP_INFO(get_node()->get_logger(), "[%s] Configured successfully.", get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

// reference 以對應 output 目前的 command 值初始化（沒有對應 output 時為 NaN）。
// 上層 activate 時讀到的就是硬體正在前往的位置，可以直接保持，不會造成跳動
controller_interface::CallbackReturn SubGroupController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
    reference_interfaces_[i] = i < command_interfaces_.size() ?
      command_interfaces_[i].get_value() : std::numeric_limits<double>::quiet_NaN();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SubGroupController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = output_interfaces_;
  return config;
}

// 不需要讀取任何 state interface
controller_interface::InterfaceConfiguration
SubGroupController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

// 只支援 chained 模式（reference 由上層 controller 寫入），沒有 topic 訂閱，故不需處理
controller_interface::return_type SubGroupController::update_reference_from_subscribers() {
  return controller_interface::return_type::OK;
}

// 依 input_interfaces 導出 reference interface，全名為 <node_name>/<input>，
// 例如 left_arm_group_controller/position
std::vector<hardware_interface::CommandInterface>
SubGroupController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(input_interfaces_.size());
  for (size_t i = 0; i < input_interfaces_.size(); ++i) {
    interfaces.emplace_back(
      get_node()->get_name(),
      input_interfaces_[i],
      &reference_interfaces_[i]);  // 指向基類的 reference_interfaces_[i]
  }
  return interfaces;
}

bool SubGroupController::on_set_chained_mode(bool chained_mode) {
  chained_mode_ = chained_mode;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[%s] Chained mode set to: %s",
    get_node()->get_name(),
    chained_mode_ ? "true" : "false");
  return true;
}

controller_interface::return_type SubGroupController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) {

  reference_publisher_->publish(reference_interfaces_, time, period);

  // 目前只使用第一個 reference，其餘僅發布至 ~/references
  const double target = reference_interfaces_[0];

  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "Received reference from main = %.3f", target);

  // 上層尚未寫入時不輸出，避免把 NaN 寫進硬體
  if (std::isnan(target)) {
    return controller_interface::return_type::OK;
  }

  for (auto & command : command_interfaces_) {
    command.set_value(target);
  }

  return controller_interface::return_type::OK;
}

}  // namespace graphen::control

PLUGINLIB_EXPORT_CLASS(
  graphen::control::SubGroupController,
  controller_interface::ChainableControllerInterface)
