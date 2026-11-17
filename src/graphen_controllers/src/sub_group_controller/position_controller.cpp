#include "graphen_controllers/sub_group_controller/position_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace graphen::control {

controller_interface::CallbackReturn PositionController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("input_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("output_interfaces", std::vector<std::string>{});
    auto_declare<double>("reference_publish_rate", 50.0);
    auto_declare<double>("tracking_publish_rate", 50.0);
    auto_declare<bool>("output_enabled", true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception caught during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionController::on_configure(
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

  output_enabled_ = get_node()->get_parameter("output_enabled").as_bool();
  if (!output_enabled_) {
    RCLCPP_WARN(get_node()->get_logger(),
      "[%s] Output disabled: commands are not written, joints hold their last command.",
      get_node()->get_name());
  }

  // 1 個 input 時廣播到所有 output；否則第 i 個 input 對應第 i 個 output
  if (input_interfaces_.size() != 1 && input_interfaces_.size() != output_interfaces_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "'input_interfaces' must contain 1 entry or as many entries as 'output_interfaces' "
      "(%zu), got %zu.", output_interfaces_.size(), input_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (std::find(input_interfaces_.begin(), input_interfaces_.end(), kSamplePeriodInterface) !=
    input_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "'input_interfaces' must not contain the reserved name '%s'.", kSamplePeriodInterface);
    return controller_interface::CallbackReturn::ERROR;
  }

  // 基類的 reference_interfaces_ 是導出介面的實際儲存空間：各 input，最後一個是 sample_period。
  // input 初始為 NaN，表示上層尚未寫入；sample_period 初始為 0（不插補）
  reference_interfaces_.assign(
    input_interfaces_.size() + 1, std::numeric_limits<double>::quiet_NaN());
  reference_interfaces_.back() = 0.0;

  // 插補狀態在這裡預先配置，RT 中不配置記憶體
  interpolators_.assign(output_interfaces_.size(), LinearInterpolator{});

  // 發布名稱與導出的 reference interface 全名一致：<node_name>/<input>
  std::vector<std::string> reference_names;
  reference_names.reserve(reference_interfaces_.size());
  for (const auto & input : input_interfaces_) {
    reference_names.push_back(std::string(get_node()->get_name()) + "/" + input);
  }
  reference_names.push_back(std::string(get_node()->get_name()) + "/" + kSamplePeriodInterface);
  reference_publisher_ = std::make_unique<StateInterfacePublisher>(
    get_node(), "~/references", reference_names,
    get_node()->get_parameter("reference_publish_rate").as_double());

  // 追蹤誤差：通道名稱即 output 全名（例如 L_KNEE_P/position），
  // 同一關節有多個 output 時也不會重名。關閉時不建立，state_interface_configuration 也不讀取
  const double tracking_rate = get_node()->get_parameter("tracking_publish_rate").as_double();
  if (tracking_rate > 0.0) {
    tracking_publisher_ = std::make_unique<TrackingPublisher>(
      get_node(), "~/tracking", output_interfaces_, tracking_rate);
  } else {
    tracking_publisher_.reset();
  }

  RCLCPP_INFO(get_node()->get_logger(), "[%s] Configured successfully.", get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

// reference 以對應 output 目前的 command 值初始化（沒有對應 output 時為 NaN）。
// 上層 activate 時讀到的就是硬體正在前往的位置，可以直接保持，不會造成跳動。
// 插補狀態也從目前的 command 開始，啟動時不會產生斜坡
controller_interface::CallbackReturn PositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  const size_t num_inputs = input_interfaces_.size();
  for (size_t i = 0; i < num_inputs; ++i) {
    reference_interfaces_[i] = i < command_interfaces_.size() ?
      command_interfaces_[i].get_value() : std::numeric_limits<double>::quiet_NaN();
  }
  reference_interfaces_[num_inputs] = 0.0;  // sample_period

  for (size_t i = 0; i < interpolators_.size() && i < command_interfaces_.size(); ++i) {
    interpolators_[i].reset(command_interfaces_[i].get_value());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = output_interfaces_;
  return config;
}

// 只有追蹤誤差需要量測值：讀取與各 output 同名的 state interface，順序與 output 相同。
// 控制本身不使用 state，關閉追蹤時不讀取，對沒有對應 state 的硬體也能啟動
controller_interface::InterfaceConfiguration
PositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  if (tracking_publisher_) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = output_interfaces_;
  } else {
    config.type = controller_interface::interface_configuration_type::NONE;
  }
  return config;
}

// 只支援 chained 模式（reference 由上層 controller 寫入），沒有 topic 訂閱，故不需處理
controller_interface::return_type PositionController::update_reference_from_subscribers() {
  return controller_interface::return_type::OK;
}

// 依 input_interfaces 導出 reference interface，全名為 <node_name>/<input>，
// 例如 left_arm_group_controller/L_SHOULDER_P
std::vector<hardware_interface::CommandInterface>
PositionController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());
  for (size_t i = 0; i < input_interfaces_.size(); ++i) {
    interfaces.emplace_back(
      get_node()->get_name(),
      input_interfaces_[i],
      &reference_interfaces_[i]);  // 指向基類的 reference_interfaces_[i]
  }
  // 上游輸出點的間隔，供插補使用
  interfaces.emplace_back(
    get_node()->get_name(), kSamplePeriodInterface, &reference_interfaces_.back());
  return interfaces;
}

bool PositionController::on_set_chained_mode(bool chained_mode) {
  chained_mode_ = chained_mode;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[%s] Chained mode set to: %s",
    get_node()->get_name(),
    chained_mode_ ? "true" : "false");
  return true;
}

controller_interface::return_type PositionController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) {

  reference_publisher_->publish(reference_interfaces_, time, period);

  // 停用輸出：reference 仍照常發布供觀察，但不寫 command，硬體保留最後的值
  if (output_enabled_) {
    const size_t num_inputs = input_interfaces_.size();
    const double sample_period =
      LinearInterpolator::sanitize_sample_period(reference_interfaces_[num_inputs]);
    const double dt = period.seconds();

    for (size_t i = 0; i < command_interfaces_.size() && i < interpolators_.size(); ++i) {
      // 1 個 input 時廣播，否則 1:1
      const double reference = reference_interfaces_[num_inputs == 1 ? 0 : i];
      // 上層尚未寫入（NaN）時不輸出，避免把 NaN 寫進硬體
      if (std::isnan(reference)) {
        continue;
      }
      command_interfaces_[i].set_value(interpolators_[i].step(reference, sample_period, dt));
    }
  }

  // 預期值取寫入後的 command（插補後、硬體實際收到的值），與同一週期讀到的量測值比較
  if (tracking_publisher_) {
    for (size_t i = 0; i < command_interfaces_.size() && i < state_interfaces_.size(); ++i) {
      tracking_publisher_->set(
        i, command_interfaces_[i].get_value(), state_interfaces_[i].get_value());
    }
    tracking_publisher_->publish(time, period);
  }

  return controller_interface::return_type::OK;
}

}  // namespace graphen::control

PLUGINLIB_EXPORT_CLASS(
  graphen::control::PositionController,
  controller_interface::ChainableControllerInterface)
