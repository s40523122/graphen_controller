#include "graphen_controllers/sub_group_controller/pd_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace graphen::control {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// 增益 reference 有效（非負的有限值）時使用它，否則用預設值
double gain_or_default(double reference, double fallback) {
  return std::isfinite(reference) && reference >= 0.0 ? reference : fallback;
}

// 各關節的預設增益與力矩上限：joint_gains.<joint>.<key>
constexpr const char * kJointGainKeys[] = {"kp", "kd", "effort_limit"};

std::string joint_gain_parameter(const std::string & joint, const char * key) {
  return "joint_gains." + joint + "." + key;
}

}  // namespace

controller_interface::CallbackReturn PdController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<double>("reference_publish_rate", 50.0);
    // 各關節的增益參數名稱依 joints 而定，所以先讀出 joints 再宣告；預設 NaN 表示未提供，
    // on_configure 檢查。joints 在 configure 前被改掉時，on_configure 會補宣告新的關節
    const auto joints = get_node()->get_parameter("joints").as_string_array();
    for (const auto & joint : joints) {
      for (const char * key : kJointGainKeys) {
        auto_declare<double>(joint_gain_parameter(joint, key), kNaN);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception caught during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PdController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  const auto logger = get_node()->get_logger();

  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(logger, "'joints' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (std::find(joints_.begin(), joints_.end(), kSamplePeriodInterface) != joints_.end()) {
    RCLCPP_ERROR(logger, "'joints' must not contain the reserved name '%s'.",
      kSamplePeriodInterface);
    return controller_interface::CallbackReturn::ERROR;
  }

  // 各關節的 joint_gains.<joint>.<key>，依 joints 的順序存成陣列；valid 檢查每個值。
  // 未提供（NaN）也在這裡擋下，避免以 0 增益或 0 力矩上限啟動
  const auto read_per_joint = [&](const char * key, std::vector<double> & values,
      bool (*valid)(double), const char * requirement) {
      values.clear();
      values.reserve(joints_.size());
      for (const auto & joint : joints_) {
        const auto name = joint_gain_parameter(joint, key);
        const double value = auto_declare<double>(name, kNaN);
        if (!valid(value)) {
          RCLCPP_ERROR(logger, "'%s' must be %s, got %f (missing in the model's gain file?).",
            name.c_str(), requirement, value);
          return false;
        }
        values.push_back(value);
      }
      return true;
    };
  const auto non_negative = [](double v) {return std::isfinite(v) && v >= 0.0;};
  const auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  if (!read_per_joint("kp", default_kp_, non_negative, ">= 0") ||
    !read_per_joint("kd", default_kd_, non_negative, ">= 0") ||
    !read_per_joint("effort_limit", effort_limits_, positive, "> 0"))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // 位置、速度、kp、kd 初始為 NaN（上層尚未寫入）；sample_period 初始為 0（不插補）
  reference_interfaces_.assign(4 * joints_.size() + 1, kNaN);
  reference_interfaces_[sample_period_ref()] = 0.0;

  interpolators_.assign(joints_.size(), LinearInterpolator{});

  // 發布名稱與導出的 reference interface 全名一致
  const std::string prefix = std::string(get_node()->get_name()) + "/";
  std::vector<std::string> reference_names;
  reference_names.reserve(reference_interfaces_.size());
  for (const char * suffix : {"", "/velocity", "/kp", "/kd"}) {
    for (const auto & joint : joints_) {
      reference_names.push_back(prefix + joint + suffix);
    }
  }
  reference_names.push_back(prefix + kSamplePeriodInterface);
  reference_publisher_ = std::make_unique<StateInterfacePublisher>(
    get_node(), "~/references", reference_names,
    get_node()->get_parameter("reference_publish_rate").as_double());

  RCLCPP_INFO(logger, "[%s] Configured with %zu joints.", get_node()->get_name(), joints_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

// 期望角度以目前量測角度初始化：上層 activate 時讀到的就是目前位置，以預設增益保持在原地
controller_interface::CallbackReturn PdController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double position = state_interfaces_[i].get_value();
    if (!std::isfinite(position)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Joint '%s' has no valid position, cannot activate.", joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    reference_interfaces_[position_ref(i)] = position;
    reference_interfaces_[velocity_ref(i)] = kNaN;
    reference_interfaces_[kp_ref(i)] = kNaN;
    reference_interfaces_[kd_ref(i)] = kNaN;
    interpolators_[i].reset(position);
  }
  reference_interfaces_[sample_period_ref()] = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PdController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

// 依序為全部關節的 position，再來是全部關節的 velocity
controller_interface::InterfaceConfiguration
PdController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const char * name : {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}) {
    for (const auto & joint : joints_) {
      config.names.push_back(joint + "/" + name);
    }
  }
  return config;
}

// 只支援 chained 模式（reference 由上層 controller 寫入），沒有 topic 訂閱，故不需處理
controller_interface::return_type PdController::update_reference_from_subscribers() {
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> PdController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());
  const std::string node_name = get_node()->get_name();
  for (size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(node_name, joints_[i], &reference_interfaces_[position_ref(i)]);
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(
      node_name, joints_[i] + "/velocity", &reference_interfaces_[velocity_ref(i)]);
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(node_name, joints_[i] + "/kp", &reference_interfaces_[kp_ref(i)]);
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(node_name, joints_[i] + "/kd", &reference_interfaces_[kd_ref(i)]);
  }
  interfaces.emplace_back(
    node_name, kSamplePeriodInterface, &reference_interfaces_[sample_period_ref()]);
  return interfaces;
}

bool PdController::on_set_chained_mode(bool chained_mode) {
  chained_mode_ = chained_mode;
  RCLCPP_INFO(get_node()->get_logger(), "[%s] Chained mode set to: %s",
    get_node()->get_name(), chained_mode_ ? "true" : "false");
  return true;
}

controller_interface::return_type PdController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period) {

  reference_publisher_->publish(reference_interfaces_, time, period);

  const double sample_period =
    LinearInterpolator::sanitize_sample_period(reference_interfaces_[sample_period_ref()]);
  const double dt = period.seconds();
  const size_t num_joints = joints_.size();

  for (size_t i = 0; i < num_joints && i < command_interfaces_.size(); ++i) {
    // 期望角度為 NaN（上層尚未寫入）時維持上一個目標
    const double target =
      interpolators_[i].step(reference_interfaces_[position_ref(i)], sample_period, dt);
    const double position = state_interfaces_[i].get_value();
    const double velocity = state_interfaces_[num_joints + i].get_value();
    if (!std::isfinite(target) || !std::isfinite(position) || !std::isfinite(velocity)) {
      // 沒有可信的量測值時不出力，避免以 NaN 或過時的誤差計算出大力矩
      command_interfaces_[i].set_value(0.0);
      continue;
    }

    const double velocity_reference = reference_interfaces_[velocity_ref(i)];
    const double target_velocity = std::isfinite(velocity_reference) ? velocity_reference : 0.0;
    const double kp = gain_or_default(reference_interfaces_[kp_ref(i)], default_kp_[i]);
    const double kd = gain_or_default(reference_interfaces_[kd_ref(i)], default_kd_[i]);

    const double effort = kp * (target - position) + kd * (target_velocity - velocity);
    command_interfaces_[i].set_value(std::clamp(effort, -effort_limits_[i], effort_limits_[i]));
  }

  return controller_interface::return_type::OK;
}

}  // namespace graphen::control

PLUGINLIB_EXPORT_CLASS(
  graphen::control::PdController,
  controller_interface::ChainableControllerInterface)
