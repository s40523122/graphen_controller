#include "graphen_controllers/switch_group_controller.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "graphen_controllers/bvh_motion.hpp"
#include "graphen_controllers/plugin_motion.hpp"
#include "graphen_controllers/target_motion.hpp"
#include "graphen_motion/bvh_clip.hpp"
#include "graphen_motion/bvh_retargeter.hpp"
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

// 模式參數以 <mode>. 為前綴，無法在 on_init 預先宣告：YAML 有寫的會被 controller node 自動宣告，
// 沒寫的在這裡以預設值宣告
template<typename T>
T get_or_declare(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::string & name, const T & default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

// package://<pkg>/<path> 轉成安裝後的絕對路徑；其他字串原樣回傳
std::string resolve_path(const std::string & path) {
  const std::string scheme = "package://";
  if (path.rfind(scheme, 0) != 0) {
    return path;
  }
  const auto rest = path.substr(scheme.size());
  const auto slash = rest.find('/');
  if (slash == std::string::npos || slash == 0) {
    throw std::invalid_argument("Invalid package path '" + path + "'");
  }
  return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) +
         rest.substr(slash);
}

// IMU 的 10 個 state interface（<sensor>/<suffix>），順序對應 ImuState 的欄位；
// 名稱與 imu_sensor_broadcaster / semantic_components::IMUSensor 的慣例相同
constexpr std::array<const char *, 10> kImuInterfaceSuffixes = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

// 以 controller node 的 <mode>.* 參數實作演算法的參數介面。
// 只讀取已宣告的參數（YAML 有寫的會被 controller node 自動宣告），沒有的回傳 nullopt
class NodeParameterSource : public locomotion::ParameterSource {
public:
  NodeParameterSource(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string prefix)
  : node_(std::move(node)), prefix_(std::move(prefix)) {}

  std::optional<bool> get_bool(const std::string & name) const override {
    const auto param = find(name);
    if (!param) {
      return std::nullopt;
    }
    expect(*param, rclcpp::ParameterType::PARAMETER_BOOL);
    return param->as_bool();
  }

  std::optional<int64_t> get_int(const std::string & name) const override {
    const auto param = find(name);
    if (!param) {
      return std::nullopt;
    }
    expect(*param, rclcpp::ParameterType::PARAMETER_INTEGER);
    return param->as_int();
  }

  std::optional<double> get_double(const std::string & name) const override {
    const auto param = find(name);
    if (!param) {
      return std::nullopt;
    }
    // YAML 寫成 1 而不是 1.0 時會是整數
    if (param->get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(param->as_int());
    }
    expect(*param, rclcpp::ParameterType::PARAMETER_DOUBLE);
    return param->as_double();
  }

  std::optional<std::string> get_string(const std::string & name) const override {
    const auto param = find(name);
    if (!param) {
      return std::nullopt;
    }
    expect(*param, rclcpp::ParameterType::PARAMETER_STRING);
    return param->as_string();
  }

  std::optional<std::vector<double>> get_double_array(const std::string & name) const override {
    const auto param = find(name);
    if (!param) {
      return std::nullopt;
    }
    if (param->get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto values = param->as_integer_array();
      return std::vector<double>(values.begin(), values.end());
    }
    expect(*param, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    return param->as_double_array();
  }

  std::optional<std::vector<std::string>> get_string_array(
    const std::string & name) const override {
    const auto param = find(name);
    if (!param) {
      return std::nullopt;
    }
    expect(*param, rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    return param->as_string_array();
  }

private:
  std::optional<rclcpp::Parameter> find(const std::string & name) const {
    const auto full_name = prefix_ + name;
    if (!node_->has_parameter(full_name)) {
      return std::nullopt;
    }
    auto param = node_->get_parameter(full_name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return std::nullopt;
    }
    return param;
  }

  static void expect(const rclcpp::Parameter & param, rclcpp::ParameterType type) {
    if (param.get_type() != type) {
      throw std::invalid_argument(
        "parameter '" + param.get_name() + "' has type " +
        rclcpp::to_string(param.get_type()) + ", expected " + rclcpp::to_string(type));
    }
  }

  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  const std::string prefix_;
};

// 演算法的 log 輸出到 controller 的 logger（rclcpp 的 log 可在任何 thread 呼叫）
locomotion::Logger make_algorithm_logger(const rclcpp::Logger & logger) {
  return [logger](locomotion::LogLevel level, const std::string & message) {
           switch (level) {
             case locomotion::LogLevel::kDebug:
               RCLCPP_DEBUG(logger, "%s", message.c_str());
               break;
             case locomotion::LogLevel::kInfo:
               RCLCPP_INFO(logger, "%s", message.c_str());
               break;
             case locomotion::LogLevel::kWarn:
               RCLCPP_WARN(logger, "%s", message.c_str());
               break;
             case locomotion::LogLevel::kError:
               RCLCPP_ERROR(logger, "%s", message.c_str());
               break;
           }
         };
}

}  // namespace

controller_interface::CallbackReturn SwitchGroupController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("sample_period_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>(
      "velocity_command_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("kp_command_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("kd_command_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>{});
    auto_declare<std::string>("sensor", std::string{});
    auto_declare<double>("state_publish_rate", 50.0);
    auto_declare<std::vector<std::string>>("modes", std::vector<std::string>{});
    auto_declare<std::string>("default_mode", std::string{});
    auto_declare<double>("planner_rate", 100.0);
    auto_declare<std::vector<double>>("planner_max_steps", std::vector<double>{});
    auto_declare<std::vector<std::string>>("target_state_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>(
      "velocity_state_interfaces", std::vector<std::string>{});
    auto_declare<std::string>("imu_sensor_name", std::string{});
    auto_declare<std::string>("base_estimator.plugin", std::string{});
    auto_declare<double>("command_timeout", 0.5);
    auto_declare<std::string>("bvh.base_link", std::string{});
    auto_declare<double>("bvh.axis_tolerance", 1e-4);
    auto_declare<int>("bvh.max_iterations", 50);
    auto_declare<double>("bvh.damping", 1e-6);
    auto_declare<double>("bvh.residual_tolerance", 1e-3);
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
  cmd_vel_subscriber_.reset();
  switch_mode_server_.reset();
  planner_.reset();
  base_estimator_.reset();

  command_interface_names_ = get_node()->get_parameter("command_interfaces").as_string_array();
  if (command_interface_names_.empty()) {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t num_commands = command_interface_names_.size();

  // 速度前饋與 PD 增益（選填，下游為 PD controller 時使用），與 command_interfaces 一一對應
  const auto read_per_command_interfaces =
    [&](const char * parameter, std::vector<std::string> & names) {
      names = get_node()->get_parameter(parameter).as_string_array();
      if (!names.empty() && names.size() != num_commands) {
        RCLCPP_ERROR(logger,
          "'%s' must be empty or contain %zu entries (one per command interface), got %zu.",
          parameter, num_commands, names.size());
        return false;
      }
      return true;
    };
  if (!read_per_command_interfaces("velocity_command_interfaces", velocity_interface_names_) ||
    !read_per_command_interfaces("kp_command_interfaces", kp_interface_names_) ||
    !read_per_command_interfaces("kd_command_interfaces", kd_interface_names_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // 下游插補用的點間隔，接在所有 command 之後 claim
  sample_period_interface_names_ =
    get_node()->get_parameter("sample_period_interfaces").as_string_array();

  // claim 的介面不可重複
  {
    const auto claimed = command_interface_configuration().names;
    for (size_t i = 0; i < claimed.size(); ++i) {
      if (std::find(claimed.begin() + static_cast<std::ptrdiff_t>(i) + 1, claimed.end(),
        claimed[i]) != claimed.end())
      {
        RCLCPP_ERROR(logger, "Command interface [ %s ] is listed more than once.",
          claimed[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  state_interface_names_ = get_node()->get_parameter("state_interfaces").as_string_array();

  // 確保 name 在 state_interfaces 中（沒有就附加在最後），回傳其 index
  const auto index_of_or_append = [this](const std::string & name) {
      auto it = std::find(state_interface_names_.begin(), state_interface_names_.end(), name);
      if (it == state_interface_names_.end()) {
        state_interface_names_.push_back(name);
        it = std::prev(state_interface_names_.end());
      }
      return static_cast<size_t>(std::distance(state_interface_names_.begin(), it));
    };

  // 若有指定 sensor interface，確保它在 state_interfaces 中，並記錄其 index
  sensor_interface_name_ = get_node()->get_parameter("sensor").as_string();
  sensor_index_.reset();
  if (!sensor_interface_name_.empty()) {
    sensor_index_ = index_of_or_append(sensor_interface_name_);
    RCLCPP_INFO(logger,
      "Sensor interface [ %s ] at state interface index %zu",
      sensor_interface_name_.c_str(), *sensor_index_);
  }

  // 交給 Motion 的量測速度（選填），與 command_interfaces 一一對應
  const auto velocity_state_names =
    get_node()->get_parameter("velocity_state_interfaces").as_string_array();
  velocity_state_indices_.clear();
  if (!velocity_state_names.empty()) {
    if (velocity_state_names.size() != num_commands) {
      RCLCPP_ERROR(logger,
        "'velocity_state_interfaces' must be empty or contain %zu entries "
        "(one per command interface), got %zu.",
        num_commands, velocity_state_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    for (const auto & name : velocity_state_names) {
      velocity_state_indices_.push_back(index_of_or_append(name));
    }
  }

  // 交給 Motion 的 IMU（選填）：<imu_sensor_name>/orientation.x … linear_acceleration.z
  const auto imu_sensor_name = get_node()->get_parameter("imu_sensor_name").as_string();
  imu_state_indices_.clear();
  if (!imu_sensor_name.empty()) {
    for (const auto * suffix : kImuInterfaceSuffixes) {
      imu_state_indices_.push_back(index_of_or_append(imu_sensor_name + "/" + suffix));
    }
    RCLCPP_INFO(logger, "IMU sensor [ %s ] provided to motions.", imu_sensor_name.c_str());
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
  joint_names_.clear();
  for (const auto & name : target_state_names) {
    // L_WRIST_P/position → L_WRIST_P
    joint_names_.push_back(name.substr(0, name.rfind('/')));
    const auto it = std::find(state_interface_names_.begin(), state_interface_names_.end(), name);
    if (it == state_interface_names_.end()) {
      RCLCPP_ERROR(logger,
        "Target state interface [ %s ] is not listed in 'state_interfaces'.", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    target_state_indices_.push_back(
      static_cast<size_t>(std::distance(state_interface_names_.begin(), it)));
  }

  planner_rate_ = get_node()->get_parameter("planner_rate").as_double();
  if (planner_rate_ <= 0.0) {
    RCLCPP_ERROR(logger, "'planner_rate' must be greater than zero, got %f.", planner_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }
  default_max_steps_ = get_node()->get_parameter("planner_max_steps").as_double_array();
  if (default_max_steps_.size() != num_commands) {
    RCLCPP_ERROR(logger,
      "'planner_max_steps' must contain %zu entries (one per command interface), got %zu.",
      num_commands, default_max_steps_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (std::any_of(default_max_steps_.begin(), default_max_steps_.end(),
    [](double s) {return !(s > 0.0);}))
  {
    RCLCPP_ERROR(logger, "Every entry of 'planner_max_steps' must be greater than zero.");
    return controller_interface::CallbackReturn::ERROR;
  }
  planner_ = std::make_unique<LocomotionPlanner>(planner_rate_, num_commands);

  // 量測狀態的緩衝，先配置好大小，update() 中只覆寫內容
  {
    RobotState initial_state;
    initial_state.measured_positions.assign(num_commands, std::numeric_limits<double>::quiet_NaN());
    initial_state.measured_velocities.assign(
      num_commands, std::numeric_limits<double>::quiet_NaN());
    initial_state.commanded_positions.assign(
      num_commands, std::numeric_limits<double>::quiet_NaN());
    state_buffer_.reset(std::move(initial_state));
  }

  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  if (!(command_timeout_ > 0.0)) {
    RCLCPP_ERROR(logger, "'command_timeout' must be greater than zero, got %f.", command_timeout_);
    return controller_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    velocity_command_ = VelocityCommand{};
  }

  try {
    base_estimator_ = create_base_estimator();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to configure base estimator: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 模式以名稱識別，名稱同時是該模式參數的前綴（<mode>.*）
  const auto modes = get_node()->get_parameter("modes").as_string_array();
  if (modes.empty()) {
    RCLCPP_ERROR(logger, "'modes' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  mode_names_.clear();
  mode_motions_.clear();
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
    try {
      mode_motions_.push_back(create_motion(mode));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Failed to configure mode [ %s ]: %s", mode.c_str(), e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    mode_names_.push_back(mode);
  }

  // 空字串表示啟動時不進入任何模式，保持目前位置，直到收到第一個模式命令
  default_mode_ = get_node()->get_parameter("default_mode").as_string();
  if (!default_mode_.empty() && motion_for_mode(default_mode_) == nullptr) {
    RCLCPP_ERROR(logger, "'default_mode' [ %s ] is not listed in 'modes'.", default_mode_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

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

  // 速度指令只取最新值，逾時由 planner 端判斷
  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      const std::array<double, 3> velocity{msg->linear.x, msg->linear.y, msg->angular.z};
      if (!std::all_of(velocity.begin(), velocity.end(), [](double v) {return std::isfinite(v);})) {
        RCLCPP_WARN(get_node()->get_logger(), "Ignored non-finite cmd_vel.");
        return;
      }
      std::lock_guard<std::mutex> lock(command_mutex_);
      velocity_command_.velocity = velocity;
      velocity_command_.stamp = std::chrono::steady_clock::now();
      velocity_command_.received = true;
    });

  using namespace std::placeholders;
  switch_mode_server_ = rclcpp_action::create_server<SwitchModeAction>(
    get_node(),
    std::string(get_node()->get_name()) + "/switch_mode",
    std::bind(&SwitchGroupController::handle_switch_mode_goal, this, _1, _2),
    std::bind(&SwitchGroupController::handle_switch_mode_cancel, this, _1),
    std::bind(&SwitchGroupController::handle_switch_mode_accepted, this, _1));

  // reconfigure 時回到預設模式，實際交給 planner 是在 activate 時
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    current_mode_ = default_mode_;
  }

  RCLCPP_INFO(logger,
    "SwitchGroupController configured with %zu command interfaces, %zu state interfaces "
    "and %zu modes (default mode [ %s ], planner %.1f Hz).",
    num_commands, state_interface_names_.size(), mode_names_.size(),
    default_mode_.empty() ? "(hold)" : default_mode_.c_str(),
    planner_rate_);

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

  // 尚未進入任何模式時保持位置。保持的目標優先取下游 reference 的值（下游 activate 時
  // 以硬體目前的 command 初始化，也就是硬體正在前往的初始姿態）；無效時才用目前的 state。
  // 若直接用 state，實機馬達還在走向初始姿態時會被停在半路
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (current_mode_.empty()) {
      for (size_t i = 0; i < initial_positions.size(); ++i) {
        const double reference = command_interfaces_[i].get_value();
        if (std::isfinite(reference)) {
          initial_positions[i] = reference;
        }
      }
      // 全部為 NaN 的目標：保持進入時的位置
      planner_->set_motion(std::make_shared<TargetMotion>(
        std::vector<double>(initial_positions.size(), std::numeric_limits<double>::quiet_NaN()),
        default_max_steps_));
      RCLCPP_INFO(get_node()->get_logger(),
        "No mode selected, holding current positions until a mode command is received.");
    } else {
      planner_->set_motion(motion_for_mode(current_mode_));
    }
  }
  // 先寫入一次目前的狀態，planner 第一個週期（含 on_enter）就有量測值可用
  state_buffer_.try_write([this](RobotState & state) {read_robot_state(state);});
  // 估測器在 planner thread 的第一個週期重設；planner thread 在這之後才啟動
  estimator_needs_reset_ = true;
  estimator_faulted_ = false;
  planner_->start(
    initial_positions, &target_buffer_,
    [this](RobotState & state) {fill_planner_state(state);});
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
  // 依序為位置、速度、kp、kd、sample_period；update() 依此順序寫入
  config.names = command_interface_names_;
  for (const auto * names : {&velocity_interface_names_, &kp_interface_names_,
      &kd_interface_names_, &sample_period_interface_names_})
  {
    config.names.insert(config.names.end(), names->begin(), names->end());
  }
  return config;
}

controller_interface::InterfaceConfiguration
SwitchGroupController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}

std::shared_ptr<Motion> SwitchGroupController::motion_for_mode(const std::string & mode) const {
  const auto it = std::find(mode_names_.begin(), mode_names_.end(), mode);
  if (it == mode_names_.end()) {
    return nullptr;
  }
  return mode_motions_[static_cast<size_t>(std::distance(mode_names_.begin(), it))];
}

std::shared_ptr<Motion> SwitchGroupController::create_motion(const std::string & mode) {
  const auto node = get_node();
  const auto logger = node->get_logger();
  const size_t num_commands = joint_names_.size();

  auto max_steps = get_or_declare<std::vector<double>>(node, mode + ".max_steps", {});
  if (max_steps.empty()) {
    max_steps = default_max_steps_;
  } else if (max_steps.size() != num_commands) {
    throw std::invalid_argument(
      "'" + mode + ".max_steps' must contain " + std::to_string(num_commands) + " entries");
  }

  const auto type = get_or_declare<std::string>(node, mode + ".type", "target");
  if (type == "target") {
    auto joints = get_or_declare<std::vector<std::string>>(node, mode + ".joints", {});
    const auto targets = get_or_declare<std::vector<double>>(node, mode + ".targets", {});
    if (joints.empty()) {
      joints = joint_names_;
    }
    if (targets.size() != joints.size()) {
      throw std::invalid_argument(
        "'" + mode + ".targets' must contain " + std::to_string(joints.size()) +
        " entries (one per joint), got " + std::to_string(targets.size()));
    }
    // 沒有列出的 joint 為 NaN：保持進入模式時的位置
    std::vector<double> full_targets(num_commands, std::numeric_limits<double>::quiet_NaN());
    for (size_t i = 0; i < joints.size(); ++i) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), joints[i]);
      if (it == joint_names_.end()) {
        throw std::invalid_argument(
          "joint '" + joints[i] + "' in '" + mode + ".joints' is not a commanded joint");
      }
      auto & target = full_targets[static_cast<size_t>(std::distance(joint_names_.begin(), it))];
      if (!std::isnan(target)) {
        throw std::invalid_argument("duplicate joint '" + joints[i] + "' in '" + mode + ".joints'");
      }
      target = targets[i];
    }
    return std::make_shared<TargetMotion>(std::move(full_targets), std::move(max_steps));
  }

  // 保持進入模式時的位置（全部 NaN）；用於停止動作，再切到其他模式時會重新 on_enter
  if (type == "hold") {
    return std::make_shared<TargetMotion>(
      std::vector<double>(num_commands, std::numeric_limits<double>::quiet_NaN()),
      std::move(max_steps));
  }

  if (type == "bvh") {
    const auto file = get_or_declare<std::string>(node, mode + ".file", "");
    if (file.empty()) {
      throw std::invalid_argument("'" + mode + ".file' is empty");
    }
    // 走向第 0 幀（含循環時接回起點）的速度，預設為 planner_max_steps
    auto approach_max_steps =
      get_or_declare<std::vector<double>>(node, mode + ".approach_max_steps", {});
    if (approach_max_steps.empty()) {
      approach_max_steps = default_max_steps_;
    } else if (approach_max_steps.size() != num_commands) {
      throw std::invalid_argument(
        "'" + mode + ".approach_max_steps' must contain " + std::to_string(num_commands) +
        " entries");
    }

    BvhMotion::Options motion_options;
    motion_options.loop = get_or_declare<bool>(node, mode + ".loop", true);
    motion_options.time_scale = get_or_declare<double>(node, mode + ".time_scale", 1.0);

    motion::BvhRetargeter::Options retarget_options;
    retarget_options.base_link = node->get_parameter("bvh.base_link").as_string();
    retarget_options.axis_tolerance = node->get_parameter("bvh.axis_tolerance").as_double();
    retarget_options.max_iterations =
      static_cast<int>(node->get_parameter("bvh.max_iterations").as_int());
    retarget_options.damping = node->get_parameter("bvh.damping").as_double();
    retarget_options.residual_tolerance =
      node->get_parameter("bvh.residual_tolerance").as_double();
    if (retarget_options.base_link.empty()) {
      throw std::invalid_argument("'bvh.base_link' is required for bvh modes");
    }

    // Humble 的 controller 沒有 robot_description API；launch 以參數檔傳給 controller_manager 的
    // robot_description 會套用到同一個 process 中的 controller node
    if (!node->has_parameter("robot_description")) {
      throw std::invalid_argument("parameter 'robot_description' is not available");
    }
    const auto urdf = node->get_parameter("robot_description").as_string();

    const auto path = resolve_path(file);
    auto clip = std::make_shared<const motion::BvhClip>(motion::parse_bvh_file(path));
    auto retargeter = std::make_shared<const motion::BvhRetargeter>(
      clip, urdf, joint_names_, retarget_options);

    RCLCPP_INFO(logger, "Mode [ %s ]: BVH '%s', %zu frames, frame time %.4f s, loop %s.",
      mode.c_str(), path.c_str(), clip->num_frames(), clip->frame_time,
      motion_options.loop ? "true" : "false");
    for (const auto & group : retargeter->groups()) {
      if (group.bvh_node.empty()) {
        continue;
      }
      std::string joints;
      for (const auto & joint : group.joints) {
        joints += (joints.empty() ? "" : ", ") + joint;
      }
      RCLCPP_INFO(logger, "  [%s] <- BVH %s", joints.c_str(), group.bvh_node.c_str());
    }
    for (size_t i = 0; i < num_commands; ++i) {
      if (!retargeter->covered()[i]) {
        RCLCPP_WARN(logger, "  Joint [ %s ] has no BVH data, it holds its position.",
          joint_names_[i].c_str());
      }
    }
    return std::make_shared<BvhMotion>(
      std::move(retargeter), motion_options, std::move(max_steps), std::move(approach_max_steps));
  }

  if (type == "plugin") {
    return create_plugin_motion(mode, std::move(max_steps));
  }

  throw std::invalid_argument(
    "unknown type '" + type + "' in '" + mode +
    ".type' (expected 'target', 'hold', 'bvh' or 'plugin')");
}

std::shared_ptr<Motion> SwitchGroupController::create_plugin_motion(
  const std::string & mode, std::vector<double> max_steps) {
  const auto node = get_node();
  const auto plugin = get_or_declare<std::string>(node, mode + ".plugin", "");
  if (plugin.empty()) {
    throw std::invalid_argument("'" + mode + ".plugin' is empty");
  }

  // loader 跨 reconfigure 保留：已建立的演算法實例需要它載入的 library
  if (!algorithm_loader_) {
    algorithm_loader_ =
      std::make_unique<pluginlib::ClassLoader<locomotion::LocomotionAlgorithm>>(
      "graphen_locomotion", "graphen::locomotion::LocomotionAlgorithm");
  }
  std::shared_ptr<locomotion::LocomotionAlgorithm> algorithm =
    algorithm_loader_->createSharedInstance(plugin);

  const NodeParameterSource params(node, mode + ".");
  const auto log = make_algorithm_logger(node->get_logger().get_child(mode));
  locomotion::Context context{
    joint_names_,
    planner_rate_,
    node->has_parameter("robot_description") ?
    node->get_parameter("robot_description").as_string() : std::string{},
    params,
    log};
  algorithm->configure(context);

  RCLCPP_INFO(node->get_logger(), "Mode [ %s ]: plugin '%s'.", mode.c_str(), plugin.c_str());
  return std::make_shared<PluginMotion>(std::move(algorithm), std::move(max_steps), mode, log);
}

std::shared_ptr<locomotion::BaseStateEstimator> SwitchGroupController::create_base_estimator() {
  const auto node = get_node();
  const auto plugin = node->get_parameter("base_estimator.plugin").as_string();
  if (plugin.empty()) {
    RCLCPP_INFO(node->get_logger(), "No base estimator configured, base state is not provided.");
    return nullptr;
  }
  if (!estimator_loader_) {
    estimator_loader_ =
      std::make_unique<pluginlib::ClassLoader<locomotion::BaseStateEstimator>>(
      "graphen_locomotion", "graphen::locomotion::BaseStateEstimator");
  }
  std::shared_ptr<locomotion::BaseStateEstimator> estimator =
    estimator_loader_->createSharedInstance(plugin);

  const NodeParameterSource params(node, "base_estimator.");
  locomotion::Context context{
    joint_names_,
    planner_rate_,
    node->has_parameter("robot_description") ?
    node->get_parameter("robot_description").as_string() : std::string{},
    params,
    make_algorithm_logger(node->get_logger().get_child("base_estimator"))};
  estimator->configure(context);

  RCLCPP_INFO(node->get_logger(), "Base estimator: plugin '%s'.", plugin.c_str());
  return estimator;
}

void SwitchGroupController::fill_planner_state(RobotState & state) {
  state_buffer_.read(state);

  // 速度指令：逾時或尚未收到時為 0
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - velocity_command_.stamp).count();
    state.command.active = velocity_command_.received && age <= command_timeout_;
    state.command.velocity = state.command.active ?
      velocity_command_.velocity : std::array<double, 3>{0.0, 0.0, 0.0};
  }

  state.base = locomotion::BaseState{};
  if (!base_estimator_ || estimator_faulted_) {
    return;
  }
  try {
    if (estimator_needs_reset_) {
      base_estimator_->reset();
      estimator_needs_reset_ = false;
      last_state_time_ = state.time;
    }
    // 以量測時間的差值作為 dt；同一筆量測被讀兩次時為 0
    const double dt = std::max(0.0, state.time - last_state_time_);
    last_state_time_ = state.time;
    base_estimator_->update(dt, state, state.base);
  } catch (const std::exception & e) {
    estimator_faulted_ = true;
    state.base = locomotion::BaseState{};
    RCLCPP_ERROR(get_node()->get_logger(),
      "Base estimator threw: %s. Base state is invalid until the controller is reactivated.",
      e.what());
  }
}

void SwitchGroupController::apply_mode(const std::string & mode) {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  auto motion = motion_for_mode(mode);
  if (!motion) {
    // 32 軸含腿部，未知模式時不移動（不再歸零）
    RCLCPP_WARN(get_node()->get_logger(), "Unknown mode [ %s ], ignored.", mode.c_str());
    return;
  }
  // 已在該模式時不重新開始（例如 BVH 不會從頭播放）
  if (current_mode_ == mode) {
    return;
  }
  current_mode_ = mode;
  planner_->set_motion(std::move(motion));
  RCLCPP_INFO(get_node()->get_logger(), "Change mode to [ %s ]", mode.c_str());
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
  if (motion_for_mode(goal->mode) == nullptr) {
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

void SwitchGroupController::read_robot_state(RobotState & state) const {
  state.time = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  for (size_t i = 0; i < target_state_indices_.size(); ++i) {
    state.measured_positions[i] = state_interfaces_[target_state_indices_[i]].get_value();
  }
  for (size_t i = 0; i < velocity_state_indices_.size(); ++i) {
    state.measured_velocities[i] = state_interfaces_[velocity_state_indices_[i]].get_value();
  }
  auto & imu = state.imu;
  if (imu_state_indices_.size() == kImuInterfaceSuffixes.size()) {
    const auto value = [this](size_t i) {
        return state_interfaces_[imu_state_indices_[i]].get_value();
      };
    for (size_t i = 0; i < 4; ++i) {
      imu.orientation[i] = value(i);
    }
    for (size_t i = 0; i < 3; ++i) {
      imu.angular_velocity[i] = value(4 + i);
      imu.linear_acceleration[i] = value(7 + i);
    }
    imu.valid = true;
    for (size_t i = 0; i < imu_state_indices_.size(); ++i) {
      imu.valid = imu.valid && std::isfinite(value(i));
    }
  }
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

  // 交給 planner 的量測狀態；拿不到 lock（planner 正在讀）就跳過這一次
  state_buffer_.try_write([this](RobotState & state) {read_robot_state(state);});

  // 輸出 planner 的最新目標；planner 尚未寫入（長度不符）時不輸出。
  // 速度與增益只在有設定對應介面時寫入（NaN 由下游解讀為沒有前饋 / 預設增益）
  const auto * output = target_buffer_.readFromRT();
  const size_t num_commands = command_interface_names_.size();
  const size_t num_claimed = num_commands + velocity_interface_names_.size() +
    kp_interface_names_.size() + kd_interface_names_.size() +
    sample_period_interface_names_.size();
  if (output != nullptr && output->positions.size() == num_commands &&
    output->velocities.size() == num_commands && output->kp.size() == num_commands &&
    output->kd.size() == num_commands && command_interfaces_.size() == num_claimed)
  {
    size_t index = 0;
    const auto write_all = [&](const std::vector<double> & values, bool enabled) {
        if (!enabled) {
          return;
        }
        for (const double value : values) {
          command_interfaces_[index++].set_value(value);
        }
      };
    write_all(output->positions, true);
    write_all(output->velocities, !velocity_interface_names_.empty());
    write_all(output->kp, !kp_interface_names_.empty());
    write_all(output->kd, !kd_interface_names_.empty());
    for (; index < command_interfaces_.size(); ++index) {
      command_interfaces_[index].set_value(output->sample_period);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace graphen::control

PLUGINLIB_EXPORT_CLASS(
  graphen::control::SwitchGroupController,
  controller_interface::ControllerInterface)
