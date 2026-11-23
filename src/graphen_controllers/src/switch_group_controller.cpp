#include "graphen_controllers/switch_group_controller.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "graphen_controllers/library_utils.hpp"
#include "graphen_controllers/parameter_sources.hpp"
#include "graphen_controllers/target_motion.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::control {

namespace {

// 類型的名稱（choreo / bvh / policy）不能拿來當系統模式的名稱：result 的 mode 欄位用它們表示類型
constexpr std::array<const char *, 3> kReservedModeNames = {
  ChoreoSource::kMode, BvhSource::kMode, PolicySource::kMode};

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

// IMU 的 10 個 state interface（<sensor>/<suffix>），順序對應 ImuState 的欄位；
// 名稱與 imu_sensor_broadcaster / semantic_components::IMUSensor 的慣例相同
constexpr std::array<const char *, 10> kImuInterfaceSuffixes = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

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
    auto_declare<std::string>("default_policy", std::string{});
    auto_declare<double>("planner_rate", 100.0);
    auto_declare<std::vector<double>>("planner_max_steps", std::vector<double>{});
    auto_declare<std::vector<std::string>>("target_state_interfaces", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>(
      "velocity_state_interfaces", std::vector<std::string>{});
    auto_declare<std::string>("imu_sensor_name", std::string{});
    auto_declare<std::string>("base_estimator.plugin", std::string{});
    auto_declare<double>("command_timeout", 0.5);
    auto_declare<std::vector<std::string>>("pose_targets.names", std::vector<std::string>{});
    auto_declare<std::string>("pose_targets.frame", std::string{"WAIST_P_S"});
    auto_declare<double>("pose_targets.timeout", 0.5);
    auto_declare<double>("switch_mode_poll_rate", 20.0);
    // 內容與 policy 的來源：庫的路徑是空字串（編舞為 plugin 是空字串）時，該類型不啟用
    auto_declare<std::string>("choreo.plugin", std::string{});
    auto_declare<std::string>("choreo.library_path", std::string{"~/.local/share/graphen/choreo"});
    auto_declare<std::vector<double>>("choreo.max_steps", std::vector<double>{});
    // 影子試播（PlaybackOptions.dry_run / shadow_token），見 ShadowGate、run_shadow
    auto_declare<bool>("choreo.shadow.require", false);
    auto_declare<double>("choreo.shadow.start_tolerance", 0.01);
    auto_declare<double>("choreo.shadow.token_ttl", 300.0);
    auto_declare<double>("choreo.shadow.max_duration", 600.0);
    auto_declare<double>("choreo.shadow.publish_rate", 50.0);
    auto_declare<std::string>("policy.library_path", std::string{});
    auto_declare<std::string>("bvh.library_path", std::string{});
    auto_declare<std::vector<double>>("bvh.max_steps", std::vector<double>{});
    auto_declare<std::vector<double>>("bvh.approach_max_steps", std::vector<double>{});
    auto_declare<bool>("bvh.interruptible", true);
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

  // reconfigure 時先釋放上一次的介面，避免舊 callback 存取即將被替換的 planner；
  // 釋放的順序與相依順序相反（server → arbiter / 來源 → planner）
  wait_timer_.reset();
  playback_servers_.clear();
  publish_choreo_service_.reset();
  list_choreos_service_.reset();
  get_choreo_service_.reset();
  preload_service_.reset();
  mode_subscriber_.reset();
  cmd_vel_subscriber_.reset();
  pose_target_subscriber_.reset();
  arbiter_.reset();
  choreo_source_.reset();
  bvh_source_.reset();
  policy_source_.reset();
  default_request_.reset();
  system_modes_.clear();
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
  // 末端位姿目標（選填）：名稱不可重複；plugin 依名稱找到自己的目標
  pose_target_names_ = get_node()->get_parameter("pose_targets.names").as_string_array();
  pose_target_frame_ = get_node()->get_parameter("pose_targets.frame").as_string();
  pose_target_timeout_ = get_node()->get_parameter("pose_targets.timeout").as_double();
  for (size_t i = 0; i < pose_target_names_.size(); ++i) {
    if (pose_target_names_[i].empty() ||
      std::find(pose_target_names_.begin() + static_cast<std::ptrdiff_t>(i) + 1,
      pose_target_names_.end(), pose_target_names_[i]) != pose_target_names_.end())
    {
      RCLCPP_ERROR(logger, "'pose_targets.names' must contain unique, non-empty names.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (!pose_target_names_.empty() && pose_target_frame_.empty()) {
    RCLCPP_ERROR(logger, "'pose_targets.frame' must not be empty when 'pose_targets.names' is set.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!(pose_target_timeout_ > 0.0)) {
    RCLCPP_ERROR(logger, "'pose_targets.timeout' must be greater than zero, got %f.",
      pose_target_timeout_);
    return controller_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(pose_target_mutex_);
    pose_target_commands_.assign(pose_target_names_.size(), PoseTargetCommand{});
  }

  planner_ = std::make_unique<LocomotionPlanner>(planner_rate_, num_commands);
  arbiter_ = std::make_unique<PlaybackArbiter>(planner_.get(), logger);

  // 所有演算法 plugin 共用的設定。Humble 的 controller 沒有 robot_description API；
  // launch 以參數檔傳給 controller_manager 的 robot_description 會套用到同一個 process 中的 controller node
  {
    PluginFactory::Setup setup;
    setup.joint_names = joint_names_;
    setup.rate_hz = planner_rate_;
    setup.robot_description = get_node()->has_parameter("robot_description") ?
      get_node()->get_parameter("robot_description").as_string() : std::string{};
    setup.logger = logger;
    setup.pose_target_names = pose_target_names_;
    setup.pose_target_frame = pose_target_frame_;
    plugin_factory_.configure(std::move(setup));
  }

  // 量測狀態的緩衝，先配置好大小，update() 中只覆寫內容
  {
    RobotState initial_state;
    initial_state.measured_positions.assign(num_commands, std::numeric_limits<double>::quiet_NaN());
    initial_state.measured_velocities.assign(
      num_commands, std::numeric_limits<double>::quiet_NaN());
    initial_state.commanded_positions.assign(
      num_commands, std::numeric_limits<double>::quiet_NaN());
    // commanded_* 由 planner 填入；這裡配置好大小，planner 讀取時不必重新配置
    initial_state.commanded_velocities.assign(num_commands, 0.0);
    // 目標由 planner 填入（fill_planner_state）；大小固定，之後只覆寫內容
    initial_state.pose_targets.assign(pose_target_names_.size(), locomotion::PoseTarget{});
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

  // 系統模式以名稱識別，名稱同時是該模式參數的前綴（<mode>.*）
  const auto modes = get_node()->get_parameter("modes").as_string_array();
  if (modes.empty()) {
    RCLCPP_ERROR(logger, "'modes' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (const auto & mode : modes) {
    if (!is_valid_mode_name(mode)) {
      RCLCPP_ERROR(logger,
        "Invalid mode name [ %s ]: must be non-empty and contain only [A-Za-z0-9_].",
        mode.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (std::find(kReservedModeNames.begin(), kReservedModeNames.end(), mode) !=
      kReservedModeNames.end())
    {
      RCLCPP_ERROR(logger,
        "Mode name [ %s ] is reserved (choreo / bvh / policy are played with their own actions).",
        mode.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (find_mode(mode) != nullptr) {
      RCLCPP_ERROR(logger, "Duplicate mode [ %s ] in 'modes'.", mode.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    try {
      system_modes_.push_back(create_mode(mode));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Failed to configure mode [ %s ]: %s", mode.c_str(), e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
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
    [this](const std_msgs::msg::String::SharedPtr msg) {
      // topic 只有系統模式的名稱；已在該模式時不重新開始，沒有回應（被拒絕時只有 log）
      const auto * mode = find_mode(msg->data);
      if (mode == nullptr) {
        // 32 軸含腿部，未知模式時不移動
        RCLCPP_WARN(get_node()->get_logger(), "~/mode: unknown mode [ %s ], ignored.",
          msg->data.c_str());
        return;
      }
      const auto outcome = arbiter_->request(request_for(*mode), OnSame::kIgnore);
      if (outcome.status == PlaybackOutcome::Status::kRejected) {
        RCLCPP_WARN(get_node()->get_logger(), "~/mode: %s", outcome.message.c_str());
      }
    });

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

  // 末端位姿目標：只取最新值，逾時由 planner 端判斷。座標轉換（相機 → 軀幹）由發送端完成
  if (!pose_target_names_.empty()) {
    pose_target_subscriber_ = get_node()->create_subscription<graphen_interfaces::msg::ArmPoseTarget>(
      "~/pose_targets", rclcpp::SystemDefaultsQoS(),
      [this](const graphen_interfaces::msg::ArmPoseTarget::SharedPtr msg) {
        on_pose_target(*msg);
      });
    std::string names;
    for (const auto & name : pose_target_names_) {
      names += (names.empty() ? "" : ", ") + name;
    }
    RCLCPP_INFO(logger, "Pose targets [ %s ] in frame '%s' on ~/pose_targets (timeout %.2f s).",
      names.c_str(), pose_target_frame_.c_str(), pose_target_timeout_);
  }

  try {
    create_playback_interfaces();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to configure playback: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 等待播完的 goal 以 timer 在 executor thread 中檢查，不碰 RT 路徑
  const double poll_rate = get_node()->get_parameter("switch_mode_poll_rate").as_double();
  if (!(poll_rate > 0.0)) {
    RCLCPP_ERROR(logger, "'switch_mode_poll_rate' must be greater than zero.");
    return controller_interface::CallbackReturn::ERROR;
  }
  wait_timer_ = get_node()->create_wall_timer(
    std::chrono::duration<double>(1.0 / poll_rate), [this]() {
      for (const auto & server : playback_servers_) {
        server->poll();
      }
      publish_playback_state();
    });

  // 啟動時的動作：實際交給 planner 是在 activate 時。空字串表示不進入任何模式，保持目前位置
  const auto default_mode = get_node()->get_parameter("default_mode").as_string();
  const auto default_policy = get_node()->get_parameter("default_policy").as_string();
  if (!default_mode.empty() && !default_policy.empty()) {
    RCLCPP_ERROR(logger, "Set only one of 'default_mode' and 'default_policy'.");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string default_label = "(hold)";
  if (!default_mode.empty()) {
    const auto * mode = find_mode(default_mode);
    if (mode == nullptr) {
      RCLCPP_ERROR(logger, "'default_mode' [ %s ] is not listed in 'modes'.",
        default_mode.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    default_request_ = request_for(*mode);
  } else if (!default_policy.empty()) {
    // 在這裡就載入：啟動時的 policy 設定有誤時，controller 直接 configure 失敗
    try {
      default_request_ = policy_source_->prepare(default_policy);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Failed to load 'default_policy' [ %s ]: %s",
        default_policy.c_str(), e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (default_request_) {
    default_label = playback_label(default_request_->mode, default_request_->item);
  }

  RCLCPP_INFO(logger,
    "SwitchGroupController configured with %zu command interfaces, %zu state interfaces "
    "and %zu system modes (starts with [ %s ], planner %.1f Hz).",
    num_commands, state_interface_names_.size(), system_modes_.size(),
    default_label.c_str(), planner_rate_);

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

  if (default_request_) {
    arbiter_->force(*default_request_);
    RCLCPP_INFO(get_node()->get_logger(), "Starting with [ %s ].",
      playback_label(default_request_->mode, default_request_->item).c_str());
  } else {
    // 尚未進入任何模式時保持位置。保持的目標優先取下游 reference 的值（下游 activate 時
    // 以硬體目前的 command 初始化，也就是硬體正在前往的初始姿態）；無效時才用目前的 state。
    // 若直接用 state，實機馬達還在走向初始姿態時會被停在半路
    for (size_t i = 0; i < initial_positions.size(); ++i) {
      const double reference = command_interfaces_[i].get_value();
      if (std::isfinite(reference)) {
        initial_positions[i] = reference;
      }
    }
    // mode 為空：只是保持，之後任何播放要求都能切入。全部為 NaN 的目標 = 保持進入時的位置
    PlaybackRequest hold;
    hold.motion = std::make_shared<TargetMotion>(
      std::vector<double>(initial_positions.size(), std::numeric_limits<double>::quiet_NaN()),
      default_max_steps_);
    arbiter_->force(std::move(hold));
    RCLCPP_INFO(get_node()->get_logger(),
      "Nothing selected to play, holding current positions until a playback command is received.");
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

const SwitchGroupController::SystemMode * SwitchGroupController::find_mode(
  const std::string & name) const
{
  const auto it = std::find_if(system_modes_.begin(), system_modes_.end(),
      [&name](const SystemMode & mode) {return mode.name == name;});
  return it == system_modes_.end() ? nullptr : &*it;
}

PlaybackRequest SwitchGroupController::request_for(const SystemMode & mode) {
  PlaybackRequest request;
  request.mode = mode.name;
  request.identity = mode.name;
  request.motion = mode.motion;
  request.policy = mode.policy;
  return request;
}

SwitchGroupController::SystemMode SwitchGroupController::create_mode(const std::string & mode) {
  const auto node = get_node();
  const size_t num_commands = joint_names_.size();

  auto max_steps = get_or_declare<std::vector<double>>(node, mode + ".max_steps", {});
  if (max_steps.empty()) {
    max_steps = default_max_steps_;
  } else if (max_steps.size() != num_commands) {
    throw std::invalid_argument(
      "'" + mode + ".max_steps' must contain " + std::to_string(num_commands) + " entries");
  }

  SystemMode result;
  result.name = mode;
  // 演算法 plugin 自己回報的可否中斷（例如內容標示為不可中斷）；其他類型一律可以
  bool motion_interruptible = true;

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
    result.motion = std::make_shared<TargetMotion>(std::move(full_targets), std::move(max_steps));
  } else if (type == "hold") {
    // 保持進入模式時的位置（全部 NaN）；用於停止動作，再切到其他模式時會重新 on_enter
    result.motion = std::make_shared<TargetMotion>(
      std::vector<double>(num_commands, std::numeric_limits<double>::quiet_NaN()),
      std::move(max_steps));
  } else if (type == "plugin") {
    const auto plugin = get_or_declare<std::string>(node, mode + ".plugin", "");
    if (plugin.empty()) {
      throw std::invalid_argument("'" + mode + ".plugin' is empty");
    }
    const NodeParameterSource params(node, mode + ".");
    auto created = plugin_factory_.create(plugin, params, mode, std::move(max_steps));
    motion_interruptible = created.algorithm->interruptible();
    result.motion = std::move(created.motion);
    RCLCPP_INFO(node->get_logger(), "Mode [ %s ]: plugin '%s'.", mode.c_str(), plugin.c_str());
  } else if (type == "bvh") {
    throw std::invalid_argument(
      "type 'bvh' is no longer a system mode: remove this mode, put the file in "
      "'bvh.library_path' and play it with the play_bvh action (see controllers.yaml.example)");
  } else {
    throw std::invalid_argument(
      "unknown type '" + type + "' in '" + mode +
      ".type' (expected 'target', 'hold' or 'plugin')");
  }

  // 停止類（hold）預設可以切入不可中斷的播放：忘了設定時，停止仍然有效。
  // 不可中斷取 YAML 與演算法兩者中較嚴格的
  result.policy.interruptible =
    get_or_declare<bool>(node, mode + ".interruptible", true) && motion_interruptible;
  result.policy.preempt = get_or_declare<bool>(node, mode + ".preempt", type == "hold");
  result.shadow_playback = get_or_declare<bool>(node, mode + ".shadow_playback", false);
  if (!result.policy.interruptible) {
    RCLCPP_INFO(node->get_logger(), "Mode [ %s ] is not interruptible while it is playing.",
      mode.c_str());
  }
  if (result.shadow_playback) {
    RCLCPP_INFO(node->get_logger(),
      "Mode [ %s ]: choreographies are only dry-run while it is playing (shadow_playback).",
      mode.c_str());
  }
  return result;
}

template<typename ActionT>
typename PlaybackServer<ActionT>::Hooks SwitchGroupController::common_hooks() {
  typename PlaybackServer<ActionT>::Hooks hooks;
  hooks.active = [this]() {
      return get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    };
  hooks.fill_state = [this](graphen_interfaces::msg::PlaybackResult & result) {
      result.interface_names = state_interface_names_;
      std::lock_guard<std::mutex> lock(state_snapshot_mutex_);
      result.values = state_snapshot_;
    };
  return hooks;
}

void SwitchGroupController::create_playback_interfaces() {
  const auto node = get_node();
  const auto logger = node->get_logger();

  // 載入內容（讀檔、解析、建立演算法實例）在這個 group 的 thread 進行；一次只處理一個要求。
  // 會載入的 action（編舞、BVH、policy）與所有 service 放在這裡
  playback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  choreo_source_ = std::make_unique<ChoreoSource>(node, &plugin_factory_, default_max_steps_);

  // 影子試播：設定錯誤時丟出例外（configure 失敗）
  ShadowGate::Config gate;
  gate.require = node->get_parameter("choreo.shadow.require").as_bool();
  gate.start_tolerance = node->get_parameter("choreo.shadow.start_tolerance").as_double();
  gate.token_ttl = node->get_parameter("choreo.shadow.token_ttl").as_double();
  shadow_gate_ = std::make_unique<ShadowGate>(gate);
  shadow_max_duration_ = node->get_parameter("choreo.shadow.max_duration").as_double();
  shadow_publish_rate_ = node->get_parameter("choreo.shadow.publish_rate").as_double();
  if (!(shadow_max_duration_ > 0.0) || !std::isfinite(shadow_max_duration_)) {
    throw std::invalid_argument("'choreo.shadow.max_duration' must be a finite number > 0");
  }
  if (!(shadow_publish_rate_ > 0.0) || !std::isfinite(shadow_publish_rate_)) {
    throw std::invalid_argument("'choreo.shadow.publish_rate' must be a finite number > 0");
  }
  // 一般的 rclcpp publisher（不受 lifecycle 啟用狀態限制）：未啟用時也要能發布空的 playback_state
  {
    std::lock_guard<std::mutex> lock(shadow_mutex_);
    shadow_replay_.reset();
  }
  shadow_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::JointState>(
    node, "~/shadow/joint_states", rclcpp::SystemDefaultsQoS());
  shadow_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / shadow_publish_rate_),
    [this]() {publish_shadow_trajectory();});
  last_playback_state_.reset();
  playback_state_publisher_ = rclcpp::create_publisher<graphen_interfaces::msg::PlaybackState>(
    node, "~/playback_state", rclcpp::QoS(1).reliable().transient_local());
  if (gate.require) {
    RCLCPP_INFO(logger, "Choreographies need a passed dry run (choreo.shadow.require).");
  }
  bvh_source_ = std::make_unique<BvhSource>(node, joint_names_, default_max_steps_);
  policy_source_ = std::make_unique<PolicySource>(node, &plugin_factory_, default_max_steps_);

  // --- 系統模式：不需要載入
  {
    auto hooks = common_hooks<SwitchModeAction>();
    hooks.validate = [this](const SwitchModeAction::Goal & goal) {
        return find_mode(goal.mode) != nullptr ? std::string{} : std::string{"unknown mode"};
      };
    hooks.start = [this](const SwitchModeAction::Goal & goal, OnSame on_same) {
        return arbiter_->request(request_for(*find_mode(goal.mode)), on_same);
      };
    hooks.describe = [](const SwitchModeAction::Goal & goal) {return goal.mode;};
    // 放在預設的 callback group（nullptr）：系統模式不需要載入，停止不能排在別人的載入後面
    playback_servers_.push_back(std::make_unique<PlaybackServer<SwitchModeAction>>(
        node, "switch_mode", nullptr, arbiter_.get(), std::move(hooks)));
  }

  // --- 編舞：舞曲庫中的名稱，或直接帶內容試播
  if (choreo_source_->enabled()) {
    auto hooks = common_hooks<PlayChoreoAction>();
    hooks.validate = [](const PlayChoreoAction::Goal & goal) -> std::string {
        if (goal.name.empty() == goal.content.empty()) {
          return "exactly one of 'name' and 'content' is required";
        }
        if (!goal.name.empty() && !is_valid_item_name(goal.name)) {
          return "invalid name";
        }
        return {};
      };
    hooks.start = [this](const PlayChoreoAction::Goal & goal, OnSame on_same) {
        auto request = goal.name.empty() ? choreo_source_->prepare_preview(goal.content) :
          choreo_source_->prepare(goal.name);
        if (goal.options.dry_run) {
          return run_dry(request, playback_label(ChoreoSource::kMode, request.item));
        }
        // 目前在示教等 shadow_playback 的模式：一律只做影子試播（帶 token 也一樣），要上機先切離該模式
        if (const auto mode = shadow_playback_mode()) {
          auto outcome = run_dry(request, playback_label(ChoreoSource::kMode, request.item));
          outcome.message = "Shadow playback in mode [ " + *mode + " ] (switch to another mode to "
            "play it for real): " + outcome.message;
          return outcome;
        }
        check_shadow_token(goal.options.shadow_token, request);
        return arbiter_->request(std::move(request), on_same);
      };
    hooks.describe = [](const PlayChoreoAction::Goal & goal) {
        return playback_label(ChoreoSource::kMode, goal.name.empty() ? "(preview)" : goal.name) +
               (goal.options.dry_run ? " (dry run)" : "");
      };
    hooks.supports_shadow = true;
    playback_servers_.push_back(std::make_unique<PlaybackServer<PlayChoreoAction>>(
        node, "play_choreo", playback_group_, arbiter_.get(), std::move(hooks)));

    using Publish = graphen_interfaces::srv::PublishChoreo;
    publish_choreo_service_ = node->create_service<Publish>(
      "~/publish_choreo",
      [this](const Publish::Request::SharedPtr request, Publish::Response::SharedPtr response) {
        try {
          response->sha256 = choreo_source_->publish(request->name, request->content);
          response->success = true;
          response->message = "Published";
          RCLCPP_INFO(get_node()->get_logger(), "Published choreography [ %s ], sha256 %s.",
            request->name.c_str(), short_hash(response->sha256).c_str());
        } catch (const std::exception & e) {
          response->success = false;
          response->message = e.what();
          RCLCPP_WARN(get_node()->get_logger(), "publish_choreo [ %s ]: %s",
            request->name.c_str(), e.what());
        }
      },
      rmw_qos_profile_services_default, playback_group_);

    using List = graphen_interfaces::srv::ListChoreos;
    list_choreos_service_ = node->create_service<List>(
      "~/list_choreos",
      [this](const List::Request::SharedPtr, List::Response::SharedPtr response) {
        for (auto & entry : choreo_source_->list()) {
          response->names.push_back(std::move(entry.name));
          response->sha256.push_back(std::move(entry.sha256));
        }
      },
      rmw_qos_profile_services_default, playback_group_);

    using Get = graphen_interfaces::srv::GetChoreo;
    get_choreo_service_ = node->create_service<Get>(
      "~/get_choreo",
      [this](const Get::Request::SharedPtr request, Get::Response::SharedPtr response) {
        try {
          response->content = choreo_source_->get(request->name, response->sha256);
          response->success = true;
        } catch (const std::exception & e) {
          response->success = false;
          response->message = e.what();
        }
      },
      rmw_qos_profile_services_default, playback_group_);
  } else {
    RCLCPP_INFO(logger, "Choreographies are not enabled ('choreo.plugin' is empty).");
  }

  // --- BVH
  if (bvh_source_->enabled()) {
    auto hooks = common_hooks<PlayBvhAction>();
    hooks.validate = [](const PlayBvhAction::Goal & goal) -> std::string {
        if (!is_valid_item_name(goal.name)) {
          return "invalid name";
        }
        if (!(goal.time_scale >= 0.0) || !std::isfinite(goal.time_scale)) {
          return "time_scale must be a finite number >= 0";
        }
        return {};
      };
    hooks.needs_loading = [this](const PlayBvhAction::Goal & goal) {
        return !bvh_source_->is_loaded(goal.name);
      };
    hooks.start = [this](const PlayBvhAction::Goal & goal, OnSame on_same) {
        // 0 視為 1.0：沒有填 time_scale 的 goal（欄位為 0）照原速播放
        const double time_scale = goal.time_scale == 0.0 ? 1.0 : goal.time_scale;
        return arbiter_->request(
          bvh_source_->prepare(goal.name, goal.loop, time_scale), on_same);
      };
    hooks.describe = [](const PlayBvhAction::Goal & goal) {
        return playback_label(BvhSource::kMode, goal.name);
      };
    playback_servers_.push_back(std::make_unique<PlaybackServer<PlayBvhAction>>(
        node, "play_bvh", playback_group_, arbiter_.get(), std::move(hooks)));
  } else {
    RCLCPP_INFO(logger, "BVH playback is not enabled ('bvh.library_path' is empty).");
  }

  // --- policy
  if (policy_source_->enabled()) {
    auto hooks = common_hooks<RunPolicyAction>();
    hooks.validate = [](const RunPolicyAction::Goal & goal) -> std::string {
        return is_valid_item_name(goal.name) ? "" : "invalid name";
      };
    hooks.needs_loading = [this](const RunPolicyAction::Goal & goal) {
        return !policy_source_->is_loaded(goal.name);
      };
    hooks.start = [this](const RunPolicyAction::Goal & goal, OnSame on_same) {
        return arbiter_->request(policy_source_->prepare(goal.name), on_same);
      };
    hooks.describe = [](const RunPolicyAction::Goal & goal) {
        return playback_label(PolicySource::kMode, goal.name);
      };
    playback_servers_.push_back(std::make_unique<PlaybackServer<RunPolicyAction>>(
        node, "run_policy", playback_group_, arbiter_.get(), std::move(hooks)));
  } else {
    RCLCPP_INFO(logger, "Policies are not enabled ('policy.library_path' is empty).");
  }

  // --- 預載：只有需要載入時間的類型（BVH、policy）
  if (bvh_source_->enabled() || policy_source_->enabled()) {
    using Preload = graphen_interfaces::srv::Preload;
    preload_service_ = node->create_service<Preload>(
      "~/preload",
      [this](const Preload::Request::SharedPtr request, Preload::Response::SharedPtr response) {
        response->success = true;
        for (const auto & item : request->items) {
          std::string message;
          try {
            if (request->mode == BvhSource::kMode) {
              bvh_source_->preload(item);
            } else if (request->mode == PolicySource::kMode) {
              policy_source_->preload(item);
            } else {
              message = "unknown mode '" + request->mode + "' (expected 'bvh' or 'policy')";
            }
          } catch (const std::exception & e) {
            message = e.what();
          }
          if (!message.empty()) {
            response->success = false;
            RCLCPP_WARN(get_node()->get_logger(), "preload [ %s ]: %s",
              playback_label(request->mode, item).c_str(), message.c_str());
          }
          response->messages.push_back(std::move(message));
        }
      },
      rmw_qos_profile_services_default, playback_group_);
  }
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

void SwitchGroupController::on_pose_target(const graphen_interfaces::msg::ArmPoseTarget & msg) {
  auto & clock = *get_node()->get_clock();
  const auto it = std::find(pose_target_names_.begin(), pose_target_names_.end(), msg.arm);
  if (it == pose_target_names_.end()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 2000,
      "~/pose_targets: unknown arm '%s', ignored.", msg.arm.c_str());
    return;
  }
  if (msg.header.frame_id != pose_target_frame_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 2000,
      "~/pose_targets: frame_id '%s' is not '%s' (transform it before publishing), ignored.",
      msg.header.frame_id.c_str(), pose_target_frame_.c_str());
    return;
  }
  const auto & p = msg.pose.position;
  const auto & o = msg.pose.orientation;
  const double norm = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
  // swivel 可以是 NaN（不指定），inf 不行
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) ||
    norm < 1e-6 || std::isinf(msg.swivel))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 2000,
      "~/pose_targets: non-finite pose or zero quaternion for '%s', ignored.", msg.arm.c_str());
    return;
  }
  locomotion::PoseTarget target;
  target.position = {p.x, p.y, p.z};
  target.orientation = {o.x / norm, o.y / norm, o.z / norm, o.w / norm};
  target.swivel = msg.swivel;
  std::lock_guard<std::mutex> lock(pose_target_mutex_);
  auto & command = pose_target_commands_[static_cast<size_t>(
    std::distance(pose_target_names_.begin(), it))];
  command.target = target;
  command.stamp = std::chrono::steady_clock::now();
  command.received = true;
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

  // 末端位姿目標：逾時或尚未收到時 active 為 false
  if (!pose_target_commands_.empty()) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(pose_target_mutex_);
    for (size_t i = 0; i < pose_target_commands_.size() && i < state.pose_targets.size(); ++i) {
      const auto & command = pose_target_commands_[i];
      auto & target = state.pose_targets[i];
      target = command.target;
      target.age = command.received ?
        std::chrono::duration<double>(now - command.stamp).count() :
        std::numeric_limits<double>::infinity();
      target.active = command.received && target.age <= pose_target_timeout_;
    }
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

PlaybackOutcome SwitchGroupController::run_dry(
  const PlaybackRequest & request, const std::string & label)
{
  const auto start = planner_ ? planner_->snapshot() : std::nullopt;
  if (!start) {
    throw std::runtime_error("the planner has not started yet; try again");
  }
  auto result = run_shadow(
    request.motion, *start, planner_rate_, shadow_max_duration_, shadow_publish_rate_);

  PlaybackOutcome outcome;
  outcome.message = result.message;
  if (result.success) {
    outcome.status = PlaybackOutcome::Status::kDryRun;
    outcome.shadow_token = shadow_gate_->issue(request.content_sha256, start->commanded_positions);
    RCLCPP_INFO(get_node()->get_logger(), "Dry run [ %s ] passed (%.2f s), shadow_token %s.",
      label.c_str(), result.duration, outcome.shadow_token.c_str());
  } else {
    outcome.status = PlaybackOutcome::Status::kRejected;
    RCLCPP_WARN(get_node()->get_logger(), "Dry run [ %s ]: %s", label.c_str(),
      result.message.c_str());
  }
  // 通過與否都發布軌跡（失敗時可以看到停在哪裡）；新的試播取代舊的
  std::lock_guard<std::mutex> lock(shadow_mutex_);
  shadow_replay_ = ShadowReplay{std::move(result), std::chrono::steady_clock::now(), 0};
  return outcome;
}

std::optional<std::string> SwitchGroupController::shadow_playback_mode() const {
  if (!arbiter_) {
    return std::nullopt;
  }
  const auto current = arbiter_->current();
  const auto * mode = find_mode(current.mode);
  if (mode == nullptr || !mode->shadow_playback) {
    return std::nullopt;
  }
  return current.mode;
}

void SwitchGroupController::check_shadow_token(
  const std::string & token, const PlaybackRequest & request) const
{
  if (token.empty() && !shadow_gate_->required()) {
    return;
  }
  const auto current = planner_ ? planner_->snapshot() : std::nullopt;
  if (!current) {
    throw std::runtime_error("the planner has not started yet; try again");
  }
  // 驗證與 planner 實際 on_enter 之間差不到一個週期，由 start_tolerance 涵蓋
  const auto reason = shadow_gate_->verify(
    token, request.content_sha256, current->commanded_positions, joint_names_);
  if (!reason.empty()) {
    throw std::runtime_error(reason);
  }
}

void SwitchGroupController::publish_playback_state() {
  graphen_interfaces::msg::PlaybackState state;
  if (arbiter_ && get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    const auto current = arbiter_->current();
    state.mode = current.mode;
    state.item = current.item;
  }
  if (last_playback_state_ && *last_playback_state_ == state) {
    return;
  }
  last_playback_state_ = state;
  playback_state_publisher_->publish(state);
}

void SwitchGroupController::publish_shadow_trajectory() {
  std::lock_guard<std::mutex> lock(shadow_mutex_);
  if (!shadow_replay_) {
    return;
  }
  auto & replay = *shadow_replay_;
  const auto & times = replay.trajectory.times;
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - replay.started).count();
  // 只發布到期的最後一筆（落後時跳過中間的取樣）
  size_t index = replay.next;
  if (index >= times.size() || times[index] > elapsed) {
    return;
  }
  while (index + 1 < times.size() && times[index + 1] <= elapsed) {
    ++index;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = get_node()->now();
  msg.name = joint_names_;
  msg.position = replay.trajectory.positions[index];
  shadow_publisher_->publish(msg);
  replay.next = index + 1;
  if (replay.next >= times.size()) {
    shadow_replay_.reset();
  }
}

}  // namespace graphen::control


PLUGINLIB_EXPORT_CLASS(
  graphen::control::SwitchGroupController,
  controller_interface::ControllerInterface)
