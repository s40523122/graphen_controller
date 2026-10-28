#include "graphen_mujoco/mujoco_system.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace graphen_mujoco {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {

// 模型中 IMU 感測器的名稱（tools/export_mjcf.py）
constexpr const char * kQuatSensor = "imu_quat";
constexpr const char * kGyroSensor = "gyro";
constexpr const char * kAccSensor = "imu_acc";
constexpr double kImuTemperature = 25.0;

// URDF 中 IMU state interface 的名稱，順序對應 ImuIndex
constexpr std::array<const char *, 11> kImuStateNames{
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
  "temperature"};

// 一個週期最多推進的步數；controller 迴圈卡住後不一次補跑太多，模擬時間會落後真實時間
constexpr int kMaxStepsPerCycle = 20;
constexpr int kWarnThrottleMs = 2000;
constexpr const char * kResetCommand = "reset";

std::string get_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

// 依類型找感測器的資料位址：只有一個時直接使用，有多個時取名稱為 preferred 的；找不到回傳 -1
int sensor_adr(const mjModel * model, mjtSensor type, const char * preferred, int dim) {
  int found = -1;
  int count = 0;
  for (int i = 0; i < model->nsensor; ++i) {
    if (model->sensor_type[i] != type || model->sensor_dim[i] != dim) {
      continue;
    }
    const char * name = mj_id2name(model, mjOBJ_SENSOR, i);
    if (name != nullptr && std::strcmp(name, preferred) == 0) {
      return model->sensor_adr[i];
    }
    found = i;
    ++count;
  }
  return count == 1 ? model->sensor_adr[found] : -1;
}

bool parse_bool(const std::string & value) {
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  throw std::invalid_argument(value);
}

}  // namespace

MujocoSystem::~MujocoSystem() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
}

CallbackReturn MujocoSystem::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & params = info_.hardware_parameters;
  const std::string model_path = get_param(params, "model_path", "");
  if (model_path.empty()) {
    RCLCPP_ERROR(logger_, "Parameter 'model_path' is required.");
    return CallbackReturn::ERROR;
  }
  try {
    initial_base_height_ = std::stod(get_param(params, "initial_base_height", "0.894"));
    fixed_base_ = parse_bool(get_param(params, "fixed_base", "false"));
    viewer_port_ = std::stoi(get_param(params, "viewer_port", "0"));
    const double viewer_rate = std::stod(get_param(params, "viewer_rate", "60.0"));
    if (!(viewer_rate > 0.0)) {
      throw std::invalid_argument("viewer_rate");
    }
    viewer_period_ = 1.0 / viewer_rate;
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_,
      "Invalid 'initial_base_height', 'fixed_base', 'viewer_port' or 'viewer_rate' parameter.");
    return CallbackReturn::ERROR;
  }
  const std::string missing_joints = get_param(params, "missing_joints", "error");
  if (missing_joints != "error" && missing_joints != "virtual") {
    RCLCPP_ERROR(logger_, "'missing_joints' must be 'error' or 'virtual', got '%s'.",
      missing_joints.c_str());
    return CallbackReturn::ERROR;
  }
  allow_virtual_joints_ = missing_joints == "virtual";
  if (viewer_port_ < 0 || viewer_port_ > 65534) {
    RCLCPP_ERROR(logger_, "'viewer_port' must be in [0, 65534], got %d.", viewer_port_);
    return CallbackReturn::ERROR;
  }

  if (!load_model(model_path)) {
    return CallbackReturn::ERROR;
  }

  // joint：依名稱對應模型的 joint 與同名致動器
  joints_.resize(info_.joints.size());
  size_t num_unsupported = 0;
  std::string virtual_names;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_info = info_.joints[i];
    auto & joint = joints_[i];
    if (joint_info.command_interfaces.size() != 1 ||
      (joint_info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
      joint_info.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(logger_,
        "Joint '%s' must have exactly one 'position' or 'effort' command interface.",
        joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint.effort_mode =
      joint_info.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT;
    for (const auto & iface : joint_info.state_interfaces) {
      if (iface.name != hardware_interface::HW_IF_POSITION &&
        iface.name != hardware_interface::HW_IF_VELOCITY &&
        iface.name != hardware_interface::HW_IF_ACCELERATION)
      {
        ++num_unsupported;
      }
      if (iface.name == hardware_interface::HW_IF_POSITION && !iface.initial_value.empty()) {
        try {
          joint.initial_position = std::stod(iface.initial_value);
        } catch (const std::exception &) {
          RCLCPP_ERROR(logger_, "Invalid initial_value '%s' of joint '%s'.",
            iface.initial_value.c_str(), joint_info.name.c_str());
          return CallbackReturn::ERROR;
        }
      }
    }

    const int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_info.name.c_str());
    if (joint_id < 0 && allow_virtual_joints_) {
      joint.virtual_joint = true;
      virtual_names += (virtual_names.empty() ? "" : ", ") + joint_info.name;
      continue;
    }
    if (joint_id < 0 || (model_->jnt_type[joint_id] != mjJNT_HINGE &&
      model_->jnt_type[joint_id] != mjJNT_SLIDE))
    {
      RCLCPP_ERROR(logger_, "Joint '%s' is not a hinge or slide joint in the MuJoCo model%s.",
        joint_info.name.c_str(),
        joint_id < 0 ? " (set missing_joints to 'virtual' to ignore missing joints)" : "");
      return CallbackReturn::ERROR;
    }
    const int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, joint_info.name.c_str());
    if (actuator_id < 0 || model_->actuator_trntype[actuator_id] != mjTRN_JOINT ||
      model_->actuator_trnid[2 * actuator_id] != joint_id)
    {
      RCLCPP_ERROR(logger_, "No actuator named '%s' driving joint '%s' in the MuJoCo model.",
        joint_info.name.c_str(), joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint.qpos_adr = model_->jnt_qposadr[joint_id];
    joint.dof_adr = model_->jnt_dofadr[joint_id];
    joint.actuator = actuator_id;

    // 致動器類型必須與 command interface 相符
    const bool fixed_gain = model_->actuator_gaintype[actuator_id] == mjGAIN_FIXED;
    const bool servo = fixed_gain && model_->actuator_biastype[actuator_id] == mjBIAS_AFFINE;
    const bool motor = fixed_gain && model_->actuator_biastype[actuator_id] == mjBIAS_NONE;
    mjtNum * gain = model_->actuator_gainprm + actuator_id * mjNGAIN;
    mjtNum * bias = model_->actuator_biasprm + actuator_id * mjNBIAS;
    const mjtNum gear = model_->actuator_gear[actuator_id * 6];
    if (!joint.effort_mode && !servo) {
      RCLCPP_ERROR(logger_,
        "Actuator '%s' is not a position servo (fixed gain, affine bias); motor actuators need "
        "the 'effort' command interface (mujoco_control_mode:=effort).", joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (joint.effort_mode) {
      if (servo) {
        // 位置伺服改成力矩馬達：ctrl 直接是力矩（forcerange 仍然有效）
        gain[0] = 1.0;
        bias[0] = bias[1] = bias[2] = 0.0;
        model_->actuator_biastype[actuator_id] = mjBIAS_NONE;
      } else if (!motor) {
        RCLCPP_ERROR(logger_, "Actuator '%s' must be a motor or a position servo for effort mode.",
          joint_info.name.c_str());
        return CallbackReturn::ERROR;
      }
      // 力矩 = gear × gain × ctrl，必須為 1 才能直接把 command 當成力矩
      if (std::abs(gain[0] * gear - 1.0) > 1e-9) {
        RCLCPP_ERROR(logger_, "Actuator '%s' must have gear × gain = 1 for effort mode, got %f.",
          joint_info.name.c_str(), gain[0] * gear);
        return CallbackReturn::ERROR;
      }
      has_effort_joints_ = true;
    }
  }
  if (!virtual_names.empty()) {
    RCLCPP_WARN(logger_,
      "Joints not in the MuJoCo model are virtual (state fixed at the initial value, commands "
      "ignored): %s", virtual_names.c_str());
  }

  unsupported_states_.assign(num_unsupported, std::numeric_limits<double>::quiet_NaN());

  // 浮動基座：模型的第一個 free joint
  for (int j = 0; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] == mjJNT_FREE) {
      free_qpos_adr_ = model_->jnt_qposadr[j];
      free_dof_adr_ = model_->jnt_dofadr[j];
      break;
    }
  }

  // IMU：最多一個 <sensor>，介面必須是 kImuStateNames 中的名稱
  if (info_.sensors.size() > 1) {
    RCLCPP_ERROR(logger_, "Expected at most 1 sensor (the IMU), got %zu.", info_.sensors.size());
    return CallbackReturn::ERROR;
  }
  if (!info_.sensors.empty()) {
    const auto & sensor = info_.sensors.front();
    for (const auto & iface : sensor.state_interfaces) {
      if (std::find_if(kImuStateNames.begin(), kImuStateNames.end(),
        [&](const char * n) {return iface.name == n;}) == kImuStateNames.end())
      {
        RCLCPP_ERROR(logger_, "Unknown state interface '%s' of sensor '%s'.",
          iface.name.c_str(), sensor.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    quat_adr_ = sensor_adr(model_, mjSENS_FRAMEQUAT, kQuatSensor, 4);
    gyro_adr_ = sensor_adr(model_, mjSENS_GYRO, kGyroSensor, 3);
    acc_adr_ = sensor_adr(model_, mjSENS_ACCELEROMETER, kAccSensor, 3);
    if (quat_adr_ < 0 || gyro_adr_ < 0 || acc_adr_ < 0) {
      RCLCPP_ERROR(logger_,
        "Sensor '%s' needs exactly one framequat, gyro and accelerometer sensor in the MuJoCo "
        "model (or ones named '%s', '%s', '%s' when there are several).",
        sensor.name.c_str(), kQuatSensor, kGyroSensor, kAccSensor);
      return CallbackReturn::ERROR;
    }
    imu_name_ = sensor.name;
  }

  if (viewer_port_ > 0 && !open_viewer_socket()) {
    return CallbackReturn::ERROR;
  }
  // [time, qpos(nq), actuator_force(nu)]；viewer 依同一份 MJCF 拆解
  viewer_buffer_.resize(1 + static_cast<size_t>(model_->nq) + static_cast<size_t>(model_->nu));

  reset_simulation();
  if (fixed_base_ && free_qpos_adr_ < 0) {
    RCLCPP_INFO(logger_, "'fixed_base' has no effect: the model has no free joint.");
  }
  RCLCPP_INFO(logger_,
    "Loaded '%s': %zu joints (%s), %s base, IMU %s, timestep %.4f s, viewer port %d.",
    model_path.c_str(), joints_.size(), has_effort_joints_ ? "effort" : "position",
    free_qpos_adr_ < 0 ? "fixed" : (fixed_base_ ? "pinned floating" : "floating"),
    imu_name_.empty() ? "disabled" : imu_name_.c_str(), model_->opt.timestep, viewer_port_);
  return CallbackReturn::SUCCESS;
}

bool MujocoSystem::load_model(const std::string & path) {
  char error[1000] = "";
  model_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to load MuJoCo model '%s': %s", path.c_str(), error);
    return false;
  }
  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to allocate mjData.");
    return false;
  }
  return true;
}

bool MujocoSystem::open_viewer_socket() {
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger_, "socket() failed: %s", std::strerror(errno));
    return false;
  }
  ::fcntl(socket_fd_, F_SETFL, ::fcntl(socket_fd_, F_GETFL, 0) | O_NONBLOCK);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(viewer_port_ + 1));
  if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger_, "Cannot bind viewer command port %d: %s",
      viewer_port_ + 1, std::strerror(errno));
    return false;
  }
  return true;
}

void MujocoSystem::reset_simulation() {
  mj_resetData(model_, data_);
  if (free_qpos_adr_ >= 0) {
    double * root = data_->qpos + free_qpos_adr_;
    root[0] = 0.0;
    root[1] = 0.0;
    root[2] = initial_base_height_;
    root[3] = 1.0;
    root[4] = 0.0;
    root[5] = 0.0;
    root[6] = 0.0;
  }
  for (auto & joint : joints_) {
    if (joint.virtual_joint) {
      joint.command = joint.effort_mode ? std::numeric_limits<double>::quiet_NaN() :
        joint.initial_position;
      continue;
    }
    data_->qpos[joint.qpos_adr] = joint.initial_position;
    if (joint.effort_mode) {
      // 物理暫停到收到新的力矩指令為止
      data_->ctrl[joint.actuator] = 0.0;
      joint.command = std::numeric_limits<double>::quiet_NaN();
    } else {
      data_->ctrl[joint.actuator] = joint.initial_position;
      joint.command = joint.initial_position;
    }
  }
  commands_ready_ = !has_effort_joints_;
  mj_forward(model_, data_);
  pending_time_ = 0.0;
  last_viewer_send_ = -1.0;
  copy_states();
}

CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // 從目前位置開始，避免啟動瞬間跳動；effort 模式等 controller 寫入力矩（之前物理暫停）
  for (auto & joint : joints_) {
    joint.command = joint.effort_mode ? std::numeric_limits<double>::quiet_NaN() : joint.position;
  }
  return CallbackReturn::SUCCESS;
}

bool MujocoSystem::effort_commands_ready() const {
  return std::all_of(joints_.begin(), joints_.end(), [](const Joint & joint) {
             return !joint.effort_mode || joint.virtual_joint || std::isfinite(joint.command);
           });
}

void MujocoSystem::pin_base() {
  double * root = data_->qpos + free_qpos_adr_;
  root[0] = 0.0;
  root[1] = 0.0;
  root[2] = initial_base_height_;
  root[3] = 1.0;
  root[4] = 0.0;
  root[5] = 0.0;
  root[6] = 0.0;
  std::fill(data_->qvel + free_dof_adr_, data_->qvel + free_dof_adr_ + 6, 0.0);
}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  size_t next_unsupported = 0;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & iface : info_.joints[i].state_interfaces) {
      double * value = nullptr;
      if (iface.name == hardware_interface::HW_IF_POSITION) {
        value = &joints_[i].position;
      } else if (iface.name == hardware_interface::HW_IF_VELOCITY) {
        value = &joints_[i].velocity;
      } else if (iface.name == hardware_interface::HW_IF_ACCELERATION) {
        value = &joints_[i].acceleration;
      } else {
        // 模擬沒有的資料（例如 ti5 的 current、error_code）：與 mock 相同，固定為 NaN
        value = &unsupported_states_[next_unsupported++];
      }
      interfaces.emplace_back(info_.joints[i].name, iface.name, value);
    }
  }
  if (!imu_name_.empty()) {
    for (const auto & iface : info_.sensors.front().state_interfaces) {
      for (size_t k = 0; k < kImuStateNames.size(); ++k) {
        if (iface.name == kImuStateNames[k]) {
          interfaces.emplace_back(imu_name_, iface.name, &imu_values_[k]);
        }
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      joints_[i].effort_mode ? hardware_interface::HW_IF_EFFORT : hardware_interface::HW_IF_POSITION,
      &joints_[i].command);
  }
  return interfaces;
}

void MujocoSystem::copy_states() {
  for (auto & joint : joints_) {
    if (joint.virtual_joint) {
      joint.position = joint.initial_position;
      joint.velocity = 0.0;
      joint.acceleration = 0.0;
      continue;
    }
    joint.position = data_->qpos[joint.qpos_adr];
    joint.velocity = data_->qvel[joint.dof_adr];
    joint.acceleration = data_->qacc[joint.dof_adr];
  }
  if (!imu_name_.empty()) {
    const mjtNum * quat = data_->sensordata + quat_adr_;  // MuJoCo 順序 w, x, y, z
    imu_values_[kOrientationX] = quat[1];
    imu_values_[kOrientationY] = quat[2];
    imu_values_[kOrientationZ] = quat[3];
    imu_values_[kOrientationW] = quat[0];
    for (size_t k = 0; k < 3; ++k) {
      imu_values_[kAngularVelocityX + k] = data_->sensordata[gyro_adr_ + static_cast<int>(k)];
      imu_values_[kLinearAccelerationX + k] = data_->sensordata[acc_adr_ + static_cast<int>(k)];
    }
    imu_values_[kTemperature] = kImuTemperature;
  }
}

return_type MujocoSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (socket_fd_ >= 0) {
    poll_viewer_commands();
  }
  copy_states();
  if (socket_fd_ >= 0 && data_->time - last_viewer_send_ >= viewer_period_) {
    send_viewer_frame();
  }
  return return_type::OK;
}

return_type MujocoSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
  // effort 模式：所有 joint 都收到第一個有效的力矩之前物理暫停，controller 啟動前不會倒下
  if (!commands_ready_) {
    if (!effort_commands_ready()) {
      return return_type::OK;
    }
    commands_ready_ = true;
    pending_time_ = 0.0;
    RCLCPP_INFO(logger_, "All effort commands received, simulation running.");
  }

  for (const auto & joint : joints_) {
    // 上層尚未寫入（NaN）時保持上一個 ctrl
    if (!joint.virtual_joint && std::isfinite(joint.command)) {
      data_->ctrl[joint.actuator] = joint.command;
    }
  }

  pending_time_ += period.seconds();
  const double timestep = model_->opt.timestep;
  const bool pinned = fixed_base_ && free_qpos_adr_ >= 0;
  int steps = 0;
  while (pending_time_ >= timestep && steps < kMaxStepsPerCycle) {
    mj_step(model_, data_);
    if (pinned) {
      pin_base();
    }
    pending_time_ -= timestep;
    ++steps;
  }
  if (pinned && steps > 0) {
    // 讓感測器（IMU）反映重設後的基座
    mj_forward(model_, data_);
  }
  if (pending_time_ >= timestep) {
    RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, kWarnThrottleMs,
      "Controller cycle too long (%.3f s), simulation is falling behind real time.",
      period.seconds());
    pending_time_ = 0.0;
  }
  if (data_->warning[mjWARN_BADQACC].number > 0) {
    RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, kWarnThrottleMs,
      "MuJoCo simulation became unstable (bad qacc) and was reset by MuJoCo.");
    data_->warning[mjWARN_BADQACC].number = 0;
  }
  return return_type::OK;
}

void MujocoSystem::send_viewer_frame() {
  viewer_buffer_[0] = data_->time;
  const auto force_begin =
    std::copy(data_->qpos, data_->qpos + model_->nq, viewer_buffer_.begin() + 1);
  std::copy(data_->actuator_force, data_->actuator_force + model_->nu, force_begin);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(viewer_port_));
  // viewer 沒有開時送出會失敗（ECONNREFUSED），忽略即可
  ::sendto(socket_fd_, viewer_buffer_.data(), viewer_buffer_.size() * sizeof(double), 0,
    reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  last_viewer_send_ = data_->time;
}

void MujocoSystem::poll_viewer_commands() {
  char buffer[64];
  ssize_t n;
  while ((n = ::recv(socket_fd_, buffer, sizeof(buffer) - 1, 0)) > 0) {
    buffer[n] = '\0';
    if (std::strncmp(buffer, kResetCommand, std::strlen(kResetCommand)) == 0) {
      RCLCPP_INFO(logger_, "Reset requested by viewer.");
      reset_simulation();
    }
  }
}

}  // namespace graphen_mujoco

PLUGINLIB_EXPORT_CLASS(graphen_mujoco::MujocoSystem, hardware_interface::SystemInterface)
