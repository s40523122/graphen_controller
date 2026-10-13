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

int sensor_adr(const mjModel * model, const char * name, int dim) {
  const int id = mj_name2id(model, mjOBJ_SENSOR, name);
  if (id < 0 || model->sensor_dim[id] != dim) {
    return -1;
  }
  return model->sensor_adr[id];
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
    viewer_port_ = std::stoi(get_param(params, "viewer_port", "0"));
    const double viewer_rate = std::stod(get_param(params, "viewer_rate", "60.0"));
    if (!(viewer_rate > 0.0)) {
      throw std::invalid_argument("viewer_rate");
    }
    viewer_period_ = 1.0 / viewer_rate;
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_,
      "Invalid 'initial_base_height', 'viewer_port' or 'viewer_rate' parameter.");
    return CallbackReturn::ERROR;
  }
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
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_info = info_.joints[i];
    auto & joint = joints_[i];
    if (joint_info.command_interfaces.size() != 1 ||
      joint_info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger_, "Joint '%s' must have exactly one 'position' command interface.",
        joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
    const int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_info.name.c_str());
    if (joint_id < 0 || (model_->jnt_type[joint_id] != mjJNT_HINGE &&
      model_->jnt_type[joint_id] != mjJNT_SLIDE))
    {
      RCLCPP_ERROR(logger_, "Joint '%s' is not a hinge or slide joint in the MuJoCo model.",
        joint_info.name.c_str());
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
    for (const auto & iface : joint_info.state_interfaces) {
      if (iface.name != hardware_interface::HW_IF_POSITION &&
        iface.name != hardware_interface::HW_IF_VELOCITY &&
        iface.name != hardware_interface::HW_IF_ACCELERATION)
      {
        ++num_unsupported;
      }
    }
    joint.qpos_adr = model_->jnt_qposadr[joint_id];
    joint.dof_adr = model_->jnt_dofadr[joint_id];
    joint.actuator = actuator_id;
    for (const auto & iface : joint_info.state_interfaces) {
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
  }

  unsupported_states_.assign(num_unsupported, std::numeric_limits<double>::quiet_NaN());

  // 浮動基座：模型的第一個 free joint
  for (int j = 0; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] == mjJNT_FREE) {
      free_qpos_adr_ = model_->jnt_qposadr[j];
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
    quat_adr_ = sensor_adr(model_, kQuatSensor, 4);
    gyro_adr_ = sensor_adr(model_, kGyroSensor, 3);
    acc_adr_ = sensor_adr(model_, kAccSensor, 3);
    if (quat_adr_ < 0 || gyro_adr_ < 0 || acc_adr_ < 0) {
      RCLCPP_ERROR(logger_,
        "Sensor '%s' needs MuJoCo sensors '%s' (framequat), '%s' (gyro) and '%s' "
        "(accelerometer); re-export the model with tools/export_mjcf.py.",
        sensor.name.c_str(), kQuatSensor, kGyroSensor, kAccSensor);
      return CallbackReturn::ERROR;
    }
    imu_name_ = sensor.name;
  }

  if (viewer_port_ > 0 && !open_viewer_socket()) {
    return CallbackReturn::ERROR;
  }
  viewer_buffer_.resize(1 + static_cast<size_t>(model_->nq));

  reset_simulation();
  RCLCPP_INFO(logger_,
    "Loaded '%s': %zu joints, %s base, IMU %s, timestep %.4f s, viewer port %d.",
    model_path.c_str(), joints_.size(), free_qpos_adr_ >= 0 ? "floating" : "fixed",
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
    data_->qpos[joint.qpos_adr] = joint.initial_position;
    data_->ctrl[joint.actuator] = joint.initial_position;
    joint.command = joint.initial_position;
  }
  mj_forward(model_, data_);
  pending_time_ = 0.0;
  last_viewer_send_ = -1.0;
  copy_states();
}

CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // 從目前位置開始，避免啟動瞬間跳動
  for (auto & joint : joints_) {
    joint.command = joint.position;
  }
  return CallbackReturn::SUCCESS;
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
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command);
  }
  return interfaces;
}

void MujocoSystem::copy_states() {
  for (auto & joint : joints_) {
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
  for (const auto & joint : joints_) {
    // 上層尚未寫入（NaN）時保持上一個 ctrl
    if (std::isfinite(joint.command)) {
      data_->ctrl[joint.actuator] = joint.command;
    }
  }

  pending_time_ += period.seconds();
  const double timestep = model_->opt.timestep;
  int steps = 0;
  while (pending_time_ >= timestep && steps < kMaxStepsPerCycle) {
    mj_step(model_, data_);
    pending_time_ -= timestep;
    ++steps;
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
  std::copy(data_->qpos, data_->qpos + model_->nq, viewer_buffer_.begin() + 1);
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
