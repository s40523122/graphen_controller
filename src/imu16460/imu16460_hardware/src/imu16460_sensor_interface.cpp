#include "imu16460_hardware/imu16460_sensor_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>

#include "imu16460_core/protocol.hpp"
#include "rclcpp/logging.hpp"

namespace imu16460_hardware {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {

constexpr double kStandardGravity = 9.80665;  // m/s² per g
constexpr double kDegToRad = M_PI / 180.0;
constexpr int kWarnThrottleMs = 1000;
constexpr auto kActivatePollPeriod = std::chrono::milliseconds(10);

// URDF 中 state interface 的名稱，順序對應 Index
constexpr std::array<const char *, 11> kStateNames{
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
  "temperature"};
constexpr size_t kNumRequired = 10;  // temperature 以外都是 imu_sensor_broadcaster 必需的

// 參數預設值（URDF 沒有寫時使用）
constexpr const char * kDefaultPort = "/dev/ttyTHS1";
constexpr int kDefaultBaudrate = 921600;
constexpr double kDefaultDataTimeoutMs = 100.0;
constexpr int kDefaultActivateTimeoutMs = 1000;

std::string get_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

bool parse_bool(const std::string & s, bool & out)
{
  if (s == "true" || s == "True" || s == "1") {
    out = true;
    return true;
  }
  if (s == "false" || s == "False" || s == "0") {
    out = false;
    return true;
  }
  return false;
}

}  // namespace

CallbackReturn Imu16460SensorInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(logger_, "Expected exactly 1 sensor, got %zu", info_.sensors.size());
    return CallbackReturn::ERROR;
  }
  const auto & sensor = info_.sensors.front();
  sensor_name_ = sensor.name;

  exported_.fill(false);
  for (const auto & iface : sensor.state_interfaces) {
    bool known = false;
    for (size_t i = 0; i < kStateNames.size(); ++i) {
      if (iface.name == kStateNames[i]) {
        if (exported_[i]) {
          RCLCPP_ERROR(logger_, "Duplicate state interface '%s'", iface.name.c_str());
          return CallbackReturn::ERROR;
        }
        exported_[i] = true;
        known = true;
      }
    }
    if (!known) {
      RCLCPP_ERROR(logger_, "Unknown state interface '%s'", iface.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  for (size_t i = 0; i < kNumRequired; ++i) {
    if (!exported_[i]) {
      RCLCPP_ERROR(
        logger_, "Sensor '%s' is missing required state interface '%s'",
        sensor_name_.c_str(), kStateNames[i]);
      return CallbackReturn::ERROR;
    }
  }

  if (!read_parameters()) {
    return CallbackReturn::ERROR;
  }

  values_.fill(0.0);
  values_[kOrientationW] = 1.0;
  values_[kTemperature] = std::numeric_limits<double>::quiet_NaN();
  return CallbackReturn::SUCCESS;
}

bool Imu16460SensorInterface::read_parameters()
{
  const auto & params = info_.hardware_parameters;

  port_ = get_param(params, "port", kDefaultPort);
  if (port_.empty()) {
    RCLCPP_ERROR(logger_, "Parameter 'port' must not be empty");
    return false;
  }

  try {
    baudrate_ = std::stoi(get_param(params, "baudrate", std::to_string(kDefaultBaudrate)));
    data_timeout_ms_ = std::stod(
      get_param(params, "data_timeout_ms", std::to_string(kDefaultDataTimeoutMs)));
    activate_timeout_ms_ = std::stoi(
      get_param(params, "activate_timeout_ms", std::to_string(kDefaultActivateTimeoutMs)));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Invalid numeric hardware parameter: %s", e.what());
    return false;
  }
  if (!imu16460::is_supported_baudrate(baudrate_)) {
    RCLCPP_ERROR(logger_, "Unsupported baudrate %d", baudrate_);
    return false;
  }
  if (data_timeout_ms_ <= 0.0) {
    RCLCPP_ERROR(logger_, "Parameter 'data_timeout_ms' must be > 0");
    return false;
  }
  if (activate_timeout_ms_ <= 0) {
    RCLCPP_ERROR(logger_, "Parameter 'activate_timeout_ms' must be > 0");
    return false;
  }

  if (!parse_bool(get_param(params, "error_on_timeout", "false"), error_on_timeout_)) {
    RCLCPP_ERROR(logger_, "Parameter 'error_on_timeout' must be true or false");
    return false;
  }

  // TODO(verify-on-hw): 手冊範例靜置時 az ≈ -1g，裝置 z 軸可能朝下（FRD）。
  // 以六面重力測試確認後再決定預設值。
  const auto axis = get_param(params, "axis_transform", "none");
  if (axis == "none") {
    axis_transform_ = AxisTransform::kNone;
  } else if (axis == "frd_to_flu") {
    axis_transform_ = AxisTransform::kFrdToFlu;
  } else {
    RCLCPP_ERROR(logger_, "Parameter 'axis_transform' must be 'none' or 'frd_to_flu'");
    return false;
  }

  // TODO(verify-on-hw): 手冊沒有說明 pitch/roll/yaw 的旋轉順序與正方向，需實際轉動模組驗證。
  const auto order = get_param(params, "euler_order", "zyx");
  if (order == "zyx") {
    euler_order_ = EulerOrder::kZyx;
  } else if (order == "xyz") {
    euler_order_ = EulerOrder::kXyz;
  } else {
    RCLCPP_ERROR(logger_, "Parameter 'euler_order' must be 'zyx' or 'xyz'");
    return false;
  }

  RCLCPP_INFO(
    logger_, "Configured sensor '%s': port=%s baudrate=%d axis_transform=%s euler_order=%s",
    sensor_name_.c_str(), port_.c_str(), baudrate_, axis.c_str(), order.c_str());
  return true;
}

CallbackReturn Imu16460SensorInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!driver_.open(port_, baudrate_)) {
    RCLCPP_ERROR(
      logger_, "Failed to open %s @ %d: %s", port_.c_str(), baudrate_,
      driver_.last_error().c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Imu16460SensorInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!driver_.start()) {
    RCLCPP_ERROR(logger_, "Failed to start driver: %s", driver_.last_error().c_str());
    return CallbackReturn::ERROR;
  }

  // 確認數據流確實有在送；只等待，不送任何命令（不改變裝置設定）
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(activate_timeout_ms_);
  while (!driver_.get_latest(sample_)) {
    if (std::chrono::steady_clock::now() >= deadline) {
      const auto stats = driver_.get_stats();
      RCLCPP_ERROR(
        logger_,
        "No IMU data within %d ms on %s @ %d (frames %lu, crc errors %lu, dropped bytes %lu). "
        "Check wiring, baudrate (imu16460_cli scan-baud) and stream mode.",
        activate_timeout_ms_, port_.c_str(), baudrate_,
        static_cast<unsigned long>(stats.frames_ok), static_cast<unsigned long>(stats.crc_errors),
        static_cast<unsigned long>(stats.bytes_dropped));
      driver_.stop();
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(kActivatePollPeriod);
  }
  write_sample(sample_);
  RCLCPP_INFO(logger_, "IMU data stream is alive");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Imu16460SensorInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  driver_.stop();
  return CallbackReturn::SUCCESS;
}

CallbackReturn Imu16460SensorInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  driver_.close();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Imu16460SensorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < kNumStates; ++i) {
    if (exported_[i]) {
      interfaces.emplace_back(sensor_name_, kStateNames[i], &values_[i]);
    }
  }
  return interfaces;
}

return_type Imu16460SensorInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO(verify-on-hw): Humble 的 imu_sensor_broadcaster 以 controller_manager 的更新時間打時間戳，
  // 無法傳遞裝置 timer（device_timer_us）。需要精確時間戳時，改用獨立 node 直接發布。

  // try_lock 拿不到（讀取 thread 正在寫入）時沿用上一筆，RT 端不等待
  if (driver_.get_latest(sample_)) {
    write_sample(sample_);
  }

  const auto age_ms = driver_.ms_since_last_sample();
  if (!age_ms || *age_ms > data_timeout_ms_) {
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, kWarnThrottleMs, "No new IMU data for %.0f ms (timeout %.0f ms)",
      age_ms.value_or(std::numeric_limits<double>::infinity()), data_timeout_ms_);
    if (error_on_timeout_) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

void Imu16460SensorInterface::write_sample(const imu16460::ImuSample & s)
{
  double gx = s.gyro_dps[0] * kDegToRad;
  double gy = s.gyro_dps[1] * kDegToRad;
  double gz = s.gyro_dps[2] * kDegToRad;
  double ax = s.acc_g[0] * kStandardGravity;
  double ay = s.acc_g[1] * kStandardGravity;
  double az = s.acc_g[2] * kStandardGravity;
  // 假設 roll 繞 x、pitch 繞 y、yaw 繞 z
  const double roll = s.roll_deg * kDegToRad;
  double pitch = s.pitch_deg * kDegToRad;
  double yaw = s.yaw_deg * kDegToRad;

  if (axis_transform_ == AxisTransform::kFrdToFlu) {
    // 繞 x 轉 180°：y、z 取負。姿態角同樣以 y、z 軸取負的方向表示
    // TODO(verify-on-hw): 姿態的參考坐標系（NED / ENU）尚未確認，目前姿態共變異數設為 -1（不提供）
    gy = -gy;
    gz = -gz;
    ay = -ay;
    az = -az;
    pitch = -pitch;
    yaw = -yaw;
  }

  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  double qw, qx, qy, qz;
  if (euler_order_ == EulerOrder::kZyx) {
    // q = Rz(yaw) * Ry(pitch) * Rx(roll)
    qw = cy * cp * cr + sy * sp * sr;
    qx = cy * cp * sr - sy * sp * cr;
    qy = cy * sp * cr + sy * cp * sr;
    qz = sy * cp * cr - cy * sp * sr;
  } else {
    // q = Rx(roll) * Ry(pitch) * Rz(yaw)
    qw = cr * cp * cy - sr * sp * sy;
    qx = sr * cp * cy + cr * sp * sy;
    qy = cr * sp * cy - sr * cp * sy;
    qz = cr * cp * sy + sr * sp * cy;
  }

  values_[kOrientationX] = qx;
  values_[kOrientationY] = qy;
  values_[kOrientationZ] = qz;
  values_[kOrientationW] = qw;
  values_[kAngularVelocityX] = gx;
  values_[kAngularVelocityY] = gy;
  values_[kAngularVelocityZ] = gz;
  values_[kLinearAccelerationX] = ax;
  values_[kLinearAccelerationY] = ay;
  values_[kLinearAccelerationZ] = az;
  values_[kTemperature] = s.temp_c;
}

}  // namespace imu16460_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imu16460_hardware::Imu16460SensorInterface, hardware_interface::SensorInterface)
