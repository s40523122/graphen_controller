#ifndef IMU16460_HARDWARE__IMU16460_SENSOR_INTERFACE_HPP_
#define IMU16460_HARDWARE__IMU16460_SENSOR_INTERFACE_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "imu16460_core/driver.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace imu16460_hardware {

// FSS-IMU16460-C 的 ros2_control sensor plugin。
//
// IMU 由 imu16460::Driver 的背景 thread 依裝置自己的頻率（約 1000 Hz）接收，
// read() 每個週期只取最新一筆、換算後寫入 state，不做 serial I/O、不阻塞、不配置記憶體。
// 匯出 imu_sensor_broadcaster 需要的 10 個 state interface，以及選用的 temperature。
//
// 不會修改 IMU 的任何設定（鮑率、頻率、坐標系等請用 imu16460_cli）。
class Imu16460SensorInterface : public hardware_interface::SensorInterface {
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // state 在 values_ 中的位置
  enum Index : size_t
  {
    kOrientationX, kOrientationY, kOrientationZ, kOrientationW,
    kAngularVelocityX, kAngularVelocityY, kAngularVelocityZ,
    kLinearAccelerationX, kLinearAccelerationY, kLinearAccelerationZ,
    kTemperature,
    kNumStates
  };

  enum class AxisTransform {kNone, kFrdToFlu};
  enum class EulerOrder {kZyx, kXyz};

  bool read_parameters();
  void write_sample(const imu16460::ImuSample & sample);

  rclcpp::Logger logger_{rclcpp::get_logger("Imu16460SensorInterface")};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // URDF <hardware> 參數
  std::string port_;
  int baudrate_{0};
  double data_timeout_ms_{0.0};
  bool error_on_timeout_{false};
  int activate_timeout_ms_{0};
  AxisTransform axis_transform_{AxisTransform::kNone};
  EulerOrder euler_order_{EulerOrder::kZyx};

  std::string sensor_name_;
  // 每個 state 是否列在 URDF 中（temperature 為選用）
  std::array<bool, kNumStates> exported_{};
  std::array<double, kNumStates> values_{};

  imu16460::Driver driver_;
  imu16460::ImuSample sample_;  // read() 用的暫存，避免在 RT 路徑配置
};

}  // namespace imu16460_hardware

#endif  // IMU16460_HARDWARE__IMU16460_SENSOR_INTERFACE_HPP_
