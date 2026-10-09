#pragma once

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dexterous_hand_hardware/rs485_comm.hpp"

namespace dexterous_hand_hardware
{

class DexterousHandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DexterousHandHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
 // ROS Control Interfaces 
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  std::vector<double> hw_commands_;
  std::vector<int16_t> last_sent_raw_;
  // Else
  std::vector<double> hw_currents_;       // Current(1164)
  std::vector<double> hw_errors_;         // ErrorCode(1178)
  std::vector<double> hw_states_;         // StateCode(1192)
  std::vector<double> hw_temperatures_;   // Temp.(1206)
  
  // Tactile
  std::vector<double> tactile_normal_;
  std::vector<double> tactile_tangent_;
  std::vector<double> tactile_direction_;

  RS485Comm comm_;
  uint8_t hand_id_ = 1;

  struct JointConversion {
    int16_t raw_min;
    int16_t raw_max;
    double  rad_min;
    double  rad_max;
  };
  std::vector<JointConversion> joint_conversions_;

  // 角度轉換：ROS rad ↔ 靈巧手原始值
  int16_t rad_to_raw(double rad, size_t joint_idx);
  double  raw_to_rad(int16_t raw, size_t joint_idx);
};

}  // namespace dexterous_hand_hardware
