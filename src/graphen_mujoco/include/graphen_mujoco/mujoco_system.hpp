#ifndef GRAPHEN_MUJOCO__MUJOCO_SYSTEM_HPP_
#define GRAPHEN_MUJOCO__MUJOCO_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mujoco/mujoco.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace graphen_mujoco {

// 以 MuJoCo 模擬 T170 本體的 ros2_control system plugin，取代 mock 或 ti5_hardware。
//
// 模型為 tools/export_mjcf.py 匯出的 MJCF：每個 joint 有一個同名的 PD 位置致動器（增益與訓練相同），
// position command 直接寫入致動器的 ctrl。write() 依 controller 週期推進物理（例如 100 Hz 時每次 2 步），
// read() 讀出 joint 的 position / velocity / acceleration，以及 IMU（framequat、gyro、accelerometer）。
// 其他 joint state interface（例如 ti5 的 current、error_code）照樣匯出，值固定為 NaN。
//
// IMU 以 <sensor> 宣告，介面名稱與 imu16460 實機相同（orientation.x … linear_acceleration.z、temperature），
// controller 設定在 mock、實機與模擬下都不需要更動。
//
// viewer_port > 0 時，read() 以 UDP 把 qpos 送到 127.0.0.1:viewer_port（tools/mujoco_viewer.py 顯示），
// 並在 127.0.0.1:viewer_port+1 接收 "reset" 指令。兩者都是非阻塞的。
class MujocoSystem : public hardware_interface::SystemInterface {
public:
  ~MujocoSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint {
    int qpos_adr{-1};
    int dof_adr{-1};
    int actuator{-1};
    double initial_position{0.0};
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    double command{0.0};
  };

  // IMU state 在 imu_values_ 中的位置
  enum ImuIndex : size_t
  {
    kOrientationX, kOrientationY, kOrientationZ, kOrientationW,
    kAngularVelocityX, kAngularVelocityY, kAngularVelocityZ,
    kLinearAccelerationX, kLinearAccelerationY, kLinearAccelerationZ,
    kTemperature,
    kNumImuStates
  };

  bool load_model(const std::string & path);
  bool open_viewer_socket();
  void reset_simulation();
  void copy_states();
  void send_viewer_frame();
  void poll_viewer_commands();

  rclcpp::Logger logger_{rclcpp::get_logger("MujocoSystem")};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  mjModel * model_{nullptr};
  mjData * data_{nullptr};

  std::vector<Joint> joints_;
  // 模擬沒有的 joint state interface，固定為 NaN（配置一次，export 時取位址）
  std::vector<double> unsupported_states_;
  // 浮動基座的 qpos 位址（固定基座模型為 -1）
  int free_qpos_adr_{-1};
  double initial_base_height_{0.894};

  // IMU（模型中的感測器位址；沒有宣告 <sensor> 時 imu_name_ 為空）
  std::string imu_name_;
  int quat_adr_{-1};
  int gyro_adr_{-1};
  int acc_adr_{-1};
  std::array<double, kNumImuStates> imu_values_{};

  // 尚未推進的模擬時間（秒）
  double pending_time_{0.0};

  // viewer
  int viewer_port_{0};
  int socket_fd_{-1};
  double viewer_period_{1.0 / 60.0};
  double last_viewer_send_{-1.0};
  std::vector<double> viewer_buffer_;
};

}  // namespace graphen_mujoco

#endif  // GRAPHEN_MUJOCO__MUJOCO_SYSTEM_HPP_
