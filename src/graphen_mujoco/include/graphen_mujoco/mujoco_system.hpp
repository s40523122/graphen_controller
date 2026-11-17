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
// 每個 joint 依名稱對應模型中同名的 joint 與致動器，command interface 為 position 或 effort（由 URDF 決定）：
//   position：致動器必須是位置伺服（fixed gain、affine bias，例如 tools/export_mjcf.py 匯出的模型），
//             command 寫入 ctrl
//   effort  ：致動器為力矩馬達（motor）；位置伺服會在載入時改成馬達（gain 1、無 bias）。command 即為力矩。
//             所有 effort joint 都收到第一個有效 command 之前（以及 viewer reset 之後）物理暫停，
//             controller 啟動前機器人停在初始姿態
// write() 依 controller 週期推進物理（例如 100 Hz 時每次 2 步），
// read() 讀出 joint 的 position / velocity / acceleration，以及 IMU（framequat、gyro、accelerometer）。
// 其他 joint state interface（例如 ti5 的 current、error_code）照樣匯出，值固定為 NaN。
//
// IMU 以 <sensor> 宣告，介面名稱與 imu16460 實機相同（orientation.x … linear_acceleration.z、temperature），
// controller 設定在 mock、實機與模擬下都不需要更動。模型中的感測器依類型尋找（framequat、gyro、
// accelerometer 各一個；有多個時取名稱為 imu_quat / gyro / imu_acc 的）。
//
// 硬體參數：
//   model_path           MJCF 路徑（必填）
//   initial_base_height  浮動基座的初始高度（m，預設 0.894）
//   fixed_base           true 時每步把 free joint 重設回初始位姿與零速度，等效吊掛（預設 false）
//   missing_joints       模型沒有的 joint：error（預設，載入失敗）或 virtual（照樣匯出介面，狀態固定在
//                        初始值、忽略指令；用於只有部分關節的模型，例如沒有頸部的模型）
//   viewer_port / viewer_rate  見下
//
// viewer_port > 0 時，read() 以 UDP 把 [time, qpos, actuator_force] 送到 127.0.0.1:viewer_port
// （tools/mujoco_viewer.py 顯示姿態與各軸力矩），
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
    bool effort_mode{false};
    // 模型中沒有這個 joint（missing_joints: virtual）：狀態固定在初始值，忽略指令
    bool virtual_joint{false};
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
  // 所有 effort joint 都有有效的 command 時回傳 true
  bool effort_commands_ready() const;
  // fixed_base：把 free joint 放回初始位姿、速度歸零
  void pin_base();
  void send_viewer_frame();
  void poll_viewer_commands();

  rclcpp::Logger logger_{rclcpp::get_logger("MujocoSystem")};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  mjModel * model_{nullptr};
  mjData * data_{nullptr};

  std::vector<Joint> joints_;
  // 模擬沒有的 joint state interface，固定為 NaN（配置一次，export 時取位址）
  std::vector<double> unsupported_states_;
  // 浮動基座的 qpos / qvel 位址（固定基座模型為 -1）
  int free_qpos_adr_{-1};
  int free_dof_adr_{-1};
  double initial_base_height_{0.894};
  bool fixed_base_{false};
  bool allow_virtual_joints_{false};
  // 有 effort joint 時，全部收到第一個有效 command 之前物理暫停
  bool has_effort_joints_{false};
  bool commands_ready_{true};

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
