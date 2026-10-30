#ifndef GRAPHEN_LOCOMOTION__ROBOT_STATE_HPP_
#define GRAPHEN_LOCOMOTION__ROBOT_STATE_HPP_

#include <array>
#include <limits>
#include <vector>

namespace graphen::locomotion {

// IMU 的最新讀值，欄位與 ros2_control 的 <sensor>/orientation.x … linear_acceleration.z 對應
struct ImuState {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  std::array<double, 4> orientation{kNaN, kNaN, kNaN, kNaN};  // 四元數 x, y, z, w
  std::array<double, 3> angular_velocity{kNaN, kNaN, kNaN};   // rad/s
  std::array<double, 3> linear_acceleration{kNaN, kNaN, kNaN};  // m/s²
  // 沒有設定 IMU，或任何一個值不是有限值時為 false
  bool valid{false};
};

// 機身狀態，由 BaseStateEstimator 估測，全部以機身座標系（估測器的 base_frame）表示
struct BaseState {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  std::array<double, 3> linear_velocity{kNaN, kNaN, kNaN};   // m/s
  std::array<double, 3> angular_velocity{kNaN, kNaN, kNaN};  // rad/s
  // 重力方向的單位向量 R_wbᵀ·[0, 0, -1]；水平靜止時為 (0, 0, -1)
  std::array<double, 3> projected_gravity{kNaN, kNaN, kNaN};
  // 沒有設定估測器、IMU 無效或估測器故障時為 false
  bool valid{false};
};

// 執行中的機身速度指令（controller 的 ~/cmd_vel），機身座標系
struct BaseCommand {
  // vx、vy（m/s）、wz（rad/s）
  std::array<double, 3> velocity{0.0, 0.0, 0.0};
  // 尚未收到指令或已逾時時為 false，此時 velocity 為 0
  bool active{false};
};

// 每個規劃週期交給演算法的機器人狀態。
// 所有 vector 的長度都等於 joint 數，順序與 Context::joint_names 相同。
struct RobotState {
  // 量測的時間（steady clock，秒），與演算法收到的時間比較可以得知資料的延遲
  double time{0.0};
  // encoder 量測的關節角度（rad）與速度（rad/s）；沒有設定對應 state interface 的為 NaN
  std::vector<double> measured_positions;
  std::vector<double> measured_velocities;
  // planner 上一次送出的命令位置（已經過 max_steps 限制），與量測值的差即為追蹤誤差
  std::vector<double> commanded_positions;
  // 原始 IMU 讀值（IMU 座標系）
  ImuState imu;
  // 估測器輸出的機身狀態（機身座標系）
  BaseState base;
  BaseCommand command;
};

// 演算法每個週期的輸出，長度等於 joint 數。
// planner 在每次 update() 之前會把全部欄位填成 NaN，沒有寫入的 joint 即為保持
struct JointCommand {
  // 期望關節角度（rad）；NaN 表示保持目前位置
  std::vector<double> positions;
  // 速度前饋（rad/s）；NaN 表示沒有前饋
  std::vector<double> velocities;
  // PD 增益（kp：Nm/rad，kd：Nm·s/rad）；NaN 表示使用下游 controller 的預設值。
  // 只有下游為 PD controller 時才生效，位置模式的下游會忽略
  std::vector<double> kp;
  std::vector<double> kd;
};

}  // namespace graphen::locomotion

#endif  // GRAPHEN_LOCOMOTION__ROBOT_STATE_HPP_
