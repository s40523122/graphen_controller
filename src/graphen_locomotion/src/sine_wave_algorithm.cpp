#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::locomotion {

// 範例演算法：以進入模式時的位置為中心，讓指定的 joint 做正弦擺動，並輸出解析微分的速度前饋。
// 用來示範 plugin 的寫法，以及驗證量測值與 IMU 的資料通路。
//
// 參數（<mode>.*）：
//   joints      要擺動的 joint（必填）
//   amplitude   振幅（rad，預設 0.1）
//   frequency   頻率（Hz，預設 0.5，必須 > 0）
//   log_period  > 0 時每隔這麼久（秒）log 一次第一個 joint 的狀態、IMU、機身狀態與速度指令（預設 0，不 log）
class SineWaveAlgorithm : public LocomotionAlgorithm {
public:
  void configure(const Context & context) override {
    const auto joints = context.params.get_string_array("joints");
    if (!joints || joints->empty()) {
      throw std::invalid_argument("'joints' must list at least one joint");
    }
    indices_.clear();
    for (const auto & joint : *joints) {
      size_t index = 0;
      while (index < context.joint_names.size() && context.joint_names[index] != joint) {
        ++index;
      }
      if (index == context.joint_names.size()) {
        throw std::invalid_argument("joint '" + joint + "' in 'joints' is not a commanded joint");
      }
      indices_.push_back(index);
    }
    amplitude_ = context.params.get_double("amplitude").value_or(0.1);
    frequency_ = context.params.get_double("frequency").value_or(0.5);
    if (!(frequency_ > 0.0)) {
      throw std::invalid_argument("'frequency' must be greater than zero");
    }
    log_period_ = context.params.get_double("log_period").value_or(0.0);
    log_ = context.log;
    first_joint_name_ = joints->front();
  }

  void on_enter(const RobotState & state) override {
    // 以命令位置為中心：起點的 sin = 0，進入時不會跳動
    center_ = state.commanded_positions;
    phase_time_ = 0.0;
    since_log_ = 0.0;
  }

  void update(double dt, const RobotState & state, JointCommand & command) override {
    phase_time_ += dt;
    const double omega = 2.0 * M_PI * frequency_;
    const double position = amplitude_ * std::sin(omega * phase_time_);
    const double velocity = amplitude_ * omega * std::cos(omega * phase_time_);
    for (const auto index : indices_) {
      command.positions[index] = center_[index] + position;
      command.velocities[index] = velocity;
    }

    if (log_period_ > 0.0 && log_) {
      since_log_ += dt;
      if (since_log_ >= log_period_) {
        since_log_ = 0.0;
        log_state(state);
      }
    }
  }

private:
  void log_state(const RobotState & state) const {
    const auto index = indices_.front();
    const auto & imu = state.imu;
    const auto & base = state.base;
    const auto & command = state.command;
    char buffer[1024];
    std::snprintf(buffer, sizeof(buffer),
      "[%s] measured pos %.4f vel %.4f, commanded %.4f | IMU valid %s "
      "quat (%.3f, %.3f, %.3f, %.3f) gyro (%.3f, %.3f, %.3f) acc (%.3f, %.3f, %.3f) | "
      "base valid %s lin_vel (%.3f, %.3f, %.3f) ang_vel (%.3f, %.3f, %.3f) "
      "gravity (%.3f, %.3f, %.3f) | command active %s (%.3f, %.3f, %.3f)",
      first_joint_name_.c_str(), state.measured_positions[index],
      state.measured_velocities[index], state.commanded_positions[index],
      imu.valid ? "true" : "false",
      imu.orientation[0], imu.orientation[1], imu.orientation[2], imu.orientation[3],
      imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2],
      imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2],
      base.valid ? "true" : "false",
      base.linear_velocity[0], base.linear_velocity[1], base.linear_velocity[2],
      base.angular_velocity[0], base.angular_velocity[1], base.angular_velocity[2],
      base.projected_gravity[0], base.projected_gravity[1], base.projected_gravity[2],
      command.active ? "true" : "false",
      command.velocity[0], command.velocity[1], command.velocity[2]);
    log_(LogLevel::kInfo, buffer);
  }

  std::vector<size_t> indices_;
  double amplitude_{0.1};
  double frequency_{0.5};
  double log_period_{0.0};
  Logger log_;
  std::string first_joint_name_;

  std::vector<double> center_;
  double phase_time_{0.0};
  double since_log_{0.0};
};

}  // namespace graphen::locomotion

PLUGINLIB_EXPORT_CLASS(
  graphen::locomotion::SineWaveAlgorithm,
  graphen::locomotion::LocomotionAlgorithm)
