#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "graphen_rl_policy/policy_runner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::rl_policy {

// FullBodyT70（mjlab）訓練的 T170 全身速度追蹤 policy 家族。
//
// obs（11 + 3N 維，N = joints 數）依序為：
//   base_ang_vel (3)       ← base.angular_velocity
//   projected_gravity (3)  ← base.projected_gravity
//   command (3)            ← command.velocity（vx, vy, wz）
//   phase (2)              sin / cos(2π·t / phase_period)；‖command‖ < phase_stand_threshold 時為 0
//   joint_pos − default (N)
//   joint_vel (N)
//   last_action (N)        上一次的 policy 原始輸出
// action → 目標：default_joint_pos + action_scale ⊙ action
// obs normalization 已包在 onnx 內（mjlab 匯出），這裡送原始值。
//
// 參數（<mode>.*，另見 PolicyRunner 的共用參數）：
//   action_scale           與 joints 對應的 action 縮放（必填）
//   phase_period           步態相位週期（秒，必填）
//   phase_stand_threshold  指令大小低於此值時相位輸入為 0（預設 0.1）
class T170VelocityPolicy : public PolicyRunner {
protected:
  std::string name() const override {return "T170VelocityPolicy";}

  size_t configure_policy(const locomotion::ParameterSource & params) override {
    const auto scale = params.get_double_array("action_scale");
    if (!scale || scale->size() != num_joints()) {
      throw std::invalid_argument(
        "'action_scale' is required and must contain " + std::to_string(num_joints()) +
        " entries");
    }
    action_scale_ = *scale;
    phase_period_ = params.get_double("phase_period").value_or(0.0);
    if (!(phase_period_ > 0.0)) {
      throw std::invalid_argument("'phase_period' is required and must be greater than zero");
    }
    stand_threshold_ = params.get_double("phase_stand_threshold").value_or(0.1);
    last_action_.assign(num_joints(), 0.0f);
    return kFixedObsDim + 3 * num_joints();
  }

  void reset_policy() override {
    std::fill(last_action_.begin(), last_action_.end(), 0.0f);
  }

  void build_obs(const PolicyInput & input, float * obs) override {
    const auto & state = input.state;
    const auto & cmd = state.command.velocity;
    for (size_t i = 0; i < 3; ++i) {
      *obs++ = static_cast<float>(state.base.angular_velocity[i]);
    }
    for (size_t i = 0; i < 3; ++i) {
      *obs++ = static_cast<float>(state.base.projected_gravity[i]);
    }
    for (size_t i = 0; i < 3; ++i) {
      *obs++ = static_cast<float>(cmd[i]);
    }
    // 指令小時相位輸入為 0（站立），時鐘照常前進
    const double cmd_norm = std::sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2]);
    const double phi = std::fmod(input.policy_time, phase_period_) / phase_period_;
    const bool walking = cmd_norm >= stand_threshold_;
    *obs++ = walking ? static_cast<float>(std::sin(2.0 * M_PI * phi)) : 0.0f;
    *obs++ = walking ? static_cast<float>(std::cos(2.0 * M_PI * phi)) : 0.0f;
    for (size_t i = 0; i < num_joints(); ++i) {
      *obs++ = static_cast<float>(input.joint_positions[i] - default_positions()[i]);
    }
    for (size_t i = 0; i < num_joints(); ++i) {
      *obs++ = static_cast<float>(input.joint_velocities[i]);
    }
    obs = std::copy(last_action_.begin(), last_action_.end(), obs);
  }

  void action_to_targets(const float * action, double * targets) override {
    for (size_t i = 0; i < num_joints(); ++i) {
      last_action_[i] = action[i];
      targets[i] = default_positions()[i] + action_scale_[i] * action[i];
    }
  }

private:
  // gyro、重力、指令各 3，相位 2
  static constexpr size_t kFixedObsDim = 11;

  std::vector<double> action_scale_;
  double phase_period_{0.0};
  double stand_threshold_{0.1};
  std::vector<float> last_action_;
};

}  // namespace graphen::rl_policy

PLUGINLIB_EXPORT_CLASS(
  graphen::rl_policy::T170VelocityPolicy,
  graphen::locomotion::LocomotionAlgorithm)
