#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "graphen_rl_policy/obs_history.hpp"
#include "graphen_rl_policy/policy_runner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::rl_policy {

// robolab（Isaac Lab DirectRL）ti5_flat 任務訓練的平地速度追蹤 policy 家族。
//
// 每幀 obs（9 + 3N 維，N = joints 數）依序為：
//   base_ang_vel × ang_vel_scale (3)
//   projected_gravity × gravity_scale (3)
//   command × command_scale (3)          vx, vy, wz（沒有步態相位）
//   (joint_pos − default) × joint_pos_scale (N)
//   joint_vel × joint_vel_scale (N)
//   last_action × action_obs_scale (N)
// 每個值再限制在 ±clip_observations。policy 的輸入為最近 history_length 幀（ObsHistory）。
// action 先限制在 ±clip_actions，目標 = default_joint_pos + action_scale × action。
// 訓練時沒有 obs normalization（agent.yaml：actor_obs_normalization: false），送原始值。
//
// TODO(verify-with-training): 下列項目目前依 env.yaml 與 onnx 權重推斷，需由訓練端確認：
//   - joints 的順序（Isaac Lab 的 robot.joint_names）
//   - 每幀 term 的順序、history 的排法（frame / term major）與新舊順序
//   - joint_pos 是否減 default、action 是否加 default
//   - last_action 為 clip 前或 clip 後的值
//
// 參數（<mode>.*，另見 PolicyRunner 的共用參數）：
//   action_scale          action 縮放（純量，必填）
//   history_length        history 幀數（必填）
//   history_layout        frame_major 或 term_major（必填）
//   history_oldest_first  true：最舊的幀在前（必填）
//   ang_vel_scale / gravity_scale / command_scale / joint_pos_scale / joint_vel_scale /
//   action_obs_scale      obs 縮放（預設 1.0）
//   clip_observations     obs 限幅（預設 100.0）
//   clip_actions          action 限幅（預設 100.0）
class Ti5FlatPolicy : public PolicyRunner {
protected:
  std::string name() const override {return "Ti5FlatPolicy";}

  size_t configure_policy(const locomotion::ParameterSource & params) override {
    action_scale_ = params.get_double("action_scale").value_or(0.0);
    if (!(action_scale_ > 0.0)) {
      throw std::invalid_argument("'action_scale' is required and must be greater than zero");
    }
    const auto length = params.get_int("history_length").value_or(0);
    if (length <= 0) {
      throw std::invalid_argument("'history_length' is required and must be greater than zero");
    }
    const auto layout = params.get_string("history_layout").value_or("");
    if (layout != "frame_major" && layout != "term_major") {
      throw std::invalid_argument("'history_layout' must be 'frame_major' or 'term_major'");
    }
    const auto oldest_first = params.get_bool("history_oldest_first");
    if (!oldest_first) {
      throw std::invalid_argument("'history_oldest_first' is required");
    }

    ang_vel_scale_ = params.get_double("ang_vel_scale").value_or(1.0);
    gravity_scale_ = params.get_double("gravity_scale").value_or(1.0);
    command_scale_ = params.get_double("command_scale").value_or(1.0);
    joint_pos_scale_ = params.get_double("joint_pos_scale").value_or(1.0);
    joint_vel_scale_ = params.get_double("joint_vel_scale").value_or(1.0);
    action_obs_scale_ = params.get_double("action_obs_scale").value_or(1.0);
    clip_observations_ = params.get_double("clip_observations").value_or(100.0);
    clip_actions_ = params.get_double("clip_actions").value_or(100.0);
    if (!(clip_observations_ > 0.0) || !(clip_actions_ > 0.0)) {
      throw std::invalid_argument("'clip_observations' and 'clip_actions' must be > 0");
    }

    const size_t n = num_joints();
    history_ = std::make_unique<ObsHistory>(
      std::vector<size_t>{3, 3, 3, n, n, n}, static_cast<size_t>(length),
      layout == "frame_major" ? ObsHistory::Layout::kFrameMajor : ObsHistory::Layout::kTermMajor,
      *oldest_first);
    frame_.assign(history_->frame_dim(), 0.0f);
    last_action_.assign(n, 0.0f);
    return history_->size();
  }

  void reset_policy() override {
    history_->reset();
    std::fill(last_action_.begin(), last_action_.end(), 0.0f);
  }

  void build_obs(const PolicyInput & input, float * obs) override {
    const auto & state = input.state;
    auto * out = frame_.data();
    const auto put = [&](double value, double scale) {
        *out++ = static_cast<float>(
          std::clamp(value * scale, -clip_observations_, clip_observations_));
      };
    for (size_t i = 0; i < 3; ++i) {
      put(state.base.angular_velocity[i], ang_vel_scale_);
    }
    for (size_t i = 0; i < 3; ++i) {
      put(state.base.projected_gravity[i], gravity_scale_);
    }
    for (size_t i = 0; i < 3; ++i) {
      put(state.command.velocity[i], command_scale_);
    }
    for (size_t i = 0; i < num_joints(); ++i) {
      put(input.joint_positions[i] - default_positions()[i], joint_pos_scale_);
    }
    for (size_t i = 0; i < num_joints(); ++i) {
      put(input.joint_velocities[i], joint_vel_scale_);
    }
    for (size_t i = 0; i < num_joints(); ++i) {
      put(last_action_[i], action_obs_scale_);
    }
    history_->push(frame_.data());
    history_->flatten(obs);
  }

  void action_to_targets(const float * action, double * targets) override {
    for (size_t i = 0; i < num_joints(); ++i) {
      const float clipped = std::clamp(
        action[i], static_cast<float>(-clip_actions_), static_cast<float>(clip_actions_));
      last_action_[i] = clipped;
      targets[i] = default_positions()[i] + action_scale_ * clipped;
    }
  }

private:
  double action_scale_{0.0};
  double ang_vel_scale_{1.0};
  double gravity_scale_{1.0};
  double command_scale_{1.0};
  double joint_pos_scale_{1.0};
  double joint_vel_scale_{1.0};
  double action_obs_scale_{1.0};
  double clip_observations_{100.0};
  double clip_actions_{100.0};
  std::unique_ptr<ObsHistory> history_;
  std::vector<float> frame_;
  std::vector<float> last_action_;
};

}  // namespace graphen::rl_policy

PLUGINLIB_EXPORT_CLASS(
  graphen::rl_policy::Ti5FlatPolicy,
  graphen::locomotion::LocomotionAlgorithm)
