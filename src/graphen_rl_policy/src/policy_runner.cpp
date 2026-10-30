#include "graphen_rl_policy/policy_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace graphen::rl_policy {

using locomotion::Context;
using locomotion::JointCommand;
using locomotion::LogLevel;
using locomotion::ParameterSource;
using locomotion::RobotState;

namespace {

// 依 name_map（"policy:controller"）把 policy 的關節名稱換成 controller 的名稱，再找出 index
size_t controller_index(
  const std::string & policy_name, const std::vector<std::string> & name_map,
  const std::vector<std::string> & controller_names)
{
  std::string name = policy_name;
  for (const auto & entry : name_map) {
    const auto colon = entry.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument("'name_map' entry '" + entry + "' is not 'policy:controller'");
    }
    if (entry.compare(0, colon, policy_name) == 0 && colon == policy_name.size()) {
      name = entry.substr(colon + 1);
      break;
    }
  }
  const auto it = std::find(controller_names.begin(), controller_names.end(), name);
  if (it == controller_names.end()) {
    throw std::invalid_argument(
      "joint '" + policy_name + "' (controller name '" + name + "') is not a commanded joint");
  }
  return static_cast<size_t>(it - controller_names.begin());
}

std::vector<double> required_array(
  const ParameterSource & params, const std::string & name, size_t size)
{
  const auto values = params.get_double_array(name);
  if (!values) {
    throw std::invalid_argument("'" + name + "' is required");
  }
  if (values->size() != size) {
    throw std::invalid_argument(
      "'" + name + "' must contain " + std::to_string(size) + " entries, got " +
      std::to_string(values->size()));
  }
  return *values;
}

// 選填的增益：沒有設定時回傳空的；設定時長度必須相符且非負
std::vector<double> optional_gains(
  const ParameterSource & params, const std::string & name, size_t size)
{
  if (!params.get_double_array(name)) {
    return {};
  }
  auto gains = required_array(params, name, size);
  if (std::any_of(gains.begin(), gains.end(),
    [](double g) {return !std::isfinite(g) || g < 0.0;}))
  {
    throw std::invalid_argument("'" + name + "' must contain finite values >= 0");
  }
  return gains;
}

}  // namespace

void PolicyRunner::configure(const Context & context) {
  log_ = context.log;
  const auto & params = context.params;

  const auto model_path = params.get_string("model_path");
  if (!model_path || model_path->empty()) {
    throw std::invalid_argument("'model_path' is required");
  }
  const auto model_sha256 = params.get_string("model_sha256").value_or("");

  joint_names_ = params.get_string_array("joints").value_or(std::vector<std::string>{});
  if (joint_names_.empty()) {
    throw std::invalid_argument("'joints' is required");
  }
  const auto name_map = params.get_string_array("name_map").value_or(std::vector<std::string>{});
  joint_indices_.clear();
  for (const auto & joint : joint_names_) {
    joint_indices_.push_back(controller_index(joint, name_map, context.joint_names));
  }
  default_positions_ = required_array(params, "default_joint_pos", num_joints());
  kp_ = optional_gains(params, "kp", num_joints());
  kd_ = optional_gains(params, "kd", num_joints());

  const auto fixed_joints =
    params.get_string_array("fixed_joints").value_or(std::vector<std::string>{});
  fixed_positions_ = fixed_joints.empty() ? std::vector<double>{} :
    required_array(params, "fixed_positions", fixed_joints.size());
  fixed_indices_.clear();
  for (const auto & joint : fixed_joints) {
    // 已經是 controller 的名稱，不經 name_map
    fixed_indices_.push_back(controller_index(joint, {}, context.joint_names));
  }
  // 同一個關節不能同時由 policy 控制又被固定
  std::vector<size_t> all_indices = joint_indices_;
  all_indices.insert(all_indices.end(), fixed_indices_.begin(), fixed_indices_.end());
  std::sort(all_indices.begin(), all_indices.end());
  if (std::adjacent_find(all_indices.begin(), all_indices.end()) != all_indices.end()) {
    throw std::invalid_argument("a joint is listed more than once in 'joints' / 'fixed_joints'");
  }

  policy_dt_ = params.get_double("policy_dt").value_or(0.0);
  if (!(policy_dt_ > 0.0)) {
    throw std::invalid_argument("'policy_dt' is required and must be greater than zero");
  }
  if (context.rate_hz > 0.0 && policy_dt_ < 1.0 / context.rate_hz - 1e-9) {
    throw std::invalid_argument("'policy_dt' is shorter than the planner period");
  }
  entry_speed_ = params.get_double("entry_speed").value_or(0.5);
  if (!(entry_speed_ >= 0.0)) {
    throw std::invalid_argument("'entry_speed' must be >= 0");
  }
  log_period_ = params.get_double("log_period").value_or(0.0);

  const size_t obs_dim = configure_policy(params);
  model_ = std::make_unique<OnnxModel>(
    resolve_package_path(*model_path), model_sha256, obs_dim, num_joints());

  // 週期中使用的緩衝在這裡配置
  obs_.assign(obs_dim, 0.0f);
  action_.assign(num_joints(), 0.0f);
  targets_.assign(num_joints(), 0.0);
  joint_positions_.assign(num_joints(), 0.0);
  joint_velocities_.assign(num_joints(), 0.0);
  entry_start_.assign(num_joints() + fixed_indices_.size(), 0.0);
  entry_goal_ = default_positions_;
  entry_goal_.insert(entry_goal_.end(), fixed_positions_.begin(), fixed_positions_.end());

  log(LogLevel::kInfo, "loaded '" + *model_path + "' (" + std::to_string(obs_dim) + " obs -> " +
    std::to_string(num_joints()) + " joints, " + std::to_string(fixed_indices_.size()) +
    " fixed), policy_dt " + std::to_string(policy_dt_) + " s, gains " +
    (kp_.empty() && kd_.empty() ? "from downstream defaults" : "from the policy") + ".");
}

void PolicyRunner::on_enter(const RobotState & state) {
  // 從 planner 目前的命令位置出發（一定是有限值），不受量測雜訊影響
  double max_distance = 0.0;
  for (size_t i = 0; i < entry_start_.size(); ++i) {
    const size_t index = i < num_joints() ? joint_indices_[i] : fixed_indices_[i - num_joints()];
    entry_start_[i] = state.commanded_positions[index];
    max_distance = std::max(max_distance, std::abs(entry_goal_[i] - entry_start_[i]));
  }
  since_log_ = 0.0;
  if (entry_speed_ > 0.0) {
    phase_ = Phase::kEntering;
    entry_duration_ = max_distance / entry_speed_;
    entry_elapsed_ = 0.0;
    log(LogLevel::kInfo, "moving to the default pose (" + std::to_string(entry_duration_) +
      " s) before running the policy.");
  } else {
    start_running();
  }
}

void PolicyRunner::update(double dt, const RobotState & state, JointCommand & command) {
  write_gains(command);

  if (phase_ == Phase::kEntering) {
    entry_elapsed_ += dt;
    const double s = entry_duration_ > 0.0 ? std::min(1.0, entry_elapsed_ / entry_duration_) : 1.0;
    for (size_t i = 0; i < entry_start_.size(); ++i) {
      const size_t index = i < num_joints() ? joint_indices_[i] : fixed_indices_[i - num_joints()];
      command.positions[index] = entry_start_[i] + s * (entry_goal_[i] - entry_start_[i]);
    }
    if (s >= 1.0) {
      start_running();
      log(LogLevel::kInfo, "default pose reached, policy running.");
    }
    return;
  }

  for (size_t i = 0; i < fixed_indices_.size(); ++i) {
    command.positions[fixed_indices_[i]] = fixed_positions_[i];
  }

  since_inference_ += dt;
  if (since_inference_ >= policy_dt_ - 1e-6) {
    if (inputs_valid(state)) {
      infer(state);
      holding_ = false;
    } else if (!holding_) {
      holding_ = true;
      has_target_ = false;
      log(LogLevel::kWarn, "base state or joint measurements invalid, holding positions");
    }
    // 落後超過一個週期時重新對齊，不連續補跑
    since_inference_ = std::min(since_inference_ - policy_dt_, policy_dt_);
  }
  if (has_target_) {
    for (size_t i = 0; i < num_joints(); ++i) {
      command.positions[joint_indices_[i]] = targets_[i];
    }
  }
  log_status(dt, state);
}

void PolicyRunner::log(LogLevel level, const std::string & message) const {
  if (log_) {
    log_(level, name() + ": " + message);
  }
}

void PolicyRunner::start_running() {
  phase_ = Phase::kRunning;
  reset_policy();
  policy_time_ = 0.0;
  // 第一個週期就推論
  since_inference_ = policy_dt_;
  has_target_ = false;
  holding_ = false;
}

bool PolicyRunner::inputs_valid(const RobotState & state) const {
  if (!state.base.valid) {
    return false;
  }
  return std::all_of(joint_indices_.begin(), joint_indices_.end(), [&](size_t index) {
             return std::isfinite(state.measured_positions[index]) &&
             std::isfinite(state.measured_velocities[index]);
           });
}

void PolicyRunner::infer(const RobotState & state) {
  for (size_t i = 0; i < num_joints(); ++i) {
    joint_positions_[i] = state.measured_positions[joint_indices_[i]];
    joint_velocities_[i] = state.measured_velocities[joint_indices_[i]];
  }
  build_obs(PolicyInput{state, joint_positions_, joint_velocities_, policy_time_}, obs_.data());
  model_->run(obs_.data(), action_.data());
  action_to_targets(action_.data(), targets_.data());
  has_target_ = true;
  policy_time_ += policy_dt_;
}

void PolicyRunner::write_gains(JointCommand & command) const {
  for (size_t i = 0; i < kp_.size(); ++i) {
    command.kp[joint_indices_[i]] = kp_[i];
  }
  for (size_t i = 0; i < kd_.size(); ++i) {
    command.kd[joint_indices_[i]] = kd_[i];
  }
}

void PolicyRunner::log_status(double dt, const RobotState & state) {
  if (!(log_period_ > 0.0)) {
    return;
  }
  since_log_ += dt;
  if (since_log_ < log_period_) {
    return;
  }
  since_log_ = 0.0;
  const auto & c = state.command.velocity;
  const auto & g = state.base.projected_gravity;
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer),
    "cmd (%.2f, %.2f, %.2f) active %s | gravity (%.3f, %.3f, %.3f) | inference %.3f ms",
    c[0], c[1], c[2], state.command.active ? "true" : "false", g[0], g[1], g[2],
    model_->last_run_ms());
  log(LogLevel::kInfo, buffer);
}

}  // namespace graphen::rl_policy
