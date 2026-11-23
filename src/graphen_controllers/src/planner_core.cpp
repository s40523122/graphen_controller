#include "graphen_controllers/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace graphen::control {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

PlannerCore::PlannerCore(size_t num_interfaces)
: num_interfaces_(num_interfaces) {
  if (num_interfaces_ == 0) {
    throw std::invalid_argument("PlannerCore: num_interfaces must be greater than zero");
  }
  reset(std::vector<double>(num_interfaces_, 0.0));
}

void PlannerCore::reset(
  const std::vector<double> & initial_positions, const std::vector<double> & initial_velocities)
{
  if (initial_positions.size() != num_interfaces_) {
    throw std::invalid_argument(
      "PlannerCore: initial_positions size does not match the number of interfaces");
  }
  if (!initial_velocities.empty() && initial_velocities.size() != num_interfaces_) {
    throw std::invalid_argument(
      "PlannerCore: initial_velocities size does not match the number of interfaces");
  }
  output_.positions = initial_positions;
  output_.velocities.assign(num_interfaces_, kNaN);
  output_.kp.assign(num_interfaces_, kNaN);
  output_.kd.assign(num_interfaces_, kNaN);
  output_.sample_period = 0.0;
  previous_positions_ = initial_positions;
  if (initial_velocities.empty()) {
    command_velocity_.assign(num_interfaces_, 0.0);
  } else {
    command_velocity_ = initial_velocities;
  }
  command_.positions.assign(num_interfaces_, kNaN);
  command_.velocities.assign(num_interfaces_, kNaN);
  command_.kp.assign(num_interfaces_, kNaN);
  command_.kd.assign(num_interfaces_, kNaN);
  motion_.reset();
  elapsed_ = 0.0;
}

void PlannerCore::fill_commanded(locomotion::RobotState & state) const {
  state.commanded_positions = output_.positions;
  state.commanded_velocities.resize(num_interfaces_);
  std::copy(command_velocity_.begin(), command_velocity_.end(),
    state.commanded_velocities.begin());
}

void PlannerCore::enter(std::shared_ptr<Motion> motion, const locomotion::RobotState & state) {
  if (!motion) {
    throw std::invalid_argument("PlannerCore: motion must not be null");
  }
  if (motion->max_steps().size() != num_interfaces_) {
    throw std::invalid_argument(
      "PlannerCore: motion max_steps size does not match the number of interfaces");
  }
  motion_ = std::move(motion);
  elapsed_ = 0.0;
  motion_->on_enter(state);
}

void PlannerCore::update(double dt, const locomotion::RobotState & state) {
  auto & current = output_.positions;
  if (motion_) {
    // 沒有寫入的介面為 NaN（保持、沒有前饋、預設增益）
    std::fill(command_.positions.begin(), command_.positions.end(), kNaN);
    std::fill(command_.velocities.begin(), command_.velocities.end(), kNaN);
    std::fill(command_.kp.begin(), command_.kp.end(), kNaN);
    std::fill(command_.kd.begin(), command_.kd.end(), kNaN);
    motion_->update(dt, state, command_);
    const auto & max_steps = motion_->max_steps();
    const auto & desired = command_.positions;
    for (size_t i = 0; i < current.size() && i < desired.size(); ++i) {
      // NaN 表示保持目前位置；inf 等異常值也一律保持
      if (std::isfinite(desired[i])) {
        current[i] += std::clamp(desired[i] - current[i], -max_steps[i], max_steps[i]);
      }
    }
    for (size_t i = 0; i < output_.velocities.size() && i < command_.velocities.size(); ++i) {
      const double velocity = command_.velocities[i];
      output_.velocities[i] = std::isfinite(velocity) ? velocity : kNaN;
    }
    // 增益必須是非負的有限值，否則改用下游的預設值
    const auto valid_gain = [](double gain) {return std::isfinite(gain) && gain >= 0.0;};
    for (size_t i = 0; i < output_.kp.size() && i < command_.kp.size(); ++i) {
      output_.kp[i] = valid_gain(command_.kp[i]) ? command_.kp[i] : kNaN;
    }
    for (size_t i = 0; i < output_.kd.size() && i < command_.kd.size(); ++i) {
      output_.kd[i] = valid_gain(command_.kd[i]) ? command_.kd[i] : kNaN;
    }
    output_.sample_period = motion_->sample_period();
    elapsed_ += dt;
  }

  // 這個週期的命令速度，下一個週期交給 Motion。dt 為 0（理論上不會發生）時保留上一次的值
  if (dt > 0.0) {
    for (size_t i = 0; i < num_interfaces_; ++i) {
      command_velocity_[i] = (current[i] - previous_positions_[i]) / dt;
      previous_positions_[i] = current[i];
    }
  }
}

MotionStatus PlannerCore::status() const {
  return motion_ ? motion_->status() : MotionStatus::kRunning;
}

}  // namespace graphen::control
