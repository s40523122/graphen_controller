#include "graphen_controllers/target_motion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace graphen::control {

TargetMotion::TargetMotion(std::vector<double> targets, std::vector<double> max_steps)
: targets_(std::move(targets)), max_steps_(std::move(max_steps)) {
  if (targets_.empty() || targets_.size() != max_steps_.size()) {
    throw std::invalid_argument(
      "TargetMotion: targets and max_steps must be non-empty and have the same size");
  }
  if (std::any_of(max_steps_.begin(), max_steps_.end(), [](double s) {return !(s > 0.0);})) {
    throw std::invalid_argument("TargetMotion: every max_step must be greater than zero");
  }
  desired_ = targets_;
}

void TargetMotion::on_enter(const RobotState & state) {
  const auto & current = state.commanded_positions;
  for (size_t i = 0; i < targets_.size(); ++i) {
    desired_[i] = std::isnan(targets_[i]) ? current[i] : targets_[i];
  }
}

void TargetMotion::update(
  double /*dt*/, const RobotState & /*state*/, JointCommand & command) {
  command.positions = desired_;
}

}  // namespace graphen::control
