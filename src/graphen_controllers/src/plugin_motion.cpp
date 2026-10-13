#include "graphen_controllers/plugin_motion.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace graphen::control {

PluginMotion::PluginMotion(
  std::shared_ptr<locomotion::LocomotionAlgorithm> algorithm,
  std::vector<double> max_steps, std::string name, locomotion::Logger log)
: algorithm_(std::move(algorithm)), max_steps_(std::move(max_steps)),
  name_(std::move(name)), log_(std::move(log)) {
  if (!algorithm_) {
    throw std::invalid_argument("PluginMotion: algorithm must not be null");
  }
  if (max_steps_.empty()) {
    throw std::invalid_argument("PluginMotion: max_steps must not be empty");
  }
  if (std::any_of(max_steps_.begin(), max_steps_.end(), [](double s) {return !(s > 0.0);})) {
    throw std::invalid_argument("PluginMotion: every max_step must be greater than zero");
  }
}

void PluginMotion::fail(const char * stage, const char * what) {
  faulted_ = true;
  if (log_) {
    log_(locomotion::LogLevel::kError,
      "Algorithm of mode [ " + name_ + " ] threw in " + stage + ": " + what +
      ". Holding positions until the mode is entered again.");
  }
}

void PluginMotion::on_enter(const RobotState & state) {
  faulted_ = false;
  try {
    algorithm_->on_enter(state);
  } catch (const std::exception & e) {
    fail("on_enter", e.what());
  } catch (...) {
    fail("on_enter", "unknown exception");
  }
}

void PluginMotion::update(double dt, const RobotState & state, JointCommand & command) {
  // 故障時保持：planner 傳進來的 command 已經是全部 NaN
  if (faulted_) {
    return;
  }
  try {
    algorithm_->update(dt, state, command);
  } catch (const std::exception & e) {
    fail("update", e.what());
  } catch (...) {
    fail("update", "unknown exception");
  }
  if (faulted_) {
    // 演算法可能已寫入部分輸出，全部改回保持
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    std::fill(command.positions.begin(), command.positions.end(), kNaN);
    std::fill(command.velocities.begin(), command.velocities.end(), kNaN);
  }
}

}  // namespace graphen::control
