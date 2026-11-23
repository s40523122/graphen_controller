#include "graphen_controllers/shadow_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "graphen_controllers/planner_core.hpp"

namespace graphen::control {

namespace {

std::string format_seconds(double seconds) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", seconds);
  return buffer;
}

// 理想追蹤：量測值 = planner 的輸出（下一個週期 Motion 看到的 commanded_*）
void track_ideally(locomotion::RobotState & state) {
  const size_t n = std::min(state.measured_positions.size(), state.commanded_positions.size());
  std::copy_n(state.commanded_positions.begin(), n, state.measured_positions.begin());
  const size_t m = std::min(state.measured_velocities.size(), state.commanded_velocities.size());
  std::copy_n(state.commanded_velocities.begin(), m, state.measured_velocities.begin());
}

}  // namespace

ShadowResult run_shadow(
  const std::shared_ptr<Motion> & motion, const locomotion::RobotState & start,
  double rate_hz, double max_duration, double sample_rate)
{
  if (!motion) {
    throw std::invalid_argument("run_shadow: motion must not be null");
  }
  if (!(rate_hz > 0.0) || !(max_duration > 0.0)) {
    throw std::invalid_argument("run_shadow: rate_hz and max_duration must be greater than zero");
  }
  const auto & initial = start.commanded_positions;
  if (initial.empty() || std::any_of(initial.begin(), initial.end(),
      [](double v) {return !std::isfinite(v);}))
  {
    throw std::invalid_argument("run_shadow: the start state has no valid commanded positions");
  }

  ShadowResult result;
  PlannerCore core(initial.size());
  core.reset(initial, start.commanded_velocities.size() == initial.size() ?
    start.commanded_velocities : std::vector<double>{});

  locomotion::RobotState state = start;
  core.fill_commanded(state);
  // 起點的量測值保留實機的值：on_enter 以它設定非播放部位的姿態（例如手臂碰撞檢查中的其他 joint）
  core.enter(motion, state);

  result.times.push_back(0.0);
  result.positions.push_back(core.output().positions);

  const double dt = 1.0 / rate_hz;
  const double sample_period = sample_rate > 0.0 ? 1.0 / sample_rate : 0.0;
  double next_sample = sample_period;
  double time = 0.0;
  while (true) {
    // planner 的順序：先填 commanded_*，再 update
    core.fill_commanded(state);
    core.update(dt, state);
    time += dt;
    state.time += dt;

    const auto status = core.status();
    const bool ended = status != MotionStatus::kRunning || time >= max_duration;
    if (sample_period > 0.0 && time + 1e-9 >= next_sample && !ended) {
      result.times.push_back(time);
      result.positions.push_back(core.output().positions);
      next_sample += sample_period;
    }
    if (ended) {
      result.times.push_back(time);
      result.positions.push_back(core.output().positions);
      result.duration = time;
      if (status == MotionStatus::kFinished) {
        result.success = true;
        result.message = "Dry run finished in " + format_seconds(time) + " s";
      } else if (status == MotionStatus::kFailed) {
        const auto reason = motion->failure_reason();
        result.message = "Dry run failed after " + format_seconds(time) + " s" +
          (reason.empty() ? std::string{} : ": " + reason);
      } else {
        result.message = "Dry run did not finish within " + format_seconds(max_duration) +
          " s (choreo.shadow.max_duration)";
      }
      return result;
    }
    // 下一步 Motion 看到的量測值
    core.fill_commanded(state);
    track_ideally(state);
  }
}

}  // namespace graphen::control
