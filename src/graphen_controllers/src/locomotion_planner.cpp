#include "graphen_controllers/locomotion_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace graphen::control {

LocomotionPlanner::LocomotionPlanner(double rate_hz, size_t num_interfaces)
: rate_hz_(rate_hz), num_interfaces_(num_interfaces) {
  if (rate_hz_ <= 0.0) {
    throw std::invalid_argument("LocomotionPlanner: rate_hz must be greater than zero");
  }
  if (num_interfaces_ == 0) {
    throw std::invalid_argument("LocomotionPlanner: num_interfaces must be greater than zero");
  }
}

LocomotionPlanner::~LocomotionPlanner() {
  stop();
}

void LocomotionPlanner::set_motion(std::shared_ptr<Motion> motion) {
  if (!motion) {
    throw std::invalid_argument("LocomotionPlanner: motion must not be null");
  }
  if (motion->max_steps().size() != num_interfaces_) {
    throw std::invalid_argument(
      "LocomotionPlanner: motion max_steps size does not match the number of interfaces");
  }
  std::lock_guard<std::mutex> lock(motion_mutex_);
  pending_motion_ = std::move(motion);
}

void LocomotionPlanner::start(const std::vector<double> & initial_positions, TargetBuffer * sink) {
  if (initial_positions.size() != num_interfaces_) {
    throw std::invalid_argument(
      "LocomotionPlanner: initial_positions size does not match the number of interfaces");
  }
  if (sink == nullptr) {
    throw std::invalid_argument("LocomotionPlanner: sink must not be null");
  }
  stop();

  output_.positions = initial_positions;
  output_.sample_period = 0.0;
  desired_ = initial_positions;
  motion_.reset();
  sink_ = sink;
  // 先寫入起點，controller 在第一個規劃週期之前就有值可以輸出
  sink_->writeFromNonRT(output_);

  running_.store(true);
  thread_ = std::thread(&LocomotionPlanner::loop, this);
}

void LocomotionPlanner::stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void LocomotionPlanner::loop() {
  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / rate_hz_));

  auto next_wakeup = Clock::now();
  auto last_cycle = next_wakeup;

  while (running_.load()) {
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (pending_motion_) {
        motion_ = std::move(pending_motion_);
        pending_motion_.reset();
        motion_->on_enter(output_.positions);
      }
    }

    // 以實際經過的時間推進動作，planner 被延遲時播放仍跟隨真實時間（輸出仍受 max_steps 限制）
    const auto now_cycle = Clock::now();
    const double dt = std::chrono::duration<double>(now_cycle - last_cycle).count();
    last_cycle = now_cycle;

    auto & current = output_.positions;
    if (motion_) {
      motion_->update(dt, current, desired_);
      const auto & max_steps = motion_->max_steps();
      for (size_t i = 0; i < current.size() && i < desired_.size(); ++i) {
        // NaN 表示保持目前位置
        if (std::isfinite(desired_[i])) {
          current[i] += std::clamp(desired_[i] - current[i], -max_steps[i], max_steps[i]);
        }
      }
      output_.sample_period = motion_->sample_period();
    }
    sink_->writeFromNonRT(output_);

    // 以絕對時間排程，維持固定頻率；落後超過一個週期時重新對齊，不連續補跑
    next_wakeup += period;
    const auto now = Clock::now();
    if (next_wakeup < now) {
      next_wakeup = now;
    }
    std::this_thread::sleep_until(next_wakeup);
  }
}

}  // namespace graphen::control
