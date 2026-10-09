#include "graphen_controllers/locomotion_planner.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace graphen::control {

LocomotionPlanner::LocomotionPlanner(double rate_hz, std::vector<double> max_steps)
: rate_hz_(rate_hz), max_steps_(std::move(max_steps)) {
  if (rate_hz_ <= 0.0) {
    throw std::invalid_argument("LocomotionPlanner: rate_hz must be greater than zero");
  }
  if (max_steps_.empty()) {
    throw std::invalid_argument("LocomotionPlanner: max_steps must not be empty");
  }
  if (std::any_of(max_steps_.begin(), max_steps_.end(), [](double s) {return s <= 0.0;})) {
    throw std::invalid_argument("LocomotionPlanner: every max_step must be greater than zero");
  }
  goal_.assign(max_steps_.size(), 0.0);
}

LocomotionPlanner::~LocomotionPlanner() {
  stop();
}

void LocomotionPlanner::set_goal(const std::vector<double> & goal) {
  if (goal.size() != max_steps_.size()) {
    throw std::invalid_argument("LocomotionPlanner: goal size does not match max_steps size");
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  goal_ = goal;
}

void LocomotionPlanner::start(const std::vector<double> & initial_positions, TargetBuffer * sink) {
  if (initial_positions.size() != max_steps_.size()) {
    throw std::invalid_argument(
      "LocomotionPlanner: initial_positions size does not match max_steps size");
  }
  if (sink == nullptr) {
    throw std::invalid_argument("LocomotionPlanner: sink must not be null");
  }
  stop();

  current_ = initial_positions;
  sink_ = sink;
  // 先寫入起點，controller 在第一個規劃週期之前就有值可以輸出
  sink_->writeFromNonRT(current_);

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

  std::vector<double> goal(max_steps_.size());
  auto next_wakeup = Clock::now();

  while (running_.load()) {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal = goal_;
    }
    for (size_t i = 0; i < current_.size(); ++i) {
      current_[i] += std::clamp(goal[i] - current_[i], -max_steps_[i], max_steps_[i]);
    }
    sink_->writeFromNonRT(current_);

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
