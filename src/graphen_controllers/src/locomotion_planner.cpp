#include "graphen_controllers/locomotion_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace graphen::control {

LocomotionPlanner::LocomotionPlanner(double rate_hz, size_t num_interfaces)
: rate_hz_(rate_hz), num_interfaces_(num_interfaces),
  core_(num_interfaces)  // num_interfaces 為 0 時丟出例外
{
  if (rate_hz_ <= 0.0) {
    throw std::invalid_argument("LocomotionPlanner: rate_hz must be greater than zero");
  }
}

LocomotionPlanner::~LocomotionPlanner() {
  stop();
}

uint64_t LocomotionPlanner::set_motion(std::shared_ptr<Motion> motion) {
  if (!motion) {
    throw std::invalid_argument("LocomotionPlanner: motion must not be null");
  }
  if (motion->max_steps().size() != num_interfaces_) {
    throw std::invalid_argument(
      "LocomotionPlanner: motion max_steps size does not match the number of interfaces");
  }
  std::lock_guard<std::mutex> lock(motion_mutex_);
  pending_motion_ = std::move(motion);
  pending_sequence_ = ++last_sequence_;
  return pending_sequence_;
}

LocomotionPlanner::Progress LocomotionPlanner::progress() const {
  std::lock_guard<std::mutex> lock(progress_mutex_);
  return progress_;
}

std::optional<RobotState> LocomotionPlanner::snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (!has_snapshot_) {
    return std::nullopt;
  }
  return snapshot_;
}

void LocomotionPlanner::start(
  const std::vector<double> & initial_positions, TargetBuffer * sink, StateReader read_state)
{
  if (initial_positions.size() != num_interfaces_) {
    throw std::invalid_argument(
      "LocomotionPlanner: initial_positions size does not match the number of interfaces");
  }
  if (sink == nullptr) {
    throw std::invalid_argument("LocomotionPlanner: sink must not be null");
  }
  stop();

  // 週期中使用的緩衝在這裡配置好大小，之後只覆寫內容
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  core_.reset(initial_positions);
  state_ = RobotState{};
  state_.measured_positions.assign(num_interfaces_, kNaN);
  state_.measured_velocities.assign(num_interfaces_, kNaN);
  core_.fill_commanded(state_);
  active_sequence_ = 0;
  {
    std::lock_guard<std::mutex> lock(progress_mutex_);
    progress_ = Progress{};
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    has_snapshot_ = false;
  }
  sink_ = sink;
  read_state_ = std::move(read_state);
  // 先寫入起點，controller 在第一個規劃週期之前就有值可以輸出
  sink_->writeFromNonRT(core_.output());

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
    // 先取得狀態，切換時的 on_enter 與 update 都使用同一份
    if (read_state_) {
      read_state_(state_);
    }
    // read_state_ 會以 controller 的緩衝覆寫整個 RobotState（不含 commanded_*），所以每週期重新填入
    core_.fill_commanded(state_);
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      snapshot_ = state_;
      has_snapshot_ = true;
    }

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (pending_motion_) {
        active_sequence_ = pending_sequence_;
        core_.enter(std::move(pending_motion_), state_);
        pending_motion_.reset();
      }
    }

    // 以實際經過的時間推進動作，planner 被延遲時播放仍跟隨真實時間（輸出仍受 max_steps 限制）
    const auto now_cycle = Clock::now();
    const double dt = std::chrono::duration<double>(now_cycle - last_cycle).count();
    last_cycle = now_cycle;

    core_.update(dt, state_);
    if (core_.has_motion()) {
      std::lock_guard<std::mutex> lock(progress_mutex_);
      progress_.sequence = active_sequence_;
      progress_.status = core_.status();
      progress_.elapsed = core_.elapsed();
    }
    sink_->writeFromNonRT(core_.output());

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
