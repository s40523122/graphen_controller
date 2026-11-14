#include "graphen_controllers/locomotion_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
  output_.positions = initial_positions;
  output_.velocities.assign(num_interfaces_, kNaN);
  output_.kp.assign(num_interfaces_, kNaN);
  output_.kd.assign(num_interfaces_, kNaN);
  output_.sample_period = 0.0;
  state_ = RobotState{};
  state_.measured_positions.assign(num_interfaces_, kNaN);
  state_.measured_velocities.assign(num_interfaces_, kNaN);
  state_.commanded_positions = initial_positions;
  state_.commanded_velocities.assign(num_interfaces_, 0.0);
  previous_positions_ = initial_positions;
  command_velocity_.assign(num_interfaces_, 0.0);
  command_.positions.assign(num_interfaces_, kNaN);
  command_.velocities.assign(num_interfaces_, kNaN);
  command_.kp.assign(num_interfaces_, kNaN);
  command_.kd.assign(num_interfaces_, kNaN);
  motion_.reset();
  active_sequence_ = 0;
  elapsed_ = 0.0;
  {
    std::lock_guard<std::mutex> lock(progress_mutex_);
    progress_ = Progress{};
  }
  sink_ = sink;
  read_state_ = std::move(read_state);
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

  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  while (running_.load()) {
    // 先取得狀態，切換時的 on_enter 與 update 都使用同一份
    if (read_state_) {
      read_state_(state_);
    }
    state_.commanded_positions = output_.positions;
    // read_state_ 會以 controller 的緩衝覆寫整個 RobotState（不含這個欄位的內容），所以每週期重新填入
    state_.commanded_velocities.resize(num_interfaces_);
    std::copy(command_velocity_.begin(), command_velocity_.end(),
      state_.commanded_velocities.begin());

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (pending_motion_) {
        motion_ = std::move(pending_motion_);
        pending_motion_.reset();
        active_sequence_ = pending_sequence_;
        elapsed_ = 0.0;
        motion_->on_enter(state_);
      }
    }

    // 以實際經過的時間推進動作，planner 被延遲時播放仍跟隨真實時間（輸出仍受 max_steps 限制）
    const auto now_cycle = Clock::now();
    const double dt = std::chrono::duration<double>(now_cycle - last_cycle).count();
    last_cycle = now_cycle;

    auto & current = output_.positions;
    if (motion_) {
      // 沒有寫入的介面為 NaN（保持、沒有前饋、預設增益）
      std::fill(command_.positions.begin(), command_.positions.end(), kNaN);
      std::fill(command_.velocities.begin(), command_.velocities.end(), kNaN);
      std::fill(command_.kp.begin(), command_.kp.end(), kNaN);
      std::fill(command_.kd.begin(), command_.kd.end(), kNaN);
      motion_->update(dt, state_, command_);
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
      std::lock_guard<std::mutex> lock(progress_mutex_);
      progress_.sequence = active_sequence_;
      progress_.status = motion_->status();
      progress_.elapsed = elapsed_;
    }
    sink_->writeFromNonRT(output_);

    // 這個週期的命令速度，下一個週期交給 Motion。dt 為 0（理論上不會發生）時保留上一次的值
    if (dt > 0.0) {
      for (size_t i = 0; i < num_interfaces_; ++i) {
        command_velocity_[i] = (current[i] - previous_positions_[i]) / dt;
        previous_positions_[i] = current[i];
      }
    }

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
