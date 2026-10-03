#include "graphen_bt_nodes/rate_control.hpp"

#include <chrono>

namespace graphen::bt {

namespace {
constexpr double kDefaultHz = 100.0;
}  // namespace

RateControl::RateControl(const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config) {}

RateControl::~RateControl() {
  halt();
}

BT::PortsList RateControl::providedPorts() {
  return {BT::InputPort<double>("hz", kDefaultHz, "Execution rate in Hz")};
}

BT::NodeStatus RateControl::tick() {
  double hz = kDefaultHz;
  if (!getInput("hz", hz)) {
    throw BT::RuntimeError("RateControl: missing input [hz]");
  }
  if (hz <= 0.0) {
    throw BT::RuntimeError("RateControl: hz must be greater than zero");
  }

  // 主樹第一次進入時啟動獨立 worker
  if (!running_) {
    stop_requested_ = false;
    running_ = true;
    worker_thread_ = std::thread(&RateControl::runLoop, this);
  }

  // 主樹不執行 child，child 完全由 worker 執行
  return BT::NodeStatus::RUNNING;
}

void RateControl::halt() {
  stop_requested_ = true;

  // 立即喚醒 worker；不在這裡直接 halt child，避免和 worker 同時操作 child
  wake_cv_.notify_all();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  running_ = false;

  BT::DecoratorNode::halt();
}

void RateControl::runLoop() {
  auto next_time = std::chrono::steady_clock::now();

  while (!stop_requested_) {
    if (child_node_ != nullptr) {
      child_node_->executeTick();
    }

    // 每輪重新讀取 hz，可以透過 blackboard 動態調整頻率
    double hz = kDefaultHz;
    if (!getInput("hz", hz) || hz <= 0.0) {
      hz = kDefaultHz;
    }

    next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / hz));

    std::unique_lock<std::mutex> lock(wait_mutex_);
    wake_cv_.wait_until(lock, next_time, [this]() {return stop_requested_.load();});
  }

  // worker 結束後再處理 child
  if (child_node_ != nullptr && child_node_->status() == BT::NodeStatus::RUNNING) {
    child_node_->halt();
  }
}

}  // namespace graphen::bt
