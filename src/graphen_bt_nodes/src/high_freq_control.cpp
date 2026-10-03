#include "graphen_bt_nodes/high_freq_control.hpp"

#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

namespace {
constexpr double kDefaultHz = 100.0;
constexpr double kRunSeconds = 5.0;  // 範例完成條件：跑滿的秒數
}  // namespace

HighFreqControlNode::HighFreqControlNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config) {}

HighFreqControlNode::~HighFreqControlNode() {
  stopThread();
}

BT::PortsList HighFreqControlNode::providedPorts() {
  return {BT::InputPort<double>("target_hz", kDefaultHz, "High frequency loop rate")};
}

// 主樹第一次 tick 到此節點時啟動背景 thread
BT::NodeStatus HighFreqControlNode::onStart() {
  double hz = kDefaultHz;
  getInput("target_hz", hz);
  if (hz <= 0.0) {
    throw BT::RuntimeError("HighFreqControlNode: target_hz must be greater than zero");
  }

  keep_running_ = true;
  is_completed_ = false;
  worker_thread_ = std::thread(&HighFreqControlNode::highFreqLoop, this, hz);

  RCLCPP_INFO(rclcpp::get_logger("BT"), "High frequency background thread launched.");
  return BT::NodeStatus::RUNNING;
}

// 主樹低頻輪詢時只檢查背景 thread 是否完成，不阻塞主樹
BT::NodeStatus HighFreqControlNode::onRunning() {
  if (is_completed_) {
    stopThread();
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

// 被 ReactiveSequence 等中斷時，安全停止背景 thread
void HighFreqControlNode::onHalted() {
  stopThread();
  RCLCPP_WARN(rclcpp::get_logger("BT"), "High frequency thread halted!");
}

void HighFreqControlNode::stopThread() {
  keep_running_ = false;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void HighFreqControlNode::highFreqLoop(double hz) {
  const auto interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / hz));
  const int log_every = std::max(1, static_cast<int>(hz));
  const int total_ticks = static_cast<int>(hz * kRunSeconds);

  int count = 0;
  while (keep_running_) {
    const auto next_tick = std::chrono::steady_clock::now() + interval;

    // 在這裡執行高頻計算 / 馬達控制 / 數據採集
    ++count;
    if (count % log_every == 0) {
      RCLCPP_INFO(rclcpp::get_logger("HighFreq"), "Running inside thread at %.0f Hz", hz);
    }

    if (count >= total_ticks) {
      is_completed_ = true;
      break;
    }

    std::this_thread::sleep_until(next_tick);
  }
}

}  // namespace graphen::bt
