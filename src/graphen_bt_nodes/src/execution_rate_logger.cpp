#include "graphen_bt_nodes/execution_rate_logger.hpp"

#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

ExecutionRateLogger::ExecutionRateLogger(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  last_reset_time_(std::chrono::steady_clock::now()) {}

BT::PortsList ExecutionRateLogger::providedPorts() {
  return {BT::InputPort<std::string>("label", "BT_Node", "Label to display in log")};
}

BT::NodeStatus ExecutionRateLogger::tick() {
  ++tick_count_;
  const auto now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = now - last_reset_time_;

  // 累積滿 1 秒才印出並重置
  if (elapsed.count() >= 1.0) {
    std::string label = "BT_Node";
    getInput("label", label);

    // 以實際經過秒數計算，避免時間誤差影響結果
    const double actual_hz = static_cast<double>(tick_count_) / elapsed.count();

    RCLCPP_INFO(
      rclcpp::get_logger("RateLogger"),
      "[%s] Actual Execution Rate: %.2f Hz (Total Ticks in %.2fs: %d)",
      label.c_str(), actual_hz, elapsed.count(), tick_count_);

    tick_count_ = 0;
    last_reset_time_ = now;
  }

  // 不阻塞其他節點，讓樹繼續往下執行
  return BT::NodeStatus::SUCCESS;
}

}  // namespace graphen::bt
