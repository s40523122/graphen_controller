#include "graphen_bt_nodes/wait.hpp"

#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

Wait::Wait(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList Wait::providedPorts() {
  return {BT::InputPort<int>("seconds")};
}

BT::NodeStatus Wait::onStart() {
  int seconds = 0;
  if (!getInput("seconds", seconds)) {
    throw BT::RuntimeError("Wait: missing input [seconds]");
  }

  start_time_ = std::chrono::steady_clock::now();
  duration_ = std::chrono::seconds(seconds);

  RCLCPP_INFO(rclcpp::get_logger("BT"), "Wait started (%d s)...", seconds);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning() {
  if (std::chrono::steady_clock::now() - start_time_ >= duration_) {
    RCLCPP_INFO(rclcpp::get_logger("BT"), "Wait finished!");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void Wait::onHalted() {
  RCLCPP_INFO(rclcpp::get_logger("BT"), "Wait halted");
}

}  // namespace graphen::bt
