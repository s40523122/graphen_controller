#include "graphen_bt_nodes/send_on_change.hpp"

namespace graphen::bt {

SendOnChange::SendOnChange(const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config) {}

BT::PortsList SendOnChange::providedPorts() {
  return {
    BT::InputPort<std::string>("value", "Command to compare with the last one sent"),
    BT::InputPort<int>("refresh_ms", 0, "Resend an unchanged command after this long, 0 = never"),
  };
}

void SendOnChange::halt() {
  last_value_.reset();
  DecoratorNode::halt();
}

BT::NodeStatus SendOnChange::tick() {
  if (child()->status() != BT::NodeStatus::RUNNING) {
    std::string value;
    if (!getInput("value", value)) {
      throw BT::RuntimeError("SendOnChange: missing input [value]");
    }
    int refresh_ms = 0;
    getInput("refresh_ms", refresh_ms);

    const auto now = std::chrono::steady_clock::now();
    const bool changed = !last_value_ || *last_value_ != value;
    const bool expired = refresh_ms > 0 &&
      now - last_sent_ >= std::chrono::milliseconds(refresh_ms);
    if (!changed && !expired) {
      return BT::NodeStatus::SUCCESS;
    }
    pending_value_ = value;
  }

  setStatus(BT::NodeStatus::RUNNING);
  const auto status = child()->executeTick();
  if (status == BT::NodeStatus::SUCCESS) {
    last_value_ = pending_value_;
    last_sent_ = std::chrono::steady_clock::now();
  }
  if (status != BT::NodeStatus::RUNNING) {
    resetChild();
  }
  return status;
}

}  // namespace graphen::bt
