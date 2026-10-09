#include "graphen_bt_nodes/compare_nodes.hpp"

#include <utility>

namespace graphen::bt {

namespace {

BT::PortsList compare_ports() {
  return {
    BT::InputPort<double>("value"),
    BT::InputPort<double>("threshold"),
  };
}

// 讀取 value / threshold，缺少時丟出例外
template<typename NodeT>
std::pair<double, double> read_inputs(const NodeT & node, const char * id) {
  double value = 0.0;
  double threshold = 0.0;
  if (!node.getInput("value", value)) {
    throw BT::RuntimeError(id, ": missing input [value]");
  }
  if (!node.getInput("threshold", threshold)) {
    throw BT::RuntimeError(id, ": missing input [threshold]");
  }
  return {value, threshold};
}

}  // namespace

IsBelow::IsBelow(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}

BT::PortsList IsBelow::providedPorts() {
  return compare_ports();
}

BT::NodeStatus IsBelow::tick() {
  const auto [value, threshold] = read_inputs(*this, "IsBelow");
  return value < threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsAbove::IsAbove(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}

BT::PortsList IsAbove::providedPorts() {
  return compare_ports();
}

BT::NodeStatus IsAbove::tick() {
  const auto [value, threshold] = read_inputs(*this, "IsAbove");
  return value > threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace graphen::bt
