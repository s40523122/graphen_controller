#include "graphen_bt_nodes/value_nodes.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace graphen::bt {

namespace {

constexpr int kMaxJoinInputs = 4;
constexpr int kDefaultPrecision = 1;

}  // namespace

Multiply::Multiply(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {}

BT::PortsList Multiply::providedPorts() {
  return {
    BT::InputPort<double>("a"),
    BT::InputPort<double>("b", 1.0, ""),
    BT::InputPort<double>("c", 1.0, ""),
    BT::OutputPort<double>("out"),
  };
}

BT::NodeStatus Multiply::tick() {
  double a = 0.0;
  double b = 1.0;
  double c = 1.0;
  if (!getInput("a", a)) {
    throw BT::RuntimeError("Multiply: missing input [a]");
  }
  getInput("b", b);
  getInput("c", c);
  setOutput("out", a * b * c);
  return BT::NodeStatus::SUCCESS;
}

JoinValues::JoinValues(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {}

BT::PortsList JoinValues::providedPorts() {
  BT::PortsList ports;
  for (int i = 1; i <= kMaxJoinInputs; ++i) {
    ports.insert(BT::InputPort<double>("in" + std::to_string(i)));
  }
  ports.insert(BT::InputPort<int>("precision", kDefaultPrecision, "Number of decimals"));
  ports.insert(BT::OutputPort<std::string>("out", "Values joined with ;"));
  return ports;
}

BT::NodeStatus JoinValues::tick() {
  int precision = kDefaultPrecision;
  getInput("precision", precision);
  if (precision < 0) {
    throw BT::RuntimeError("JoinValues: [precision] must not be negative");
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(precision);
  int count = 0;
  for (int i = 1; i <= kMaxJoinInputs; ++i) {
    const auto port = "in" + std::to_string(i);
    // 沒有寫在 XML 中的 port 會被略過；寫了但讀不到值（blackboard 尚未設定）是樹的錯誤
    if (config().input_ports.count(port) == 0) {
      continue;
    }
    double value = 0.0;
    if (!getInput(port, value)) {
      throw BT::RuntimeError("JoinValues: cannot read [" + port + "]");
    }
    // -0.0 與 0.0 輸出相同的字串，避免「命令改變」的誤判
    if (std::abs(value) < 0.5 * std::pow(10.0, -precision)) {
      value = 0.0;
    }
    out << (count++ == 0 ? "" : ";") << value;
  }
  if (count == 0) {
    throw BT::RuntimeError("JoinValues: at least one of in1..in4 is required");
  }
  setOutput("out", out.str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace graphen::bt
