#include "graphen_bt_nodes/is_ok.hpp"

namespace graphen::bt {

IsOk::IsOk(const std::string & name, const BT::NodeConfiguration & config)
: BT::SimpleConditionNode(name, [this](BT::TreeNode &) {return checkStatus();}, config) {}

BT::PortsList IsOk::providedPorts() {
  return {BT::InputPort<bool>("is_ok_val")};
}

BT::NodeStatus IsOk::checkStatus() {
  bool ok_status = false;
  if (getInput("is_ok_val", ok_status) && ok_status) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace graphen::bt
