#include "graphen_bt_nodes/say_hello.hpp"

#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

SayHello::SayHello(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {}

BT::NodeStatus SayHello::tick() {
  RCLCPP_INFO(rclcpp::get_logger("BT"), "Hello from Behavior Tree!");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace graphen::bt
