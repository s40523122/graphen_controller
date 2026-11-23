#include "graphen_bt_nodes/playback_nodes.hpp"

#include <cmath>

namespace graphen::bt {

namespace {

constexpr const char * kSwitchModeAction = "/switch_group_controller/switch_mode";
constexpr const char * kPlayChoreoAction = "/switch_group_controller/play_choreo";
constexpr const char * kPlayBvhAction = "/switch_group_controller/play_bvh";
constexpr const char * kRunPolicyAction = "/switch_group_controller/run_policy";

constexpr bool kDefaultBvhLoop = true;
constexpr double kDefaultBvhTimeScale = 1.0;

// 必填的字串 port；沒有設定或是空字串時丟出 BT::RuntimeError
std::string required_input(
  const BT::TreeNode & node, const std::string & type, const std::string & port)
{
  std::string value;
  if (!node.getInput(port, value) || value.empty()) {
    throw BT::RuntimeError(type + ": missing input [" + port + "]");
  }
  return value;
}

}  // namespace

SwitchMode::SwitchMode(const std::string & name, const BT::NodeConfiguration & config)
: PlaybackActionNode(name, config, "SwitchMode", kSwitchModeAction) {}

BT::PortsList SwitchMode::providedPorts() {
  auto ports = common_ports(kSwitchModeAction);
  ports.insert(BT::InputPort<std::string>("mode", "System mode to switch to"));
  return ports;
}

std::string SwitchMode::fill_goal(Goal & goal) {
  goal.mode = required_input(*this, node_type_name(), "mode");
  return goal.mode;
}

PlayChoreo::PlayChoreo(const std::string & name, const BT::NodeConfiguration & config)
: PlaybackActionNode(name, config, "PlayChoreo", kPlayChoreoAction) {}

BT::PortsList PlayChoreo::providedPorts() {
  auto ports = common_ports(kPlayChoreoAction);
  ports.insert(BT::InputPort<std::string>("choreo", "Name of a published choreography"));
  ports.insert(BT::InputPort<std::string>("content",
    "Choreography script (YAML) to play directly; exclusive with choreo"));
  return ports;
}

std::string PlayChoreo::fill_goal(Goal & goal) {
  std::string name;
  std::string content;
  getInput("choreo", name);
  getInput("content", content);
  if (name.empty() == content.empty()) {
    throw BT::RuntimeError(node_type_name() + ": exactly one of [choreo] and [content] is required");
  }
  goal.name = name;
  goal.content = content;
  return name.empty() ? "<content>" : name;
}

PlayBvh::PlayBvh(const std::string & name, const BT::NodeConfiguration & config)
: PlaybackActionNode(name, config, "PlayBvh", kPlayBvhAction) {}

BT::PortsList PlayBvh::providedPorts() {
  auto ports = common_ports(kPlayBvhAction);
  ports.insert(BT::InputPort<std::string>("bvh", "BVH file name without the .bvh extension"));
  ports.insert(BT::InputPort<bool>("loop", kDefaultBvhLoop, "Loop when the end is reached"));
  ports.insert(BT::InputPort<double>("time_scale", kDefaultBvhTimeScale, "Playback speed factor"));
  return ports;
}

std::string PlayBvh::fill_goal(Goal & goal) {
  goal.name = required_input(*this, node_type_name(), "bvh");
  goal.loop = kDefaultBvhLoop;
  getInput("loop", goal.loop);
  goal.time_scale = kDefaultBvhTimeScale;
  getInput("time_scale", goal.time_scale);
  if (!(goal.time_scale > 0.0) || !std::isfinite(goal.time_scale)) {
    throw BT::RuntimeError(node_type_name() + ": [time_scale] must be greater than zero");
  }
  return goal.name;
}

RunPolicy::RunPolicy(const std::string & name, const BT::NodeConfiguration & config)
: PlaybackActionNode(name, config, "RunPolicy", kRunPolicyAction) {}

BT::PortsList RunPolicy::providedPorts() {
  auto ports = common_ports(kRunPolicyAction);
  ports.insert(BT::InputPort<std::string>("policy", "Name of a policy in the policy library"));
  return ports;
}

std::string RunPolicy::fill_goal(Goal & goal) {
  goal.name = required_input(*this, node_type_name(), "policy");
  return goal.name;
}

}  // namespace graphen::bt
