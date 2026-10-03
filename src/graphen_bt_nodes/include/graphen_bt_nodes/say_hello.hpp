#ifndef GRAPHEN_BT_NODES__SAY_HELLO_HPP_
#define GRAPHEN_BT_NODES__SAY_HELLO_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace graphen::bt {

// 範例節點：印出問候訊息後回傳 SUCCESS
class SayHello : public BT::SyncActionNode {
public:
  SayHello(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__SAY_HELLO_HPP_
