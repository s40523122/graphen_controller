#ifndef GRAPHEN_BT_NODES__COMPARE_NODES_HPP_
#define GRAPHEN_BT_NODES__COMPARE_NODES_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace graphen::bt {

// 數值比較條件，讓判斷寫在 tree 的分支結構中，在 Groot 裡可以直接看出走了哪條路。
// 缺少 port 時丟出 BT::RuntimeError。
//
// Ports：
//   value      (double)
//   threshold  (double)

// value < threshold 時回傳 SUCCESS，否則 FAILURE
class IsBelow : public BT::ConditionNode {
public:
  IsBelow(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

// value > threshold 時回傳 SUCCESS，否則 FAILURE
class IsAbove : public BT::ConditionNode {
public:
  IsAbove(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__COMPARE_NODES_HPP_
