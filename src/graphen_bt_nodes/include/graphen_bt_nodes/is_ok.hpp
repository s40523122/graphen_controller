#ifndef GRAPHEN_BT_NODES__IS_OK_HPP_
#define GRAPHEN_BT_NODES__IS_OK_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace graphen::bt {

// 快速的 True/False 判斷：is_ok_val 為 true 回傳 SUCCESS，
// 否則（含讀不到）回傳 FAILURE，觸發 ReactiveSequence 中斷。
//
// Ports：
//   is_ok_val  (bool)
class IsOk : public BT::SimpleConditionNode {
public:
  IsOk(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkStatus();
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__IS_OK_HPP_
