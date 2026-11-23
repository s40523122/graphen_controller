#ifndef GRAPHEN_BT_NODES__VALUE_NODES_HPP_
#define GRAPHEN_BT_NODES__VALUE_NODES_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace graphen::bt {

// 樹中的簡單數值運算，讓決策留在 tree 的結構中（BT.CPP v3 沒有 script）。

// out = a × b × c（b、c 預設 1）。也可以只給 a，當作「寫入一個 double」使用
// （例如 a="0" 或 a="{v_fast}"），blackboard 的型別維持 double。
//   a, b, c (double)
//   out     (double, output)
class Multiply : public BT::SyncActionNode {
public:
  Multiply(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

// 以固定小數位數把 in1..in4 中有設定的值串成 "a;b;..."（例如 "5.0;0.0"）。
// 結果可以直接給 vector<double> 的 port，也適合當作 SendOnChange 的比較值：
// 固定位數避免浮點雜訊被當成「命令改變」。
//   in1..in4  (double) 選填，依序串接有設定的值；至少要有一個
//   precision (int)    預設 1，小數位數
//   out       (string, output)
class JoinValues : public BT::SyncActionNode {
public:
  JoinValues(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__VALUE_NODES_HPP_
