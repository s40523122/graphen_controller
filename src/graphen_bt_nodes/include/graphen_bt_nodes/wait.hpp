#ifndef GRAPHEN_BT_NODES__WAIT_HPP_
#define GRAPHEN_BT_NODES__WAIT_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace graphen::bt {

// 非阻塞等待：經過指定秒數後回傳 SUCCESS，期間回傳 RUNNING。
//
// Ports：
//   seconds  (int) 等待秒數
class Wait : public BT::StatefulActionNode {
public:
  Wait(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::seconds duration_{0};
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__WAIT_HPP_
