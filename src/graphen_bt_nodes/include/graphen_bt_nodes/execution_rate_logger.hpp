#ifndef GRAPHEN_BT_NODES__EXECUTION_RATE_LOGGER_HPP_
#define GRAPHEN_BT_NODES__EXECUTION_RATE_LOGGER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace graphen::bt {

// 統計自身被 tick 的次數，每滿 1 秒印出實際執行頻率，永遠回傳 SUCCESS。
//
// Ports：
//   label  (string) log 中顯示的標籤
class ExecutionRateLogger : public BT::SyncActionNode {
public:
  ExecutionRateLogger(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  int tick_count_{0};
  std::chrono::steady_clock::time_point last_reset_time_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__EXECUTION_RATE_LOGGER_HPP_
