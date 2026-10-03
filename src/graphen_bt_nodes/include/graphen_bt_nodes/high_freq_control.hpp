#ifndef GRAPHEN_BT_NODES__HIGH_FREQ_CONTROL_HPP_
#define GRAPHEN_BT_NODES__HIGH_FREQ_CONTROL_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"

namespace graphen::bt {

// 在背景 thread 以 target_hz 執行高頻迴圈，主樹低頻輪詢時只檢查是否完成。
// 目前為範例：跑滿 5 秒後視為完成。
//
// Ports：
//   target_hz  (double) 背景迴圈頻率
class HighFreqControlNode : public BT::StatefulActionNode {
public:
  HighFreqControlNode(const std::string & name, const BT::NodeConfiguration & config);
  ~HighFreqControlNode() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void stopThread();
  void highFreqLoop(double hz);

  std::thread worker_thread_;
  std::atomic<bool> keep_running_{false};
  std::atomic<bool> is_completed_{false};
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__HIGH_FREQ_CONTROL_HPP_
