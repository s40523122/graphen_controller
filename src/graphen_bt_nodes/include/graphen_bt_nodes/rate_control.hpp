#ifndef GRAPHEN_BT_NODES__RATE_CONTROL_HPP_
#define GRAPHEN_BT_NODES__RATE_CONTROL_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace graphen::bt {

// 以獨立 worker thread 依 hz 頻率 tick child，主樹 tick 時只回傳 RUNNING，
// 讓 child 的執行頻率不受主樹 tick 週期限制。
//
// Ports：
//   hz  (double) child 的執行頻率，必須 > 0
class RateControl : public BT::DecoratorNode {
public:
  RateControl(const std::string & name, const BT::NodeConfiguration & config);
  ~RateControl() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void runLoop();

  std::thread worker_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  std::mutex wait_mutex_;
  std::condition_variable wake_cv_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__RATE_CONTROL_HPP_
