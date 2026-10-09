#ifndef GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_
#define GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "realtime_tools/realtime_buffer.hpp"

namespace graphen::control {

// 以固定頻率產生位置目標的 planner，在獨立的 non-RT thread 中執行。
//
// 每個週期把目前的目標往 goal 內插，每個介面最多移動 max_steps[i]，
// 然後寫入呼叫端提供的 TargetBuffer。controller 的 update() 只要讀取最新值輸出即可，
// 不需要在 RT thread 中做任何規劃計算。
//
// set_goal() 可以在任何 thread 呼叫；未啟動時設定的 goal 會保留到 start() 之後使用。
class LocomotionPlanner {
public:
  using TargetBuffer = realtime_tools::RealtimeBuffer<std::vector<double>>;

  // max_steps 的長度決定介面數，每個值必須 > 0；rate_hz 必須 > 0
  LocomotionPlanner(double rate_hz, std::vector<double> max_steps);
  ~LocomotionPlanner();

  LocomotionPlanner(const LocomotionPlanner &) = delete;
  LocomotionPlanner & operator=(const LocomotionPlanner &) = delete;

  // 設定目標位置，長度必須等於介面數
  void set_goal(const std::vector<double> & goal);

  // 以 initial_positions 為起點開始規劃，結果寫入 sink（呼叫端需保證 sink 在 stop() 前有效）
  void start(const std::vector<double> & initial_positions, TargetBuffer * sink);

  // 停止規劃 thread 並等待結束；未啟動時不做任何事
  void stop();

  bool is_running() const {return running_.load();}

private:
  void loop();

  const double rate_hz_;
  const std::vector<double> max_steps_;

  std::mutex goal_mutex_;
  std::vector<double> goal_;

  // 只在規劃 thread 中存取
  std::vector<double> current_;
  TargetBuffer * sink_{nullptr};

  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_
