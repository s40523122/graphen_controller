#ifndef GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_
#define GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "graphen_controllers/motion.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace graphen::control {

// 以固定頻率播放 Motion 的 planner，在獨立的 non-RT thread 中執行。
//
// 每個週期向目前的 Motion 取得期望位置，每個介面最多移動 Motion::max_steps()[i]，
// 連同 Motion::sample_period() 寫入呼叫端提供的 TargetBuffer。controller 的 update() 只要讀取最新值輸出即可，
// 規劃計算（包含 BVH 的轉換）都不在 RT thread 中執行。
//
// set_motion() 可以在任何 thread 呼叫，切換在 planner thread 的下一個週期執行；
// 未啟動時設定的 motion 會保留到 start() 之後使用。
class LocomotionPlanner {
public:
  struct Output {
    std::vector<double> positions;
    // 目前 Motion 的輸出點間隔（秒），供下游插補；0 表示連續
    double sample_period{0.0};
  };
  using TargetBuffer = realtime_tools::RealtimeBuffer<Output>;

  // num_interfaces 必須 > 0；rate_hz 必須 > 0
  LocomotionPlanner(double rate_hz, size_t num_interfaces);
  ~LocomotionPlanner();

  LocomotionPlanner(const LocomotionPlanner &) = delete;
  LocomotionPlanner & operator=(const LocomotionPlanner &) = delete;

  // 切換動作；motion 的 max_steps 長度必須等於介面數。
  // 切換後 motion 只會在 planner thread 中被呼叫，呼叫端不可同時使用它
  void set_motion(std::shared_ptr<Motion> motion);

  // 以 initial_positions 為起點開始規劃，結果寫入 sink（呼叫端需保證 sink 在 stop() 前有效）。
  // 還沒有設定 motion 時保持起點
  void start(const std::vector<double> & initial_positions, TargetBuffer * sink);

  // 停止規劃 thread 並等待結束；未啟動時不做任何事
  void stop();

  bool is_running() const {return running_.load();}

private:
  void loop();

  const double rate_hz_;
  const size_t num_interfaces_;

  std::mutex motion_mutex_;
  std::shared_ptr<Motion> pending_motion_;  // 等待 planner thread 切換

  // 只在規劃 thread 中存取
  std::shared_ptr<Motion> motion_;
  Output output_;  // positions 即目前的輸出位置
  std::vector<double> desired_;
  TargetBuffer * sink_{nullptr};

  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_
