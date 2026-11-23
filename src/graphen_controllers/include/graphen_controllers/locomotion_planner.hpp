#ifndef GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_
#define GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "graphen_controllers/motion.hpp"
#include "graphen_controllers/planner_core.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace graphen::control {

// 以固定頻率播放 Motion 的 planner，在獨立的 non-RT thread 中執行。
//
// 每個週期先以呼叫端提供的 StateReader 取得量測狀態（關節角度 / 速度、IMU），交給目前的 Motion
// 取得期望位置與速度前饋，每個介面最多移動 Motion::max_steps()[i]，
// 連同 Motion::sample_period() 寫入呼叫端提供的 TargetBuffer。controller 的 update() 只要讀取最新值輸出即可，
// 規劃計算（包含 BVH 的轉換）都不在 RT thread 中執行。
//
// set_motion() 可以在任何 thread 呼叫，切換在 planner thread 的下一個週期執行；
// 未啟動時設定的 motion 會保留到 start() 之後使用。
// 每次 set_motion() 會得到一個遞增的序號，progress() 回報目前播放中的序號與 Motion::status()，
// 呼叫端可以據此判斷「自己設定的那一次」是否已經播完、或已被後來的 set_motion() 取代。
// 每週期的計算在 PlannerCore（影子試播共用）；這裡只負責計時、切換與緩衝。
class LocomotionPlanner {
public:
  using Output = PlannerOutput;
  using TargetBuffer = realtime_tools::RealtimeBuffer<Output>;

  struct Progress {
    // 播放中的 Motion 的序號（set_motion() 的回傳值）；0 表示 start() 之後還沒有開始播放任何 Motion
    uint64_t sequence{0};
    MotionStatus status{MotionStatus::kRunning};
    // 進入這個 Motion 之後經過的時間（秒）
    double elapsed{0.0};
  };
  // 在 planner thread 中每週期呼叫一次，填入量測值與 IMU（commanded_positions 由 planner 填入）。
  // 傳入的 RobotState 已依介面數配置好大小
  using StateReader = std::function<void(RobotState &)>;

  // num_interfaces 必須 > 0；rate_hz 必須 > 0
  LocomotionPlanner(double rate_hz, size_t num_interfaces);
  ~LocomotionPlanner();

  LocomotionPlanner(const LocomotionPlanner &) = delete;
  LocomotionPlanner & operator=(const LocomotionPlanner &) = delete;

  // 切換動作；motion 的 max_steps 長度必須等於介面數。
  // 切換後 motion 只會在 planner thread 中被呼叫，呼叫端不可同時使用它。
  // 回傳這次設定的序號（> 0，遞增）；同一個 motion 再設定一次會重新 on_enter()
  uint64_t set_motion(std::shared_ptr<Motion> motion);

  // 目前播放中的 Motion 的進度，可以在任何 non-RT thread 呼叫
  Progress progress() const;

  // 以 initial_positions 為起點開始規劃，結果寫入 sink（呼叫端需保證 sink 在 stop() 前有效）。
  // read_state 可以為空（量測值保持 NaN）。還沒有設定 motion 時保持起點
  void start(
    const std::vector<double> & initial_positions, TargetBuffer * sink,
    StateReader read_state = nullptr);

  // 停止規劃 thread 並等待結束；未啟動時不做任何事
  void stop();

  bool is_running() const {return running_.load();}

  // 最近一個規劃週期開始時的狀態（量測值、IMU，以及 Motion 看到的 commanded_positions / velocities），
  // 影子試播以它為起點。start() 之後第一個週期之前、或未啟動時為 nullopt。可以在任何 non-RT thread 呼叫
  std::optional<RobotState> snapshot() const;
  double rate_hz() const {return rate_hz_;}
  size_t num_interfaces() const {return num_interfaces_;}

private:
  void loop();

  const double rate_hz_;
  const size_t num_interfaces_;

  std::mutex motion_mutex_;
  std::shared_ptr<Motion> pending_motion_;  // 等待 planner thread 切換
  uint64_t pending_sequence_{0};            // pending_motion_ 的序號
  uint64_t last_sequence_{0};               // 最後發出的序號

  mutable std::mutex progress_mutex_;
  Progress progress_;

  mutable std::mutex snapshot_mutex_;
  RobotState snapshot_;          // 最近一個週期開始時的狀態（含 commanded_*）
  bool has_snapshot_{false};

  // 只在規劃 thread 中存取
  PlannerCore core_;
  uint64_t active_sequence_{0};  // core_ 中 Motion 的序號
  RobotState state_;
  TargetBuffer * sink_{nullptr};
  StateReader read_state_;

  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__LOCOMOTION_PLANNER_HPP_
