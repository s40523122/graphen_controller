#ifndef GRAPHEN_CONTROLLERS__SHADOW_RUNNER_HPP_
#define GRAPHEN_CONTROLLERS__SHADOW_RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "graphen_controllers/motion.hpp"

namespace graphen::control {

// 影子試播的結果
struct ShadowResult {
  bool success{false};
  std::string message;
  // 模擬的播放時間（秒）：播完、失敗或逾時的時間點
  double duration{0.0};
  // 取樣的軌跡（planner 的輸出位置），times 與 positions 一一對應；第一筆為起點（t = 0）
  std::vector<double> times;
  std::vector<std::vector<double>> positions;
};

// 影子試播：以 start（LocomotionPlanner::snapshot()）為起點，用 PlannerCore（和實際播放同一份計算）
// 以固定步長 1 / rate_hz 空跑 motion，直到播完（kFinished）、失敗（kFailed）或超過 max_duration。
//
// 理想追蹤：每一步令量測值 = 命令值（measured_* := commanded_*），IMU、base、速度指令與末端目標
// 都沿用 start 的值。只適合開環的動作（編舞），閉環的演算法在這個假設下沒有意義。
// 不碰任何 interface，也不經過 PlaybackArbiter；呼叫端的 thread 中同步執行。
// sample_rate（Hz）為軌跡的取樣頻率，<= 0 時不記錄軌跡（只有起點與終點）。
// motion 必須是新建立、沒有在 planner 中使用的實例。
ShadowResult run_shadow(
  const std::shared_ptr<Motion> & motion, const locomotion::RobotState & start,
  double rate_hz, double max_duration, double sample_rate);

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SHADOW_RUNNER_HPP_
