#ifndef GRAPHEN_CONTROLLERS__TARGET_MOTION_HPP_
#define GRAPHEN_CONTROLLERS__TARGET_MOTION_HPP_

#include <vector>

#include "graphen_controllers/motion.hpp"

namespace graphen::control {

// 往一組固定目標移動。targets 中為 NaN 的介面保持進入此動作時的位置。
// 輸出到達全部目標時 status() 為 kFinished（全部為 NaN 的保持動作會立即完成）。
class TargetMotion : public Motion {
public:
  // targets 與 max_steps 長度必須相同且不為空，每個 max_step 必須 > 0
  TargetMotion(std::vector<double> targets, std::vector<double> max_steps);

  void on_enter(const RobotState & state) override;
  void update(double dt, const RobotState & state, JointCommand & command) override;
  const std::vector<double> & max_steps() const override {return max_steps_;}
  MotionStatus status() const override {
    return reached_ ? MotionStatus::kFinished : MotionStatus::kRunning;
  }

private:
  const std::vector<double> targets_;
  const std::vector<double> max_steps_;
  std::vector<double> desired_;  // targets_ 的 NaN 以進入時的位置取代
  bool reached_{false};          // planner 的輸出已到達 desired_
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__TARGET_MOTION_HPP_
