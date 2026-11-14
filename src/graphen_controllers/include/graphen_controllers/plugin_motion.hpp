#ifndef GRAPHEN_CONTROLLERS__PLUGIN_MOTION_HPP_
#define GRAPHEN_CONTROLLERS__PLUGIN_MOTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "graphen_controllers/motion.hpp"
#include "graphen_locomotion/locomotion_algorithm.hpp"

namespace graphen::control {

// 把外部的 LocomotionAlgorithm（pluginlib plugin）包成 Motion，交給 LocomotionPlanner 播放。
//
// 演算法在 on_enter() / update() 丟出例外時，不讓例外離開 planner thread：
// 記錄一次 ERROR，之後全部保持位置，直到下一次進入這個模式（重新 on_enter）；此時 status() 為 kFailed。
// 演算法的 finished() 為 true 時 status() 為 kFinished。
class PluginMotion : public Motion {
public:
  // algorithm 必須已經 configure；max_steps 不可為空且每個值必須 > 0。
  // name 只用在 log 訊息中（例如模式名稱）
  PluginMotion(
    std::shared_ptr<locomotion::LocomotionAlgorithm> algorithm,
    std::vector<double> max_steps, std::string name, locomotion::Logger log);

  void on_enter(const RobotState & state) override;
  void update(double dt, const RobotState & state, JointCommand & command) override;
  const std::vector<double> & max_steps() const override {return max_steps_;}
  MotionStatus status() const override {
    if (faulted_) {
      return MotionStatus::kFailed;
    }
    return finished_ ? MotionStatus::kFinished : MotionStatus::kRunning;
  }

private:
  // 記錄演算法的例外並進入保持狀態
  void fail(const char * stage, const char * what);

  const std::shared_ptr<locomotion::LocomotionAlgorithm> algorithm_;
  const std::vector<double> max_steps_;
  const std::string name_;
  const locomotion::Logger log_;
  bool faulted_{false};
  bool finished_{false};  // 最近一次 update() 之後演算法的 finished()
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__PLUGIN_MOTION_HPP_
