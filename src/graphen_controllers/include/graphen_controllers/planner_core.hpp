#ifndef GRAPHEN_CONTROLLERS__PLANNER_CORE_HPP_
#define GRAPHEN_CONTROLLERS__PLANNER_CORE_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "graphen_controllers/motion.hpp"

namespace graphen::control {

// planner 一個週期的輸出
struct PlannerOutput {
  std::vector<double> positions;
  // Motion 的速度前饋（rad/s），NaN 表示沒有
  std::vector<double> velocities;
  // Motion 指定的 PD 增益，NaN 表示使用下游的預設值
  std::vector<double> kp;
  std::vector<double> kd;
  // 目前 Motion 的輸出點間隔（秒），供下游插補；0 表示連續
  double sample_period{0.0};
};

// planner 每個週期的計算（不含計時、thread 與緩衝）：
//   fill_commanded()  把目前的輸出位置與命令速度填入 RobotState（Motion 看到的 commanded_*）
//   enter()           切換到新的 Motion（on_enter）
//   update()          Motion::update → 每個介面最多移動 max_steps → 更新命令速度
//
// LocomotionPlanner（實機播放）與 ShadowRunner（影子試播）共用這一份，
// 讓試播和實際播放走完全相同的限速與速度計算。不是 thread-safe，由呼叫端保證只在一個 thread 中使用。
// reset() 之後週期中只覆寫內容、不配置記憶體（Motion 本身除外）。
class PlannerCore {
public:
  // num_interfaces 必須 > 0
  explicit PlannerCore(size_t num_interfaces);

  // 以 initial_positions 為起點重新開始（長度必須等於介面數）；initial_velocities 為空時命令速度為 0。
  // 清除目前的 Motion
  void reset(
    const std::vector<double> & initial_positions,
    const std::vector<double> & initial_velocities = {});

  // 把目前的輸出位置與命令速度寫入 state.commanded_positions / commanded_velocities
  void fill_commanded(locomotion::RobotState & state) const;

  // 切換到 motion（max_steps 長度必須等於介面數），以 state 呼叫 on_enter；經過時間歸零
  void enter(std::shared_ptr<Motion> motion, const locomotion::RobotState & state);

  // 一個週期：有 Motion 時 update 並限制變化量、累計經過時間；之後更新命令速度（dt <= 0 時保留上一次）
  void update(double dt, const locomotion::RobotState & state);

  const PlannerOutput & output() const {return output_;}
  bool has_motion() const {return static_cast<bool>(motion_);}
  // 沒有 Motion 時為 kRunning
  MotionStatus status() const;
  // 進入目前的 Motion 之後經過的時間（秒）
  double elapsed() const {return elapsed_;}
  size_t num_interfaces() const {return num_interfaces_;}

private:
  const size_t num_interfaces_;
  std::shared_ptr<Motion> motion_;
  double elapsed_{0.0};
  PlannerOutput output_;                    // positions 即目前的輸出位置
  std::vector<double> previous_positions_;  // 上一個週期的輸出位置，用來算命令速度
  std::vector<double> command_velocity_;    // 上一個週期的命令速度
  locomotion::JointCommand command_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__PLANNER_CORE_HPP_
