#ifndef GRAPHEN_CONTROLLERS__MOTION_HPP_
#define GRAPHEN_CONTROLLERS__MOTION_HPP_

#include <vector>

#include "graphen_locomotion/robot_state.hpp"

namespace graphen::control {

using locomotion::JointCommand;
using locomotion::RobotState;

// LocomotionPlanner 播放的動作（一個模式對應一個 Motion）。
//
// 只在 planner thread 中呼叫：切換到這個動作時呼叫一次 on_enter()，之後每個規劃週期呼叫 update()。
// planner 會以 max_steps() 限制每週期的變化量後再輸出，所以 update() 回傳的期望位置可以跳動。
// state 含量測的關節角度 / 速度、IMU，以及 planner 目前的輸出位置（commanded_positions）。
// 同一個實例可以重複進入，on_enter() 需重設內部狀態。
class Motion {
public:
  virtual ~Motion() = default;

  virtual void on_enter(const RobotState & state) = 0;

  // dt：距離上一個週期的實際時間（秒）。
  // command 進來時已填滿 NaN（長度等於介面數）；positions 為 NaN 表示保持目前位置，
  // velocities 為 NaN 表示沒有速度前饋
  virtual void update(double dt, const RobotState & state, JointCommand & command) = 0;

  // 每個介面每週期最多移動的量；planner 在每次 update() 之後讀取，可以依動作的階段改變
  virtual const std::vector<double> & max_steps() const = 0;

  // 輸出點之間的時間間隔（秒），planner 在每次 update() 之後讀取並傳給下游插補；
  // 0 表示輸出已經是連續的，下游不插補
  virtual double sample_period() const {return 0.0;}
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__MOTION_HPP_
