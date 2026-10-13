#ifndef GRAPHEN_LOCOMOTION__BASE_STATE_ESTIMATOR_HPP_
#define GRAPHEN_LOCOMOTION__BASE_STATE_ESTIMATOR_HPP_

#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "graphen_locomotion/robot_state.hpp"

namespace graphen::locomotion {

// 機身狀態估測器的 plugin 基類（pluginlib）。
//
// controller 以 YAML 的 base_estimator.plugin 建立一個實例，在 planner thread 中、演算法之前執行，
// 結果寫入 RobotState::base。估測與演算法分開，換估測器（例如改成 IMU + 腿部運動學）只要改 YAML，
// 演算法不需要修改。
// 呼叫順序：
//   configure()：controller configure 時呼叫一次。Context::params 的前綴為 "base_estimator."；
//                設定錯誤時丟出例外，controller 會 configure 失敗
//   reset()：controller 每次 activate 後、第一次 update() 之前，在 planner thread 中呼叫
//   update()：之後每個規劃週期呼叫，與目前的模式無關（切換模式時估測不中斷）
// update() 丟出例外時，base.valid 為 false，直到下一次 activate。
class BaseStateEstimator {
public:
  virtual ~BaseStateEstimator() = default;

  virtual void configure(const Context & context) = 0;

  // 清除內部狀態（例如積分值）
  virtual void reset() = 0;

  // dt：兩次量測之間的時間（秒，取自 RobotState::time 的差值，可能為 0）。
  // state 的 base 尚未填入；結果寫到 base
  virtual void update(double dt, const RobotState & state, BaseState & base) = 0;
};

}  // namespace graphen::locomotion

#endif  // GRAPHEN_LOCOMOTION__BASE_STATE_ESTIMATOR_HPP_
