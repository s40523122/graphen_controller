#ifndef GRAPHEN_LOCOMOTION__LOCOMOTION_ALGORITHM_HPP_
#define GRAPHEN_LOCOMOTION__LOCOMOTION_ALGORITHM_HPP_

#include <functional>
#include <string>
#include <vector>

#include "graphen_locomotion/parameter_source.hpp"
#include "graphen_locomotion/robot_state.hpp"

namespace graphen::locomotion {

enum class LogLevel {kDebug, kInfo, kWarn, kError};

// 由 controller 提供的 log 函式（輸出到 controller 的 ROS logger），可在任何 thread 呼叫
using Logger = std::function<void(LogLevel, const std::string &)>;

// configure() 時提供給演算法的設定
struct Context {
  // 受控 joint 的名稱，順序即 RobotState / JointCommand 中 vector 的 index
  std::vector<std::string> joint_names;
  // update() 的呼叫頻率（Hz）；實際間隔以 update() 的 dt 為準
  double rate_hz{0.0};
  // URDF（controller 有 robot_description 參數時），否則為空字串
  std::string robot_description;
  // 已套用 "<mode>." 前綴的參數；只在 configure() 期間有效，不可保存參考
  const ParameterSource & params;
  // 可以複製保存，在 update() 中使用
  Logger log;
  // 這個模式實際生效的每週期最大移動量（<mode>.max_steps，沒有設定時為 planner_max_steps），
  // 順序同 joint_names。演算法可用來確認自己的輸出不會被限速；不由 controller 建立時可能為空
  std::vector<double> max_steps{};
};

// 外部 locomotion 演算法的 plugin 基類（pluginlib）。
//
// controller 以 YAML 的 <mode>.type: plugin、<mode>.plugin: <類別名稱> 建立，一個模式一個實例。
// 呼叫順序：
//   configure()：controller configure 時呼叫一次（controller 的 thread）。設定錯誤時丟出例外，
//                controller 會 configure 失敗
//   on_enter()：每次切換到這個模式時，在 planner thread 中呼叫
//   update()：之後每個規劃週期在 planner thread 中呼叫（non-RT，但應避免長時間阻塞）
// on_enter() / update() 都只在 planner thread 中呼叫，不需要額外的同步。
// on_enter() / update() 丟出例外時，該模式改為保持位置，直到下一次進入這個模式。
//
// update() 的輸出會再經過 planner 的 <mode>.max_steps 限制（每週期最大移動量）才送往下游。
//
// 有終點的演算法（例如播放一段動作）可以覆寫 finished()，讓 controller 回報「這個模式播完了」
// （SwitchMode action 的 wait_until_finished）；持續執行的演算法（步行、policy）不需要覆寫。
class LocomotionAlgorithm {
public:
  virtual ~LocomotionAlgorithm() = default;

  virtual void configure(const Context & context) = 0;

  // state：進入模式時的狀態；同一個實例可以重複進入，需在這裡重設內部狀態
  virtual void on_enter(const RobotState & state) = 0;

  // dt：距離上一個週期的實際時間（秒）。
  // command 進來時已填滿 NaN（長度為 joint 數），只需要寫入要控制的 joint
  virtual void update(double dt, const RobotState & state, JointCommand & command) = 0;

  // 這次進入模式後動作是否已經完成（之後的 update() 只是保持）。
  // 每次 update() 之後在 planner thread 中呼叫；on_enter() 之後要回到 false
  virtual bool finished() const {return false;}
};

}  // namespace graphen::locomotion

#endif  // GRAPHEN_LOCOMOTION__LOCOMOTION_ALGORITHM_HPP_
