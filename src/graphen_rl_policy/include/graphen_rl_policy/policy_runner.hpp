#ifndef GRAPHEN_RL_POLICY__POLICY_RUNNER_HPP_
#define GRAPHEN_RL_POLICY__POLICY_RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "graphen_rl_policy/onnx_model.hpp"

namespace graphen::rl_policy {

// 子類組 obs 時可用的輸入；關節量測已依 policy 的關節順序排好
struct PolicyInput {
  const locomotion::RobotState & state;
  const std::vector<double> & joint_positions;   // rad，policy 順序
  const std::vector<double> & joint_velocities;  // rad/s，policy 順序
  // 這次推論之前已經推論過的時間（推論次數 × policy_dt），可用於步態相位
  double policy_time;
};

// ONNX policy plugin 的共用基類。一個 policy 家族（相同的 obs / action 組法）一個子類，
// 子類只實作「組 obs」與「action → 關節目標」，其餘流程在這裡：
//
//   on_enter：以 entry_speed 從目前命令位置插補到 default_joint_pos（fixed_joints 到 fixed_positions），
//             到達後才開始推論，避免 policy 第一個輸出離目前姿態太遠
//   推論    ：dt 累積滿 policy_dt 才推論一次，其餘週期輸出上一次的目標
//   保持    ：機身狀態無效或關節量測有 NaN 時不輸出位置（planner 保持），恢復後繼續
//   增益    ：有設定 kp / kd 時每週期輸出（下游為 PdController 才生效）
//
// 共用參數（<mode>.*）：
//   model_path          onnx 路徑（必填），可寫 package://<套件>/<相對路徑>
//   model_sha256        onnx 的 sha256（必填），用來確認 onnx 與這份設定是同一次訓練的產物
//   joints              policy 的關節名稱，依 policy 的 obs / action 順序（必填）
//   name_map            （選填）"policy名稱:controller名稱"，沒有列出的視為相同
//   default_joint_pos   與 joints 對應的預設姿態（rad，必填）
//   policy_dt           推論間隔（秒，必填），必須 >= planner 週期
//   kp / kd             （選填）與 joints 對應的 PD 增益；省略時使用下游的預設值
//   fixed_joints        （選填）policy 不控制、但進入時要固定的關節（controller 名稱）
//   fixed_positions     與 fixed_joints 對應的角度（rad）
//   entry_speed         進入時走向預設姿態的速度（rad/s，預設 0.5；0 表示不插補，直接開始推論）
//   log_period          > 0 時每隔這麼久（秒）log 一次指令、重力投影與推論時間（預設 0）
class PolicyRunner : public locomotion::LocomotionAlgorithm {
public:
  void configure(const locomotion::Context & context) final;
  void on_enter(const locomotion::RobotState & state) final;
  void update(
    double dt, const locomotion::RobotState & state, locomotion::JointCommand & command) final;

protected:
  // log 與例外訊息使用的名稱
  virtual std::string name() const = 0;

  // 讀取子類自己的參數（共用參數已讀完，可以使用下面的 accessor），回傳 obs 的維度。
  // 設定錯誤時丟出 std::invalid_argument
  virtual size_t configure_policy(const locomotion::ParameterSource & params) = 0;

  // 開始推論前呼叫（每次進入模式、插補到預設姿態之後），重設 last action、history 等內部狀態
  virtual void reset_policy() = 0;

  // 組出 obs（長度為 configure_policy() 的回傳值）
  virtual void build_obs(const PolicyInput & input, float * obs) = 0;

  // policy 輸出（長度為 num_joints()）→ 關節目標角度（rad，policy 順序）
  virtual void action_to_targets(const float * action, double * targets) = 0;

  size_t num_joints() const {return joint_names_.size();}
  const std::vector<double> & default_positions() const {return default_positions_;}
  void log(locomotion::LogLevel level, const std::string & message) const;

private:
  enum class Phase {kEntering, kRunning};

  void start_running();
  bool inputs_valid(const locomotion::RobotState & state) const;
  void infer(const locomotion::RobotState & state);
  void write_gains(locomotion::JointCommand & command) const;
  void log_status(double dt, const locomotion::RobotState & state);

  locomotion::Logger log_;
  std::unique_ptr<OnnxModel> model_;

  // policy 關節（policy 順序）
  std::vector<std::string> joint_names_;
  std::vector<size_t> joint_indices_;  // → controller joint index
  std::vector<double> default_positions_;
  std::vector<double> kp_;  // 空的表示不指定
  std::vector<double> kd_;
  // policy 不控制、進入時固定的關節
  std::vector<size_t> fixed_indices_;
  std::vector<double> fixed_positions_;

  double policy_dt_{0.0};
  double entry_speed_{0.5};
  double log_period_{0.0};

  // 執行狀態（只在 planner thread 中存取）
  Phase phase_{Phase::kEntering};
  std::vector<double> entry_start_;     // policy 關節 + fixed 關節（依序）
  std::vector<double> entry_goal_;
  double entry_duration_{0.0};
  double entry_elapsed_{0.0};

  std::vector<float> obs_;
  std::vector<float> action_;
  std::vector<double> targets_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  bool has_target_{false};
  bool holding_{false};
  double policy_time_{0.0};
  double since_inference_{0.0};
  double since_log_{0.0};
};

}  // namespace graphen::rl_policy

#endif  // GRAPHEN_RL_POLICY__POLICY_RUNNER_HPP_
