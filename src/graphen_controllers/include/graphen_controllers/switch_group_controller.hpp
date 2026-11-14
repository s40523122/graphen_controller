#ifndef GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
#define GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "graphen_control_utils/state_interface_publisher.hpp"
#include "graphen_control_utils/try_lock_buffer.hpp"
#include "graphen_controllers/locomotion_planner.hpp"
#include "graphen_controllers/motion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "graphen_interfaces/action/switch_mode.hpp"
#include "graphen_locomotion/base_state_estimator.hpp"
#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"

namespace graphen::control {

// 頂層切換 controller：依目前模式，寫入下游子群組 controller（PositionController / PdController）
// 的 reference interface。
//
// 每個模式對應一個 Motion（固定目標、BVH 動作或外部演算法 plugin），由 LocomotionPlanner 在 non-RT thread 中以
// planner_rate 播放、每週期最多移動 max_steps，寫入 target_buffer_；update() 只讀取最新值輸出。
// controller 本身只認得模式名稱，不包含任何特定模式的邏輯。
// update() 另外把量測的關節角度 / 速度與 IMU 寫入 state_buffer_。planner 每週期讀取後，
// 補上 ~/cmd_vel 的速度指令、執行機身狀態估測器（base_estimator），再交給 Motion。
//
// 參數：
//   command_interfaces       下游 reference interface（例如 left_arm_group_controller/L_SHOULDER_P）
//   sample_period_interfaces （選填）下游的 sample_period reference（例如 left_arm_group_controller/sample_period），
//                            每週期寫入目前 Motion 的輸出點間隔，供下游插補
//   velocity_command_interfaces / kp_command_interfaces / kd_command_interfaces
//                            （選填）與 command_interfaces 一一對應的下游速度前饋與 PD 增益 reference
//                            （例如 left_arm_group_controller/L_SHOULDER_P/kp），下游為 PdController 時設定。
//                            每週期寫入 Motion 的輸出，沒有指定的值為 NaN（下游使用預設值）
//   state_interfaces         要讀取並發布的 state interface
//   sensor                   （選填）sensor state interface，會自動加入 state_interfaces
//   state_publish_rate       ~/states 的發布頻率 (Hz)
//   target_state_interfaces  與 command_interfaces 一一對應的 state interface，activate 時作為
//                            planner 的起點，也是交給 Motion 的量測角度；
//                            其 prefix（最後一個 '/' 之前）即為該介面的 joint 名稱
//   velocity_state_interfaces（選填）與 command_interfaces 一一對應的速度 state interface，
//                            交給 Motion 的量測速度；空的時候為 NaN。會自動加入 state_interfaces
//   imu_sensor_name          （選填）IMU sensor 名稱（例如 imu_sensor），讀取 <name>/orientation.x …
//                            linear_acceleration.z 共 10 個 state interface 交給 Motion，
//                            會自動加入 state_interfaces
//   planner_rate             planner 的規劃頻率 (Hz)
//   planner_max_steps        每個 command interface 在每個規劃週期最多移動的量（各模式的預設值）
//   modes                    可用的模式名稱，同時作為下面各模式參數的前綴（只允許 [A-Za-z0-9_]）
//   <mode>.type              target（預設）、hold 或 bvh
//   <mode>.max_steps         （選填）該模式的每週期最大移動量（bvh 為播放時），預設為 planner_max_steps
//   <mode>.interruptible     （選填，預設 true）false 時，這個模式播放中（尚未結束）拒絕切換到其他模式
//                            或重播，只有 preempt 的模式可以切入；播完或失敗之後恢復可切換
//   <mode>.preempt           （選填，hold 預設 true，其他預設 false）true 時可以切入不可中斷的模式，
//                            用於停止類的模式（安全底線）
//   type: target
//     <mode>.joints          （選填）要移動的 joint；省略時為全部，依 command_interfaces 順序
//     <mode>.targets         對應 joints 的目標位置；沒有列出的 joint 保持進入模式時的位置
//   base_estimator.plugin    （選填）graphen_locomotion::BaseStateEstimator 的 plugin 類別名稱；
//                            其餘 base_estimator.* 參數由估測器自行讀取。空字串表示不估測（base.valid 為 false）
//   command_timeout          ~/cmd_vel 的逾時（秒）；超過時速度指令視為 0（command.active 為 false）
//   type: hold               保持進入模式時的位置（停止），沒有其他參數
//   type: plugin
//     <mode>.plugin          graphen_locomotion::LocomotionAlgorithm 的 plugin 類別名稱；
//                            其餘 <mode>.* 參數由演算法自行讀取
//   type: bvh
//     <mode>.file            BVH 檔案路徑，支援 package://<pkg>/<path>
//     <mode>.loop            播放到結尾後是否循環（預設 true）
//     <mode>.time_scale      播放速度倍率（預設 1.0）
//     <mode>.approach_max_steps  （選填）走向第 0 幀時的每週期最大移動量，預設為 planner_max_steps
//   bvh.base_link            BVH 與 URDF 共同的基準 link（使用 bvh 模式時必填）
//   bvh.axis_tolerance / bvh.max_iterations / bvh.damping / bvh.residual_tolerance
//                            BvhRetargeter 的分組與求解設定
//   default_mode             configure 時的初始模式名稱；空字串表示不進入任何模式，
//                            activate 時保持目前位置，直到收到第一個模式命令
//   switch_mode_poll_rate    檢查「等待模式播完」的 switch_mode goal 的頻率 (Hz)，也是 feedback 的頻率
//   robot_description        （使用 bvh 模式時必要）URDF，由 launch 傳給 controller_manager
//
// 介面：
//   ~/mode        (std_msgs/String, 訂閱)                切換模式；未知模式會被忽略
//   ~/cmd_vel     (geometry_msgs/Twist, 訂閱)            機身速度指令（linear.x、linear.y、angular.z）
//   ~/switch_mode (graphen_interfaces/action/SwitchMode) 切換模式，result 附 state 快照；
//                                                         wait_until_finished 時等動作播完才回傳
//   ~/states      (control_msgs/DynamicJointState)        發布所有 state interface
class SwitchGroupController : public controller_interface::ControllerInterface {
public:
  using SwitchModeAction = graphen_interfaces::action::SwitchMode;
  using SwitchModeGoalHandle = rclcpp_action::ServerGoalHandle<SwitchModeAction>;

  SwitchGroupController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // 模式的中斷設定（<mode>.interruptible / <mode>.preempt）
  struct ModePolicy {
    bool interruptible{true};
    bool preempt{false};
  };

  // 依模式名稱取得 Motion；未知模式回傳 nullptr
  std::shared_ptr<Motion> motion_for_mode(const std::string & mode) const;
  // 依模式名稱取得中斷設定；未知模式回傳 nullptr
  const ModePolicy * policy_for_mode(const std::string & mode) const;

  // 讀取 <mode>.* 參數並建立該模式的 Motion；設定錯誤時丟出例外
  std::shared_ptr<Motion> create_motion(const std::string & mode);

  // 讀取 <mode>.plugin 並建立、configure 外部演算法；設定錯誤時丟出例外
  std::shared_ptr<Motion> create_plugin_motion(
    const std::string & mode, std::vector<double> max_steps);

  using OnSameMode = int8_t;  // SwitchModeAction::Goal::ON_SAME_MODE_*

  struct ModeRequest {
    enum class Outcome {
      kSwitched,   // 已切換（或重播）
      kUnchanged,  // 已在該模式，依 on_same_mode 不重新開始
      kRejected,   // 目前的模式不可中斷
      kUnknown,    // 未知的模式
    };
    Outcome outcome{Outcome::kUnknown};
    uint64_t sequence{0};  // 目前播放的序號（kSwitched / kUnchanged 時有效）
    std::string message;   // kRejected / kUnknown 的原因
  };

  // 切換模式並交給 planner 播放（non-RT）。已在該模式時依 on_same_mode 決定是否重新開始；
  // 目前的模式不可中斷且還在播放時，除非目標模式是 preempt，否則拒絕
  ModeRequest apply_mode(const std::string & mode, OnSameMode on_same_mode);

  // 目前這次播放是否已經結束（完成或失敗）。呼叫端需持有 mode_mutex_
  bool current_play_ended() const;

  // 以目前的模式與 state 快照建立 action result
  std::shared_ptr<SwitchModeAction::Result> make_switch_mode_result(
    bool success, const std::string & message);

  // 定時（executor thread）檢查等待中的 goal：播完、失敗、被取代、取消或 controller 停用時結束它
  void poll_waiting_goals();

  // 以 state_interfaces_ 的目前值更新量測狀態（不配置記憶體，RT 可呼叫）
  void read_robot_state(RobotState & state) const;

  // planner thread 每週期呼叫：取得量測狀態，補上速度指令，並執行機身狀態估測器
  void fill_planner_state(RobotState & state);

  // 讀取 base_estimator.plugin 並建立、configure 估測器；未設定時回傳 nullptr，設定錯誤時丟出例外
  std::shared_ptr<locomotion::BaseStateEstimator> create_base_estimator();

  // action server callback（皆在 executor thread 執行）
  rclcpp_action::GoalResponse handle_switch_mode_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SwitchModeAction::Goal> goal);
  rclcpp_action::CancelResponse handle_switch_mode_cancel(
    std::shared_ptr<SwitchModeGoalHandle> goal_handle);
  void handle_switch_mode_accepted(std::shared_ptr<SwitchModeGoalHandle> goal_handle);

  // 外部演算法的 plugin loader：必須比 mode_motions_ 與 planner_ 中的演算法實例活得久，所以宣告在最前面
  std::unique_ptr<pluginlib::ClassLoader<locomotion::LocomotionAlgorithm>> algorithm_loader_;
  std::unique_ptr<pluginlib::ClassLoader<locomotion::BaseStateEstimator>> estimator_loader_;

  std::vector<std::string> command_interface_names_;
  // 速度前饋與 PD 增益的下游介面；沒有設定時為空（下游為位置模式）
  std::vector<std::string> velocity_interface_names_;
  std::vector<std::string> kp_interface_names_;
  std::vector<std::string> kd_interface_names_;
  std::vector<std::string> sample_period_interface_names_;
  std::vector<std::string> state_interface_names_;

  // 各 command interface 對應的 joint 名稱（取自 target_state_interfaces）
  std::vector<std::string> joint_names_;
  std::vector<double> default_max_steps_;  // planner_max_steps
  double planner_rate_{0.0};

  // mode_names_[i] 對應 mode_motions_[i]，於 on_configure 建立，之後只有 planner thread 使用 Motion
  std::vector<std::string> mode_names_;
  std::vector<std::shared_ptr<Motion>> mode_motions_;
  std::vector<ModePolicy> mode_policies_;  // 順序同 mode_names_
  std::string default_mode_;

  // ~/mode 與 action 可能在不同 executor thread 同時切換模式
  std::mutex mode_mutex_;
  std::string current_mode_;
  // 目前模式在 planner 中的序號（mode_mutex_ 保護）；與等待中 goal 的序號不同表示已被取代
  uint64_t mode_sequence_{0};

  // wait_until_finished 的 goal，由 wait_timer_ 定時檢查。
  // 鎖的順序：waiting_mutex_ → mode_mutex_ / state_snapshot_mutex_
  struct WaitingGoal {
    std::shared_ptr<SwitchModeGoalHandle> handle;
    uint64_t sequence{0};
  };
  std::mutex waiting_mutex_;
  std::vector<WaitingGoal> waiting_goals_;
  rclcpp::TimerBase::SharedPtr wait_timer_{nullptr};

  std::string sensor_interface_name_;
  std::optional<size_t> sensor_index_;  // sensor 在 state_interfaces_ 中的 index
  // 最新的 sensor 讀值，供後續控制邏輯使用
  double sensor_value_{std::numeric_limits<double>::quiet_NaN()};

  // target_state_interfaces 在 state_interfaces_ 中的 index，順序對應 command_interfaces
  std::vector<size_t> target_state_indices_;
  // velocity_state_interfaces 在 state_interfaces_ 中的 index；沒有設定時為空
  std::vector<size_t> velocity_state_indices_;
  // IMU 10 個 state interface 在 state_interfaces_ 中的 index（orientation.xyzw、
  // angular_velocity.xyz、linear_acceleration.xyz）；沒有設定時為空
  std::vector<size_t> imu_state_indices_;

  // update()（RT）以 try_lock 寫入量測狀態，planner thread 每週期讀取
  TryLockBuffer<RobotState> state_buffer_;

  // 機身狀態估測器；以下狀態在 activate 時設定，之後只在 planner thread 中存取
  // （planner thread 在設定之後才啟動，不需要額外同步）
  std::shared_ptr<locomotion::BaseStateEstimator> base_estimator_;
  bool estimator_needs_reset_{true};
  bool estimator_faulted_{false};
  double last_state_time_{0.0};

  // ~/cmd_vel 的最新指令：subscription callback 寫入、planner thread 讀取，兩者都不是 RT
  struct VelocityCommand {
    std::array<double, 3> velocity{0.0, 0.0, 0.0};  // vx, vy, wz
    std::chrono::steady_clock::time_point stamp;
    bool received{false};
  };
  std::mutex command_mutex_;
  VelocityCommand velocity_command_;
  double command_timeout_{0.5};

  // planner（non-RT thread）寫入，update() 從 RT 端讀取
  std::unique_ptr<LocomotionPlanner> planner_;
  LocomotionPlanner::TargetBuffer target_buffer_;

  // update() 以 try_lock 寫入最新的 state 值，供 action result 讀取快照
  std::mutex state_snapshot_mutex_;
  std::vector<double> state_snapshot_;

  // 將所有 state interface 發布至 ~/states
  std::unique_ptr<StateInterfacePublisher> state_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_{nullptr};
  rclcpp_action::Server<SwitchModeAction>::SharedPtr switch_mode_server_{nullptr};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
