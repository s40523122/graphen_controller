#ifndef GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
#define GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "graphen_control_utils/state_interface_publisher.hpp"
#include "graphen_controllers/locomotion_planner.hpp"
#include "graphen_controllers/motion.hpp"
#include "graphen_interfaces/action/switch_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"

namespace graphen::control {

// 頂層切換 controller：依目前模式，寫入下游 SubGroupController 的 reference interface。
//
// 每個模式對應一個 Motion（固定目標或 BVH 動作），由 LocomotionPlanner 在 non-RT thread 中以
// planner_rate 播放、每週期最多移動 max_steps，寫入 target_buffer_；update() 只讀取最新值輸出。
// controller 本身只認得模式名稱，不包含任何特定模式的邏輯。
//
// 參數：
//   command_interfaces       下游 reference interface（例如 left_arm_group_controller/L_SHOULDER_P）
//   sample_period_interfaces （選填）下游的 sample_period reference（例如 left_arm_group_controller/sample_period），
//                            每週期寫入目前 Motion 的輸出點間隔，供下游插補
//   state_interfaces         要讀取並發布的 state interface
//   sensor                   （選填）sensor state interface，會自動加入 state_interfaces
//   state_publish_rate       ~/states 的發布頻率 (Hz)
//   target_state_interfaces  與 command_interfaces 一一對應的 state interface，activate 時作為
//                            planner 的起點；其 prefix（最後一個 '/' 之前）即為該介面的 joint 名稱
//   planner_rate             planner 的規劃頻率 (Hz)
//   planner_max_steps        每個 command interface 在每個規劃週期最多移動的量（各模式的預設值）
//   modes                    可用的模式名稱，同時作為下面各模式參數的前綴（只允許 [A-Za-z0-9_]）
//   <mode>.type              target（預設）或 bvh
//   <mode>.max_steps         （選填）該模式的每週期最大移動量（bvh 為播放時），預設為 planner_max_steps
//   type: target
//     <mode>.joints          （選填）要移動的 joint；省略時為全部，依 command_interfaces 順序
//     <mode>.targets         對應 joints 的目標位置；沒有列出的 joint 保持進入模式時的位置
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
//   robot_description        （使用 bvh 模式時必要）URDF，由 launch 傳給 controller_manager
//
// 介面：
//   ~/mode        (std_msgs/String, 訂閱)                切換模式；未知模式會被忽略
//   ~/switch_mode (graphen_interfaces/action/SwitchMode) 切換模式，result 附 state 快照
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
  // 依模式名稱取得 Motion；未知模式回傳 nullptr
  std::shared_ptr<Motion> motion_for_mode(const std::string & mode) const;

  // 讀取 <mode>.* 參數並建立該模式的 Motion；設定錯誤時丟出例外
  std::shared_ptr<Motion> create_motion(const std::string & mode);

  // 切換模式並交給 planner 播放（non-RT）；未知模式或與目前模式相同時不做任何事
  void apply_mode(const std::string & mode);

  // action server callback（皆在 executor thread 執行）
  rclcpp_action::GoalResponse handle_switch_mode_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SwitchModeAction::Goal> goal);
  rclcpp_action::CancelResponse handle_switch_mode_cancel(
    std::shared_ptr<SwitchModeGoalHandle> goal_handle);
  void handle_switch_mode_accepted(std::shared_ptr<SwitchModeGoalHandle> goal_handle);

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> sample_period_interface_names_;
  std::vector<std::string> state_interface_names_;

  // 各 command interface 對應的 joint 名稱（取自 target_state_interfaces）
  std::vector<std::string> joint_names_;
  std::vector<double> default_max_steps_;  // planner_max_steps

  // mode_names_[i] 對應 mode_motions_[i]，於 on_configure 建立，之後只有 planner thread 使用 Motion
  std::vector<std::string> mode_names_;
  std::vector<std::shared_ptr<Motion>> mode_motions_;
  std::string default_mode_;

  // ~/mode 與 action 可能在不同 executor thread 同時切換模式
  std::mutex mode_mutex_;
  std::string current_mode_;

  std::string sensor_interface_name_;
  std::optional<size_t> sensor_index_;  // sensor 在 state_interfaces_ 中的 index
  // 最新的 sensor 讀值，供後續控制邏輯使用
  double sensor_value_{std::numeric_limits<double>::quiet_NaN()};

  // target_state_interfaces 在 state_interfaces_ 中的 index，順序對應 command_interfaces
  std::vector<size_t> target_state_indices_;

  // planner（non-RT thread）寫入，update() 從 RT 端讀取
  std::unique_ptr<LocomotionPlanner> planner_;
  LocomotionPlanner::TargetBuffer target_buffer_;

  // update() 以 try_lock 寫入最新的 state 值，供 action result 讀取快照
  std::mutex state_snapshot_mutex_;
  std::vector<double> state_snapshot_;

  // 將所有 state interface 發布至 ~/states
  std::unique_ptr<StateInterfacePublisher> state_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_{nullptr};
  rclcpp_action::Server<SwitchModeAction>::SharedPtr switch_mode_server_{nullptr};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
