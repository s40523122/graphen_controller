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
#include "graphen_interfaces/action/switch_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"

namespace graphen::control {

// 頂層切換 controller：依目前模式，寫入下游 SubGroupController 的 reference interface。
//
// 模式只決定「目標位置」；實際輸出由 LocomotionPlanner 以 planner_rate 的頻率、
// 每週期最多 planner_max_steps 的幅度往目標內插，寫入 target_buffer_，
// update() 只讀取最新值輸出。
//
// 參數：
//   command_interfaces       下游 reference interface（例如 arm_group_controller/position）
//   state_interfaces         要讀取並發布的 state interface
//   sensor                   （選填）sensor state interface，會自動加入 state_interfaces
//   state_publish_rate       ~/states 的發布頻率 (Hz)
//   modes                    可用的模式名稱，同時作為下面各模式參數的前綴
//                            （只允許 [A-Za-z0-9_]）
//   <mode>.targets           該模式的目標位置，長度 = command_interfaces 數量
//   default_mode             configure 時的初始模式名稱
//   planner_rate             planner 的規劃頻率 (Hz)
//   planner_max_steps        每個 command interface 在每個規劃週期最多移動的量
//   target_state_interfaces  與 command_interfaces 一一對應的 state interface，
//                            activate 時作為 planner 的起點
//
// 介面：
//   ~/mode        (std_msgs/String, 訂閱)                切換模式
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
  // 依模式取得各 command interface 的目標值；未知模式回傳 nullptr
  const std::vector<double> * targets_for_mode(const std::string & mode) const;

  // 切換模式並更新 planner 的目標（non-RT）；未知模式時目標全部設為 0
  void apply_mode(const std::string & mode);

  // action server callback（皆在 executor thread 執行）
  rclcpp_action::GoalResponse handle_switch_mode_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SwitchModeAction::Goal> goal);
  rclcpp_action::CancelResponse handle_switch_mode_cancel(
    std::shared_ptr<SwitchModeGoalHandle> goal_handle);
  void handle_switch_mode_accepted(std::shared_ptr<SwitchModeGoalHandle> goal_handle);

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  // mode_names_[i] 對應 mode_targets_[i]，於 on_configure 建立，之後唯讀
  std::vector<std::string> mode_names_;
  std::vector<std::vector<double>> mode_targets_;
  std::vector<double> zero_targets_;  // 未知模式時的目標
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
