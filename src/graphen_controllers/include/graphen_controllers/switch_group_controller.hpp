#ifndef GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
#define GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "graphen_control_utils/state_interface_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/int32.hpp"

namespace graphen::control {

// 頂層切換 controller：依目前模式，寫入下游 SubGroupController 的 reference interface。
//
// 參數：
//   command_interfaces  下游 reference interface（例如 arm_group_controller/position）
//   state_interfaces    要讀取並發布的 state interface
//   sensor              （選填）sensor state interface，會自動加入 state_interfaces
//   state_publish_rate  ~/states 的發布頻率 (Hz)
//   modes               可用的模式 id
//   mode_targets        各模式的目標值，依 modes 順序攤平，
//                       長度 = modes 數量 × command_interfaces 數量
//   default_mode        configure 時的初始模式
//
// Topic：
//   ~/mode   (std_msgs/Int32, 訂閱)          切換模式
//   ~/states (control_msgs/DynamicJointState) 發布所有 state interface
class SwitchGroupController : public controller_interface::ControllerInterface {
public:
  SwitchGroupController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // 依模式取得各 command interface 的目標值；未知模式回傳 nullptr
  const std::vector<double> * targets_for_mode(int mode) const;

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  // mode_ids_[i] 對應 mode_targets_[i]，於 on_configure 建立，update() 中只讀不配置
  std::vector<int> mode_ids_;
  std::vector<std::vector<double>> mode_targets_;
  int default_mode_{1};

  std::string sensor_interface_name_;
  std::optional<size_t> sensor_index_;  // sensor 在 state_interfaces_ 中的 index
  // 最新的 sensor 讀值，供後續控制邏輯使用
  double sensor_value_{std::numeric_limits<double>::quiet_NaN()};

  // 將所有 state interface 發布至 ~/states
  std::unique_ptr<StateInterfacePublisher> state_publisher_;

  // ~/mode 由 non-RT callback 寫入 mode_buffer_，update() 從 RT 端讀取
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_subscriber_{nullptr};
  realtime_tools::RealtimeBuffer<int> mode_buffer_;

  int current_mode_{1};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
