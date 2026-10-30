#ifndef GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__POSITION_CONTROLLER_HPP_
#define GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "graphen_control_utils/state_interface_publisher.hpp"
#include "graphen_controllers/sub_group_controller/linear_interpolator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graphen::control {

// 位置模式的子群組 chainable controller：導出 reference interface 給上層 controller 寫入，
// 再把 reference 寫到底層硬體的 command interface（通常是 position）。
//
// 參數：
//   input_interfaces        導出的 reference interface 名稱（全名為 <node_name>/<input>）
//   output_interfaces       要寫入的硬體 command interface
//   reference_publish_rate  ~/references 的發布頻率 (Hz)
//   output_enabled          false 時不寫 command，硬體保留最後的 command，關節維持在最後狀態
//                           （例如停用會碰撞或故障的部位）；只在 configure 時讀取
//
// input 只有 1 個時寫到所有 output（廣播）；否則 input 與 output 數量必須相同，第 i 個 reference
// 寫到第 i 個 output。所有 reference 皆發布到 ~/references 供觀察。
//
// 另外導出 <node_name>/sample_period：上游輸出點的間隔（秒），以 LinearInterpolator 插補。
// activate 時 reference 以對應 output 目前的 command 值初始化，上層可據此保持位置。
class PositionController : public controller_interface::ChainableControllerInterface {
public:
  PositionController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_reference_from_subscribers() override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

private:
  static constexpr const char * kSamplePeriodInterface = "sample_period";

  std::vector<std::string> input_interfaces_;
  std::vector<std::string> output_interfaces_;
  bool chained_mode_{false};  // 是否由上層 controller 透過 reference interface 驅動
  bool output_enabled_{true};
  std::vector<LinearInterpolator> interpolators_;  // 與 output_interfaces_ 對應

  // 將所有 reference 值發布至 ~/references
  std::unique_ptr<StateInterfacePublisher> reference_publisher_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__POSITION_CONTROLLER_HPP_
