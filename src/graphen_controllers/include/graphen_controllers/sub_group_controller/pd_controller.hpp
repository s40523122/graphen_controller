#ifndef GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__PD_CONTROLLER_HPP_
#define GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__PD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "graphen_control_utils/state_interface_publisher.hpp"
#include "graphen_controllers/sub_group_controller/linear_interpolator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graphen::control {

// PD 模式的子群組 chainable controller：以關節 PD 計算力矩，寫到硬體的 effort command。
//
//   τ = kp·(q* − q) + kd·(q̇* − q̇)，再限制在 ±effort_limit
//
// 導出的 reference interface（全名為 <node_name>/<名稱>）：
//   <joint>           期望角度 q*（rad），名稱與 PositionController 相同，上層不需要改接線
//   <joint>/velocity  期望速度 q̇*（rad/s）；NaN 時為 0
//   <joint>/kp        kp（Nm/rad）；NaN 或負值時用該關節的預設 kp
//   <joint>/kd        kd（Nm·s/rad）；NaN 或負值時用該關節的預設 kd
//   sample_period     上游輸出點的間隔（秒），q* 以 LinearInterpolator 插補
//
// 讀取 <joint>/position、<joint>/velocity state，寫入 <joint>/effort command。
// activate 時 q* 以目前量測角度初始化，以預設增益保持在原地。
// 量測值無效（NaN）的關節輸出 0 力矩。
//
// 參數：
//   joints                  關節名稱
//   joint_gains.<joint>.kp / .kd       各關節的預設增益（≥ 0，必填）
//   joint_gains.<joint>.effort_limit   各關節的力矩上限（> 0，必填）
//   reference_publish_rate  ~/references 的發布頻率 (Hz)
//
// 增益以關節為 key，是為了讓模型套件提供一份與 controller 分組無關的增益檔
// （以 /** 套到所有子群組，各自只讀自己的 joints）；子群組怎麼分組都不必改增益檔。
//
// TODO(verify-on-hw): 目前只用於 MuJoCo 模擬。實機（Ti5Hardware）沒有 effort 介面，
// 需先驗證馬達電流模式、力矩常數與 CAN 上可達到的迴圈頻率
class PdController : public controller_interface::ChainableControllerInterface {
public:
  PdController() = default;

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

  // reference_interfaces_ 的排列：[位置 ×N][速度 ×N][kp ×N][kd ×N][sample_period]
  size_t position_ref(size_t i) const {return i;}
  size_t velocity_ref(size_t i) const {return joints_.size() + i;}
  size_t kp_ref(size_t i) const {return 2 * joints_.size() + i;}
  size_t kd_ref(size_t i) const {return 3 * joints_.size() + i;}
  size_t sample_period_ref() const {return 4 * joints_.size();}

  std::vector<std::string> joints_;
  std::vector<double> default_kp_;
  std::vector<double> default_kd_;
  std::vector<double> effort_limits_;
  bool chained_mode_{false};
  std::vector<LinearInterpolator> interpolators_;  // 與 joints_ 對應

  std::unique_ptr<StateInterfacePublisher> reference_publisher_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__PD_CONTROLLER_HPP_
