#ifndef GRAPHEN_CONTROLLERS__PLUGIN_FACTORY_HPP_
#define GRAPHEN_CONTROLLERS__PLUGIN_FACTORY_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "graphen_controllers/motion.hpp"
#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "graphen_locomotion/parameter_source.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/logger.hpp"

namespace graphen::control {

// 演算法的 log 輸出到 logger（rclcpp 的 log 可在任何 thread 呼叫）
locomotion::Logger make_algorithm_logger(const rclcpp::Logger & logger);

// 建立並 configure graphen_locomotion::LocomotionAlgorithm plugin，包成 Motion（PluginMotion）。
// 系統模式（type: plugin）、編舞與 policy 共用。
//
// 持有 pluginlib 的 loader：已建立的演算法實例需要它載入的 library，所以這個物件必須比所有
// 建立出來的 Motion 活得久（宣告在持有 Motion 的成員之前，reconfigure 時也不重建）。
// create() 可以在任何 non-RT thread 呼叫（內部以 mutex 保護）。
class PluginFactory {
public:
  // 所有演算法共用的設定（locomotion::Context 中與模式無關的部分）
  struct Setup {
    std::vector<std::string> joint_names;
    double rate_hz{0.0};
    std::string robot_description;  // 沒有時為空字串
    rclcpp::Logger logger{rclcpp::get_logger("plugin_factory")};
  };

  struct Created {
    std::shared_ptr<locomotion::LocomotionAlgorithm> algorithm;  // 已 configure
    std::shared_ptr<Motion> motion;
  };

  void configure(Setup setup);

  // plugin：pluginlib 的類別名稱；params：演算法的參數；name：log 中的名稱（也是 logger 的子名稱）；
  // max_steps：這個 Motion 的每週期最大移動量（長度必須等於 joint 數，每個值 > 0）。
  // plugin 不存在或演算法 configure 失敗時丟出例外
  Created create(
    const std::string & plugin, const locomotion::ParameterSource & params,
    const std::string & name, std::vector<double> max_steps);

private:
  std::mutex mutex_;
  std::unique_ptr<pluginlib::ClassLoader<locomotion::LocomotionAlgorithm>> loader_;
  Setup setup_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__PLUGIN_FACTORY_HPP_
