#include "graphen_controllers/plugin_factory.hpp"

#include <stdexcept>
#include <utility>

#include "graphen_controllers/plugin_motion.hpp"
#include "rclcpp/logging.hpp"

namespace graphen::control {

locomotion::Logger make_algorithm_logger(const rclcpp::Logger & logger) {
  return [logger](locomotion::LogLevel level, const std::string & message) {
           switch (level) {
             case locomotion::LogLevel::kDebug:
               RCLCPP_DEBUG(logger, "%s", message.c_str());
               break;
             case locomotion::LogLevel::kInfo:
               RCLCPP_INFO(logger, "%s", message.c_str());
               break;
             case locomotion::LogLevel::kWarn:
               RCLCPP_WARN(logger, "%s", message.c_str());
               break;
             case locomotion::LogLevel::kError:
               RCLCPP_ERROR(logger, "%s", message.c_str());
               break;
           }
         };
}

void PluginFactory::configure(Setup setup) {
  std::lock_guard<std::mutex> lock(mutex_);
  setup_ = std::move(setup);
}

PluginFactory::Created PluginFactory::create(
  const std::string & plugin, const locomotion::ParameterSource & params,
  const std::string & name, std::vector<double> max_steps)
{
  if (plugin.empty()) {
    throw std::invalid_argument("no algorithm plugin given for [ " + name + " ]");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_steps.size() != setup_.joint_names.size()) {
    throw std::invalid_argument(
            "max_steps of [ " + name + " ] must contain " +
            std::to_string(setup_.joint_names.size()) + " entries, got " +
            std::to_string(max_steps.size()));
  }
  // loader 跨 reconfigure 保留：已建立的演算法實例需要它載入的 library
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<locomotion::LocomotionAlgorithm>>(
      "graphen_locomotion", "graphen::locomotion::LocomotionAlgorithm");
  }
  Created created;
  created.algorithm = loader_->createSharedInstance(plugin);

  const auto log = make_algorithm_logger(setup_.logger.get_child(name));
  const locomotion::Context context{
    setup_.joint_names, setup_.rate_hz, setup_.robot_description, params, log, max_steps,
    setup_.pose_target_names, setup_.pose_target_frame};
  created.algorithm->configure(context);

  created.motion =
    std::make_shared<PluginMotion>(created.algorithm, std::move(max_steps), name, log);
  return created;
}

}  // namespace graphen::control
