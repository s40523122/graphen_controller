#include "graphen_controllers/bvh_source.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "graphen_controllers/bvh_motion.hpp"
#include "graphen_controllers/library_utils.hpp"
#include "graphen_motion/bvh_clip.hpp"
#include "rclcpp/logging.hpp"

namespace graphen::control {

namespace {

constexpr const char * kExtension = ".bvh";

// 空的時候使用預設值，否則長度必須相符
std::vector<double> steps_or_default(
  const rclcpp_lifecycle::LifecycleNode & node, const std::string & name,
  const std::vector<double> & default_steps)
{
  auto steps = node.get_parameter(name).as_double_array();
  if (steps.empty()) {
    return default_steps;
  }
  if (steps.size() != default_steps.size()) {
    throw std::invalid_argument(
            "'" + name + "' must be empty or contain " + std::to_string(default_steps.size()) +
            " entries, got " + std::to_string(steps.size()));
  }
  return steps;
}

}  // namespace

BvhSource::BvhSource(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  std::vector<std::string> joint_names, const std::vector<double> & default_max_steps)
: node_(std::move(node)), joint_names_(std::move(joint_names))
{
  library_path_ = resolve_path(node_->get_parameter("bvh.library_path").as_string());
  if (library_path_.empty()) {
    return;
  }
  max_steps_ = steps_or_default(*node_, "bvh.max_steps", default_max_steps);
  approach_max_steps_ = steps_or_default(*node_, "bvh.approach_max_steps", default_max_steps);
  interruptible_ = node_->get_parameter("bvh.interruptible").as_bool();

  retarget_options_.base_link = node_->get_parameter("bvh.base_link").as_string();
  retarget_options_.axis_tolerance = node_->get_parameter("bvh.axis_tolerance").as_double();
  retarget_options_.max_iterations =
    static_cast<int>(node_->get_parameter("bvh.max_iterations").as_int());
  retarget_options_.damping = node_->get_parameter("bvh.damping").as_double();
  retarget_options_.residual_tolerance =
    node_->get_parameter("bvh.residual_tolerance").as_double();

  // Humble 的 controller 沒有 robot_description API；launch 以參數檔傳給 controller_manager 的
  // robot_description 會套用到同一個 process 中的 controller node
  if (node_->has_parameter("robot_description")) {
    robot_description_ = node_->get_parameter("robot_description").as_string();
  }
}

std::string BvhSource::path_of(const std::string & name) const {
  if (!enabled()) {
    throw std::runtime_error("BVH playback is not enabled ('bvh.library_path' is empty)");
  }
  require_valid_item_name("BVH", name);
  return library_path_ + "/" + name + kExtension;
}

bool BvhSource::is_loaded(const std::string & name) {
  if (!enabled() || !is_valid_item_name(name)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = cache_.find(name);
  std::error_code error;
  return it != cache_.end() &&
         std::filesystem::last_write_time(path_of(name), error) == it->second.modified && !error;
}

std::shared_ptr<const motion::BvhRetargeter> BvhSource::load(const std::string & name) {
  const auto path = path_of(name);
  std::error_code error;
  const auto modified = std::filesystem::last_write_time(path, error);
  if (error) {
    throw std::runtime_error("BVH '" + name + "' not found (" + path + ")");
  }
  if (const auto it = cache_.find(name); it != cache_.end() && it->second.modified == modified) {
    return it->second.retargeter;
  }

  if (retarget_options_.base_link.empty()) {
    throw std::invalid_argument("'bvh.base_link' is required to play BVH motions");
  }
  if (robot_description_.empty()) {
    throw std::invalid_argument("parameter 'robot_description' is not available");
  }
  const auto logger = node_->get_logger();
  auto clip = std::make_shared<const motion::BvhClip>(motion::parse_bvh_file(path));
  auto retargeter = std::make_shared<const motion::BvhRetargeter>(
    clip, robot_description_, joint_names_, retarget_options_);

  RCLCPP_INFO(logger, "Loaded BVH [ %s ]: '%s', %zu frames, frame time %.4f s.",
    name.c_str(), path.c_str(), clip->num_frames(), clip->frame_time);
  for (const auto & group : retargeter->groups()) {
    if (group.bvh_node.empty()) {
      continue;
    }
    std::string joints;
    for (const auto & joint : group.joints) {
      joints += (joints.empty() ? "" : ", ") + joint;
    }
    RCLCPP_INFO(logger, "  [%s] <- BVH %s", joints.c_str(), group.bvh_node.c_str());
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (!retargeter->covered()[i]) {
      RCLCPP_WARN(logger, "  Joint [ %s ] has no BVH data, it holds its position.",
        joint_names_[i].c_str());
    }
  }
  cache_[name] = Loaded{modified, retargeter};
  return retargeter;
}

void BvhSource::preload(const std::string & name) {
  std::lock_guard<std::mutex> lock(mutex_);
  load(name);
}

PlaybackRequest BvhSource::prepare(const std::string & name, bool loop, double time_scale) {
  if (!(time_scale > 0.0) || !std::isfinite(time_scale)) {
    throw std::invalid_argument("time_scale must be greater than zero");
  }
  std::shared_ptr<const motion::BvhRetargeter> retargeter;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    retargeter = load(name);
  }
  BvhMotion::Options options;
  options.loop = loop;
  options.time_scale = time_scale;

  PlaybackRequest request;
  request.mode = kMode;
  request.item = name;
  // 播放選項不同就不是同一個播放：例如循環中再送一次不循環的指令，要換成不循環
  request.identity = std::string(kMode) + ":" + name + (loop ? "@loop" : "@once") +
    "@" + std::to_string(time_scale);
  request.detail = std::string(loop ? "loop" : "once") + ", time scale " +
    std::to_string(time_scale);
  request.motion = std::make_shared<BvhMotion>(
    std::move(retargeter), options, max_steps_, approach_max_steps_);
  request.policy.interruptible = interruptible_;
  return request;
}

}  // namespace graphen::control
