#include "graphen_controllers/policy_source.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include "graphen_controllers/library_utils.hpp"
#include "graphen_controllers/parameter_sources.hpp"
#include "graphen_locomotion/sha256.hpp"
#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

namespace graphen::control {

namespace {

constexpr const char * kExtension = ".policy.yaml";

}  // namespace

PolicySource::PolicySource(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, PluginFactory * factory,
  const std::vector<double> & default_max_steps)
: node_(std::move(node)), factory_(factory), default_max_steps_(default_max_steps)
{
  library_path_ = resolve_path(node_->get_parameter("policy.library_path").as_string());
}

std::string PolicySource::path_of(const std::string & name) const {
  if (!enabled()) {
    throw std::runtime_error("policies are not enabled ('policy.library_path' is empty)");
  }
  require_valid_item_name("policy", name);
  return library_path_ + "/" + name + kExtension;
}

bool PolicySource::is_loaded(const std::string & name) {
  if (!enabled() || !is_valid_item_name(name)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = cache_.find(name);
  if (it == cache_.end()) {
    return false;
  }
  try {
    return locomotion::Sha256::file_hex_digest(path_of(name)) == it->second.sha256;
  } catch (const std::exception &) {
    return false;
  }
}

PlaybackRequest PolicySource::prepare(const std::string & name) {
  const auto path = path_of(name);
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("policy '" + name + "' not found (" + path + ")");
  }
  const auto text = read_text_file(path);
  const auto sha256 = locomotion::Sha256::hex_digest_of(text);

  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(name);
  if (it == cache_.end() || it->second.sha256 != sha256) {
    YAML::Node root;
    try {
      root = YAML::Load(text);
    } catch (const YAML::Exception & e) {
      throw std::invalid_argument("'" + path + "': YAML syntax error: " + e.what());
    }
    // controller 的 <name>.* 參數覆寫檔案中的同名鍵
    const YamlParameterSource file(root, "'" + path + "'");
    const NodeParameterSource overrides(node_, name + ".");
    const OverlayParameterSource params({&overrides, &file});

    const auto plugin = params.get_string("plugin").value_or("");
    if (plugin.empty()) {
      throw std::invalid_argument("'" + path + "': 'plugin' is required");
    }
    auto max_steps = params.get_double_array("max_steps").value_or(default_max_steps_);
    if (max_steps.size() != default_max_steps_.size()) {
      throw std::invalid_argument(
              "'" + path + "': 'max_steps' must contain " +
              std::to_string(default_max_steps_.size()) + " entries, got " +
              std::to_string(max_steps.size()));
    }
    if (std::any_of(max_steps.begin(), max_steps.end(), [](double s) {return !(s > 0.0);})) {
      throw std::invalid_argument("'" + path + "': every entry of 'max_steps' must be > 0");
    }
    const auto started = std::chrono::steady_clock::now();
    auto created = factory_->create(plugin, params, name, std::move(max_steps));
    Loaded loaded;
    loaded.sha256 = sha256;
    loaded.motion = std::move(created.motion);
    loaded.interruptible =
      params.get_bool("interruptible").value_or(true) && created.algorithm->interruptible();
    RCLCPP_INFO(node_->get_logger(),
      "Loaded policy [ %s ]: plugin '%s', config sha256 %s (%.0f ms).",
      name.c_str(), plugin.c_str(), short_hash(sha256).c_str(),
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
    it = cache_.insert_or_assign(name, std::move(loaded)).first;
  }

  PlaybackRequest request;
  request.mode = kMode;
  request.item = name;
  request.identity = std::string(kMode) + ":" + name + "@" + sha256;
  request.detail = "config sha256 " + short_hash(sha256);
  request.motion = it->second.motion;
  request.policy.interruptible = it->second.interruptible;
  return request;
}

}  // namespace graphen::control
