#include "graphen_controllers/choreo_source.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "graphen_controllers/library_utils.hpp"
#include "graphen_controllers/parameter_sources.hpp"
#include "graphen_locomotion/sha256.hpp"
#include "yaml-cpp/yaml.h"

namespace graphen::control {

namespace {

constexpr const char * kExtension = ".choreo.yaml";
// 試播（不在舞曲庫中）的項目名稱
constexpr const char * kPreviewItem = "(preview)";

}  // namespace

ChoreoSource::ChoreoSource(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, PluginFactory * factory,
  const std::vector<double> & default_max_steps)
: node_(std::move(node)), factory_(factory)
{
  plugin_ = node_->get_parameter("choreo.plugin").as_string();
  if (plugin_.empty()) {
    return;
  }
  library_path_ = resolve_path(node_->get_parameter("choreo.library_path").as_string());
  if (library_path_.empty()) {
    throw std::invalid_argument("'choreo.library_path' is empty");
  }
  max_steps_ = node_->get_parameter("choreo.max_steps").as_double_array();
  if (max_steps_.empty()) {
    max_steps_ = default_max_steps;
  } else if (max_steps_.size() != default_max_steps.size()) {
    throw std::invalid_argument(
            "'choreo.max_steps' must be empty or contain " +
            std::to_string(default_max_steps.size()) + " entries, got " +
            std::to_string(max_steps_.size()));
  }
}

std::string ChoreoSource::path_of(const std::string & name) const {
  require_valid_item_name("choreography", name);
  return library_path_ + "/" + name + kExtension;
}

PlaybackRequest ChoreoSource::build(
  const std::string & item, const std::string & content, const std::string & source) const
{
  if (!enabled()) {
    throw std::runtime_error("choreographies are not enabled ('choreo.plugin' is empty)");
  }
  const auto sha256 = locomotion::Sha256::hex_digest_of(content);

  // 內容與名稱疊在 controller 的 choreo.* 之上：上限等設定只來自 controller，腳本裡沒有對應的鍵
  YAML::Node extra;
  extra["content"] = content;
  extra["label"] = item;
  const YamlParameterSource overlay(extra, "PlayChoreo");
  const NodeParameterSource base(node_, "choreo.");
  const OverlayParameterSource params({&overlay, &base});

  // 每次都建立新的實例：播放中的那一個不受之後的發佈或試播影響
  auto created = factory_->create(plugin_, params, kMode, max_steps_);

  PlaybackRequest request;
  request.mode = kMode;
  request.item = item;
  request.identity = std::string(kMode) + ":" + item + "@" + sha256;
  request.detail = source + ", sha256 " + short_hash(sha256);
  request.motion = std::move(created.motion);
  request.policy.interruptible = created.algorithm->interruptible();
  return request;
}

PlaybackRequest ChoreoSource::prepare(const std::string & name) const {
  const auto path = path_of(name);
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("choreography '" + name + "' is not published");
  }
  return build(name, read_text_file(path), "published");
}

PlaybackRequest ChoreoSource::prepare_preview(const std::string & content) const {
  return build(kPreviewItem, content, "preview");
}

std::string ChoreoSource::publish(const std::string & name, const std::string & content) const {
  const auto path = path_of(name);
  // 與播放相同的檢查；不通過時丟出例外，不寫入
  auto request = build(name, content, "publish check");
  request.motion.reset();

  std::error_code error;
  std::filesystem::create_directories(library_path_, error);
  if (error) {
    throw std::runtime_error(
            "cannot create library directory '" + library_path_ + "': " + error.message());
  }
  // 先寫到暫存檔再 rename：讀取端不會看到寫到一半的檔案
  const auto temporary = path + ".tmp";
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    file << content;
    file.flush();
    if (!file) {
      throw std::runtime_error("cannot write '" + temporary + "'");
    }
  }
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("cannot write '" + path + "': " + error.message());
  }
  return locomotion::Sha256::hex_digest_of(content);
}

std::vector<ChoreoSource::Entry> ChoreoSource::list() const {
  std::vector<Entry> entries;
  std::error_code error;
  if (!enabled() || !std::filesystem::is_directory(library_path_, error)) {
    return entries;
  }
  const std::string extension = kExtension;
  for (const auto & file : std::filesystem::directory_iterator(library_path_, error)) {
    const auto filename = file.path().filename().string();
    if (filename.size() <= extension.size() ||
      filename.compare(filename.size() - extension.size(), extension.size(), extension) != 0)
    {
      continue;
    }
    const auto name = filename.substr(0, filename.size() - extension.size());
    // 手動放進目錄、名稱不合規則的檔案無法播放，不列出
    if (!is_valid_item_name(name)) {
      continue;
    }
    entries.push_back({name, locomotion::Sha256::file_hex_digest(file.path().string())});
  }
  std::sort(entries.begin(), entries.end(),
    [](const Entry & a, const Entry & b) {return a.name < b.name;});
  return entries;
}

std::string ChoreoSource::get(const std::string & name, std::string & sha256) const {
  if (!enabled()) {
    throw std::runtime_error("choreographies are not enabled ('choreo.plugin' is empty)");
  }
  const auto path = path_of(name);
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("choreography '" + name + "' is not published");
  }
  auto content = read_text_file(path);
  sha256 = locomotion::Sha256::hex_digest_of(content);
  return content;
}

}  // namespace graphen::control
