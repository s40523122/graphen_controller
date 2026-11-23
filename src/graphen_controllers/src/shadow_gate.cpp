#include "graphen_controllers/shadow_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <utility>

namespace graphen::control {

namespace {

// 同時保留的 token 上限：超過時捨棄最舊的
constexpr size_t kMaxEntries = 64;

std::string hex(uint64_t value) {
  char buffer[17];
  std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(value));
  return buffer;
}

}  // namespace

ShadowGate::ShadowGate(Config config)
: config_(config) {
  if (!(config_.start_tolerance >= 0.0) || !std::isfinite(config_.start_tolerance)) {
    throw std::invalid_argument("'choreo.shadow.start_tolerance' must be a finite number >= 0");
  }
  if (!(config_.token_ttl > 0.0)) {
    throw std::invalid_argument("'choreo.shadow.token_ttl' must be greater than zero");
  }
  std::random_device device;
  salt_ = (static_cast<uint64_t>(device()) << 32) ^ device();
}

void ShadowGate::purge(Clock::time_point now) {
  const auto ttl = std::chrono::duration<double>(config_.token_ttl);
  entries_.erase(
    std::remove_if(entries_.begin(), entries_.end(),
    [&](const Entry & entry) {return now - entry.issued > ttl;}),
    entries_.end());
}

std::string ShadowGate::issue(const std::string & sha256, const std::vector<double> & start_positions) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Clock::now();
  purge(now);
  if (entries_.size() >= kMaxEntries) {
    entries_.erase(entries_.begin());
  }
  // 內容 hash 的前 8 碼方便人工辨識，其餘為這個實例的序號
  const auto token = sha256.substr(0, 8) + "-" + hex(salt_ ^ ++counter_);
  entries_.push_back({token, sha256, start_positions, now});
  return token;
}

std::string ShadowGate::verify(
  const std::string & token, const std::string & sha256,
  const std::vector<double> & current_positions, const std::vector<std::string> & names) const
{
  if (token.empty()) {
    return config_.require ?
           "a dry run is required first (choreo.shadow.require is true): "
           "send the goal with options.dry_run and pass the returned shadow_token" :
           std::string{};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Clock::now();
  const auto it = std::find_if(entries_.begin(), entries_.end(),
      [&](const Entry & entry) {return entry.token == token;});
  if (it == entries_.end() ||
    now - it->issued > std::chrono::duration<double>(config_.token_ttl))
  {
    return "unknown or expired shadow_token (valid for choreo.shadow.token_ttl = " +
           std::to_string(config_.token_ttl) + " s); run the dry run again";
  }
  if (it->sha256 != sha256) {
    return "the content differs from the one that passed the dry run";
  }
  if (current_positions.size() != it->start_positions.size()) {
    return "the number of interfaces changed since the dry run";
  }
  double worst = 0.0;
  size_t worst_index = 0;
  for (size_t i = 0; i < current_positions.size(); ++i) {
    const double diff = std::abs(current_positions[i] - it->start_positions[i]);
    if (!(diff <= worst) || std::isnan(diff)) {
      worst = diff;
      worst_index = i;
      if (std::isnan(diff)) {
        break;
      }
    }
  }
  if (!(worst <= config_.start_tolerance)) {
    const auto name = worst_index < names.size() ? names[worst_index] :
      "interface " + std::to_string(worst_index);
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "%.4f rad, choreo.shadow.start_tolerance %.4f",
      worst, config_.start_tolerance);
    return "the robot moved since the dry run (" + name + " differs by " + buffer +
           "); run the dry run again";
  }
  return {};
}

}  // namespace graphen::control
