#include "graphen_choreographer/path_collision.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace graphen::choreographer {

std::string CollisionSpan::describe() const {
  char head[96];
  if (end > start) {
    std::snprintf(head, sizeof(head), "collision at t=%.2f-%.2f s: ", start, end);
  } else {
    std::snprintf(head, sizeof(head), "collision at t=%.2f s: ", start);
  }
  std::string text = head;
  for (size_t i = 0; i < pairs.size(); ++i) {
    text += (i == 0 ? "" : ", ") + pairs[i];
  }
  char depth[48];
  std::snprintf(depth, sizeof(depth), " (max %.1f mm)", max_depth * 1e3);
  return text + depth;
}

PathCollisionChecker::PathCollisionChecker(
  std::shared_ptr<const arm_ik::ArmModel> model, Choreography choreography)
: choreography_(std::move(choreography)), ik_(std::move(model)) {
  const auto & m = ik_.model();
  for (const auto & track : choreography_.tracks) {
    std::vector<int> qpos;
    for (const auto & joint : track.joints) {
      qpos.push_back(m.qpos_address(joint));
    }
    track_qpos_.push_back(std::move(qpos));
    const size_t n = track.joints.size();
    positions_.emplace_back(n, 0.0);
    velocities_.emplace_back(n, 0.0);
    // 第一個姿態（沒有任何姿態的軌道維持 0；這種軌道只會 hold，check_script 時不動）
    std::vector<double> first(n, 0.0);
    for (const auto & keyframe : track.keyframes) {
      if (!keyframe.positions.empty()) {
        first = keyframe.positions;
        break;
      }
    }
    first_poses_.push_back(std::move(first));
  }
  starts_.assign(choreography_.tracks.size(), nullptr);
  start_velocities_.assign(choreography_.tracks.size(), nullptr);
  cursors_.assign(choreography_.tracks.size(), 0);
  contacts_.reserve(64);
}

bool PathCollisionChecker::set_base_position(const std::string & joint, double value) {
  const int adr = ik_.model().qpos_address(joint);
  if (adr < 0 || !std::isfinite(value)) {
    return false;
  }
  const auto it = std::find_if(base_.begin(), base_.end(), [adr](const auto & e) {return e.first == adr;});
  if (it != base_.end()) {
    it->second = value;
  } else {
    base_.emplace_back(adr, value);
  }
  return true;
}

void PathCollisionChecker::sample(double time, std::vector<CollisionSpan> & spans, bool & in_span) {
  static const std::vector<double> kAtRest;
  for (size_t t = 0; t < choreography_.tracks.size(); ++t) {
    sample_track(choreography_.tracks[t], *starts_[t],
      start_velocities_[t] ? *start_velocities_[t] : kAtRest, time, cursors_[t], positions_[t],
      velocities_[t]);
    const auto & qpos = track_qpos_[t];
    for (size_t j = 0; j < qpos.size(); ++j) {
      ik_.set_position(qpos[j], positions_[t][j]);
    }
  }
  if (ik_.check_collisions(-1, contacts_) == 0) {
    in_span = false;
    return;
  }
  if (!in_span) {
    spans.emplace_back();
    spans.back().start = time;
    in_span = true;
  }
  auto & span = spans.back();
  span.end = time;
  const auto & m = ik_.model();
  for (const auto & c : contacts_) {
    span.max_depth = std::max(span.max_depth, c.depth);
    const auto pair = m.body_name(c.body1) + "|" + m.body_name(c.body2);
    if (std::find(span.pairs.begin(), span.pairs.end(), pair) == span.pairs.end()) {
      span.pairs.push_back(pair);
    }
  }
}

std::vector<CollisionSpan> PathCollisionChecker::check_script(double sample_rate) {
  if (!(sample_rate > 0.0)) {
    throw std::invalid_argument("collision sample rate must be greater than zero");
  }
  ik_.reset();
  for (const auto & [adr, value] : base_) {
    ik_.set_position(adr, value);
  }
  for (size_t t = 0; t < choreography_.tracks.size(); ++t) {
    starts_[t] = &first_poses_[t];
    start_velocities_[t] = nullptr;
    cursors_[t] = 0;
  }
  std::vector<CollisionSpan> spans;
  bool in_span = false;
  const double duration = choreography_.duration();
  const auto steps = static_cast<size_t>(std::ceil(duration * sample_rate));
  for (size_t i = 0; i <= steps; ++i) {
    // 最後一個點剛好落在 duration（最後的姿態）
    sample(std::min(static_cast<double>(i) / sample_rate, duration), spans, in_span);
  }
  return spans;
}

std::vector<CollisionSpan> PathCollisionChecker::check_entry(
  const std::vector<std::vector<double>> & starts,
  const std::vector<std::vector<double>> & start_velocities, int samples)
{
  const size_t n = choreography_.tracks.size();
  if (starts.size() != n || start_velocities.size() != n) {
    throw std::invalid_argument("check_entry: one start per track is required");
  }
  ik_.reset();
  for (const auto & [adr, value] : base_) {
    ik_.set_position(adr, value);
  }
  double end = 0.0;
  for (size_t t = 0; t < n; ++t) {
    starts_[t] = &starts[t];
    start_velocities_[t] = &start_velocities[t];
    cursors_[t] = 0;
    end = std::max(end, choreography_.tracks[t].keyframes.front().time);
  }
  std::vector<CollisionSpan> spans;
  bool in_span = false;
  const int count = end > 0.0 ? std::max(samples, 1) : 0;
  for (int i = 0; i <= count; ++i) {
    sample(count > 0 ? end * i / count : 0.0, spans, in_span);
  }
  return spans;
}

}  // namespace graphen::choreographer
