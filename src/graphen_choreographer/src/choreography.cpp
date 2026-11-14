#include "graphen_choreographer/choreography.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

namespace graphen::choreographer {

namespace {

constexpr int kSupportedVersion = 1;
constexpr Interpolation kDefaultInterpolation = Interpolation::kQuintic;

using Pose = std::map<std::string, double>;

[[noreturn]] void fail(const std::string & where, const std::string & what) {
  throw std::invalid_argument(where + ": " + what);
}

std::string to_text(const YAML::Node & node, const std::string & where) {
  if (!node.IsScalar()) {
    fail(where, "expected a string");
  }
  return node.Scalar();
}

double to_number(const YAML::Node & node, const std::string & where) {
  double value = 0.0;
  // yaml-cpp 會把 .nan / .inf 也轉成 double，這裡一律不接受
  if (!node.IsScalar() || !YAML::convert<double>::decode(node, value) || !std::isfinite(value)) {
    fail(where, "expected a finite number");
  }
  return value;
}

// 只接受列出的 key，打錯字（例如 interp 寫成 intrep）時直接報錯，而不是默默忽略
void check_keys(
  const YAML::Node & node, std::initializer_list<const char *> allowed, const std::string & where)
{
  if (!node.IsMap()) {
    fail(where, "expected a map");
  }
  for (const auto & entry : node) {
    const auto key = to_text(entry.first, where + " (key)");
    if (std::none_of(allowed.begin(), allowed.end(), [&](const char * a) {return key == a;})) {
      std::string list;
      for (const char * a : allowed) {
        list += list.empty() ? "" : ", ";
        list += a;
      }
      fail(where, "unknown key '" + key + "' (allowed: " + list + ")");
    }
  }
}

Interpolation to_interpolation(const YAML::Node & node, const std::string & where) {
  const auto name = to_text(node, where);
  const auto type = parse_interpolation(name);
  if (!type) {
    fail(where, "unknown interpolation '" + name + "' (expected linear, cubic or quintic)");
  }
  return *type;
}

// joint → 角度的 map
Pose to_pose(const YAML::Node & node, const std::string & where) {
  if (!node.IsMap() || node.size() == 0) {
    fail(where, "expected a non-empty map of joint: angle");
  }
  Pose pose;
  for (const auto & entry : node) {
    const auto joint = to_text(entry.first, where + " (joint name)");
    if (!pose.emplace(joint, to_number(entry.second, where + "." + joint)).second) {
      fail(where, "joint '" + joint + "' is listed more than once");
    }
  }
  return pose;
}

Keyframe parse_keyframe(
  const YAML::Node & node, const std::string & where, const Track & track,
  const std::map<std::string, Pose> & poses, Interpolation default_interpolation,
  const Keyframe * previous)
{
  check_keys(node, {"t", "pose", "positions", "hold", "interp"}, where);

  Keyframe keyframe;
  if (!node["t"]) {
    fail(where, "missing 't'");
  }
  keyframe.time = to_number(node["t"], where + ".t");
  if (keyframe.time < 0.0) {
    fail(where, "'t' must not be negative");
  }
  if (previous != nullptr && !(keyframe.time > previous->time)) {
    fail(where, "'t' must be greater than the previous keyframe's");
  }
  keyframe.interpolation = node["interp"] ?
    to_interpolation(node["interp"], where + ".interp") : default_interpolation;

  bool hold = false;
  if (node["hold"] && !YAML::convert<bool>::decode(node["hold"], hold)) {
    fail(where + ".hold", "expected true or false");
  }
  const int sources = (node["pose"] ? 1 : 0) + (node["positions"] ? 1 : 0) + (hold ? 1 : 0);
  if (sources != 1) {
    fail(where, "expected exactly one of 'pose', 'positions' or 'hold: true'");
  }

  if (hold) {
    // 維持上一個 keyframe；前面沒有任何姿態時 positions 留空，表示維持起點
    if (previous != nullptr) {
      keyframe.positions = previous->positions;
    }
    return keyframe;
  }

  const Pose * pose = nullptr;
  Pose inline_pose;
  std::string source;
  if (node["pose"]) {
    const auto name = to_text(node["pose"], where + ".pose");
    const auto it = poses.find(name);
    if (it == poses.end()) {
      fail(where, "pose '" + name + "' is not defined in 'poses'");
    }
    pose = &it->second;
    source = "pose '" + name + "'";
  } else {
    inline_pose = to_pose(node["positions"], where + ".positions");
    // 直接寫的數值必須剛好是這條軌道的 joint，多出來的多半是打錯字
    for (const auto & entry : inline_pose) {
      const auto & joint = entry.first;
      if (std::find(track.joints.begin(), track.joints.end(), joint) == track.joints.end()) {
        fail(where, "joint '" + joint + "' in 'positions' does not belong to this track");
      }
    }
    pose = &inline_pose;
    source = "'positions'";
  }

  keyframe.positions.reserve(track.joints.size());
  for (const auto & joint : track.joints) {
    const auto it = pose->find(joint);
    if (it == pose->end()) {
      fail(where, source + " does not give joint '" + joint + "'");
    }
    keyframe.positions.push_back(it->second);
  }
  return keyframe;
}

}  // namespace

double Choreography::duration() const {
  double duration = 0.0;
  for (const auto & track : tracks) {
    duration = std::max(duration, track.duration());
  }
  return duration;
}

Choreography parse_choreography(const std::string & yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    throw std::invalid_argument(std::string("YAML syntax error: ") + e.what());
  }

  try {
    check_keys(root, {"version", "default_interp", "poses", "tracks"}, "top level");

    int version = 0;
    if (!root["version"] || !YAML::convert<int>::decode(root["version"], version)) {
      fail("version", "missing or not an integer");
    }
    if (version != kSupportedVersion) {
      fail("version", "unsupported version " + std::to_string(version) +
        " (expected " + std::to_string(kSupportedVersion) + ")");
    }

    const Interpolation default_interpolation = root["default_interp"] ?
      to_interpolation(root["default_interp"], "default_interp") : kDefaultInterpolation;

    std::map<std::string, Pose> poses;
    if (const auto node = root["poses"]) {
      if (!node.IsMap()) {
        fail("poses", "expected a map of pose name: {joint: angle}");
      }
      for (const auto & entry : node) {
        const auto name = to_text(entry.first, "poses (pose name)");
        if (!poses.emplace(name, to_pose(entry.second, "poses." + name)).second) {
          fail("poses", "pose '" + name + "' is defined more than once");
        }
      }
    }

    const auto tracks_node = root["tracks"];
    if (!tracks_node || !tracks_node.IsMap() || tracks_node.size() == 0) {
      fail("tracks", "expected a non-empty map of track name: {joints, keyframes}");
    }

    Choreography choreography;
    std::map<std::string, std::string> joint_owner;  // joint → 軌道名稱
    for (const auto & entry : tracks_node) {
      Track track;
      track.name = to_text(entry.first, "tracks (track name)");
      const std::string where = "tracks." + track.name;
      const auto & node = entry.second;
      if (std::any_of(choreography.tracks.begin(), choreography.tracks.end(),
        [&](const Track & t) {return t.name == track.name;}))
      {
        fail("tracks", "track '" + track.name + "' is defined more than once");
      }
      check_keys(node, {"joints", "keyframes"}, where);

      const auto joints_node = node["joints"];
      if (!joints_node || !joints_node.IsSequence() || joints_node.size() == 0) {
        fail(where + ".joints", "expected a non-empty list of joint names");
      }
      for (const auto & joint_node : joints_node) {
        const auto joint = to_text(joint_node, where + ".joints");
        const auto [it, inserted] = joint_owner.emplace(joint, track.name);
        if (!inserted) {
          fail(where + ".joints", "joint '" + joint + "' already belongs to track '" +
            it->second + "' (a joint can only be in one track)");
        }
        track.joints.push_back(joint);
      }

      const auto keyframes_node = node["keyframes"];
      if (!keyframes_node || !keyframes_node.IsSequence() || keyframes_node.size() == 0) {
        fail(where + ".keyframes", "expected a non-empty list of keyframes");
      }
      track.keyframes.reserve(keyframes_node.size());
      for (size_t i = 0; i < keyframes_node.size(); ++i) {
        track.keyframes.push_back(parse_keyframe(
            keyframes_node[i], where + ".keyframes[" + std::to_string(i) + "]", track, poses,
            default_interpolation, track.keyframes.empty() ? nullptr : &track.keyframes.back()));
      }
      choreography.tracks.push_back(std::move(track));
    }
    return choreography;
  } catch (const YAML::Exception & e) {
    throw std::invalid_argument(std::string("YAML error: ") + e.what());
  }
}

Choreography load_choreography(const std::string & path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open choreography file '" + path + "'");
  }
  std::ostringstream text;
  text << file.rdbuf();
  try {
    return parse_choreography(text.str());
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument("'" + path + "': " + e.what());
  }
}

EntrySample sample_entry(
  Interpolation type, double from, double to, double start_velocity, double duration, double u)
{
  const Blend s = blend(type, u);
  const double delta = to - from;
  EntrySample sample{from + delta * s.position, delta * s.velocity / duration};
  if (start_velocity != 0.0 && supports_entry_velocity(type)) {
    const Blend h = entry_velocity_blend(u);
    sample.position += start_velocity * duration * h.position;
    sample.velocity += start_velocity * h.velocity;
  }
  return sample;
}

void sample_track(
  const Track & track, const std::vector<double> & start,
  const std::vector<double> & start_velocity, double time, size_t & cursor,
  std::vector<double> & positions, std::vector<double> & velocities)
{
  const auto & keyframes = track.keyframes;
  // 第 k 段：從第 k-1 個 keyframe（k = 0 時為 t = 0 的起點）到第 k 個 keyframe
  while (cursor < keyframes.size() && time >= keyframes[cursor].time) {
    ++cursor;
  }
  // positions 為空的 keyframe 表示維持起點
  const auto positions_of = [&start](const Keyframe & keyframe) -> const std::vector<double> & {
      return keyframe.positions.empty() ? start : keyframe.positions;
    };

  if (cursor >= keyframes.size()) {
    // 播完：保持最後的位置
    const auto & last = keyframes.empty() ? start : positions_of(keyframes.back());
    for (size_t i = 0; i < positions.size(); ++i) {
      positions[i] = last[i];
      velocities[i] = 0.0;
    }
    return;
  }

  const auto & to = keyframes[cursor];
  const double from_time = cursor == 0 ? 0.0 : keyframes[cursor - 1].time;
  const auto & from_positions = cursor == 0 ? start : positions_of(keyframes[cursor - 1]);
  const auto & to_positions = positions_of(to);
  // 上面的 while 保證 time < to.time，且 t 嚴格遞增，所以 duration > 0
  const double duration = to.time - from_time;
  const double u = std::clamp((time - from_time) / duration, 0.0, 1.0);
  if (cursor == 0) {
    // 起點 → 第一個 keyframe：可以帶入起點速度
    for (size_t i = 0; i < positions.size(); ++i) {
      const double v0 = start_velocity.empty() ? 0.0 : start_velocity[i];
      const auto sample =
        sample_entry(to.interpolation, from_positions[i], to_positions[i], v0, duration, u);
      positions[i] = sample.position;
      velocities[i] = sample.velocity;
    }
    return;
  }
  const Blend s = blend(to.interpolation, u);
  for (size_t i = 0; i < positions.size(); ++i) {
    const double delta = to_positions[i] - from_positions[i];
    positions[i] = from_positions[i] + delta * s.position;
    velocities[i] = delta * s.velocity / duration;
  }
}

std::vector<JointSummary> summarize(const Choreography & choreography) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  std::vector<JointSummary> summaries;
  for (const auto & track : choreography.tracks) {
    for (size_t j = 0; j < track.joints.size(); ++j) {
      JointSummary summary{track.name, track.joints[j], kNaN, kNaN, 0.0, 0.0};
      const Keyframe * previous = nullptr;  // 上一個有姿態的 keyframe（之後的 hold 也有 positions）
      for (const auto & keyframe : track.keyframes) {
        if (keyframe.positions.empty()) {
          continue;
        }
        const double position = keyframe.positions[j];
        summary.min_position = std::isnan(summary.min_position) ? position :
          std::min(summary.min_position, position);
        summary.max_position = std::isnan(summary.max_position) ? position :
          std::max(summary.max_position, position);
        if (previous != nullptr) {
          const double velocity = std::abs(position - previous->positions[j]) /
            (keyframe.time - previous->time) * peak_velocity_factor(keyframe.interpolation);
          if (velocity > summary.peak_velocity) {
            summary.peak_velocity = velocity;
            summary.peak_time = previous->time;
          }
        }
        previous = &keyframe;
      }
      summaries.push_back(std::move(summary));
    }
  }
  return summaries;
}

JointLimits parse_joint_limits(const std::string & urdf_xml) {
  const auto model = urdf::parseURDF(urdf_xml);
  if (!model) {
    throw std::invalid_argument("failed to parse the URDF");
  }
  JointLimits limits;
  for (const auto & [name, joint] : model->joints_) {
    if (!joint || !joint->limits) {
      continue;
    }
    JointLimit limit;
    // continuous joint 沒有位置限位
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
      limit.lower = joint->limits->lower;
      limit.upper = joint->limits->upper;
    }
    // URDF 沒寫 velocity 時為 0，視為沒有限制
    if (joint->limits->velocity > 0.0) {
      limit.velocity = joint->limits->velocity;
    }
    limits.emplace(name, limit);
  }
  return limits;
}

std::vector<std::string> check_limits(
  const std::vector<JointSummary> & summaries, const JointLimits & limits, double velocity_scale)
{
  std::vector<std::string> violations;
  const auto format = [](double value) {
      std::ostringstream out;
      out.precision(4);
      out << value;
      return out.str();
    };
  for (const auto & summary : summaries) {
    const auto it = limits.find(summary.joint);
    if (it == limits.end()) {
      continue;
    }
    const auto & limit = it->second;
    const std::string who = "track '" + summary.track + "' joint '" + summary.joint + "'";
    if (limit.lower && summary.min_position < *limit.lower) {
      violations.push_back(who + ": position " + format(summary.min_position) +
        " rad is below the lower limit " + format(*limit.lower));
    }
    if (limit.upper && summary.max_position > *limit.upper) {
      violations.push_back(who + ": position " + format(summary.max_position) +
        " rad is above the upper limit " + format(*limit.upper));
    }
    if (limit.velocity && summary.peak_velocity > *limit.velocity * velocity_scale) {
      violations.push_back(who + ": peak velocity " + format(summary.peak_velocity) +
        " rad/s (segment starting at t = " + format(summary.peak_time) +
        " s) exceeds the limit " + format(*limit.velocity * velocity_scale) + " rad/s");
    }
  }
  return violations;
}

}  // namespace graphen::choreographer
