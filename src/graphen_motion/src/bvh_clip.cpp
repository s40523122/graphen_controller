#include "graphen_motion/bvh_clip.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

namespace graphen::motion {

namespace {

constexpr int kEndSite = -2;  // 堆疊中代表 End Site 區塊（不是真正的 joint）

[[noreturn]] void fail(const std::string & message) {
  throw std::runtime_error("BVH parse error: " + message);
}

std::string next_token(std::istream & in, const char * context) {
  std::string token;
  if (!(in >> token)) {
    fail(std::string("unexpected end of file while reading ") + context);
  }
  return token;
}

void expect_token(std::istream & in, const std::string & expected) {
  const auto token = next_token(in, expected.c_str());
  if (token != expected) {
    fail("expected '" + expected + "', got '" + token + "'");
  }
}

double read_double(std::istream & in, const char * context) {
  const auto token = next_token(in, context);
  try {
    size_t used = 0;
    const double value = std::stod(token, &used);
    if (used != token.size()) {
      fail(std::string("invalid number '") + token + "' in " + context);
    }
    return value;
  } catch (const std::logic_error &) {
    fail(std::string("invalid number '") + token + "' in " + context);
  }
}

BvhChannel parse_channel(const std::string & name) {
  if (name == "Xposition") {return BvhChannel::kXposition;}
  if (name == "Yposition") {return BvhChannel::kYposition;}
  if (name == "Zposition") {return BvhChannel::kZposition;}
  if (name == "Xrotation") {return BvhChannel::kXrotation;}
  if (name == "Yrotation") {return BvhChannel::kYrotation;}
  if (name == "Zrotation") {return BvhChannel::kZrotation;}
  fail("unknown channel '" + name + "'");
}

}  // namespace

std::optional<size_t> BvhClip::find_joint(const std::string & name) const {
  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].name == name) {
      return i;
    }
  }
  return std::nullopt;
}

Eigen::Matrix3d BvhClip::local_rotation(size_t frame, size_t joint) const {
  const auto & j = joints.at(joint);
  const auto & values = frames.at(frame);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  for (size_t c = 0; c < j.channels.size(); ++c) {
    const double angle = values[j.channel_offset + c] * M_PI / 180.0;
    switch (j.channels[c]) {
      case BvhChannel::kXrotation:
        rotation *= Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
        break;
      case BvhChannel::kYrotation:
        rotation *= Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
        break;
      case BvhChannel::kZrotation:
        rotation *= Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        break;
      default:
        break;  // 位移 channel 不影響旋轉
    }
  }
  return rotation;
}

BvhClip parse_bvh(std::istream & in) {
  BvhClip clip;
  size_t num_channels = 0;

  expect_token(in, "HIERARCHY");

  // 堆疊存放目前所在的區塊：joint index 或 kEndSite
  std::vector<int> stack;
  int pending = -1;          // 已讀到名稱、等待 '{' 的區塊
  bool has_pending = false;
  bool root_done = false;

  while (true) {
    const auto token = next_token(in, "HIERARCHY");
    if (token == "MOTION") {
      break;
    }
    if (token == "ROOT" || token == "JOINT") {
      if (token == "ROOT" && (root_done || !stack.empty())) {
        fail("multiple ROOT joints are not supported");
      }
      if (token == "JOINT" && (stack.empty() || stack.back() < 0)) {
        fail("JOINT outside of a joint block");
      }
      BvhJoint joint;
      joint.name = next_token(in, "joint name");
      if (clip.find_joint(joint.name)) {
        fail("duplicate joint name '" + joint.name + "'");
      }
      joint.parent = stack.empty() ? -1 : stack.back();
      clip.joints.push_back(std::move(joint));
      pending = static_cast<int>(clip.joints.size()) - 1;
      has_pending = true;
    } else if (token == "End") {
      expect_token(in, "Site");
      if (stack.empty() || stack.back() < 0) {
        fail("End Site outside of a joint block");
      }
      pending = kEndSite;
      has_pending = true;
    } else if (token == "{") {
      if (!has_pending) {
        fail("unexpected '{'");
      }
      stack.push_back(pending);
      has_pending = false;
    } else if (token == "}") {
      if (stack.empty()) {
        fail("unbalanced '}'");
      }
      stack.pop_back();
      if (stack.empty()) {
        root_done = true;
      }
    } else if (token == "OFFSET") {
      if (stack.empty()) {
        fail("OFFSET outside of a block");
      }
      Eigen::Vector3d offset;
      for (int k = 0; k < 3; ++k) {
        offset[k] = read_double(in, "OFFSET");
      }
      if (stack.back() >= 0) {
        clip.joints[static_cast<size_t>(stack.back())].offset = offset;
      }
    } else if (token == "CHANNELS") {
      if (stack.empty() || stack.back() < 0) {
        fail("CHANNELS outside of a joint block");
      }
      const double count = read_double(in, "CHANNELS count");
      if (count < 0 || count != std::floor(count)) {
        fail("invalid CHANNELS count");
      }
      auto & joint = clip.joints[static_cast<size_t>(stack.back())];
      joint.channel_offset = num_channels;
      for (int k = 0; k < static_cast<int>(count); ++k) {
        joint.channels.push_back(parse_channel(next_token(in, "channel name")));
      }
      num_channels += joint.channels.size();
    } else {
      fail("unexpected token '" + token + "' in HIERARCHY");
    }
  }

  if (!stack.empty() || !root_done) {
    fail("HIERARCHY is incomplete");
  }

  expect_token(in, "Frames:");
  const double num_frames = read_double(in, "Frames");
  if (num_frames < 1 || num_frames != std::floor(num_frames)) {
    fail("'Frames' must be a positive integer");
  }
  expect_token(in, "Frame");
  expect_token(in, "Time:");
  clip.frame_time = read_double(in, "Frame Time");
  if (!(clip.frame_time > 0.0)) {
    fail("'Frame Time' must be greater than zero");
  }

  clip.frames.resize(static_cast<size_t>(num_frames));
  for (auto & frame : clip.frames) {
    frame.resize(num_channels);
    for (auto & value : frame) {
      value = read_double(in, "MOTION data");
    }
  }
  return clip;
}

BvhClip parse_bvh_file(const std::string & path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Cannot open BVH file '" + path + "'");
  }
  return parse_bvh(file);
}

}  // namespace graphen::motion
