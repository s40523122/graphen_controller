#include "graphen_arm_ik/arm_config.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace graphen::arm_ik {

namespace {

constexpr int kVersion = 1;

[[noreturn]] void fail(const std::string & where, const std::string & what) {
  throw std::invalid_argument(where + ": " + what);
}

const YAML::Node require(const YAML::Node & parent, const std::string & key, const std::string & where) {
  const auto node = parent[key];
  if (!node) {
    fail(where, "missing '" + key + "'");
  }
  return node;
}

template<typename T>
T as(const YAML::Node & node, const std::string & where) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception &) {
    fail(where, "invalid value");
  }
}

double finite(const YAML::Node & node, const std::string & where) {
  const auto value = as<double>(node, where);
  if (!std::isfinite(value)) {
    fail(where, "must be finite");
  }
  return value;
}

template<size_t N>
std::array<double, N> vector_of(const YAML::Node & node, const std::string & where) {
  if (!node.IsSequence() || node.size() != N) {
    fail(where, "must be a list of " + std::to_string(N) + " numbers");
  }
  std::array<double, N> out{};
  for (size_t i = 0; i < N; ++i) {
    out[i] = finite(node[i], where + "[" + std::to_string(i) + "]");
  }
  return out;
}

// {xyz: [..], rpy: [..]}（皆選填）→ 位姿；rpy 為固定軸 roll-pitch-yaw（與 URDF 相同）
Eigen::Isometry3d parse_pose(const YAML::Node & node, const std::string & where) {
  if (!node.IsMap()) {
    fail(where, "must be a map with xyz / rpy");
  }
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (key != "xyz" && key != "rpy") {
      fail(where, "unknown key '" + key + "'");
    }
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (node["xyz"]) {
    const auto xyz = vector_of<3>(node["xyz"], where + ".xyz");
    pose.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  }
  if (node["rpy"]) {
    const auto rpy = vector_of<3>(node["rpy"], where + ".rpy");
    pose.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  return pose;
}

void check_keys(const YAML::Node & node, const std::set<std::string> & allowed, const std::string & where) {
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (allowed.count(key) == 0) {
      fail(where, "unknown key '" + key + "'");
    }
  }
}

ArmDefinition parse_arm(const std::string & name, const YAML::Node & node, const std::string & where) {
  if (!node.IsMap()) {
    fail(where, "must be a map");
  }
  check_keys(node, {"joints", "tip_body", "elbow_body", "shoulder_body", "tip_offset",
      "swivel_reference", "seed"}, where);
  ArmDefinition arm;
  arm.name = name;
  const auto joints = require(node, "joints", where);
  if (!joints.IsSequence() || joints.size() != kArmDof) {
    fail(where + ".joints", "must list exactly " + std::to_string(kArmDof) + " joints");
  }
  for (size_t i = 0; i < kArmDof; ++i) {
    arm.joints[i] = as<std::string>(joints[i], where + ".joints[" + std::to_string(i) + "]");
  }
  arm.tip_body = as<std::string>(require(node, "tip_body", where), where + ".tip_body");
  arm.elbow_body = as<std::string>(require(node, "elbow_body", where), where + ".elbow_body");
  arm.shoulder_body = as<std::string>(require(node, "shoulder_body", where), where + ".shoulder_body");
  if (node["tip_offset"]) {
    arm.tip_offset = parse_pose(node["tip_offset"], where + ".tip_offset");
  }
  if (node["swivel_reference"]) {
    const auto ref = vector_of<3>(node["swivel_reference"], where + ".swivel_reference");
    arm.swivel_reference = Eigen::Vector3d(ref[0], ref[1], ref[2]);
    if (arm.swivel_reference.norm() < 1e-9) {
      fail(where + ".swivel_reference", "must not be zero");
    }
    arm.swivel_reference.normalize();
  }
  arm.seed = vector_of<kArmDof>(require(node, "seed", where), where + ".seed");
  return arm;
}

void parse_solver(const YAML::Node & node, SolverOptions & out) {
  const std::string where = "solver";
  check_keys(node, {"damping", "iterations", "position_tolerance", "orientation_tolerance",
      "swivel_tolerance", "orientation_weight", "max_step", "joint_limit_margin", "restarts"}, where);
  const auto positive = [&](const char * key, double & value) {
      if (node[key]) {
        value = finite(node[key], where + "." + key);
        if (!(value > 0.0)) {
          fail(where + "." + key, "must be > 0");
        }
      }
    };
  positive("damping", out.damping);
  positive("position_tolerance", out.position_tolerance);
  positive("orientation_tolerance", out.orientation_tolerance);
  positive("swivel_tolerance", out.swivel_tolerance);
  positive("orientation_weight", out.orientation_weight);
  positive("max_step", out.max_step);
  if (node["iterations"]) {
    out.iterations = as<int>(node["iterations"], where + ".iterations");
    if (out.iterations < 1) {
      fail(where + ".iterations", "must be >= 1");
    }
  }
  if (node["restarts"]) {
    out.restarts = as<int>(node["restarts"], where + ".restarts");
    if (out.restarts < 0) {
      fail(where + ".restarts", "must be >= 0");
    }
  }
  if (node["joint_limit_margin"]) {
    out.joint_limit_margin = finite(node["joint_limit_margin"], where + ".joint_limit_margin");
    if (out.joint_limit_margin < 0.0) {
      fail(where + ".joint_limit_margin", "must be >= 0");
    }
  }
}

void parse_collision(const YAML::Node & node, CollisionOptions & out) {
  const std::string where = "collision";
  check_keys(node, {"enabled", "include_floor", "min_penetration", "ignore_within_links",
      "allow_initial_contacts", "allowed_pairs"}, where);
  if (node["ignore_within_links"]) {
    out.ignore_within_links = as<int>(node["ignore_within_links"], where + ".ignore_within_links");
    if (out.ignore_within_links < 1) {
      fail(where + ".ignore_within_links", "must be >= 1");
    }
  }
  if (node["enabled"]) {
    out.enabled = as<bool>(node["enabled"], where + ".enabled");
  }
  if (node["include_floor"]) {
    out.include_floor = as<bool>(node["include_floor"], where + ".include_floor");
  }
  if (node["allow_initial_contacts"]) {
    out.allow_initial_contacts =
      as<bool>(node["allow_initial_contacts"], where + ".allow_initial_contacts");
  }
  if (node["min_penetration"]) {
    out.min_penetration = finite(node["min_penetration"], where + ".min_penetration");
    if (out.min_penetration < 0.0) {
      fail(where + ".min_penetration", "must be >= 0");
    }
  }
  if (node["allowed_pairs"]) {
    const auto pairs = node["allowed_pairs"];
    if (!pairs.IsSequence()) {
      fail(where + ".allowed_pairs", "must be a list of [body, body]");
    }
    for (size_t i = 0; i < pairs.size(); ++i) {
      const auto at = where + ".allowed_pairs[" + std::to_string(i) + "]";
      if (!pairs[i].IsSequence() || pairs[i].size() != 2) {
        fail(at, "must be [body, body]");
      }
      out.allowed_pairs.emplace_back(as<std::string>(pairs[i][0], at), as<std::string>(pairs[i][1], at));
    }
  }
}

Obstacle parse_obstacle(const YAML::Node & node, const std::string & where) {
  check_keys(node, {"name", "shape", "size", "xyz", "rpy"}, where);
  Obstacle obstacle;
  obstacle.name = as<std::string>(require(node, "name", where), where + ".name");
  const auto shape = as<std::string>(require(node, "shape", where), where + ".shape");
  if (shape == "box") {
    obstacle.shape = Obstacle::Shape::kBox;
  } else if (shape == "sphere") {
    obstacle.shape = Obstacle::Shape::kSphere;
  } else if (shape == "cylinder") {
    obstacle.shape = Obstacle::Shape::kCylinder;
  } else if (shape == "capsule") {
    obstacle.shape = Obstacle::Shape::kCapsule;
  } else {
    fail(where + ".shape", "must be box / sphere / cylinder / capsule");
  }
  // sphere 只用 size[0]，cylinder / capsule 用 [半徑, 半長]；沒用到的位置填 0
  const auto size = require(node, "size", where);
  if (!size.IsSequence() || size.size() < 1 || size.size() > 3) {
    fail(where + ".size", "must be a list of 1 to 3 numbers");
  }
  for (size_t i = 0; i < size.size(); ++i) {
    obstacle.size[i] = finite(size[i], where + ".size[" + std::to_string(i) + "]");
    if (!(obstacle.size[i] > 0.0)) {
      fail(where + ".size", "must be > 0");
    }
  }
  YAML::Node pose;
  if (node["xyz"]) {
    pose["xyz"] = node["xyz"];
  }
  if (node["rpy"]) {
    pose["rpy"] = node["rpy"];
  }
  obstacle.pose = parse_pose(pose, where);
  return obstacle;
}

}  // namespace

std::string resolve_package_path(const std::string & path) {
  const std::string scheme = "package://";
  if (path.compare(0, scheme.size(), scheme) != 0) {
    return path;
  }
  const auto rest = path.substr(scheme.size());
  const auto slash = rest.find('/');
  if (slash == std::string::npos || slash == 0) {
    throw std::invalid_argument("'" + path + "' must be package://<package>/<path>");
  }
  try {
    return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) + rest.substr(slash);
  } catch (const std::exception & e) {
    throw std::invalid_argument("cannot resolve '" + path + "': " + e.what());
  }
}

ArmConfig parse_arm_config(const std::string & yaml_text, const std::string & base_dir) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    throw std::invalid_argument(std::string("invalid YAML: ") + e.what());
  }
  if (!root.IsMap()) {
    throw std::invalid_argument("arm_ik config must be a map");
  }
  check_keys(root, {"version", "mjcf", "root_body", "arms", "solver", "collision", "scene"}, "(top)");
  if (as<int>(require(root, "version", "(top)"), "version") != kVersion) {
    fail("version", "must be " + std::to_string(kVersion));
  }

  ArmConfig config;
  auto mjcf = resolve_package_path(as<std::string>(require(root, "mjcf", "(top)"), "mjcf"));
  if (!mjcf.empty() && std::filesystem::path(mjcf).is_relative()) {
    mjcf = (std::filesystem::path(base_dir) / mjcf).lexically_normal().string();
  }
  config.mjcf = mjcf;
  config.root_body = as<std::string>(require(root, "root_body", "(top)"), "root_body");

  const auto arms = require(root, "arms", "(top)");
  if (!arms.IsMap() || arms.size() == 0) {
    fail("arms", "must be a non-empty map of <name>: {...}");
  }
  for (const auto & item : arms) {
    const auto name = item.first.as<std::string>();
    config.arms.push_back(parse_arm(name, item.second, "arms." + name));
  }
  if (root["solver"]) {
    parse_solver(root["solver"], config.solver);
  }
  if (root["collision"]) {
    parse_collision(root["collision"], config.collision);
  }
  if (root["scene"]) {
    const auto scene = root["scene"];
    if (!scene.IsSequence()) {
      fail("scene", "must be a list of obstacles");
    }
    for (size_t i = 0; i < scene.size(); ++i) {
      config.scene.push_back(parse_obstacle(scene[i], "scene[" + std::to_string(i) + "]"));
    }
  }
  return config;
}

ArmConfig load_arm_config(const std::string & path) {
  const auto resolved = resolve_package_path(path);
  std::ifstream file(resolved);
  if (!file) {
    throw std::runtime_error("cannot open arm_ik config '" + resolved + "'");
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  try {
    return parse_arm_config(buffer.str(), std::filesystem::path(resolved).parent_path().string());
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument(resolved + ": " + e.what());
  }
}

}  // namespace graphen::arm_ik
