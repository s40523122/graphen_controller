// 離線把 BVH 轉成 URDF joint 角度，用來檢查分組與轉換結果。
//
// 用法：bvh_retarget_cli <bvh> <urdf> [--base <link>] [--damping <v>] [--max-iterations <n>] [joint ...]
//   不指定 joint 時輸出所有有 BVH 資料的 joint。
//   stdout：CSV（time + 各 joint 角度，rad）
//   stderr：分組、每組最大殘差、各 joint 的範圍 / 限位 / 最大速度、限位截斷次數

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "graphen_motion/bvh_clip.hpp"
#include "graphen_motion/bvh_retargeter.hpp"

namespace {

void print_usage() {
  std::cerr << "Usage: bvh_retarget_cli <bvh> <urdf> [options] [joint ...]\n"
               "  --base <link>            base link present in both URDF and BVH (default: BASE_S)\n"
               "  --damping <value>        damped least squares damping\n"
               "  --max-iterations <n>     solver iteration limit per group\n";
}

}  // namespace

int main(int argc, char ** argv) {
  using graphen::motion::BvhRetargeter;

  std::vector<std::string> positional;
  std::string base_link = "BASE_S";
  BvhRetargeter::Options defaults;
  double damping = defaults.damping;
  int max_iterations = defaults.max_iterations;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--base" && i + 1 < argc) {
      base_link = argv[++i];
    } else if (arg == "--damping" && i + 1 < argc) {
      damping = std::stod(argv[++i]);
    } else if (arg == "--max-iterations" && i + 1 < argc) {
      max_iterations = std::stoi(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      print_usage();
      return 0;
    } else {
      positional.push_back(arg);
    }
  }
  if (positional.size() < 2) {
    print_usage();
    return 1;
  }

  try {
    auto clip = std::make_shared<const graphen::motion::BvhClip>(
      graphen::motion::parse_bvh_file(positional[0]));

    std::ifstream urdf_file(positional[1]);
    if (!urdf_file) {
      std::cerr << "Cannot open URDF file '" << positional[1] << "'\n";
      return 1;
    }
    std::stringstream urdf_xml;
    urdf_xml << urdf_file.rdbuf();

    BvhRetargeter::Options options;
    options.base_link = base_link;
    options.damping = damping;
    options.max_iterations = max_iterations;

    std::vector<std::string> joints(positional.begin() + 2, positional.end());
    if (joints.empty()) {
      // 先只建立分組，再取出所有有 BVH 資料的 joint
      const BvhRetargeter probe(clip, urdf_xml.str(), {}, options);
      for (const auto & group : probe.groups()) {
        if (!group.bvh_node.empty()) {
          joints.insert(joints.end(), group.joints.begin(), group.joints.end());
        }
      }
    }
    const BvhRetargeter retargeter(clip, urdf_xml.str(), joints, options);

    std::cerr << "Frames: " << clip->num_frames() << ", frame time: " << clip->frame_time
              << " s\nGroups:\n";
    const auto & groups = retargeter.groups();
    for (const auto & group : groups) {
      std::cerr << "  [";
      for (size_t k = 0; k < group.joints.size(); ++k) {
        std::cerr << (k ? ", " : "") << group.joints[k];
      }
      std::cerr << "] <- " << (group.bvh_node.empty() ? "(no BVH data)" : group.bvh_node) << "\n";
    }

    const size_t n = joints.size();
    std::vector<double> q(n, std::numeric_limits<double>::quiet_NaN());
    std::vector<double> previous;
    std::vector<double> min_value(n, std::numeric_limits<double>::infinity());
    std::vector<double> max_value(n, -std::numeric_limits<double>::infinity());
    std::vector<double> max_velocity(n, 0.0);
    std::vector<double> group_max_residual(groups.size(), 0.0);
    size_t clamped = 0;

    std::cout << "time";
    for (const auto & name : joints) {
      std::cout << "," << name;
    }
    std::cout << "\n";

    for (size_t frame = 0; frame < clip->num_frames(); ++frame) {
      const auto stats = retargeter.solve(frame, q);
      clamped += stats.clamped;
      for (size_t g = 0; g < stats.group_residuals.size(); ++g) {
        group_max_residual[g] = std::max(group_max_residual[g], stats.group_residuals[g]);
      }
      std::cout << frame * clip->frame_time;
      for (size_t i = 0; i < n; ++i) {
        std::cout << "," << q[i];
        min_value[i] = std::min(min_value[i], q[i]);
        max_value[i] = std::max(max_value[i], q[i]);
        if (!previous.empty()) {
          max_velocity[i] =
            std::max(max_velocity[i], std::abs(q[i] - previous[i]) / clip->frame_time);
        }
      }
      std::cout << "\n";
      previous = q;
    }

    std::cerr << "Max residual per group [rad]:\n";
    for (size_t g = 0; g < groups.size(); ++g) {
      if (!groups[g].bvh_node.empty()) {
        std::cerr << "  " << groups[g].bvh_node << ": " << group_max_residual[g] << "\n";
      }
    }
    std::cerr << "Joints (min / max, limit, max velocity [rad/s]):\n";
    for (size_t i = 0; i < n; ++i) {
      const auto [lower, upper] = retargeter.limits()[i];
      char line[256];
      std::snprintf(line, sizeof(line), "  %-14s %8.4f / %8.4f  [%8.4f, %8.4f]  %7.3f\n",
        joints[i].c_str(), min_value[i], max_value[i], lower, upper, max_velocity[i]);
      std::cerr << line;
    }
    std::cerr << "Clamped by joint limits: " << clamped << " joint-frames\n";
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
