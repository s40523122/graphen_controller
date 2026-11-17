// 離線檢查編舞腳本：列出各軌道的時間範圍、各 joint 的範圍與峰值速度、建議的 max_steps，
// 有 URDF 時檢查限位與姿態中的 joint 名稱，並可以把整份編舞取樣成 CSV。
//
//   choreo_check <腳本> [--urdf <展開後的 URDF>] [--rate <Hz>] [--velocity-scale <倍率>] [--csv <輸出>]
//
// 從「開始播放時的位置」到各軌道第一個姿態的那一段取決於當下的位置，這裡無法檢查
// （ChoreographerAlgorithm 在進入模式時檢查）；CSV 假設起點就是第一個姿態。
// 回傳值：0 通過、1 腳本有誤或違反限位、2 用法錯誤。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "graphen_choreographer/choreography.hpp"

namespace {

using graphen::choreographer::JointLimits;
using graphen::choreographer::Choreography;

// 建議的 max_steps 相對於峰值速度的餘裕（planner 的週期會抖動）
constexpr double kMaxStepMargin = 1.2;

struct Options {
  std::string file;
  std::string urdf;
  std::string csv;
  double rate{100.0};
  double velocity_scale{1.0};
};

void print_usage() {
  std::cerr <<
    "Usage: choreo_check <choreography.yaml> [--urdf <file>] [--rate <Hz>]\n"
    "                      [--velocity-scale <factor>] [--csv <file>]\n"
    "  --urdf            expanded URDF (xacro <file> > out.urdf) to check joint limits against\n"
    "  --rate            planner rate used for the max_steps suggestion and CSV sampling "
    "(default 100)\n"
    "  --velocity-scale  velocity limit = URDF limit x factor (default 1.0)\n"
    "  --csv             write the sampled choreography (time + one column per joint)\n";
}

std::optional<double> to_positive(const std::string & text) {
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || !(value > 0.0) || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<Options> parse_args(int argc, char ** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::optional<std::string> {
        if (i + 1 >= argc) {
          std::cerr << "Missing value for " << arg << "\n";
          return std::nullopt;
        }
        return std::string(argv[++i]);
      };
    if (arg == "--urdf" || arg == "--csv") {
      const auto text = value();
      if (!text) {
        return std::nullopt;
      }
      (arg == "--urdf" ? options.urdf : options.csv) = *text;
    } else if (arg == "--rate" || arg == "--velocity-scale") {
      const auto text = value();
      const auto number = text ? to_positive(*text) : std::nullopt;
      if (!number) {
        std::cerr << arg << " expects a number greater than zero\n";
        return std::nullopt;
      }
      (arg == "--rate" ? options.rate : options.velocity_scale) = *number;
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option " << arg << "\n";
      return std::nullopt;
    } else if (options.file.empty()) {
      options.file = arg;
    } else {
      std::cerr << "Unexpected argument " << arg << "\n";
      return std::nullopt;
    }
  }
  if (options.file.empty()) {
    return std::nullopt;
  }
  return options;
}

std::string read_file(const std::string & path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open '" + path + "'");
  }
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

void print_report(
  const Choreography & choreography, const std::vector<graphen::choreographer::JointSummary> & summaries,
  const JointLimits & limits, const Options & options)
{
  size_t num_joints = 0;
  for (const auto & track : choreography.tracks) {
    num_joints += track.joints.size();
  }
  std::printf("Choreography: %s\n", options.file.c_str());
  std::printf("Duration: %.2f s, %zu tracks, %zu joints\n",
    choreography.duration(), choreography.tracks.size(), num_joints);
  std::printf("Interruptible while playing: %s\n", choreography.interruptible ? "yes" : "no");

  const auto limit_text = [](const std::optional<double> & value) {
      char buffer[32];
      if (value) {
        std::snprintf(buffer, sizeof(buffer), "%8.3f", *value);
      } else {
        std::snprintf(buffer, sizeof(buffer), "%8s", "-");
      }
      return std::string(buffer);
    };

  size_t index = 0;  // summarize() 的順序與這裡相同（軌道 → joint）
  for (const auto & track : choreography.tracks) {
    std::printf("\nTrack '%s': %zu keyframes, ends at %.2f s (first keyframe at %.2f s)\n",
      track.name.c_str(), track.keyframes.size(), track.duration(),
      track.keyframes.front().time);
    std::printf("  %-16s %8s %8s %8s %8s %9s %9s %9s %14s\n",
      "joint", "min", "max", "lower", "upper", "peak_vel", "at_t", "vel_limit",
      "max_step");
    for (size_t j = 0; j < track.joints.size(); ++j) {
      const auto & summary = summaries[index++];
      graphen::choreographer::JointLimit limit;
      if (const auto it = limits.find(summary.joint); it != limits.end()) {
        limit = it->second;
      }
      const std::optional<double> velocity_limit = limit.velocity ?
        std::optional<double>(*limit.velocity * options.velocity_scale) : std::nullopt;
      std::printf("  %-16s %8.3f %8.3f %s %s %9.3f %9.2f  %s %14.5f\n",
        summary.joint.c_str(), summary.min_position, summary.max_position,
        limit_text(limit.lower).c_str(), limit_text(limit.upper).c_str(),
        summary.peak_velocity, summary.peak_time, limit_text(velocity_limit).c_str(),
        summary.peak_velocity / options.rate * kMaxStepMargin);
    }
  }
  std::printf(
    "\nUnits: rad, rad/s. max_step = peak_vel / %.0f Hz x %.1f: the smallest <mode>.max_steps "
    "entry that\ndoes not slow this joint down (joints outside the choreography keep their usual "
    "value).\n"
    "Not covered: the move from the pose at mode entry to each track's first pose; it depends\n"
    "on where the robot is and is checked when the mode is entered.\n",
    options.rate, kMaxStepMargin);
}

void write_csv(const Choreography & choreography, const Options & options) {
  std::ofstream csv(options.csv);
  if (!csv) {
    throw std::runtime_error("cannot write '" + options.csv + "'");
  }
  struct TrackSampler {
    std::vector<double> start;
    std::vector<double> positions;
    std::vector<double> velocities;
    size_t cursor{0};
  };
  std::vector<TrackSampler> samplers;
  csv << "time";
  for (const auto & track : choreography.tracks) {
    TrackSampler sampler;
    // 起點假設為第一個姿態（沒有任何姿態的軌道為 0）
    sampler.start.assign(track.joints.size(), 0.0);
    for (const auto & keyframe : track.keyframes) {
      if (!keyframe.positions.empty()) {
        sampler.start = keyframe.positions;
        break;
      }
    }
    sampler.positions.assign(track.joints.size(), 0.0);
    sampler.velocities.assign(track.joints.size(), 0.0);
    samplers.push_back(std::move(sampler));
    for (const auto & joint : track.joints) {
      csv << ',' << joint;
    }
  }
  csv << '\n';

  const auto steps = static_cast<size_t>(std::ceil(choreography.duration() * options.rate));
  csv.precision(6);
  csv << std::fixed;
  for (size_t step = 0; step <= steps; ++step) {
    const double time = static_cast<double>(step) / options.rate;
    csv << time;
    for (size_t t = 0; t < choreography.tracks.size(); ++t) {
      auto & sampler = samplers[t];
      // 從靜止開始（起點速度只有在播放中切換模式時才會有）
      graphen::choreographer::sample_track(
        choreography.tracks[t], sampler.start, {}, time, sampler.cursor, sampler.positions,
        sampler.velocities);
      for (const double position : sampler.positions) {
        csv << ',' << position;
      }
    }
    csv << '\n';
  }
  std::printf("\nWrote %zu samples at %.0f Hz to %s\n", steps + 1, options.rate,
    options.csv.c_str());
}

}  // namespace

int main(int argc, char ** argv) {
  const auto options = parse_args(argc, argv);
  if (!options) {
    print_usage();
    return 2;
  }

  try {
    const auto choreography = graphen::choreographer::load_choreography(options->file);
    const auto summaries = graphen::choreographer::summarize(choreography);

    JointLimits limits;
    if (!options->urdf.empty()) {
      limits = graphen::choreographer::parse_joint_limits(read_file(options->urdf));
    }
    print_report(choreography, summaries, limits, *options);
    if (!options->csv.empty()) {
      write_csv(choreography, *options);
    }

    if (options->urdf.empty()) {
      std::printf("\nNo --urdf given: joint limits were not checked.\n");
      return 0;
    }
    for (const auto & summary : summaries) {
      if (limits.find(summary.joint) == limits.end()) {
        std::printf("\nWARNING: joint '%s' has no limits in the URDF, not checked.",
          summary.joint.c_str());
      }
    }
    auto violations =
      graphen::choreographer::check_limits(summaries, limits, options->velocity_scale);
    // 姿態中沒有被任何軌道使用的 joint 也要是機器人上的 joint（打錯字時不會有其他檢查發現）
    std::vector<std::string> urdf_joints;
    for (const auto & entry : limits) {
      urdf_joints.push_back(entry.first);
    }
    for (const auto & joint :
      graphen::choreographer::unknown_pose_joints(choreography, urdf_joints))
    {
      violations.push_back("poses use joint '" + joint + "', which is not a joint of the URDF");
    }
    if (violations.empty()) {
      std::printf("\nOK: all keyframes are within the joint limits.\n");
      return 0;
    }
    std::printf("\n%zu violation(s):\n", violations.size());
    for (const auto & violation : violations) {
      std::printf("  - %s\n", violation.c_str());
    }
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
