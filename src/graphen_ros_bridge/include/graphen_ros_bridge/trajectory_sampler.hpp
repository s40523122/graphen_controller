#ifndef GRAPHEN_ROS_BRIDGE__TRAJECTORY_SAMPLER_HPP_
#define GRAPHEN_ROS_BRIDGE__TRAJECTORY_SAMPLER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "graphen_locomotion/robot_state.hpp"

namespace graphen::ros_bridge {

// 關節軌跡的取樣，與傳輸方式（ROS topic 等）無關，也不依賴 rclcpp。
// 訊息端（例如 trajectory_msgs/JointTrajectory）先轉成 Trajectory，再由 validate() 檢查。

struct TrajectoryPoint {
  double time{0.0};                  // s，相對軌跡起點
  std::vector<double> positions;     // rad，順序同 Trajectory::command_index
  std::vector<double> velocities;    // rad/s，空 = 沒有速度
};

struct Trajectory {
  // 軌跡中每個關節對應的命令 index（Context::joint_names）；-1 表示不是受控關節，略過
  std::vector<int> command_index;
  std::vector<TrajectoryPoint> points;

  double end_time() const {return points.empty() ? 0.0 : points.back().time;}
};

// 回傳錯誤訊息；沒有問題時回傳 std::nullopt。
// 至少要有一個受控關節與一個點；每個點的位置（與速度，有的話）長度要對上 command_index，
// 數值必須有限；time 必須有限、>= 0 且不遞減
inline std::optional<std::string> validate(const Trajectory & trajectory) {
  const auto joints = trajectory.command_index.size();
  const bool any_commanded = std::any_of(
    trajectory.command_index.begin(), trajectory.command_index.end(),
    [](int index) {return index >= 0;});
  if (joints == 0 || trajectory.points.empty() || !any_commanded) {
    return "no commanded joint or no point in the trajectory";
  }
  const auto finite = [](const std::vector<double> & values) {
      return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
    };
  double previous_time = 0.0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    if (point.positions.size() != joints || !finite(point.positions)) {
      return "point " + std::to_string(i) + ": positions must be finite, one per joint name";
    }
    if (!point.velocities.empty() && (point.velocities.size() != joints || !finite(point.velocities))) {
      return "point " + std::to_string(i) + ": velocities must be empty or finite, one per joint name";
    }
    if (!std::isfinite(point.time) || point.time < 0.0 || (i > 0 && point.time < previous_time)) {
      return "point " + std::to_string(i) + ": time_from_start must be finite, >= 0 and non-decreasing";
    }
    previous_time = point.time;
  }
  return std::nullopt;
}

// 在時間 t（s，相對軌跡起點）取樣，寫入受控關節的 positions / velocities，其他欄位不動。
// 第一個點之前不寫入（保持）；兩點之間位置線性插值，速度在兩點都有 velocities 時線性插值、
// 否則以該段位置的斜率當前饋；最後一個點之後停在最後一點，速度沿用最後一點的 velocities（沒有則為 0）。
// 呼叫端必須先通過 validate()。回傳 t 是否已經超過最後一個點
inline bool sample(const Trajectory & trajectory, double t, locomotion::JointCommand & command) {
  const auto & points = trajectory.points;
  if (t < points.front().time) {
    return false;
  }
  const bool past_end = t >= points.back().time;
  size_t upper = points.size();
  if (!past_end) {
    const auto it = std::upper_bound(
      points.begin(), points.end(), t,
      [](double time, const TrajectoryPoint & point) {return time < point.time;});
    upper = static_cast<size_t>(it - points.begin());
  }

  for (size_t joint = 0; joint < trajectory.command_index.size(); ++joint) {
    const int index = trajectory.command_index[joint];
    if (index < 0) {
      continue;
    }
    double position = 0.0;
    double velocity = 0.0;
    if (past_end) {
      const auto & last = points.back();
      position = last.positions[joint];
      velocity = last.velocities.empty() ? 0.0 : last.velocities[joint];
    } else {
      const auto & left = points[upper - 1];
      const auto & right = points[upper];
      const double span = right.time - left.time;
      const double alpha = span > 0.0 ? (t - left.time) / span : 1.0;
      position = left.positions[joint] + alpha * (right.positions[joint] - left.positions[joint]);
      if (!left.velocities.empty() && !right.velocities.empty()) {
        velocity = left.velocities[joint] + alpha * (right.velocities[joint] - left.velocities[joint]);
      } else if (span > 0.0) {
        velocity = (right.positions[joint] - left.positions[joint]) / span;
      }
    }
    const auto i = static_cast<size_t>(index);
    command.positions[i] = position;
    command.velocities[i] = velocity;
  }
  return past_end;
}

}  // namespace graphen::ros_bridge

#endif  // GRAPHEN_ROS_BRIDGE__TRAJECTORY_SAMPLER_HPP_
