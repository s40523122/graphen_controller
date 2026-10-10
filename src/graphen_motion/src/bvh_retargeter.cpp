#include "graphen_motion/bvh_retargeter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <urdf_parser/urdf_parser.h>

namespace graphen::motion {

namespace {

Eigen::Matrix3d to_matrix(const urdf::Rotation & rotation) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  rotation.getQuaternion(x, y, z, w);
  return Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
}

// 旋轉矩陣的 log map（旋轉向量）
Eigen::Vector3d rotation_log(const Eigen::Matrix3d & rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

// 組內軸線的交點狀態：只有一個 joint 時是一條線，兩個以上時是一個點
struct GroupGeometry {
  Eigen::Vector3d line_point;
  Eigen::Vector3d line_direction;
  std::optional<Eigen::Vector3d> point;
};

// 新 joint 的軸線是否通過組的共同點；可以時更新共同點
bool try_join(
  GroupGeometry & geometry, const Eigen::Vector3d & point, const Eigen::Vector3d & direction,
  double tolerance) {
  if (geometry.point) {
    return (*geometry.point - point).cross(direction).norm() <= tolerance;
  }
  // 兩條線：平行（含共線）不構成球形關節，不合併
  const Eigen::Vector3d & p1 = geometry.line_point;
  const Eigen::Vector3d & d1 = geometry.line_direction;
  const Eigen::Vector3d normal = d1.cross(direction);
  if (normal.norm() < 1e-9) {
    return false;
  }
  const Eigen::Vector3d w0 = p1 - point;
  if (std::abs(w0.dot(normal)) / normal.norm() > tolerance) {
    return false;
  }
  // 兩線最近點的中點作為共同點
  const double b = d1.dot(direction);
  const double d = d1.dot(w0);
  const double e = direction.dot(w0);
  const double denom = 1.0 - b * b;
  const double s = (b * e - d) / denom;
  const double t = (e - b * d) / denom;
  geometry.point = 0.5 * ((p1 + s * d1) + (point + t * direction));
  return true;
}

}  // namespace

BvhRetargeter::BvhRetargeter(
  std::shared_ptr<const BvhClip> clip, const std::string & urdf_xml,
  std::vector<std::string> joint_names, Options options)
: clip_(std::move(clip)), joint_names_(std::move(joint_names)), options_(std::move(options)) {
  if (!clip_ || clip_->joints.empty() || clip_->frames.empty()) {
    throw std::invalid_argument("BvhRetargeter: BVH clip is empty");
  }
  if (options_.max_iterations < 1) {
    throw std::invalid_argument("BvhRetargeter: max_iterations must be at least 1");
  }
  if (!(options_.axis_tolerance > 0.0)) {
    throw std::invalid_argument("BvhRetargeter: axis_tolerance must be greater than zero");
  }

  const auto model = urdf::parseURDF(urdf_xml);
  if (!model) {
    throw std::invalid_argument("BvhRetargeter: failed to parse URDF");
  }
  const auto base = model->getLink(options_.base_link);
  if (!base) {
    throw std::invalid_argument(
      "BvhRetargeter: base_link '" + options_.base_link + "' not found in URDF");
  }
  const auto base_node = clip_->find_joint(options_.base_link);
  if (!base_node) {
    throw std::invalid_argument(
      "BvhRetargeter: base_link '" + options_.base_link + "' not found in BVH");
  }
  base_node_ = *base_node;

  // 以 DFS 收集 base_link 之下的 joint：父 link 一定在子 link 之前，單一鏈上的 joint 連續出現
  std::vector<Eigen::Vector3d> origin_translations;
  links_.push_back(base->name);
  std::function<void(const urdf::LinkConstSharedPtr &, size_t)> visit =
    [&](const urdf::LinkConstSharedPtr & link, size_t link_index) {
      for (const auto & urdf_joint : link->child_joints) {
        Joint joint;
        joint.name = urdf_joint->name;
        joint.parent_link = link_index;
        joint.origin_rotation = to_matrix(urdf_joint->parent_to_joint_origin_transform.rotation);
        const auto & t = urdf_joint->parent_to_joint_origin_transform.position;
        origin_translations.emplace_back(t.x, t.y, t.z);
        joint.movable = urdf_joint->type == urdf::Joint::REVOLUTE ||
          urdf_joint->type == urdf::Joint::CONTINUOUS;
        if (joint.movable) {
          const Eigen::Vector3d axis(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
          if (axis.norm() < 1e-9) {
            throw std::invalid_argument("BvhRetargeter: joint '" + joint.name + "' has zero axis");
          }
          joint.axis = axis.normalized();
        }
        if (urdf_joint->type == urdf::Joint::REVOLUTE && urdf_joint->limits) {
          joint.limited = true;
          joint.lower = urdf_joint->limits->lower;
          joint.upper = urdf_joint->limits->upper;
        }
        links_.push_back(urdf_joint->child_link_name);
        joint.child_link = links_.size() - 1;
        joints_.push_back(joint);

        const auto child = model->getLink(urdf_joint->child_link_name);
        if (child) {
          visit(child, joint.child_link);
        }
      }
    };
  visit(base, 0);

  build_groups(origin_translations);

  // 輸出 joint 對應到內部 joint
  std::unordered_map<std::string, size_t> joint_index;
  for (size_t i = 0; i < joints_.size(); ++i) {
    joint_index.emplace(joints_[i].name, i);
  }
  for (const auto & name : joint_names_) {
    const auto it = joint_index.find(name);
    if (it == joint_index.end()) {
      throw std::invalid_argument(
        "BvhRetargeter: joint '" + name + "' is not below base_link '" +
        options_.base_link + "' in URDF");
    }
    const auto & joint = joints_[it->second];
    if (!joint.movable) {
      throw std::invalid_argument(
        "BvhRetargeter: joint '" + name + "' is not a revolute or continuous joint");
    }
    output_to_joint_.push_back(it->second);
    covered_.push_back(groups_[*joint.group].bvh_node.has_value());
    if (joint.limited) {
      output_limits_.emplace_back(joint.lower, joint.upper);
    } else {
      constexpr double inf = std::numeric_limits<double>::infinity();
      output_limits_.emplace_back(-inf, inf);
    }
  }
}

void BvhRetargeter::build_groups(const std::vector<Eigen::Vector3d> & origin_translations) {
  // 零位時各 link 在 base_link 座標下的姿態與位置；組內軸線交於一點的性質與關節角度無關
  std::vector<Eigen::Matrix3d> link_rotation(links_.size(), Eigen::Matrix3d::Identity());
  std::vector<Eigen::Vector3d> link_position(links_.size(), Eigen::Vector3d::Zero());
  std::vector<size_t> child_count(links_.size(), 0);
  std::vector<std::optional<size_t>> parent_joint(links_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = joints_[i];
    link_rotation[joint.child_link] = link_rotation[joint.parent_link] * joint.origin_rotation;
    link_position[joint.child_link] =
      link_position[joint.parent_link] + link_rotation[joint.parent_link] * origin_translations[i];
    ++child_count[joint.parent_link];
    parent_joint[joint.child_link] = i;
  }

  std::vector<GroupGeometry> geometries;
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & joint = joints_[i];
    if (!joint.movable) {
      steps_.push_back({false, i});
      continue;
    }
    const Eigen::Vector3d point = link_position[joint.child_link];
    const Eigen::Vector3d direction = link_rotation[joint.child_link] * joint.axis;

    // 只有在父 link 不分岔、且上一個 joint 是同一組的最後一個時，才考慮合併
    bool joined = false;
    const auto previous = parent_joint[joint.parent_link];
    if (previous && child_count[joint.parent_link] == 1 && joints_[*previous].group) {
      const size_t group = *joints_[*previous].group;
      if (groups_[group].joints.back() == *previous &&
        groups_[group].joints.size() < static_cast<size_t>(kMaxGroupDof) &&
        try_join(geometries[group], point, direction, options_.axis_tolerance))
      {
        groups_[group].joints.push_back(i);
        joint.group = group;
        joined = true;
      }
    }
    if (!joined) {
      joint.group = groups_.size();
      groups_.push_back(Group{{i}, std::nullopt, Eigen::Matrix3d::Identity()});
      geometries.push_back(GroupGeometry{point, direction, std::nullopt});
      steps_.push_back({true, *joint.group});
    }
  }

  // 每組的目標：組內最後一個在 BVH 中有同名節點的 link
  for (auto & group : groups_) {
    for (size_t k = group.joints.size(); k-- > 0; ) {
      const auto node = clip_->find_joint(links_[joints_[group.joints[k]].child_link]);
      if (!node) {
        continue;
      }
      group.bvh_node = node;
      for (size_t m = k + 1; m < group.joints.size(); ++m) {
        group.post_rotation *= joints_[group.joints[m]].origin_rotation;
      }
      break;
    }

    GroupInfo info;
    for (const auto index : group.joints) {
      info.joints.push_back(joints_[index].name);
    }
    if (group.bvh_node) {
      info.bvh_node = clip_->joints[*group.bvh_node].name;
    }
    group_infos_.push_back(std::move(info));
  }
}

BvhRetargeter::GroupResult BvhRetargeter::solve_from(
  const Group & group, const Eigen::Matrix3d & parent_rotation, const Eigen::Matrix3d & target,
  const GroupState & seed) const {
  const auto dof = static_cast<Eigen::Index>(group.joints.size());
  GroupResult result;
  result.q = seed;

  Eigen::Matrix<double, 3, Eigen::Dynamic, 0, 3, kMaxGroupDof> jacobian(3, dof);
  std::array<bool, kMaxGroupDof> clamped{};

  for (int iteration = 0; ; ++iteration) {
    // 組內 FK：末端姿態，以及各軸在 base_link 座標下的方向（Jacobian 的欄）
    Eigen::Matrix3d rotation = parent_rotation;
    for (Eigen::Index k = 0; k < dof; ++k) {
      const auto & joint = joints_[group.joints[static_cast<size_t>(k)]];
      rotation = rotation * joint.origin_rotation;
      jacobian.col(k) = rotation * joint.axis;
      rotation = rotation *
        Eigen::AngleAxisd(result.q[static_cast<size_t>(k)], joint.axis).toRotationMatrix();
    }
    const Eigen::Vector3d error = rotation_log(target * rotation.transpose());
    result.residual = error.norm();
    if (iteration >= options_.max_iterations) {
      break;
    }

    // damped least squares：dq = (JᵀJ + λI)⁻¹ Jᵀ e
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxGroupDof, kMaxGroupDof> jtj =
      jacobian.transpose() * jacobian;
    jtj.diagonal().array() += options_.damping;
    const Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxGroupDof, 1> delta =
      jtj.ldlt().solve(jacobian.transpose() * error);

    double max_change = 0.0;
    for (Eigen::Index k = 0; k < dof; ++k) {
      const auto index = static_cast<size_t>(k);
      const auto & joint = joints_[group.joints[index]];
      double & value = result.q[index];
      const double previous = value;
      value += delta[k];
      clamped[index] = joint.limited && (value < joint.lower || value > joint.upper);
      if (clamped[index]) {
        value = std::clamp(value, joint.lower, joint.upper);
      }
      max_change = std::max(max_change, std::abs(value - previous));
    }
    if (max_change < options_.convergence_tolerance) {
      break;
    }
  }
  result.clamped = static_cast<size_t>(std::count(clamped.begin(), clamped.end(), true));
  return result;
}

BvhRetargeter::GroupResult BvhRetargeter::solve_group(
  const Group & group, const Eigen::Matrix3d & parent_rotation, const Eigen::Matrix3d & target,
  const GroupState & warm) const {
  // 大多數幀從上一幀出發就能精確解出，直接採用以維持連續
  auto best = solve_from(group, parent_rotation, target, warm);
  if (best.residual <= options_.residual_tolerance) {
    return best;
  }

  // 解不出來時（奇異點附近走到錯誤的分支、或撞到限位），從一組固定的初始值重新求解：
  // 每個 joint 取 {-π/2, 0, π/2} 的所有組合。殘差在最佳值 + residual_tolerance 以內的解中，
  // 選離上一幀最近的
  std::vector<GroupResult> candidates{best};
  const size_t dof = group.joints.size();
  size_t combinations = 1;
  for (size_t k = 0; k < dof; ++k) {
    combinations *= kSeedValues.size();
  }
  for (size_t c = 0; c < combinations; ++c) {
    GroupState seed{};
    size_t code = c;
    for (size_t k = 0; k < dof; ++k) {
      seed[k] = kSeedValues[code % kSeedValues.size()];
      code /= kSeedValues.size();
    }
    candidates.push_back(solve_from(group, parent_rotation, target, seed));
  }

  double best_residual = std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    best_residual = std::min(best_residual, candidate.residual);
  }
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    if (candidate.residual > best_residual + options_.residual_tolerance) {
      continue;
    }
    double distance = 0.0;
    for (size_t k = 0; k < dof; ++k) {
      distance += (candidate.q[k] - warm[k]) * (candidate.q[k] - warm[k]);
    }
    if (distance < best_distance) {
      best_distance = distance;
      best = candidate;
    }
  }
  return best;
}

BvhRetargeter::Stats BvhRetargeter::solve(size_t frame, std::vector<double> & q_inout) const {
  if (frame >= clip_->num_frames()) {
    throw std::out_of_range("BvhRetargeter: frame index out of range");
  }
  if (q_inout.size() != output_to_joint_.size()) {
    throw std::invalid_argument("BvhRetargeter: q_inout size does not match joint_names");
  }

  // 熱啟動：以呼叫端給的值作為初始值，其他 joint 從 0 開始
  std::vector<double> q(joints_.size(), 0.0);
  for (size_t i = 0; i < output_to_joint_.size(); ++i) {
    if (std::isfinite(q_inout[i])) {
      q[output_to_joint_[i]] = q_inout[i];
    }
  }

  // BVH 各節點的全域姿態（父節點一定在前）
  const auto & nodes = clip_->joints;
  std::vector<Eigen::Matrix3d> node_rotation(nodes.size());
  for (size_t n = 0; n < nodes.size(); ++n) {
    const Eigen::Matrix3d local = clip_->local_rotation(frame, n);
    node_rotation[n] = nodes[n].parent < 0 ?
      local : Eigen::Matrix3d(node_rotation[static_cast<size_t>(nodes[n].parent)] * local);
  }
  const Eigen::Matrix3d base_inverse = node_rotation[base_node_].transpose();

  Stats stats;
  std::vector<Eigen::Matrix3d> link_rotation(links_.size(), Eigen::Matrix3d::Identity());

  for (const auto & step : steps_) {
    if (!step.is_group) {
      const auto & joint = joints_[step.index];
      link_rotation[joint.child_link] = link_rotation[joint.parent_link] * joint.origin_rotation;
      continue;
    }

    const auto & group = groups_[step.index];
    const Eigen::Matrix3d parent_rotation = link_rotation[joints_[group.joints.front()].parent_link];

    double residual = 0.0;
    if (group.bvh_node) {
      const Eigen::Matrix3d target =
        base_inverse * node_rotation[*group.bvh_node] * group.post_rotation;
      GroupState warm;
      for (size_t k = 0; k < group.joints.size(); ++k) {
        warm[k] = q[group.joints[k]];
      }
      const auto result = solve_group(group, parent_rotation, target, warm);
      for (size_t k = 0; k < group.joints.size(); ++k) {
        q[group.joints[k]] = result.q[k];
      }
      residual = result.residual;
      stats.clamped += result.clamped;
    }

    // 以最終角度更新組內各 link 的姿態，供下游的組使用
    Eigen::Matrix3d rotation = parent_rotation;
    for (const auto index : group.joints) {
      const auto & joint = joints_[index];
      rotation = rotation * joint.origin_rotation *
        Eigen::AngleAxisd(q[index], joint.axis).toRotationMatrix();
      link_rotation[joint.child_link] = rotation;
    }
    stats.group_residuals.push_back(residual);
    stats.max_residual = std::max(stats.max_residual, residual);
  }

  for (size_t i = 0; i < output_to_joint_.size(); ++i) {
    q_inout[i] = covered_[i] ? q[output_to_joint_[i]] : std::numeric_limits<double>::quiet_NaN();
  }
  return stats;
}

}  // namespace graphen::motion
