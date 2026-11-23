#include "graphen_arm_ik/arm_ik.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace graphen::arm_ik {

namespace {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6x7 = Eigen::Matrix<double, 6, kArmDof>;
using Matrix3x7 = Eigen::Matrix<double, 3, kArmDof>;
using VectorArm = Eigen::Matrix<double, kArmDof, 1>;

// 比這個短的投影向量視為與 swivel 的參考軸平行（swivel 無定義）
constexpr double kDegenerateLength = 1e-6;

Eigen::Vector3d vec3(const mjtNum * p) {return Eigen::Vector3d(p[0], p[1], p[2]);}

Eigen::Matrix3d mat3(const mjtNum * p) {
  Eigen::Matrix3d r;
  r << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8];
  return r;
}

// v 在 axis（單位向量）垂直平面上的單位投影；太短時回傳 false
bool perpendicular(const Eigen::Vector3d & v, const Eigen::Vector3d & axis, Eigen::Vector3d & out) {
  out = v - v.dot(axis) * axis;
  const double norm = out.norm();
  if (norm < kDegenerateLength) {
    return false;
  }
  out /= norm;
  return true;
}

}  // namespace

double wrap_angle(double angle) {
  return std::remainder(angle, 2.0 * M_PI);
}

ArmIk::ArmIk(std::shared_ptr<const ArmModel> model)
: model_(std::move(model)), m_(model_->model())
{
  d_ = mj_makeData(m_);
  jacp_.resize(3 * static_cast<size_t>(m_->nv));
  jacr_.resize(3 * static_cast<size_t>(m_->nv));
  reset();
}

ArmIk::~ArmIk() {
  if (d_) {
    mj_deleteData(d_);
  }
}

void ArmIk::reset() {
  mj_resetData(m_, d_);
  for (size_t i = 0; i < model_->arms().size(); ++i) {
    set_arm(static_cast<int>(i), model_->arms()[i].definition.seed);
  }
  update_kinematics();
}

void ArmIk::set_position(int qpos_adr, double value) {
  if (qpos_adr >= 0 && qpos_adr < m_->nq && std::isfinite(value)) {
    d_->qpos[qpos_adr] = value;
  }
}

void ArmIk::set_arm(int arm, const ArmVector & positions) {
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  for (size_t i = 0; i < kArmDof; ++i) {
    d_->qpos[a.qpos_adr[i]] = std::clamp(positions[i], a.lower[i], a.upper[i]);
  }
}

ArmVector ArmIk::arm_positions(int arm) const {
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  ArmVector out{};
  for (size_t i = 0; i < kArmDof; ++i) {
    out[i] = d_->qpos[a.qpos_adr[i]];
  }
  return out;
}

void ArmIk::update_kinematics() {
  mj_kinematics(m_, d_);
}

Eigen::Isometry3d ArmIk::root_pose() const {
  const int root = model_->root_body();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = vec3(d_->xpos + 3 * root);
  pose.linear() = mat3(d_->xmat + 9 * root);
  return pose;
}

Eigen::Isometry3d ArmIk::tip_pose(int arm) const {
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = vec3(d_->xpos + 3 * a.tip_body);
  pose.linear() = mat3(d_->xmat + 9 * a.tip_body);
  return pose * a.definition.tip_offset;
}

double ArmIk::swivel_world(int arm) const {
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  const Eigen::Vector3d shoulder = vec3(d_->xpos + 3 * a.shoulder_body);
  const Eigen::Vector3d elbow = vec3(d_->xpos + 3 * a.elbow_body);
  // swivel 的軸取 tip body 的原點（手腕中心），不含工具點偏移
  Eigen::Vector3d axis = vec3(d_->xpos + 3 * a.tip_body) - shoulder;
  if (axis.norm() < kDegenerateLength) {
    return kNaN;
  }
  axis.normalize();
  const Eigen::Vector3d reference_world = root_pose().linear() * a.definition.swivel_reference;
  Eigen::Vector3d ref;
  Eigen::Vector3d v;
  if (!perpendicular(reference_world, axis, ref) || !perpendicular(elbow - shoulder, axis, v)) {
    return kNaN;
  }
  return std::atan2(ref.cross(v).dot(axis), ref.dot(v));
}

ArmPose ArmIk::forward(int arm) {
  update_kinematics();
  const Eigen::Isometry3d local = root_pose().inverse() * tip_pose(arm);
  ArmPose pose;
  pose.position = local.translation();
  pose.orientation = Eigen::Quaterniond(local.linear()).normalized();
  pose.swivel = swivel_world(arm);
  return pose;
}

ArmIk::Errors ArmIk::errors(int arm, const Eigen::Isometry3d & target_world) const {
  const Eigen::Isometry3d tip = tip_pose(arm);
  Errors e;
  e.position = target_world.translation() - tip.translation();
  // 目標相對目前姿態的旋轉（世界座標），取最短的那一邊
  Eigen::Quaterniond q(target_world.linear() * tip.linear().transpose());
  if (q.w() < 0.0) {
    q.coeffs() = -q.coeffs();
  }
  const Eigen::AngleAxisd angle_axis(q.normalized());
  e.orientation = angle_axis.angle() * angle_axis.axis();
  return e;
}

Eigen::Matrix<double, kArmDof, 1> ArmIk::compute_step(
  int arm, const Eigen::Matrix<double, 6, 1> & error, double target_swivel, double swivel_scale)
{
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  const auto & solver = model_->config().solver;
  const int nv = m_->nv;

  // mj_jac 以子樹質心為參考點，需要 mj_comPos
  mj_comPos(m_, d_);
  const Eigen::Vector3d tip = tip_pose(arm).translation();
  const mjtNum point[3] = {tip.x(), tip.y(), tip.z()};
  mj_jac(m_, d_, jacp_.data(), jacr_.data(), point, a.tip_body);
  Matrix6x7 jac;
  for (size_t c = 0; c < kArmDof; ++c) {
    const int dof = a.dof_adr[c];
    for (int r = 0; r < 3; ++r) {
      jac(r, static_cast<int>(c)) = jacp_[static_cast<size_t>(r * nv + dof)];
      jac(r + 3, static_cast<int>(c)) =
        solver.orientation_weight * jacr_[static_cast<size_t>(r * nv + dof)];
    }
  }

  // DLS：dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e（接近奇異點時仍穩定）
  const Eigen::Matrix<double, 6, 6> jjt =
    jac * jac.transpose() + solver.damping * solver.damping * Eigen::Matrix<double, 6, 6>::Identity();
  VectorArm dq = jac.transpose() * jjt.ldlt().solve(error);

  if (!std::isfinite(target_swivel)) {
    return dq;
  }
  const double current = swivel_world(arm);
  if (!std::isfinite(current)) {
    return dq;
  }
  const double delta = wrap_angle(target_swivel - current);
  if (std::abs(delta) < 1e-12) {
    return dq;
  }
  // 手肘目標點：目前的手肘繞「肩 → 腕」軸轉 delta
  const Eigen::Vector3d shoulder = vec3(d_->xpos + 3 * a.shoulder_body);
  const Eigen::Vector3d elbow = vec3(d_->xpos + 3 * a.elbow_body);
  const Eigen::Vector3d axis = (vec3(d_->xpos + 3 * a.tip_body) - shoulder).normalized();
  const Eigen::Vector3d goal = shoulder + Eigen::AngleAxisd(delta, axis) * (elbow - shoulder);

  // 主任務的 1 維 null space：V 的最後一個 column
  const Eigen::JacobiSVD<Matrix6x7> svd(jac, Eigen::ComputeFullV);
  const VectorArm null_direction = svd.matrixV().col(kArmDof - 1);

  mj_jacBody(m_, d_, jacp_.data(), nullptr, a.elbow_body);
  Matrix3x7 jac_elbow;
  for (size_t c = 0; c < kArmDof; ++c) {
    for (int r = 0; r < 3; ++r) {
      jac_elbow(r, static_cast<int>(c)) = jacp_[static_cast<size_t>(r * nv + a.dof_adr[c])];
    }
  }
  const Eigen::Vector3d elbow_direction = jac_elbow * null_direction;
  const double norm_sq = elbow_direction.squaredNorm();
  if (norm_sq > 1e-12) {
    // 沿 n 走的純量步長：讓手肘最接近目標點（最小平方）
    dq += swivel_scale * elbow_direction.dot(goal - elbow) / norm_sq * null_direction;
  }
  return dq;
}

void ArmIk::apply(int arm, const Eigen::Matrix<double, kArmDof, 1> & dq) {
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  for (size_t i = 0; i < kArmDof; ++i) {
    const auto adr = a.qpos_adr[i];
    d_->qpos[adr] = std::clamp(d_->qpos[adr] + dq(static_cast<int>(i)), a.lower[i], a.upper[i]);
  }
  update_kinematics();
}

void ArmIk::iterate(
  int arm, const Eigen::Isometry3d & target_world, double target_swivel, bool use_swivel)
{
  const auto & solver = model_->config().solver;
  const bool control_swivel = use_swivel && std::isfinite(target_swivel);
  for (int i = 0; i < solver.iterations; ++i) {
    const Errors e = errors(arm, target_world);
    bool done = e.position.norm() < solver.position_tolerance &&
      e.orientation.norm() < solver.orientation_tolerance;
    if (done && control_swivel) {
      const double current = swivel_world(arm);
      // swivel 無定義（手臂與參考軸平行）時無法控制，視為完成
      done = !std::isfinite(current) ||
        std::abs(wrap_angle(target_swivel - current)) < solver.swivel_tolerance;
    }
    if (done) {
      return;
    }
    Vector6 error;
    error << e.position, solver.orientation_weight * e.orientation;
    VectorArm dq = compute_step(arm, error, control_swivel ? target_swivel : kNaN, 1.0);
    dq = dq.cwiseMax(-solver.max_step).cwiseMin(solver.max_step);
    apply(arm, dq);
  }
}

IkResult ArmIk::solve_once(
  int arm, const Eigen::Isometry3d & target_world, double target_swivel, const ArmVector & seed)
{
  const auto & solver = model_->config().solver;
  set_arm(arm, seed);
  update_kinematics();
  iterate(arm, target_world, target_swivel, true);
  Errors e = errors(arm, target_world);
  const bool pose_ok = e.position.norm() < solver.position_tolerance &&
    e.orientation.norm() < solver.orientation_tolerance;
  if (!pose_ok && std::isfinite(target_swivel)) {
    // 末端優先：swivel 次任務在 clamp 下可能拖住主任務，最後只跑主任務收尾
    iterate(arm, target_world, kNaN, false);
    e = errors(arm, target_world);
  }

  IkResult result;
  result.positions = arm_positions(arm);
  result.position_error = e.position.norm();
  result.orientation_error = e.orientation.norm();
  result.converged = result.position_error < solver.position_tolerance &&
    result.orientation_error < solver.orientation_tolerance;
  result.swivel = swivel_world(arm);
  if (std::isfinite(target_swivel)) {
    result.swivel_error = std::isfinite(result.swivel) ?
      std::abs(wrap_angle(target_swivel - result.swivel)) : kNaN;
    result.swivel_reached = std::isfinite(result.swivel_error) &&
      result.swivel_error < solver.swivel_tolerance;
  }
  return result;
}

IkResult ArmIk::solve(int arm, const Target & target, const ArmVector & seed, bool allow_restarts) {
  const auto & solver = model_->config().solver;
  const auto & a = model_->arms()[static_cast<size_t>(arm)];
  Eigen::Isometry3d target_local = Eigen::Isometry3d::Identity();
  target_local.translation() = target.position;
  target_local.linear() = target.orientation.normalized().toRotationMatrix();
  // 只有手臂的 joint 會變，root_body 的位姿在求解期間固定
  update_kinematics();
  const Eigen::Isometry3d target_world = root_pose() * target_local;

  // 等級：末端收斂且 swivel 到達 > 末端收斂 > 沒有收斂。重試的解只有等級較高時才取代：
  // 同等級時保留從 seed 出發的解（離目前姿態最近，不會跳到另一個分支）
  const auto rank = [](const IkResult & r) {
      return r.converged ? (r.swivel_reached ? 2 : 1) : 0;
    };

  IkResult best = solve_once(arm, target_world, target.swivel, seed);
  // 固定的種子：同樣的輸入永遠得到同樣的結果
  std::mt19937 rng(0);
  for (int r = 0; allow_restarts && r < solver.restarts && rank(best) < 2; ++r) {
    ArmVector start{};
    for (size_t i = 0; i < kArmDof; ++i) {
      const double lo = std::isfinite(a.lower[i]) ? a.lower[i] : -M_PI;
      const double hi = std::isfinite(a.upper[i]) ? a.upper[i] : M_PI;
      start[i] = std::uniform_real_distribution<double>(lo, hi)(rng);
    }
    const IkResult candidate = solve_once(arm, target_world, target.swivel, start);
    if (rank(candidate) > rank(best)) {
      best = candidate;
    }
  }
  // 模型停在最好的解
  set_arm(arm, best.positions);
  update_kinematics();
  return best;
}

ArmVector ArmIk::step(
  int arm, const Target & target, const ArmVector & current, double dt, const ServoGains & gains)
{
  set_arm(arm, current);
  update_kinematics();
  if (!(dt > 0.0)) {
    return arm_positions(arm);
  }
  const auto & solver = model_->config().solver;
  Eigen::Isometry3d target_local = Eigen::Isometry3d::Identity();
  target_local.translation() = target.position;
  target_local.linear() = target.orientation.normalized().toRotationMatrix();
  const Errors e = errors(arm, root_pose() * target_local);

  Vector6 error;
  error << gains.position_gain * dt * e.position,
    solver.orientation_weight * gains.orientation_gain * dt * e.orientation;
  VectorArm dq = compute_step(arm, error, target.swivel, std::min(1.0, gains.swivel_gain * dt));
  const double limit = gains.max_joint_velocity * dt;
  dq = dq.cwiseMax(-limit).cwiseMin(limit);
  apply(arm, dq);
  return arm_positions(arm);
}

size_t ArmIk::check_collisions(int arm, std::vector<Contact> & out) {
  out.clear();
  const auto & options = model_->config().collision;
  if (!options.enabled) {
    return 0;
  }
  update_kinematics();
  mj_collision(m_, d_);
  const auto & arms = model_->arms();
  for (int i = 0; i < d_->ncon; ++i) {
    const auto & contact = d_->contact[i];
    const double depth = -contact.dist;
    if (depth <= options.min_penetration) {
      continue;
    }
    const int b1 = m_->geom_bodyid[contact.geom[0]];
    const int b2 = m_->geom_bodyid[contact.geom[1]];
    if (model_->pair_allowed(b1, b2)) {
      continue;
    }
    bool involved = false;
    if (arm >= 0) {
      const auto & bodies = arms[static_cast<size_t>(arm)].bodies;
      involved = bodies[static_cast<size_t>(b1)] || bodies[static_cast<size_t>(b2)];
    } else {
      for (const auto & a : arms) {
        involved = involved || a.bodies[static_cast<size_t>(b1)] || a.bodies[static_cast<size_t>(b2)];
      }
    }
    if (!involved) {
      continue;
    }
    // 同一對 body 可能有多個接觸點：合併成一筆，保留最深的
    const auto same = std::find_if(out.begin(), out.end(), [&](const Contact & c) {
        return (c.body1 == b1 && c.body2 == b2) || (c.body1 == b2 && c.body2 == b1);
      });
    if (same != out.end()) {
      same->depth = std::max(same->depth, depth);
    } else {
      out.push_back({b1, b2, depth});
    }
  }
  return out.size();
}

}  // namespace graphen::arm_ik
