#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <urdf_parser/urdf_parser.h>

#include "graphen_locomotion/base_state_estimator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::locomotion {

namespace {

Eigen::Quaterniond to_quaternion(const urdf::Rotation & rotation) {
  return Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);
}

// 沿 fixed joint 從 imu_frame 往上走到 base_frame，求出 IMU 相對機身的固定旋轉 R_bi
Eigen::Quaterniond fixed_rotation(
  const std::string & urdf_xml, const std::string & base_frame, const std::string & imu_frame) {
  if (urdf_xml.empty()) {
    throw std::invalid_argument("robot_description is empty");
  }
  const auto model = urdf::parseURDF(urdf_xml);
  if (!model) {
    throw std::invalid_argument("failed to parse robot_description");
  }
  if (!model->getLink(base_frame)) {
    throw std::invalid_argument("base_frame '" + base_frame + "' is not a link in the URDF");
  }
  auto link = model->getLink(imu_frame);
  if (!link) {
    throw std::invalid_argument("imu_frame '" + imu_frame + "' is not a link in the URDF");
  }

  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  while (link->name != base_frame) {
    const auto joint = link->parent_joint;
    if (!joint) {
      throw std::invalid_argument(
        "imu_frame '" + imu_frame + "' is not a descendant of base_frame '" + base_frame + "'");
    }
    // 中間有可動 joint 時，IMU 與機身的相對姿態不是固定的
    if (joint->type != urdf::Joint::FIXED) {
      throw std::invalid_argument(
        "joint '" + joint->name + "' between base_frame and imu_frame is not fixed");
    }
    rotation = to_quaternion(joint->parent_to_joint_origin_transform.rotation) * rotation;
    link = model->getLink(joint->parent_link_name);
  }
  return rotation.normalized();
}

}  // namespace

// 只用 IMU 的機身狀態估測器。
//
// - 角速度：IMU 角速度轉到機身座標系
// - 重力投影：由 IMU 姿態求出機身姿態 R_wb，取 R_wbᵀ·[0, 0, -1]
// - 線速度：在世界座標系對「扣除重力的加速度」做 leaky 積分，再轉到機身座標系。
//   積分值以時間常數 leak_time_constant 衰減回 0，所以只反映短時間的速度變化，
//   無法提供長時間準確的速度（bias 與姿態誤差會被積分進去）。之後應改用 IMU + 腿部運動學的估測器
// - 忽略 IMU 相對機身的槓桿臂效應（ω×(ω×r) 與角加速度項）
//
// 參數（base_estimator.*）：
//   base_frame          機身 link（預設 base_link）
//   imu_frame           IMU link（預設 imu_link），必須以 fixed joint 連到 base_frame
//   leak_time_constant  線速度積分的衰減時間常數（秒，預設 2.0，必須 > 0）
//   gravity             重力加速度（m/s²，預設 9.81）
class ImuLeakyEstimator : public BaseStateEstimator {
public:
  void configure(const Context & context) override {
    const auto base_frame = context.params.get_string("base_frame").value_or("base_link");
    const auto imu_frame = context.params.get_string("imu_frame").value_or("imu_link");
    // TODO(verify-on-hw): imu_link 相對 base_link 的安裝姿態（xacro 的 imu_rpy）尚未以實機確認
    base_from_imu_ = fixed_rotation(context.robot_description, base_frame, imu_frame);
    time_constant_ = context.params.get_double("leak_time_constant").value_or(2.0);
    if (!(time_constant_ > 0.0)) {
      throw std::invalid_argument("'leak_time_constant' must be greater than zero");
    }
    gravity_ = context.params.get_double("gravity").value_or(9.81);
    if (!(gravity_ > 0.0)) {
      throw std::invalid_argument("'gravity' must be greater than zero");
    }
  }

  void reset() override {
    velocity_world_.setZero();
  }

  void update(double dt, const RobotState & state, BaseState & base) override {
    const auto & imu = state.imu;
    const Eigen::Quaterniond world_from_imu_raw(
      imu.orientation[3], imu.orientation[0], imu.orientation[1], imu.orientation[2]);
    if (!imu.valid || world_from_imu_raw.norm() < 1e-6) {
      base = BaseState{};
      velocity_world_.setZero();
      return;
    }
    // TODO(verify-on-hw): IMU16460 姿態的參考座標系（NED / ENU）與 Euler 順序尚未以實機確認；
    // 這裡假設 orientation 為 IMU 相對 z 軸朝上的世界座標系
    const Eigen::Quaterniond world_from_imu = world_from_imu_raw.normalized();
    const Eigen::Quaterniond world_from_base = world_from_imu * base_from_imu_.conjugate();
    const Eigen::Matrix3d base_from_world = world_from_base.toRotationMatrix().transpose();

    const Eigen::Vector3d angular_velocity = base_from_imu_ * Eigen::Vector3d(
      imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
    const Eigen::Vector3d gravity = base_from_world * Eigen::Vector3d(0.0, 0.0, -1.0);

    // 加速度計量到的是比力（靜止時為 +g 向上），扣掉重力才是實際加速度
    const Eigen::Vector3d specific_force(
      imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]);
    const Eigen::Vector3d acceleration_world =
      world_from_imu * specific_force - Eigen::Vector3d(0.0, 0.0, gravity_);
    // dt 為 0 表示同一筆量測被讀了兩次，不重複積分
    if (dt > 0.0) {
      velocity_world_ = velocity_world_ * std::exp(-dt / time_constant_) + acceleration_world * dt;
    }
    const Eigen::Vector3d linear_velocity = base_from_world * velocity_world_;

    for (size_t i = 0; i < 3; ++i) {
      base.linear_velocity[i] = linear_velocity[static_cast<Eigen::Index>(i)];
      base.angular_velocity[i] = angular_velocity[static_cast<Eigen::Index>(i)];
      base.projected_gravity[i] = gravity[static_cast<Eigen::Index>(i)];
    }
    base.valid = true;
  }

private:
  Eigen::Quaterniond base_from_imu_{Eigen::Quaterniond::Identity()};  // R_bi
  double time_constant_{2.0};
  double gravity_{9.81};
  Eigen::Vector3d velocity_world_{Eigen::Vector3d::Zero()};
};

}  // namespace graphen::locomotion

PLUGINLIB_EXPORT_CLASS(
  graphen::locomotion::ImuLeakyEstimator,
  graphen::locomotion::BaseStateEstimator)
