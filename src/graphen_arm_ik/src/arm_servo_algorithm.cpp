// 視覺伺服（末端位姿 + elbow）的 planner plugin：graphen_arm_ik/ArmServoAlgorithm。
//
// 以系統模式（type: plugin）使用，SwitchMode 進入、hold 類的模式停止。目標來自 controller 的
// ~/pose_targets（RobotState::pose_targets，名稱 = 手臂名稱），每個規劃週期對每隻手臂解一步 IK
// （ArmIk::step，速度 = gain × 誤差、單一 joint 不超過 max_joint_velocity）。
//
// 碰撞只偵測、不閃避：這一步會產生新的碰撞（或讓原本的碰撞更深）時，該手臂不走這一步（保持），
// 並 log 碰撞的 body 對。目標沒有收到或逾時時，該手臂保持。其他 joint 不寫入。
// 目標的 swivel 為 NaN 時，維持進入模式時的 swivel。
//
// 參數（<mode>.*）：
//   arm_ik_config       arm_ik.yaml（支援 package://）；bringup 依 robot 帶入。空字串時這個模式無法進入
//   arms                （選填）要伺服的手臂，預設為 arm_ik.yaml 中所有也在 pose_targets.names 的手臂
//   position_gain       末端位置誤差 → 速度（1/s，預設 2.0）
//   orientation_gain    （1/s，預設 2.0）
//   swivel_gain         （1/s，預設 2.0）
//   max_joint_velocity  單一 joint 的速度上限（rad/s，預設 1.0）；之後還有 <mode>.max_steps 的限制
//   collision_log_period  碰撞 log 的最短間隔（s，預設 1.0）
//
// TODO(verify-on-hw): 增益與速度上限需要在實機上調整；視覺的延遲會讓高增益振盪
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "graphen_arm_ik/arm_ik.hpp"
#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::arm_ik {

namespace {

double total_depth(const std::vector<Contact> & contacts) {
  double depth = 0.0;
  for (const auto & c : contacts) {
    depth += c.depth;
  }
  return depth;
}

// 碰撞的深度可能只因數值誤差而微幅增加；超過這個量（m）才算變深
constexpr double kDepthTolerance = 1e-4;

}  // namespace

class ArmServoAlgorithm : public locomotion::LocomotionAlgorithm {
public:
  void configure(const locomotion::Context & context) override {
    log_ = context.log;
    servos_.clear();
    ik_.reset();
    const auto config = context.params.get_string("arm_ik_config").value_or("");
    if (config.empty()) {
      // 不讓整個 controller configure 失敗（例如模型沒有 arm_ik.yaml），只有進入這個模式時失敗
      log_(locomotion::LogLevel::kWarn, "'arm_ik_config' is empty: this mode cannot be entered.");
      return;
    }

    gains_.position_gain = positive(context, "position_gain", gains_.position_gain);
    gains_.orientation_gain = positive(context, "orientation_gain", gains_.orientation_gain);
    gains_.swivel_gain = positive(context, "swivel_gain", gains_.swivel_gain);
    gains_.max_joint_velocity = positive(context, "max_joint_velocity", gains_.max_joint_velocity);
    collision_log_period_ = positive(context, "collision_log_period", 1.0);

    const auto model = ArmModel::load_cached(config);
    if (context.pose_target_frame != model->config().root_body) {
      throw std::invalid_argument(
              "controller 'pose_targets.frame' is '" + context.pose_target_frame +
              "' but the IK targets are in '" + model->config().root_body + "' (root_body of " +
              config + ")");
    }

    std::vector<std::string> arms;
    if (const auto listed = context.params.get_string_array("arms")) {
      arms = *listed;
    } else {
      for (const auto & arm : model->arms()) {
        const auto & name = arm.definition.name;
        if (std::find(context.pose_target_names.begin(), context.pose_target_names.end(), name) !=
          context.pose_target_names.end())
        {
          arms.push_back(name);
        }
      }
    }
    if (arms.empty()) {
      throw std::invalid_argument(
              "no arm to servo: none of the arms in " + config +
              " is listed in the controller's 'pose_targets.names'");
    }

    // 所有受控 joint 在模型中的位址：每週期把量測值寫進模型（腰、頭等會影響 root_body 與碰撞）
    joint_qpos_.clear();
    for (const auto & joint : context.joint_names) {
      joint_qpos_.push_back(model->qpos_address(joint));
    }

    for (const auto & name : arms) {
      Servo servo;
      servo.name = name;
      servo.arm = model->arm_index(name);
      if (servo.arm < 0) {
        throw std::invalid_argument("'arms': '" + name + "' is not an arm of " + config);
      }
      const auto target = std::find(
        context.pose_target_names.begin(), context.pose_target_names.end(), name);
      if (target == context.pose_target_names.end()) {
        throw std::invalid_argument(
                "'arms': '" + name + "' is not listed in the controller's 'pose_targets.names'");
      }
      servo.target = static_cast<size_t>(std::distance(context.pose_target_names.begin(), target));
      const auto & definition = model->arms()[static_cast<size_t>(servo.arm)].definition;
      for (size_t j = 0; j < kArmDof; ++j) {
        const auto it = std::find(
          context.joint_names.begin(), context.joint_names.end(), definition.joints[j]);
        if (it == context.joint_names.end()) {
          throw std::invalid_argument(
                  "arm '" + name + "': joint '" + definition.joints[j] + "' is not a commanded joint");
        }
        servo.joints[j] = static_cast<size_t>(std::distance(context.joint_names.begin(), it));
      }
      servos_.push_back(servo);
    }

    ik_ = std::make_unique<ArmIk>(model);
    contacts_.reserve(64);
    next_contacts_.reserve(64);
    std::string names;
    for (const auto & servo : servos_) {
      names += (names.empty() ? "" : ", ") + servo.name;
    }
    log_(locomotion::LogLevel::kInfo,
      "Arm servo on [" + names + "] with " + model->config().mjcf + ", collision " +
      (model->config().collision.enabled ? "checked" : "not checked") + ".");
  }

  void on_enter(const locomotion::RobotState & state) override {
    if (!ik_) {
      throw std::runtime_error("'arm_ik_config' is not set for this mode");
    }
    if (state.pose_targets.size() <= max_target_index()) {
      throw std::runtime_error("the controller provides no pose targets");
    }
    apply_state(state);
    for (auto & servo : servos_) {
      ik_->set_arm(servo.arm, current_positions(servo, state));
    }
    for (auto & servo : servos_) {
      // 目標沒有指定 swivel 時維持這個值，手肘不會在 null space 中慢慢漂移
      servo.entry_swivel = ik_->forward(servo.arm).swivel;
      servo.blocked = false;
      servo.since_log = collision_log_period_;
    }
  }

  void update(
    double dt, const locomotion::RobotState & state, locomotion::JointCommand & command) override
  {
    apply_state(state);
    for (auto & servo : servos_) {
      servo.current = current_positions(servo, state);
      ik_->set_arm(servo.arm, servo.current);
    }
    for (auto & servo : servos_) {
      servo.since_log += dt;
      const auto & goal = state.pose_targets[servo.target];
      if (!goal.active) {
        continue;  // 保持（NaN）
      }
      Target target;
      target.position = Eigen::Vector3d(goal.position[0], goal.position[1], goal.position[2]);
      target.orientation = Eigen::Quaterniond(
        goal.orientation[3], goal.orientation[0], goal.orientation[1], goal.orientation[2]);
      target.swivel = std::isfinite(goal.swivel) ? goal.swivel : servo.entry_swivel;
      const auto next = ik_->step(servo.arm, target, servo.current, dt, gains_);

      if (ik_->check_collisions(servo.arm, next_contacts_) > 0) {
        // 只在碰撞變多 / 變深時擋下：已經在碰撞中的手臂仍然可以往外移動
        ik_->set_arm(servo.arm, servo.current);
        const size_t current_count = ik_->check_collisions(servo.arm, contacts_);
        if (current_count == 0 ||
          total_depth(next_contacts_) > total_depth(contacts_) + kDepthTolerance)
        {
          report_collision(servo);
          continue;  // 不走這一步；模型已回到 current
        }
        ik_->set_arm(servo.arm, next);
      }
      servo.blocked = false;
      for (size_t j = 0; j < kArmDof; ++j) {
        command.positions[servo.joints[j]] = next[j];
      }
    }
  }

private:
  struct Servo {
    std::string name;
    int arm{-1};
    size_t target{0};                   // RobotState::pose_targets 的 index
    std::array<size_t, kArmDof> joints{};  // RobotState / JointCommand 的 index
    double entry_swivel{kNaN};
    ArmVector current{};
    bool blocked{false};
    double since_log{0.0};
  };

  static double positive(const locomotion::Context & context, const std::string & name, double fallback) {
    const double value = context.params.get_double(name).value_or(fallback);
    if (!(value > 0.0) || !std::isfinite(value)) {
      throw std::invalid_argument("'" + name + "' must be a positive number");
    }
    return value;
  }

  size_t max_target_index() const {
    size_t index = 0;
    for (const auto & servo : servos_) {
      index = std::max(index, servo.target);
    }
    return index;
  }

  // 量測的關節角度寫進模型（手臂之後會被 current_positions 蓋過）
  void apply_state(const locomotion::RobotState & state) {
    const size_t n = std::min(joint_qpos_.size(), state.measured_positions.size());
    for (size_t i = 0; i < n; ++i) {
      ik_->set_position(joint_qpos_[i], state.measured_positions[i]);
    }
  }

  // 從 planner 的命令位置繼續（已經過 max_steps 限制，不會累積落後）；沒有時用量測值
  static ArmVector current_positions(const Servo & servo, const locomotion::RobotState & state) {
    ArmVector q{};
    for (size_t j = 0; j < kArmDof; ++j) {
      const size_t index = servo.joints[j];
      const double commanded =
        index < state.commanded_positions.size() ? state.commanded_positions[index] : kNaN;
      q[j] = std::isfinite(commanded) ? commanded : state.measured_positions[index];
    }
    return q;
  }

  void report_collision(Servo & servo) {
    if (servo.blocked && servo.since_log < collision_log_period_) {
      return;
    }
    std::string pairs;
    for (const auto & c : next_contacts_) {
      pairs += (pairs.empty() ? "" : ", ") + ik_->model().body_name(c.body1) + "|" +
        ik_->model().body_name(c.body2);
    }
    log_(locomotion::LogLevel::kWarn,
      "Arm '" + servo.name + "' stopped: the next step would collide" +
      (pairs.empty() ? std::string{} : " (" + pairs + ")") + ".");
    servo.blocked = true;
    servo.since_log = 0.0;
  }

  locomotion::Logger log_;
  std::unique_ptr<ArmIk> ik_;
  std::vector<Servo> servos_;
  std::vector<int> joint_qpos_;
  ServoGains gains_;
  double collision_log_period_{1.0};
  std::vector<Contact> contacts_;       // 目前姿態的碰撞
  std::vector<Contact> next_contacts_;  // 下一步的碰撞
};

}  // namespace graphen::arm_ik

PLUGINLIB_EXPORT_CLASS(graphen::arm_ik::ArmServoAlgorithm, graphen::locomotion::LocomotionAlgorithm)
