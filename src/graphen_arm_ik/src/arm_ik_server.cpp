// 手臂 IK 的 ROS service（兩套 UI 共用：MuJoCo viewer 的示教模式、外部 ROS 程式的指令）。
// 只做計算，不送任何指令給 controller。
//
// 參數：
//   arm_ik_config        arm_ik.yaml（支援 package://）；bringup 依 robot 帶入
//   joint_states_topic   全身目前角度的來源（預設 /joint_states），給碰撞檢查與預設的 seed
// service：
//   ~/solve     (graphen_interfaces/srv/SolveArmIk)
//   ~/forward   (graphen_interfaces/srv/ArmForward)
//   ~/tcp_jog   (graphen_interfaces/srv/TcpJog)：以基準姿態為起點沿一個軸單步，UI 再以 PlayChoreo 播放
//
// 使用單執行緒 executor：subscription 與 service callback 不會同時執行，ArmIk 不需要鎖
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "graphen_arm_ik/arm_ik.hpp"
#include "graphen_interfaces/srv/arm_forward.hpp"
#include "graphen_interfaces/srv/solve_arm_ik.hpp"
#include "graphen_interfaces/srv/tcp_jog.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace graphen::arm_ik {

namespace {

using SolveArmIk = graphen_interfaces::srv::SolveArmIk;
using ArmForward = graphen_interfaces::srv::ArmForward;
using TcpJog = graphen_interfaces::srv::TcpJog;
using JointState = sensor_msgs::msg::JointState;

std::vector<std::string> collision_names(const ArmModel & model, const std::vector<Contact> & contacts) {
  std::vector<std::string> names;
  names.reserve(contacts.size());
  for (const auto & c : contacts) {
    names.push_back(model.body_name(c.body1) + "|" + model.body_name(c.body2));
  }
  return names;
}

std::string join(const std::vector<std::string> & items) {
  std::string text;
  for (const auto & item : items) {
    text += (text.empty() ? "" : ", ") + item;
  }
  return text;
}

}  // namespace

class ArmIkServer : public rclcpp::Node {
public:
  ArmIkServer()
  : Node("arm_ik_server") {
    const auto config_path = declare_parameter<std::string>("arm_ik_config", "");
    const auto topic = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    if (config_path.empty()) {
      throw std::invalid_argument("parameter 'arm_ik_config' is empty");
    }
    model_ = ArmModel::load(load_arm_config(config_path));
    ik_ = std::make_unique<ArmIk>(model_);
    contacts_.reserve(64);

    std::string arms;
    for (const auto & arm : model_->arms()) {
      arms += (arms.empty() ? "" : ", ") + arm.definition.name;
    }
    std::string allowed;
    for (const auto & [a, b] : model_->initial_contacts()) {
      allowed += (allowed.empty() ? "" : ", ") + a + "|" + b;
    }
    RCLCPP_INFO(get_logger(), "Loaded %s: arms [%s], root_body %s, collision %s",
      model_->config().mjcf.c_str(), arms.c_str(), model_->config().root_body.c_str(),
      model_->config().collision.enabled ? "enabled" : "disabled");
    if (!allowed.empty()) {
      RCLCPP_INFO(get_logger(), "Contacts allowed because present in the initial pose: %s", allowed.c_str());
    }

    state_subscriber_ = create_subscription<JointState>(topic, rclcpp::SensorDataQoS(),
        [this](JointState::ConstSharedPtr msg) {latest_state_ = msg;});
    solve_service_ = create_service<SolveArmIk>("~/solve",
        [this](const std::shared_ptr<SolveArmIk::Request> request,
        std::shared_ptr<SolveArmIk::Response> response) {on_solve(*request, *response);});
    forward_service_ = create_service<ArmForward>("~/forward",
        [this](const std::shared_ptr<ArmForward::Request> request,
        std::shared_ptr<ArmForward::Response> response) {on_forward(*request, *response);});
    tcp_jog_service_ = create_service<TcpJog>("~/tcp_jog",
        [this](const std::shared_ptr<TcpJog::Request> request,
        std::shared_ptr<TcpJog::Response> response) {on_tcp_jog(*request, *response);});
  }

private:
  // 模型回到初始姿態，再套用 state（請求的，或最新的 /joint_states）。回傳實際使用的來源說明
  std::string apply_state(const JointState & requested) {
    ik_->reset();
    const JointState * state = nullptr;
    std::string source;
    if (!requested.name.empty()) {
      state = &requested;
      source = "request state";
    } else if (latest_state_) {
      state = latest_state_.get();
      source = "latest joint states";
    } else {
      return "no joint states received, using the initial pose";
    }
    const size_t n = std::min(state->name.size(), state->position.size());
    for (size_t i = 0; i < n; ++i) {
      ik_->set_position(model_->qpos_address(state->name[i]), state->position[i]);
    }
    return source;
  }

  // 該手臂的 joint 是否都在 state 中（決定預設 seed 用目前角度還是設定檔的 seed）
  bool state_has_arm(const JointState & requested, const ArmModel::Arm & arm) const {
    const JointState * state = !requested.name.empty() ? &requested : latest_state_.get();
    if (state == nullptr) {
      return false;
    }
    for (const auto & joint : arm.definition.joints) {
      if (std::find(state->name.begin(), state->name.end(), joint) == state->name.end()) {
        return false;
      }
    }
    return true;
  }

  std::optional<int> find_arm(const std::string & name, std::string & message) const {
    const int arm = model_->arm_index(name);
    if (arm < 0) {
      std::string arms;
      for (const auto & a : model_->arms()) {
        arms += (arms.empty() ? "" : ", ") + a.definition.name;
      }
      message = "unknown arm '" + name + "' (available: " + arms + ")";
      return std::nullopt;
    }
    return arm;
  }

  void on_solve(const SolveArmIk::Request & request, SolveArmIk::Response & response) {
    const auto arm = find_arm(request.arm, response.message);
    if (!arm) {
      RCLCPP_WARN(get_logger(), "solve rejected: %s", response.message.c_str());
      return;
    }
    const auto & definition = model_->arms()[static_cast<size_t>(*arm)].definition;
    response.joint_names.assign(definition.joints.begin(), definition.joints.end());

    const auto & frame = request.target.header.frame_id;
    if (!frame.empty() && frame != model_->config().root_body) {
      response.message = "target frame_id '" + frame + "' must be '" + model_->config().root_body +
        "' (transform it before calling)";
      RCLCPP_WARN(get_logger(), "solve rejected: %s", response.message.c_str());
      return;
    }
    const auto & p = request.target.pose.position;
    const auto & o = request.target.pose.orientation;
    const Eigen::Quaterniond orientation(o.w, o.x, o.y, o.z);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !orientation.coeffs().allFinite() || orientation.norm() < 1e-6)
    {
      response.message = "target pose is not finite or the quaternion is zero";
      RCLCPP_WARN(get_logger(), "solve rejected: %s", response.message.c_str());
      return;
    }
    if (!request.seed.empty() && request.seed.size() != kArmDof) {
      response.message = "seed must be empty or have " + std::to_string(kArmDof) + " values";
      RCLCPP_WARN(get_logger(), "solve rejected: %s", response.message.c_str());
      return;
    }

    const auto source = apply_state(request.state);
    ArmVector seed = definition.seed;
    if (!request.seed.empty()) {
      std::copy(request.seed.begin(), request.seed.end(), seed.begin());
    } else if (state_has_arm(request.state, model_->arms()[static_cast<size_t>(*arm)])) {
      seed = ik_->arm_positions(*arm);
    }

    Target target;
    target.position = Eigen::Vector3d(p.x, p.y, p.z);
    target.orientation = orientation.normalized();
    target.swivel = request.swivel;
    solve_into(*arm, target, seed, !request.local, response);
    response.message += " [" + source + "]";
    RCLCPP_DEBUG(get_logger(), "solve %s: %s", request.arm.c_str(), response.message.c_str());
  }

  void on_tcp_jog(const TcpJog::Request & request, TcpJog::Response & response) {
    const auto arm = find_arm(request.arm, response.message);
    if (!arm) {
      RCLCPP_WARN(get_logger(), "tcp_jog rejected: %s", response.message.c_str());
      return;
    }
    const auto & definition = model_->arms()[static_cast<size_t>(*arm)].definition;
    response.joint_names.assign(definition.joints.begin(), definition.joints.end());

    const auto reject = [&](const std::string & message) {
        response.message = message;
        RCLCPP_WARN(get_logger(), "tcp_jog rejected: %s", message.c_str());
      };
    if (request.axis > TcpJog::Request::AXIS_ELBOW) {
      return reject("unknown axis " + std::to_string(request.axis) + " (0-6: X, Y, Z, ROLL, PITCH, YAW, ELBOW)");
    }
    if (request.frame > TcpJog::Request::FRAME_ROOT) {
      return reject("unknown frame " + std::to_string(request.frame) + " (0: TOOL, 1: ROOT)");
    }
    if (!std::isfinite(request.delta)) {
      return reject("delta is not finite");
    }
    if (!request.reference.empty() && request.reference.size() != kArmDof) {
      return reject("reference must be empty or have " + std::to_string(kArmDof) + " values");
    }
    if (!std::all_of(request.reference.begin(), request.reference.end(),
      [](double v) {return std::isfinite(v);}))
    {
      return reject("reference has a value that is not finite");
    }

    const auto source = apply_state(request.state);
    ArmVector reference{};
    if (!request.reference.empty()) {
      std::copy(request.reference.begin(), request.reference.end(), reference.begin());
    } else if (state_has_arm(request.state, model_->arms()[static_cast<size_t>(*arm)])) {
      reference = ik_->arm_positions(*arm);
    } else {
      return reject("no reference given and the joint states have no '" + request.arm + "' arm joints");
    }

    // 基準姿態的末端位姿與 swivel，加上這一步的增量
    ik_->set_arm(*arm, reference);
    const auto pose = ik_->forward(*arm);
    Target target;
    target.position = pose.position;
    target.orientation = pose.orientation;
    target.swivel = pose.swivel;  // 平移 / 旋轉時維持手肘的絕對角度（NaN：基準姿態沒有定義，不限制）
    const bool tool = request.frame == TcpJog::Request::FRAME_TOOL;
    if (request.axis <= TcpJog::Request::AXIS_Z) {
      const Eigen::Vector3d step = Eigen::Vector3d::Unit(request.axis) * request.delta;
      target.position += tool ? Eigen::Vector3d(pose.orientation * step) : step;
    } else if (request.axis <= TcpJog::Request::AXIS_YAW) {
      const Eigen::Quaterniond turn(
        Eigen::AngleAxisd(request.delta, Eigen::Vector3d::Unit(request.axis - TcpJog::Request::AXIS_ROLL)));
      // 手腕座標：繞手腕目前的軸（右乘）；軀幹座標：繞固定的軸（左乘）
      target.orientation = (tool ? pose.orientation * turn : turn * pose.orientation).normalized();
    } else {
      if (!std::isfinite(pose.swivel)) {
        return reject("the elbow swivel is undefined at the reference pose (shoulder-wrist axis "
                 "parallel to the swivel reference)");
      }
      target.swivel = wrap_angle(pose.swivel + request.delta);
    }

    // 只從基準姿態迭代：單步不會跳到另一個分支
    solve_into(*arm, target, reference, false, response);
    double max_delta = 0.0;
    for (size_t j = 0; j < kArmDof; ++j) {
      max_delta = std::max(max_delta, std::abs(response.positions[j] - reference[j]));
    }
    response.max_joint_delta = max_delta;
    response.target.header.frame_id = model_->config().root_body;
    response.target.header.stamp = now();
    response.target.pose.position.x = target.position.x();
    response.target.pose.position.y = target.position.y();
    response.target.pose.position.z = target.position.z();
    response.target.pose.orientation.w = target.orientation.w();
    response.target.pose.orientation.x = target.orientation.x();
    response.target.pose.orientation.y = target.orientation.y();
    response.target.pose.orientation.z = target.orientation.z();
    response.message += " [" + source + "]";
    RCLCPP_DEBUG(get_logger(), "tcp_jog %s axis %u delta %.4f: %s", request.arm.c_str(),
      request.axis, request.delta, response.message.c_str());
  }

  // 解 IK、檢查解的碰撞，填入 SolveArmIk / TcpJog 共同的回應欄位。模型停在解的姿態
  template<typename Response>
  void solve_into(int arm, const Target & target, const ArmVector & seed, bool allow_restarts,
    Response & response)
  {
    const auto result = ik_->solve(arm, target, seed, allow_restarts);
    ik_->set_arm(arm, result.positions);
    ik_->check_collisions(arm, contacts_);

    response.positions.assign(result.positions.begin(), result.positions.end());
    response.position_error = result.position_error;
    response.orientation_error = result.orientation_error;
    response.swivel = result.swivel;
    response.swivel_error = result.swivel_error;
    response.collisions = collision_names(*model_, contacts_);
    response.success = result.converged && result.swivel_reached && contacts_.empty();
    if (!result.converged) {
      response.message = "not converged (unreachable or blocked by joint limits)";
    } else if (!result.swivel_reached) {
      response.message = "tip reached but the swivel is blocked by joint limits";
    } else if (!contacts_.empty()) {
      response.message = "solution collides: " + join(response.collisions);
    } else {
      response.message = "ok";
    }
  }

  void on_forward(const ArmForward::Request & request, ArmForward::Response & response) {
    const auto arm = find_arm(request.arm, response.message);
    if (!arm) {
      RCLCPP_WARN(get_logger(), "forward rejected: %s", response.message.c_str());
      return;
    }
    const auto & definition = model_->arms()[static_cast<size_t>(*arm)].definition;
    const auto source = apply_state(request.state);
    const auto pose = ik_->forward(*arm);
    ik_->check_collisions(*arm, contacts_);

    const auto positions = ik_->arm_positions(*arm);
    response.joint_names.assign(definition.joints.begin(), definition.joints.end());
    response.positions.assign(positions.begin(), positions.end());
    response.pose.header.frame_id = model_->config().root_body;
    response.pose.header.stamp = now();
    response.pose.pose.position.x = pose.position.x();
    response.pose.pose.position.y = pose.position.y();
    response.pose.pose.position.z = pose.position.z();
    response.pose.pose.orientation.w = pose.orientation.w();
    response.pose.pose.orientation.x = pose.orientation.x();
    response.pose.pose.orientation.y = pose.orientation.y();
    response.pose.pose.orientation.z = pose.orientation.z();
    response.swivel = pose.swivel;
    response.collisions = collision_names(*model_, contacts_);
    response.success = true;
    response.message = source;
  }

  std::shared_ptr<const ArmModel> model_;
  std::unique_ptr<ArmIk> ik_;
  std::vector<Contact> contacts_;
  JointState::ConstSharedPtr latest_state_;
  rclcpp::Subscription<JointState>::SharedPtr state_subscriber_;
  rclcpp::Service<SolveArmIk>::SharedPtr solve_service_;
  rclcpp::Service<ArmForward>::SharedPtr forward_service_;
  rclcpp::Service<TcpJog>::SharedPtr tcp_jog_service_;
};

}  // namespace graphen::arm_ik

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<graphen::arm_ik::ArmIkServer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("arm_ik_server"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
