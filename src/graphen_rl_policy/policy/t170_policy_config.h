// Auto-generated 2026-10-12 from FullBodyT70 training run
// logs/rsl_rl/t170_velocity_fullbody/2026-10-11_21-48-15 (T170-FullBody-Flat,
// checkpoint model_10000.pt, A-pose arms). Do not hand-edit numbers.
//
// Sources: env.yaml (actions.joint_pos.scale, entities.robot.init_state.joint_pos),
// entity joint order verified via mjlab Entity(get_t170fb_robot_cfg()).joint_names.
#pragma once

namespace t170 {

constexpr int kNumJoints = 32;
constexpr int kObsDim = 107;
constexpr int kActDim = 32;
constexpr float kCtrlDt = 0.02f;    // 50 Hz control period (sim dt 0.005 x decimation 4)
constexpr float kPhasePeriod = 0.6f;

// Observation vector layout (concatenation order of the training obs group):
//   [0..2]    base_ang_vel      (rad/s, base frame, IMU gyro)
//   [3..5]    projected_gravity (R_base^T @ [0,0,-1])
//   [6..8]    command           (vx, vy, vyaw)
//   [9..10]   phase             (sin, cos); forced to (0,0) when ||cmd|| < 0.1
//   [11..42]  joint_pos - q_home
//   [43..74]  joint_vel
//   [75..106] last_action
constexpr int kObsGyro = 0;
constexpr int kObsGravity = 3;
constexpr int kObsCmd = 6;
constexpr int kObsPhase = 9;
constexpr int kObsJointPos = 11;
constexpr int kObsJointVel = 43;
constexpr int kObsLastAction = 75;

inline constexpr const char* kJointNames[kNumJoints] = {
  "L_HIP_P", "L_HIP_R", "L_HIP_Y", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R",
  "R_HIP_P", "R_HIP_R", "R_HIP_Y", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
  "WAIST_Y", "WAIST_R", "WAIST_P",
  "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_Y",
  "L_WRIST_P", "L_WRIST_Y", "L_WRIST_R",
  "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_Y",
  "R_WRIST_P", "R_WRIST_Y", "R_WRIST_R",
  "NECK_Y1", "NECK_P", "NECK_R",
};

// Home (default) joint angles [rad]: hanging arms (A-pose), everything else 0.
// L_SHOULDER_R = -1.19 (idx 16), R_SHOULDER_R = +1.05 (idx 23).
inline constexpr float kQHome[kNumJoints] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,
  0.0f, -1.19f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 1.05f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,
};

// Per-joint action scale = 0.25 * effort_limit / stiffness (training config).
inline constexpr float kActionScale[kNumJoints] = {
  0.06763189008512659f, 0.06649202676908512f, 0.06385345427824841f,
  0.06763189008512659f, 0.06649202676908512f, 0.07282460074709322f,
  0.06763189008512659f, 0.06649202676908512f, 0.06385345427824841f,
  0.06763189008512659f, 0.06649202676908512f, 0.07282460074709322f,
  0.06649202676908512f, 0.06649202676908512f, 0.06649202676908512f,
  0.19947608030725536f, 0.19947608030725536f, 0.13298405353817025f,
  0.13298405353817025f, 0.2912984029883729f, 0.10512072803493458f,
  0.10512072803493458f, 0.19947608030725536f, 0.19947608030725536f,
  0.13298405353817025f, 0.13298405353817025f, 0.2912984029883729f,
  0.10512072803493458f, 0.10512072803493458f, 0.05256036401746729f,
  0.014564920149418644f, 0.014564920149418644f,
};

}  // namespace t170
