#ifndef GRAPHEN_ARM_IK__ARM_IK_HPP_
#define GRAPHEN_ARM_IK__ARM_IK_HPP_

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "graphen_arm_ik/arm_config.hpp"
#include "graphen_arm_ik/arm_model.hpp"
#include "mujoco/mujoco.h"

namespace graphen::arm_ik {

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// 末端目標（root_body 座標系）。swivel 為手肘繞「肩 → 腕」軸的角度（rad，0 = swivel_reference 方向）；
// NaN 表示不指定：手肘不另外控制（最小變動，大致維持）
struct Target {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double swivel{kNaN};
};

// 末端的位姿（root_body 座標系）與 swivel；手臂的肩 → 腕軸與 swivel_reference 平行時 swivel 為 NaN
struct ArmPose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double swivel{kNaN};
};

struct IkResult {
  ArmVector positions{};
  double position_error{0.0};     // m
  double orientation_error{0.0};  // rad
  double swivel{kNaN};            // 解的 swivel
  double swivel_error{0.0};       // rad，沒有指定 swivel 時為 0
  bool converged{false};          // 末端位置與姿態都在容差內
  bool swivel_reached{true};      // 指定的 swivel 在容差內（沒有指定時為 true）
};

struct Contact {
  int body1{-1};
  int body2{-1};
  double depth{0.0};  // 穿透深度（m，> 0）
};

// 伺服（step）的增益
struct ServoGains {
  double position_gain{2.0};       // 1/s：末端位置誤差 → 速度
  double orientation_gain{2.0};    // 1/s
  double swivel_gain{2.0};         // 1/s
  double max_joint_velocity{1.0};  // rad/s，單一 joint
};

// 以 ArmModel 求解手臂 IK 與檢查碰撞。每個實例有自己的 mjData（不可跨 thread 共用同一個實例）。
//
// 演算法：末端 6D 誤差（位置 + 姿態旋轉向量）以 damped least squares 求解；7 DOF 對 6D 只剩 1 維
// null space，以 SVD 取得方向向量 n，沿 n 走純量步長讓 swivel 接近目標（不用 pinv(J_e·N)：理論上
// rank 1，數值雜訊的小奇異值會被放大成很大的步長）。每步 clamp 到限位。末端優先於 swivel。
//
// 其他關節（腰、腿、頭）的角度會影響 root_body 的位置與碰撞，使用前以 set_position() 設成目前的值。
class ArmIk {
public:
  explicit ArmIk(std::shared_ptr<const ArmModel> model);
  ~ArmIk();
  ArmIk(const ArmIk &) = delete;
  ArmIk & operator=(const ArmIk &) = delete;

  const ArmModel & model() const {return *model_;}

  // 全部回到模型的初始姿態，手臂放在設定檔的 seed
  void reset();
  // 設定單一 joint（qpos 位址由 ArmModel::qpos_address 取得；-1 時忽略）
  void set_position(int qpos_adr, double value);
  void set_arm(int arm, const ArmVector & positions);
  ArmVector arm_positions(int arm) const;

  // 目前姿態下的末端位姿與 swivel（會先更新運動學）
  ArmPose forward(int arm);

  // 從 seed 開始收斂到目標（迭代次數與容差見 SolverOptions）。結束時模型停在解的姿態。
  // 指定的 swivel 做不到時末端優先（swivel_reached 為 false）。從 seed 解不到時（allow_restarts）
  // 依 restarts 從其他起點重試，只有等級較高的解才取代（末端收斂且 swivel 到達 > 末端收斂 > 沒有收斂；
  // 同等級保留 seed 的解，避免跳到另一個分支）。allow_restarts 為 false 時只從 seed 迭代（拖曳示教：
  // 解要連續）。耗時不固定，給 non-RT 使用
  IkResult solve(int arm, const Target & target, const ArmVector & seed, bool allow_restarts = true);

  // 伺服：從 current 往目標走一步（dt 秒），回傳下一步的關節角；不配置記憶體。
  // 速度 = gain × 誤差，單一 joint 不超過 max_joint_velocity。模型停在回傳的姿態
  ArmVector step(int arm, const Target & target, const ArmVector & current, double dt,
    const ServoGains & gains);

  // 目前姿態的碰撞（arm < 0 表示所有手臂；只列出至少一邊屬於該手臂的接觸）。
  // 結果寫入 out（先清空），回傳數量；out 的容量足夠時不配置記憶體
  size_t check_collisions(int arm, std::vector<Contact> & out);

private:
  struct Errors {
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;  // 世界座標的旋轉向量
  };

  void update_kinematics();
  Eigen::Isometry3d root_pose() const;
  Eigen::Isometry3d tip_pose(int arm) const;  // 世界座標，含 tip_offset
  double swivel_world(int arm) const;  // 目前的 swivel；無定義時為 NaN
  Errors errors(int arm, const Eigen::Isometry3d & target_world) const;
  // 計算一步的 dq（不含 clamp）。target_swivel 為 NaN 時不控制手肘；swivel_scale 為次任務的步長倍率
  Eigen::Matrix<double, kArmDof, 1> compute_step(int arm, const Eigen::Matrix<double, 6, 1> & error,
    double target_swivel, double swivel_scale);
  // 迭代到收斂或達到上限；use_swivel 為 false 時只跑主任務
  void iterate(int arm, const Eigen::Isometry3d & target_world, double target_swivel, bool use_swivel);
  void apply(int arm, const Eigen::Matrix<double, kArmDof, 1> & dq);
  // 從 seed 解一次（不重試）
  IkResult solve_once(int arm, const Eigen::Isometry3d & target_world, double target_swivel,
    const ArmVector & seed);

  std::shared_ptr<const ArmModel> model_;
  const mjModel * m_;
  mjData * d_{nullptr};
  std::vector<mjtNum> jacp_;
  std::vector<mjtNum> jacr_;
};

// 角度包到 (-π, π]
double wrap_angle(double angle);

}  // namespace graphen::arm_ik

#endif  // GRAPHEN_ARM_IK__ARM_IK_HPP_
