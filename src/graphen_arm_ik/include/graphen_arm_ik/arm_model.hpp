#ifndef GRAPHEN_ARM_IK__ARM_MODEL_HPP_
#define GRAPHEN_ARM_IK__ARM_MODEL_HPP_

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "graphen_arm_ik/arm_config.hpp"
#include "mujoco/mujoco.h"

namespace graphen::arm_ik {

// IK 與碰撞檢查用的 MuJoCo 模型（唯讀，可以由多個 ArmIk 共用）。
//
// 由 arm_ik.yaml 指定的 MJCF 修改而來（MjSpec，不改檔案）：
//   - 移除 free joint：基座固定，模型只描述「機器人相對自己」的姿態
//   - collision.include_floor 為 false 時移除地面（worldbody 上的 plane geom）
//   - 加入 scene 中的障礙物（固定在基座座標系）
// 只做運動學（mj_kinematics）與碰撞偵測（mj_collision），不推進物理。
class ArmModel {
public:
  // 一隻手臂在模型中的位址
  struct Arm {
    ArmDefinition definition;
    std::array<int, kArmDof> qpos_adr{};
    std::array<int, kArmDof> dof_adr{};
    // 已扣掉 joint_limit_margin；沒有限位的 joint 為 ±inf
    std::array<double, kArmDof> lower{};
    std::array<double, kArmDof> upper{};
    int tip_body{-1};
    int elbow_body{-1};
    int shoulder_body{-1};
    // body id → 是否屬於這隻手臂（第一個 joint 所在 body 的子樹，含手掌與手指）
    std::vector<bool> bodies;
  };

  // 載入並檢查；MJCF 無法載入、找不到 body / joint、joint 不是 hinge 等丟出 std::invalid_argument
  static std::shared_ptr<const ArmModel> load(const ArmConfig & config);
  // 依設定檔路徑（支援 package://）載入，同一個路徑在 process 內只載入一次（thread-safe）。
  // 編舞每次播放 / 發佈都會建立新的 plugin 實例，不必每次重新編譯 MJCF；修改設定檔後要重新啟動才會生效
  static std::shared_ptr<const ArmModel> load_cached(const std::string & config_path);

  ~ArmModel();
  ArmModel(const ArmModel &) = delete;
  ArmModel & operator=(const ArmModel &) = delete;

  const mjModel * model() const {return model_;}
  const ArmConfig & config() const {return config_;}
  const std::vector<Arm> & arms() const {return arms_;}
  // 依名稱找手臂；找不到時回傳 -1
  int arm_index(const std::string & name) const;
  int root_body() const {return root_body_;}

  // joint 名稱 → qpos 位址；模型沒有這個 joint、或不是 hinge / slide 時回傳 -1
  int qpos_address(const std::string & joint) const;
  std::string body_name(int body) const;

  // 兩個 body 之間的接觸不算碰撞（設定的 allowed_pairs 或初始姿態就接觸的）
  bool pair_allowed(int body1, int body2) const {
    return allowed_[static_cast<size_t>(body1) * nbody_ + static_cast<size_t>(body2)];
  }
  // 初始姿態就接觸、因此自動允許的 body 對（載入時 log 用）
  const std::vector<std::pair<std::string, std::string>> & initial_contacts() const {
    return initial_contacts_;
  }

private:
  ArmModel() = default;
  void allow(int body1, int body2);

  ArmConfig config_;
  mjModel * model_{nullptr};
  std::vector<Arm> arms_;
  int root_body_{-1};
  size_t nbody_{0};
  std::vector<bool> allowed_;
  std::vector<std::pair<std::string, std::string>> initial_contacts_;
};

}  // namespace graphen::arm_ik

#endif  // GRAPHEN_ARM_IK__ARM_MODEL_HPP_
