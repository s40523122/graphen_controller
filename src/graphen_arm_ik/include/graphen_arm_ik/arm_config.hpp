#ifndef GRAPHEN_ARM_IK__ARM_CONFIG_HPP_
#define GRAPHEN_ARM_IK__ARM_CONFIG_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace graphen::arm_ik {

// 手臂固定為 7 DOF（肩 3 + 肘 1 + 腕 3）：末端 6D 之外剩 1 維冗餘，就是手肘的 swivel
inline constexpr std::size_t kArmDof = 7;
using ArmVector = std::array<double, kArmDof>;

// 一隻手臂的定義（arm_ik.yaml 的 arms.<名稱>）
struct ArmDefinition {
  std::string name;
  std::array<std::string, kArmDof> joints;  // 由肩到腕的順序
  std::string tip_body;       // 末端（wrist）所在的 body
  std::string elbow_body;     // swivel 用的手肘點：此 body 的原點
  std::string shoulder_body;  // swivel 用的肩點：此 body 的原點
  // 工具點相對 tip_body 的位姿（例如掌心）；目標位姿指的是這個點
  Eigen::Isometry3d tip_offset{Eigen::Isometry3d::Identity()};
  // swivel 0 的方向，root_body 座標系。手臂的「肩 → 腕」軸與它平行時 swivel 無定義
  Eigen::Vector3d swivel_reference{0.0, 0.0, -1.0};
  // 沒有給初始值時使用，決定 IK 解的分支
  ArmVector seed{};
};

struct SolverOptions {
  double damping{0.01};               // damped least squares 的 λ
  int iterations{100};                // solve() 的最大迭代次數（主任務 + swivel、以及收尾各一輪）
  double position_tolerance{1e-4};    // m
  double orientation_tolerance{1e-3};  // rad
  double swivel_tolerance{1e-3};      // rad
  double orientation_weight{0.5};     // 姿態誤差相對位置誤差的權重（rad → m 的尺度換算）
  double max_step{0.1};               // solve() 每次迭代單一 joint 的最大變化（rad）
  double joint_limit_margin{0.01};    // 解離限位至少保留的距離（rad）
  // 從 seed 解不到（局部最小值、卡在限位）時，改從幾個固定的起點重試，取最好的解。
  // 起點由固定的亂數種子產生，同樣的輸入永遠得到同樣的結果
  int restarts{8};
};

// 環境障礙物（預留）：位姿以機器人基座（free joint 所在的 body）座標系表示
struct Obstacle {
  enum class Shape {kBox, kSphere, kCylinder, kCapsule};
  std::string name;
  Shape shape{Shape::kBox};
  std::array<double, 3> size{};  // MuJoCo 的 geom size（box 為半邊長）
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

// 只檢查至少一邊屬於手臂的接觸（手臂 ↔ 手臂、手臂 ↔ 軀幹 / 腿 / 頭 / 障礙物）
struct CollisionOptions {
  bool enabled{true};
  bool include_floor{false};      // 地面（plane geom）是否參與碰撞
  double min_penetration{0.0};    // m，穿透深度超過才算碰撞（容許 mesh 誤差）
  // 運動鏈上相距這麼多個 link 以內的 body 不檢查（關節附近的 mesh 本來就重疊）
  int ignore_within_links{2};
  // 模型初始姿態（所有關節 0）就接觸的 body 對自動允許（載入時列出）
  bool allow_initial_contacts{true};
  std::vector<std::pair<std::string, std::string>> allowed_pairs;
};

struct ArmConfig {
  std::string mjcf;       // 已解析成絕對路徑
  std::string root_body;  // 目標位姿的參考座標系（軀幹）
  std::vector<ArmDefinition> arms;
  SolverOptions solver;
  CollisionOptions collision;
  std::vector<Obstacle> scene;
};

// 解析 arm_ik.yaml 的內容。mjcf 支援 package://<套件>/<路徑>、絕對路徑與相對於 base_dir 的路徑。
// 格式或內容有誤時丟出 std::invalid_argument（訊息指出欄位）
ArmConfig parse_arm_config(const std::string & yaml_text, const std::string & base_dir);

// 讀取並解析 arm_ik.yaml；path 支援 package://。無法開啟時丟出 std::runtime_error
ArmConfig load_arm_config(const std::string & path);

// package://<套件>/<路徑> 轉成絕對路徑；其他路徑原樣回傳。套件不存在時丟出 std::invalid_argument
std::string resolve_package_path(const std::string & path);

}  // namespace graphen::arm_ik

#endif  // GRAPHEN_ARM_IK__ARM_CONFIG_HPP_
