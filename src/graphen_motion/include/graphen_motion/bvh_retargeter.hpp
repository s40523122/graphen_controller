#ifndef GRAPHEN_MOTION__BVH_RETARGETER_HPP_
#define GRAPHEN_MOTION__BVH_RETARGETER_HPP_

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "graphen_motion/bvh_clip.hpp"

namespace graphen::motion {

// 把 BVH（每個節點 3 個旋轉自由度）轉成 URDF 1-DOF revolute joint 的角度。
//
// 前提：BVH 的節點名稱等於 URDF link 名稱，rest pose 等於 URDF 零位（兩邊 link 的長度與方向相同）。
// BVH 可以把一組關節的旋轉集中在組內任一個節點，因此以「組」為單位求解：
//   - 分組：沿 URDF 運動鏈，把連續、軸線交於同一點的可動 joint 歸成一組（例如髖 P/R/Y），
//     遇到分岔的 link 或 fixed joint 就切組。這樣的組合旋轉可以精確分解成各軸角度
//   - 目標：組內最後一個在 BVH 中有同名節點的 link，相對 base_link 的 BVH 姿態
//   - 求解：依樹的順序，以已解出的上游 joint 做 FK 得到組的父 link 姿態，再以 damped least squares
//     讓組末端 link 的姿態逼近目標。少於 3 軸的組是最小平方投影；丟掉的分量會由下游的組以實際 FK 補償
// BVH 的 root 位移與 base_link 本身的旋轉（浮動基座）會被忽略。
class BvhRetargeter {
public:
  struct Options {
    std::string base_link;            // URDF 與 BVH 中都要存在，姿態以它為基準
    double axis_tolerance{1e-4};      // 判斷軸線交於一點的距離容差（URDF 單位，m）
    int max_iterations{50};           // 每組的迭代上限
    double convergence_tolerance{1e-10};  // 單次更新量（rad）小於此值即視為收斂
    double damping{1e-6};             // damped least squares 的阻尼
    // 從上一幀出發的殘差超過此值時，改以多組初始值重新求解；
    // 也是選解時視為「殘差相同」的範圍（rad）
    double residual_tolerance{1e-3};
  };

  struct GroupInfo {
    std::vector<std::string> joints;  // URDF joint 名稱，依運動鏈順序
    std::string bvh_node;             // 作為目標的 BVH 節點；空字串表示沒有 BVH 資料
  };

  struct Stats {
    double max_residual{0.0};          // 所有組中最大的姿態誤差（rad）
    std::vector<double> group_residuals;  // 順序同 groups()
    size_t clamped{0};                 // 被限位截斷的 joint 數
  };

  // joint_names：輸出的 joint 順序。必須是 base_link 之下的 revolute / continuous joint。
  // 設定錯誤（找不到 link / joint、URDF 解析失敗等）時丟出 std::invalid_argument
  BvhRetargeter(
    std::shared_ptr<const BvhClip> clip, const std::string & urdf_xml,
    std::vector<std::string> joint_names, Options options);

  // 解出第 frame 幀的角度。q_inout 進來時作為初始值（熱啟動；NaN 視為 0），
  // 回傳時為結果；沒有 BVH 資料的 joint 為 NaN
  Stats solve(size_t frame, std::vector<double> & q_inout) const;

  const BvhClip & clip() const {return *clip_;}
  const std::vector<std::string> & joint_names() const {return joint_names_;}
  const std::vector<GroupInfo> & groups() const {return group_infos_;}
  // 第 i 個輸出 joint 是否有 BVH 資料
  const std::vector<bool> & covered() const {return covered_;}
  // 第 i 個輸出 joint 的 URDF 限位；continuous joint 為 ±inf
  const std::vector<std::pair<double, double>> & limits() const {return output_limits_;}

private:
  struct Joint {
    std::string name;
    size_t parent_link{0};
    size_t child_link{0};
    Eigen::Matrix3d origin_rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
    bool movable{false};
    bool limited{false};
    double lower{0.0};
    double upper{0.0};
    std::optional<size_t> group;  // 所屬的組（只有可動 joint 有）
  };

  // 3 個軸已涵蓋所有姿態，組內最多 3 個 joint
  static constexpr int kMaxGroupDof = 3;
  static constexpr std::array<double, 3> kSeedValues{-M_PI / 2.0, 0.0, M_PI / 2.0};
  using GroupState = std::array<double, kMaxGroupDof>;

  struct GroupResult {
    GroupState q{};
    double residual{0.0};
    size_t clamped{0};
  };

  struct Group {
    std::vector<size_t> joints;  // 內部 joint index
    std::optional<size_t> bvh_node;
    // 目標節點之後（組內）的 joint origin 旋轉，用來把 BVH 節點姿態換算成組末端 link 的姿態
    Eigen::Matrix3d post_rotation{Eigen::Matrix3d::Identity()};
  };

  // 依樹的順序執行的步驟：fixed joint 直接傳遞姿態，組則求解
  struct Step {
    bool is_group{false};
    size_t index{0};  // joint index 或 group index
  };

  // 分組並決定每組的 BVH 目標節點；origin_translations[i] 為 joints_[i] 的 origin 位移
  // 從 seed 出發以 damped least squares 求解一組（每步依限位截斷）
  GroupResult solve_from(
    const Group & group, const Eigen::Matrix3d & parent_rotation,
    const Eigen::Matrix3d & target, const GroupState & seed) const;
  // 先從上一幀求解，失敗時改用多組初始值，選出殘差最小且最接近上一幀的解
  GroupResult solve_group(
    const Group & group, const Eigen::Matrix3d & parent_rotation,
    const Eigen::Matrix3d & target, const GroupState & warm) const;

  void build_groups(const std::vector<Eigen::Vector3d> & origin_translations);

  std::shared_ptr<const BvhClip> clip_;
  std::vector<std::string> joint_names_;
  Options options_;

  std::vector<std::string> links_;  // index 0 為 base_link
  std::vector<Joint> joints_;       // base_link 之下的所有 joint，依 DFS 順序
  std::vector<Group> groups_;
  std::vector<Step> steps_;
  size_t base_node_{0};             // base_link 在 BVH 中的節點 index

  std::vector<size_t> output_to_joint_;  // 輸出順序 → 內部 joint index
  std::vector<GroupInfo> group_infos_;
  std::vector<bool> covered_;
  std::vector<std::pair<double, double>> output_limits_;
};

}  // namespace graphen::motion

#endif  // GRAPHEN_MOTION__BVH_RETARGETER_HPP_
