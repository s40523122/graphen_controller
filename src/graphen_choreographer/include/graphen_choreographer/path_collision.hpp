#ifndef GRAPHEN_CHOREOGRAPHER__PATH_COLLISION_HPP_
#define GRAPHEN_CHOREOGRAPHER__PATH_COLLISION_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "graphen_arm_ik/arm_ik.hpp"
#include "graphen_choreographer/choreography.hpp"

namespace graphen::choreographer {

// 一段連續碰撞的時間區間（取樣點之間視為連續）
struct CollisionSpan {
  double start{0.0};  // 第一個碰撞的取樣時間（s）
  double end{0.0};    // 最後一個碰撞的取樣時間（s）
  std::vector<std::string> pairs;  // 這段期間出現過的 body 對，"bodyA|bodyB"
  double max_depth{0.0};           // m

  // 例如 "collision at t=2.35-2.60 s: R_WRIST_R_S|WAIST_P_S (12.3 mm)"
  std::string describe() const;
};

// 以 graphen_arm_ik 的模型檢查編舞的整條路徑（與實際播放同一條曲線，sample_track()）是否讓手臂碰撞。
// 只偵測，不修改路徑。不在任何軌道中、但在模型中的 joint 使用「基準姿態」：預設為模型的初始姿態
// （手臂為 arm_ik.yaml 的 seed），可用 set_base_position() 覆寫（configure 時的 nominal_pose、
// 進入時的量測值）。不在模型中的 joint（例如靈巧手）忽略。
//
// 一個實例有自己的 mjData，不可跨 thread 同時使用。
class PathCollisionChecker {
public:
  PathCollisionChecker(std::shared_ptr<const arm_ik::ArmModel> model, Choreography choreography);

  const arm_ik::ArmModel & model() const {return ik_.model();}

  // 覆寫基準姿態中的一個 joint（軌道中的 joint 會被取樣值蓋過；不在模型中時忽略，回傳 false）
  bool set_base_position(const std::string & joint, double value);

  // 整份腳本：各軌道的起點假設為該軌道的第一個姿態（從靜止開始），以 sample_rate 取樣 [0, duration]。
  // 「開始播放時的位置 → 第一個姿態」那一段取決於播放時的位置，由 check_entry() 檢查
  std::vector<CollisionSpan> check_script(double sample_rate);

  // 進入時：以實際的起點與起點速度（順序同各軌道的 joints），在 [0, 最晚的第一個 keyframe 時間]
  // 均勻取 samples + 1 個點。所有緩衝在建構時配置，這裡不配置記憶體（除了發現碰撞時的結果）
  std::vector<CollisionSpan> check_entry(
    const std::vector<std::vector<double>> & starts,
    const std::vector<std::vector<double>> & start_velocities, int samples);

private:
  // 在 time 取樣所有軌道並寫入模型、檢查碰撞；有碰撞時合併進 spans
  void sample(double time, std::vector<CollisionSpan> & spans, bool & in_span);

  Choreography choreography_;  // 自己保存一份：呼叫端的腳本可能是暫存的
  arm_ik::ArmIk ik_;
  std::vector<std::pair<int, double>> base_;  // (qpos 位址, 角度)
  std::vector<std::vector<int>> track_qpos_;  // 各軌道 joint 的 qpos 位址（不在模型中為 -1）
  // 取樣用
  std::vector<const std::vector<double> *> starts_;
  std::vector<const std::vector<double> *> start_velocities_;
  std::vector<size_t> cursors_;
  std::vector<std::vector<double>> positions_;
  std::vector<std::vector<double>> velocities_;
  std::vector<std::vector<double>> first_poses_;
  std::vector<arm_ik::Contact> contacts_;
};

}  // namespace graphen::choreographer

#endif  // GRAPHEN_CHOREOGRAPHER__PATH_COLLISION_HPP_
