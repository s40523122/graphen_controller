#ifndef GRAPHEN_CONTROLLERS__BVH_MOTION_HPP_
#define GRAPHEN_CONTROLLERS__BVH_MOTION_HPP_

#include <memory>
#include <vector>

#include "graphen_controllers/motion.hpp"
#include "graphen_motion/bvh_retargeter.hpp"

namespace graphen::control {

// 播放 BVH 動作。每一幀在 planner thread 中才以 BvhRetargeter 轉成 joint 角度。
// 依檔案頻率輸出原始幀（幀與幀之間保持），並以 sample_period() 告知幀間隔，
// 插補由下游的子群組 controller（PositionController / PdController）在 RT 迴圈中進行。
//
// 流程：
//   Approach：以 approach_max_steps 走向第 0 幀，所有介面都在一步之內時開始播放（sample_period 為 0）
//   Play：依實際時間 × time_scale 前進（sample_period = frame_time / time_scale）；
//         播完最後一幀時 loop 為 true 則回到 Approach（限速接回第 0 幀），否則保持最後一幀
// 沒有 BVH 資料的介面保持進入此動作時的位置。
class BvhMotion : public Motion {
public:
  struct Options {
    bool loop{true};
    double time_scale{1.0};  // 播放速度倍率，必須 > 0
  };

  // retargeter 的 joint_names 順序必須與介面順序相同。
  // play_max_steps：播放時每週期的最大移動量；approach_max_steps：走向第 0 幀時的最大移動量
  BvhMotion(
    std::shared_ptr<const motion::BvhRetargeter> retargeter, Options options,
    std::vector<double> play_max_steps, std::vector<double> approach_max_steps);

  void on_enter(const RobotState & state) override;
  void update(double dt, const RobotState & state, JointCommand & command) override;
  // Approach 時回傳 approach_max_steps，其餘回傳 play_max_steps
  const std::vector<double> & max_steps() const override {
    return phase_ == Phase::kApproach ? approach_max_steps_ : play_max_steps_;
  }
  // Play 時為 frame_time / time_scale，其餘為 0
  double sample_period() const override;

private:
  enum class Phase {kApproach, kPlay, kHold};

  // 解出 frame 的角度：q 進來時為初始值，沒有資料的介面以 hold_ 填入
  void solve_frame(size_t frame, std::vector<double> & q) const;

  const std::shared_ptr<const motion::BvhRetargeter> retargeter_;
  const Options options_;
  const std::vector<double> play_max_steps_;
  const std::vector<double> approach_max_steps_;

  Phase phase_{Phase::kApproach};
  std::vector<double> hold_;         // 進入時的位置
  std::vector<double> first_frame_;  // 第 0 幀（每次進入時解一次）
  size_t frame_{0};                  // 目前輸出的幀
  double time_in_frame_{0.0};        // 距離 frame_ 開始的時間（秒）
  std::vector<double> frame_q_;      // frame_ 的角度
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__BVH_MOTION_HPP_
