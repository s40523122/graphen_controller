#include "graphen_controllers/bvh_motion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace graphen::control {

BvhMotion::BvhMotion(
  std::shared_ptr<const motion::BvhRetargeter> retargeter, Options options,
  std::vector<double> play_max_steps, std::vector<double> approach_max_steps)
: retargeter_(std::move(retargeter)), options_(options),
  play_max_steps_(std::move(play_max_steps)), approach_max_steps_(std::move(approach_max_steps)) {
  if (!retargeter_) {
    throw std::invalid_argument("BvhMotion: retargeter must not be null");
  }
  const size_t num_joints = retargeter_->joint_names().size();
  for (const auto * steps : {&play_max_steps_, &approach_max_steps_}) {
    if (steps->empty() || steps->size() != num_joints) {
      throw std::invalid_argument(
        "BvhMotion: max_steps size must match the number of retargeted joints");
    }
    if (std::any_of(steps->begin(), steps->end(), [](double s) {return !(s > 0.0);})) {
      throw std::invalid_argument("BvhMotion: every max_step must be greater than zero");
    }
  }
  if (!(options_.time_scale > 0.0)) {
    throw std::invalid_argument("BvhMotion: time_scale must be greater than zero");
  }
}

void BvhMotion::solve_frame(size_t frame, std::vector<double> & q) const {
  retargeter_->solve(frame, q);
  for (size_t i = 0; i < q.size(); ++i) {
    if (std::isnan(q[i])) {
      q[i] = hold_[i];
    }
  }
}

void BvhMotion::on_enter(const std::vector<double> & current) {
  hold_ = current;
  // 以目前位置作為初始值，從多個等價解中選出最接近目前姿態的
  first_frame_ = current;
  solve_frame(0, first_frame_);
  phase_ = Phase::kApproach;
}

void BvhMotion::update(
  double dt, const std::vector<double> & current, std::vector<double> & desired) {
  const size_t num_frames = retargeter_->clip().num_frames();
  const double frame_time = retargeter_->clip().frame_time;

  if (phase_ == Phase::kApproach) {
    desired = first_frame_;
    // 每個介面都能在一步之內到達第 0 幀時開始播放，避免播放時的第一段被限速拉長
    for (size_t i = 0; i < first_frame_.size(); ++i) {
      if (std::abs(first_frame_[i] - current[i]) > approach_max_steps_[i]) {
        return;
      }
    }
    frame_ = 0;
    time_in_frame_ = 0.0;
    frame_q_ = first_frame_;
    phase_ = num_frames > 1 ? Phase::kPlay : Phase::kHold;
    return;
  }

  if (phase_ == Phase::kPlay) {
    // 依檔案頻率輸出原始幀（幀與幀之間保持），插補交給下游的 SubGroupController
    time_in_frame_ += dt * options_.time_scale;
    while (time_in_frame_ >= frame_time) {
      time_in_frame_ -= frame_time;
      ++frame_;
      if (frame_ >= num_frames) {
        // 最後一幀已播完：循環時限速接回第 0 幀，否則停在最後一幀
        phase_ = options_.loop ? Phase::kApproach : Phase::kHold;
        break;
      }
      // 以上一幀作為初始值，維持連續
      solve_frame(frame_, frame_q_);
    }
  }

  desired = frame_q_;  // kPlay / kHold
}

double BvhMotion::sample_period() const {
  return phase_ == Phase::kPlay ?
         retargeter_->clip().frame_time / options_.time_scale : 0.0;
}

}  // namespace graphen::control
