#ifndef GRAPHEN_RL_POLICY__OBS_HISTORY_HPP_
#define GRAPHEN_RL_POLICY__OBS_HISTORY_HPP_

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace graphen::rl_policy {

// 觀測的 history 緩衝（環形），把最近 length 幀攤平成 policy 的輸入。
//
// 一幀由數個 term 串接（term_sizes，例如 {3, 3, 3, 29, 29, 29}）。攤平方式：
//   kFrameMajor：[幀 0 的全部 term][幀 1]…（每幀完整排在一起）
//   kTermMajor ：[term 0 的 length 幀][term 1 的 length 幀]…（Isaac Lab ObservationManager 的排法）
// 幀的順序由 oldest_first 決定（true：最舊的在前）。
//
// reset() 之後第一次 push() 會把整個 history 填成該幀，與 Isaac Lab CircularBuffer 的行為相同，
// 剛進入時 policy 看到的是「一直靜止在這個狀態」而不是 0。
// 建構後只覆寫內容，push() / flatten() 不配置記憶體。
class ObsHistory {
public:
  enum class Layout {kFrameMajor, kTermMajor};

  ObsHistory(std::vector<size_t> term_sizes, size_t length, Layout layout, bool oldest_first)
  : term_sizes_(std::move(term_sizes)), length_(length), layout_(layout),
    oldest_first_(oldest_first)
  {
    if (term_sizes_.empty() || length_ == 0) {
      throw std::invalid_argument("ObsHistory: term_sizes and length must not be empty");
    }
    frame_dim_ = std::accumulate(term_sizes_.begin(), term_sizes_.end(), size_t{0});
    frames_.assign(length_ * frame_dim_, 0.0f);
  }

  size_t frame_dim() const {return frame_dim_;}
  size_t length() const {return length_;}
  size_t size() const {return frame_dim_ * length_;}

  void reset() {
    count_ = 0;
    newest_ = 0;
  }

  // frame 長度為 frame_dim()
  void push(const float * frame) {
    if (count_ == 0) {
      for (size_t k = 0; k < length_; ++k) {
        std::copy(frame, frame + frame_dim_, frames_.begin() + static_cast<long>(k * frame_dim_));
      }
      newest_ = length_ - 1;
      count_ = length_;
      return;
    }
    newest_ = (newest_ + 1) % length_;
    std::copy(frame, frame + frame_dim_, frames_.begin() + static_cast<long>(newest_ * frame_dim_));
  }

  // 依 layout 攤平到 out（長度為 size()）；必須至少 push() 過一次
  void flatten(float * out) const {
    if (layout_ == Layout::kFrameMajor) {
      for (size_t k = 0; k < length_; ++k) {
        const float * frame = slot(k);
        out = std::copy(frame, frame + frame_dim_, out);
      }
      return;
    }
    size_t term_offset = 0;
    for (const size_t term_size : term_sizes_) {
      for (size_t k = 0; k < length_; ++k) {
        const float * term = slot(k) + term_offset;
        out = std::copy(term, term + term_size, out);
      }
      term_offset += term_size;
    }
  }

private:
  // 攤平後第 k 個位置的幀
  const float * slot(size_t k) const {
    // 最舊的幀在 newest_ 的下一格
    const size_t age_order = oldest_first_ ? k : length_ - 1 - k;
    const size_t index = (newest_ + 1 + age_order) % length_;
    return frames_.data() + index * frame_dim_;
  }

  std::vector<size_t> term_sizes_;
  size_t length_;
  Layout layout_;
  bool oldest_first_;
  size_t frame_dim_{0};
  std::vector<float> frames_;
  size_t newest_{0};
  size_t count_{0};
};

}  // namespace graphen::rl_policy

#endif  // GRAPHEN_RL_POLICY__OBS_HISTORY_HPP_
