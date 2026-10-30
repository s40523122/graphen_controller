#ifndef GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__LINEAR_INTERPOLATOR_HPP_
#define GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__LINEAR_INTERPOLATOR_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace graphen::control {

// 子群組 controller 共用的 reference 插補（RT 安全，不配置記憶體）。
//
// reference 改變時，從目前輸出線性走到新值，花費 max(sample_period, 上一段剩下的時間)；
// sample_period 為 0 時直接輸出。上游依 BVH 檔頻率送出原始幀時，以此在 RT 迴圈中做幀間插補
// （輸出約晚一個點）。
class LinearInterpolator {
public:
  // 從 value 開始，不產生斜坡（activate 時使用）
  void reset(double value) {
    start_ = value;
    target_ = value;
    duration_ = 0.0;
    elapsed_ = 0.0;
    output_ = value;
  }

  // 推進 dt 秒並回傳輸出。reference 為 NaN（上層尚未寫入）時維持目前輸出
  double step(double reference, double sample_period, double dt) {
    if (std::isnan(reference)) {
      return output_;
    }
    if (reference != target_) {
      // 新的點：時間至少為上一段剩下的時間，間隔變成 0（例如從 BVH 切到其他模式）時
      // 不會把還沒走完的一段瞬間跳完
      const double remaining = std::max(0.0, duration_ - elapsed_);
      start_ = std::isfinite(output_) ? output_ : reference;
      target_ = reference;
      duration_ = std::max(sample_period, remaining);
      elapsed_ = 0.0;
    }
    elapsed_ += dt;
    output_ = elapsed_ < duration_ ?
      start_ + (elapsed_ / duration_) * (target_ - start_) : target_;
    return output_;
  }

  double output() const {return output_;}

  // 上游送出的 sample_period 無效（NaN、負值）時視為 0（不插補）
  static double sanitize_sample_period(double value) {
    return std::isfinite(value) && value > 0.0 ? value : 0.0;
  }

private:
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  double start_{kNaN};
  double target_{kNaN};
  double duration_{0.0};
  double elapsed_{0.0};
  double output_{kNaN};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SUB_GROUP_CONTROLLER__LINEAR_INTERPOLATOR_HPP_
