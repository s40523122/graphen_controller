#ifndef GRAPHEN_CHOREOGRAPHER__INTERPOLATION_HPP_
#define GRAPHEN_CHOREOGRAPHER__INTERPOLATION_HPP_

#include <optional>
#include <string_view>

namespace graphen::choreographer {

// 兩個 keyframe 之間的插值方式。三種都是單調的（不會超出兩端的值）：
//   linear   等速，兩端速度不連續
//   cubic    兩端速度為 0（smoothstep）
//   quintic  兩端速度與加速度都為 0（minimum jerk），對實機最溫和
enum class Interpolation {kLinear, kCubic, kQuintic};

// 正規化的插值曲線 s(u)，u ∈ [0, 1]，s(0) = 0、s(1) = 1
struct Blend {
  double position;  // s(u)
  double velocity;  // ds/du
};

constexpr Blend blend(Interpolation type, double u) {
  switch (type) {
    case Interpolation::kLinear:
      return {u, 1.0};
    case Interpolation::kCubic:
      return {u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u)};
    case Interpolation::kQuintic:
      return {u * u * u * (10.0 + u * (-15.0 + 6.0 * u)),
        30.0 * u * u * (1.0 - u) * (1.0 - u)};
  }
  return {u, 1.0};
}

// 起點帶速度時加上的修正項 h(u)（五次 Hermite 的起點速度基底）：
//   h(0) = h(1) = 0、h'(0) = 1、h'(1) = 0、h''(0) = h''(1) = 0
// 位置 p(u) = p0 + Δ·s(u) + v0·T·h(u)，速度 = Δ·s'(u)/T + v0·h'(u)。
// s 的起點速度必須為 0（cubic / quintic），起點速度才會剛好是 v0；配合 quintic 時兩端加速度也是 0。
// 曲線會先順著 v0 的方向多走一段再轉向，所以不再是單調的
constexpr Blend entry_velocity_blend(double u) {
  return {u * (1.0 + u * u * (-6.0 + u * (8.0 - 3.0 * u))),
    1.0 + u * u * (-18.0 + u * (32.0 - 15.0 * u))};
}

// 這種插值能不能帶入起點速度（linear 的起點速度本來就不是 0，加上修正項也不會等於 v0）
constexpr bool supports_entry_velocity(Interpolation type) {
  return type != Interpolation::kLinear;
}

// ds/du 的最大值（出現在 u = 0.5）：一段的峰值速度 = |Δ| / T × 這個係數
constexpr double peak_velocity_factor(Interpolation type) {
  return blend(type, 0.5).velocity;
}

constexpr std::optional<Interpolation> parse_interpolation(std::string_view name) {
  if (name == "linear") {
    return Interpolation::kLinear;
  }
  if (name == "cubic") {
    return Interpolation::kCubic;
  }
  if (name == "quintic") {
    return Interpolation::kQuintic;
  }
  return std::nullopt;
}

constexpr const char * to_string(Interpolation type) {
  switch (type) {
    case Interpolation::kLinear:
      return "linear";
    case Interpolation::kCubic:
      return "cubic";
    case Interpolation::kQuintic:
      return "quintic";
  }
  return "?";
}

}  // namespace graphen::choreographer

#endif  // GRAPHEN_CHOREOGRAPHER__INTERPOLATION_HPP_
