#ifndef GRAPHEN_CHOREOGRAPHER__CHOREOGRAPHY_HPP_
#define GRAPHEN_CHOREOGRAPHER__CHOREOGRAPHY_HPP_

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "graphen_choreographer/interpolation.hpp"

namespace graphen::choreographer {

// 編舞腳本：多條軌道，每條軌道控制自己的一組 joint，依各自的 keyframe 播放。
// 軌道之間只共用時鐘，彼此不等待（開環）：某個 joint 跟不上不會拖慢其他軌道。
//
// 腳本格式（YAML）：
//   version: 1
//   default_interp: quintic            # （選填）linear / cubic / quintic，預設 quintic
//   poses:                             # （選填）命名姿態：joint → 角度（rad），可以跨多條軌道
//     r_arm_up: {R_SHOULDER_P: -1.2, R_ELBOW_Y: 0.8, ...}
//   tracks:
//     right_arm:
//       joints: [R_SHOULDER_P, R_ELBOW_Y, ...]
//       keyframes:
//         - {t: 3.0, pose: r_arm_up}                    # 在 3.0 s 到達姿態
//         - {t: 4.0, hold: true}                        # 停留到 4.0 s
//         - {t: 6.0, positions: {R_SHOULDER_P: 0.0, R_ELBOW_Y: 0.3, ...}, interp: linear}
//
// 語意：
//   - 每條軌道在 t = 0 有一個隱含的起點：開始播放時的位置。到第一個 keyframe 之前從起點插值過去，
//     所以第一個 keyframe 的 t 就是走過去的時間。
//   - 每一段的插值方式由「終點」keyframe 的 interp 決定。
//   - keyframe 三選一：pose（命名姿態，必須涵蓋軌道的全部 joint）、positions（直接寫數值，
//     必須剛好是軌道的全部 joint）、hold: true（維持上一個 keyframe；前面沒有姿態時維持起點）。
//   - t 必須 ≥ 0 且嚴格遞增；一個 joint 只能屬於一條軌道。
//   - 軌道播完後保持最後的位置；不在任何軌道中的 joint 不受控制。

struct Keyframe {
  double time{0.0};
  // 到達這個 keyframe 的那一段使用的插值
  Interpolation interpolation{Interpolation::kQuintic};
  // 順序同 Track::joints。空的表示「維持軌道的起點」（hold，且前面沒有任何姿態）
  std::vector<double> positions;
};

struct Track {
  std::string name;
  std::vector<std::string> joints;
  std::vector<Keyframe> keyframes;  // 至少一個，time 嚴格遞增

  // 最後一個 keyframe 的時間
  double duration() const {return keyframes.empty() ? 0.0 : keyframes.back().time;}
};

struct Choreography {
  std::vector<Track> tracks;  // 腳本中的順序

  // 最長的軌道的長度
  double duration() const;
};

// 解析腳本；格式或內容有誤時丟出 std::invalid_argument（訊息會指出位置，例如
// "tracks.right_arm.keyframes[2]: ..."）
Choreography parse_choreography(const std::string & yaml_text);

// 讀取並解析腳本檔；無法開啟時丟出 std::runtime_error，內容有誤時丟出 std::invalid_argument
Choreography load_choreography(const std::string & path);

// 取樣一條軌道在 time（從開始播放起算，秒）的位置與速度。
//   start      軌道的起點（t = 0 的位置），長度同 track.joints
//   start_velocity  起點的速度（rad/s），長度同 track.joints；空的表示從靜止開始。
//              只影響「起點 → 第一個 keyframe」那一段，而且該段的插值要支援（cubic / quintic），
//              見 entry_velocity_blend()
//   cursor     目前所在的段落，呼叫端保存並在重新播放時歸零；time 必須不遞減（只往前找）
//   positions / velocities  輸出，長度必須同 track.joints
// 不配置記憶體。
void sample_track(
  const Track & track, const std::vector<double> & start,
  const std::vector<double> & start_velocity, double time, size_t & cursor,
  std::vector<double> & positions, std::vector<double> & velocities);

// 「起點 → 第一個 keyframe」那一段（cursor 為 0 時）的位置與速度，u ∈ [0, 1]。
// sample_track() 與進入時的檢查共用，確保檢查的就是實際播放的曲線
struct EntrySample {
  double position;
  double velocity;
};
EntrySample sample_entry(
  Interpolation type, double from, double to, double start_velocity, double duration, double u);

// 一個 joint 在整份腳本中的統計（只看 keyframe 之間；從起點到第一個姿態的那一段取決於播放時的位置）
struct JointSummary {
  std::string track;
  std::string joint;
  // keyframe 的範圍；插值是單調的，所以也是整條軌跡的範圍。軌道沒有任何姿態時為 NaN
  double min_position;
  double max_position;
  // 各段峰值速度（|Δ| / T × 插值的係數）的最大值（rad/s），以及該段的開始時間
  double peak_velocity{0.0};
  double peak_time{0.0};
};

std::vector<JointSummary> summarize(const Choreography & choreography);

// URDF 的關節限位；沒有該項限制時為 nullopt（例如 continuous joint 沒有位置限位）
struct JointLimit {
  std::optional<double> lower;
  std::optional<double> upper;
  std::optional<double> velocity;
};
using JointLimits = std::map<std::string, JointLimit>;

// 從 URDF 取得所有可動 joint 的限位；URDF 無法解析時丟出 std::invalid_argument
JointLimits parse_joint_limits(const std::string & urdf_xml);

// 檢查位置與速度限位（速度上限為 URDF 的值 × velocity_scale），回傳每一項違規的說明；
// 不在 limits 中的 joint 不檢查。沒有違規時回傳空的 vector
std::vector<std::string> check_limits(
  const std::vector<JointSummary> & summaries, const JointLimits & limits, double velocity_scale);

}  // namespace graphen::choreographer

#endif  // GRAPHEN_CHOREOGRAPHER__CHOREOGRAPHY_HPP_
