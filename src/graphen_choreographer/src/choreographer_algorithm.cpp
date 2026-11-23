#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "graphen_choreographer/choreography.hpp"
#include "graphen_choreographer/path_collision.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace graphen::choreographer {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
// 判斷「輸出已經到達最後的位置」的容差（rad）：planner 到達時只剩浮點誤差
constexpr double kReachedTolerance = 1e-9;
// 建議的 max_steps 相對於峰值速度的餘裕（planner 的週期會抖動）
constexpr double kMaxStepMargin = 1.2;
// 檢查帶起點速度的第一段時的取樣數：曲線不再單調，峰值速度與範圍以取樣求出
constexpr int kEntrySamples = 64;
// 進入時檢查「起點 → 第一個 keyframe」碰撞的預設取樣數（每點約 0.1 ms）
constexpr int64_t kDefaultCollisionEntrySamples = 10;

// package://<pkg>/<path> → <pkg 的 share 目錄>/<path>；其他字串原樣回傳
// TODO: graphen_controllers（BVH）與 graphen_rl_policy（onnx）各有一份相同的邏輯，之後抽成共用元件
std::string resolve_package_path(const std::string & path) {
  const std::string scheme = "package://";
  if (path.compare(0, scheme.size(), scheme) != 0) {
    return path;
  }
  const auto rest = path.substr(scheme.size());
  const auto slash = rest.find('/');
  if (slash == std::string::npos || slash == 0) {
    throw std::invalid_argument("'" + path + "' must be package://<package>/<path>");
  }
  try {
    return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) +
           rest.substr(slash);
  } catch (const std::exception & e) {
    throw std::invalid_argument("cannot resolve '" + path + "': " + e.what());
  }
}

// 訊息用：固定小數位數的數字
std::string num(double value, int precision = 4) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return buffer;
}

}  // namespace

// 依編舞腳本播放多軌 keyframe 的組合動作（格式見 choreography.hpp）。
//
// 進入模式時，各軌道從當下的命令位置開始，依腳本的時間前進；各軌道獨立、時間可以重疊，
// 不在任何軌道中的 joint 不寫入（保持）。輸出位置與解析微分的速度前饋。全部播完並到達最後的位置時
// finished() 為 true，之後保持；要重播需重新進入（播放類 action 的 options.on_same）。
//
// 速度平滑化：播放途中從另一個動作切過來時，手臂可能還在移動。第一段（起點 → 第一個 keyframe）
// 會帶入切換當下的命令速度（RobotState::commanded_velocities），沿著原本的方向自然減速再轉向，
// 而不是在一個週期內煞停。從靜止開始時速度為 0，行為和沒有平滑化相同。
// 只對 cubic / quintic 的第一段有效；第一個 keyframe 在 t = 0、或第一段是 linear 時不帶入。
// 起點速度先限制在該 joint 的速度上限與 max_steps 之內（BVH 等逐幀跳動的動作會出現速度尖峰）。
//
// 時間是開環的：只依時鐘輸出，不等待實際位置。所以輸出不能被 planner 限速，否則時間會被拉長：
// configure 時會確認每一段的峰值速度都在 <mode>.max_steps 之內。
//
// 腳本頂層的 interruptible: false 會讓這段編舞播放中不可被打斷（interruptible()）。
//
// 參數（<mode>.*，或由 controller 的 PlayChoreo 以 choreo.* 加上 content / label 提供）：
//   file                  腳本路徑，可寫 package://<套件>/<相對路徑>；與 content 擇一
//   content               腳本的 YAML 內容（不經過檔案，用於舞曲庫與試播）；與 file 擇一
//   label                 （選填）log 與錯誤訊息中用來稱呼這份腳本的名稱，預設為 file 或 "(content)"
//   start_tolerance       第一個 keyframe 在 t = 0（表示「必須已經在這個姿態」）時，
//                         允許與進入時位置的最大差距（rad，預設 0.05）
//   velocity_limit_scale  速度上限 = URDF 的 velocity limit × 這個值（預設 1.0，必須 > 0）
//   entry_velocity        是否帶入切換當下的速度（預設 true）
//   max_steps             （由 controller 讀取）每週期最大移動量；必須容得下腳本的峰值速度，
//                         可用 choreo_check 算出建議值
//   arm_ik_config         （選填）graphen_arm_ik 的 arm_ik.yaml（支援 package://）；設定時檢查整條路徑
//                         是否讓手臂碰撞（只偵測，不閃避）。空字串 = 不檢查。bringup 依 robot 帶入
//   collision.sample_rate      路徑碰撞的取樣頻率（Hz，預設 = planner 頻率）
//   collision.entry_samples    進入時「起點 → 第一個 keyframe」的取樣數（預設 10）
//   collision.nominal_joints / collision.nominal_positions
//                         configure 時不在任何軌道中的 joint（腰、腿、頭）的角度；沒有列出的用
//                         模型的初始姿態（手臂為 arm_ik.yaml 的 seed）。進入時改用量測值
//
// 檢查（任何一項不通過，configure 就失敗）：
//   - 腳本的 joint 都是受控的 joint；poses 中的 joint 名稱也都必須是受控的 joint，
//     即使沒有被任何軌道使用（全身姿態只取一部分時，其餘名稱打錯也要報錯）
//   - 有 robot_description 時：每個 keyframe 都在位置限位內、每一段的峰值速度不超過速度上限
//   - 每一段的峰值速度不會被 max_steps 限速
//   - 設定 arm_ik_config 時：整條路徑（各軌道從第一個姿態開始）沒有碰撞，錯誤訊息指出時間與 body 對
// 從進入時的位置到第一個姿態的那一段，在 on_enter 以同樣的條件（含位置限位，因為帶起點速度的曲線
// 會超出兩端；碰撞則以實際的起點與量測的其他關節）檢查，不通過時不執行（保持並回報失敗）。
// 限制：configure 時其他 joint 用 nominal 姿態；播放時腰 / 腿與 nominal 差很多時，keyframe 之間的
// 碰撞檢查是近似的。
class ChoreographerAlgorithm : public locomotion::LocomotionAlgorithm {
public:
  void configure(const locomotion::Context & context) override {
    const auto file = context.params.get_string("file");
    const auto content = context.params.get_string("content");
    const bool has_file = file && !file->empty();
    const bool has_content = content && !content->empty();
    if (has_file == has_content) {
      throw std::invalid_argument("exactly one of 'file' and 'content' is required");
    }
    const std::string label =
      context.params.get_string("label").value_or(has_file ? *file : "(content)");
    start_tolerance_ = context.params.get_double("start_tolerance").value_or(0.05);
    if (!(start_tolerance_ >= 0.0)) {
      throw std::invalid_argument("'start_tolerance' must not be negative");
    }
    const double velocity_scale = context.params.get_double("velocity_limit_scale").value_or(1.0);
    if (!(velocity_scale > 0.0)) {
      throw std::invalid_argument("'velocity_limit_scale' must be greater than zero");
    }
    entry_velocity_ = context.params.get_bool("entry_velocity").value_or(true);
    if (!(context.rate_hz > 0.0)) {
      throw std::invalid_argument("the planner rate must be greater than zero");
    }

    const auto choreography = has_file ?
      load_choreography(resolve_package_path(*file)) : parse_choreography(*content);
    const auto summaries = summarize(choreography);

    // 姿態可以涵蓋軌道以外的 joint，但名稱必須存在：打錯字的 joint 不會被任何軌道取用，
    // 不檢查的話永遠不會被發現
    if (const auto unknown = unknown_pose_joints(choreography, context.joint_names);
      !unknown.empty())
    {
      std::string names;
      for (const auto & joint : unknown) {
        names += (names.empty() ? "'" : ", '") + joint + "'";
      }
      throw std::invalid_argument(
              "choreography '" + label + "': poses use " + names +
              ", which " + (unknown.size() == 1 ? "is not a commanded joint" :
              "are not commanded joints"));
    }

    std::vector<std::string> problems;

    JointLimits limits;
    if (context.robot_description.empty()) {
      context.log(locomotion::LogLevel::kWarn,
        "No robot_description: joint position / velocity limits of the choreography are not checked.");
    } else {
      limits = parse_joint_limits(context.robot_description);
      problems = check_limits(summaries, limits, velocity_scale);
      std::string unknown;
      for (const auto & summary : summaries) {
        if (limits.find(summary.joint) == limits.end()) {
          unknown += (unknown.empty() ? "" : ", ") + summary.joint;
        }
      }
      if (!unknown.empty()) {
        context.log(locomotion::LogLevel::kWarn,
          "No limits in the URDF for: " + unknown + ". These joints are not checked.");
      }
    }

    // max_steps 會限制每週期的移動量：容不下峰值速度時，該 joint 會落後編舞
    const bool has_max_steps = context.max_steps.size() == context.joint_names.size();

    tracks_.clear();
    num_joints_ = 0;
    size_t summary_index = 0;
    for (const auto & source : choreography.tracks) {
      TrackState track;
      track.track = source;
      const size_t size = source.joints.size();
      for (const auto & joint : source.joints) {
        const auto it =
          std::find(context.joint_names.begin(), context.joint_names.end(), joint);
        if (it == context.joint_names.end()) {
          throw std::invalid_argument(
                  "track '" + source.name + "': joint '" + joint + "' is not a commanded joint");
        }
        const auto index = static_cast<size_t>(std::distance(context.joint_names.begin(), it));
        track.indices.push_back(index);

        const auto limit = limits.find(joint);
        const bool has_limit = limit != limits.end();
        track.velocity_limits.push_back(
          has_limit && limit->second.velocity ?
          *limit->second.velocity * velocity_scale : kInfinity);
        track.lower_limits.push_back(
          has_limit && limit->second.lower ? *limit->second.lower : -kInfinity);
        track.upper_limits.push_back(
          has_limit && limit->second.upper ? *limit->second.upper : kInfinity);
        const double step_limit = has_max_steps ? context.max_steps[index] : kInfinity;
        track.step_limits.push_back(step_limit);

        // summarize() 的順序與這裡相同（軌道 → joint）
        const auto & summary = summaries[summary_index++];
        const double step = summary.peak_velocity / context.rate_hz;
        if (step > step_limit) {
          problems.push_back(
            "track '" + source.name + "' joint '" + joint + "': peak velocity " +
            num(summary.peak_velocity) + " rad/s needs " + num(step, 5) +
            " rad per cycle, but max_steps allows " + num(step_limit, 5) +
            " (set it to at least " + num(step * kMaxStepMargin, 5) +
            ", or slow the choreography down)");
        }
      }
      for (size_t k = 0; k < source.keyframes.size(); ++k) {
        if (!source.keyframes[k].positions.empty()) {
          track.first_pose = k;
          break;
        }
      }
      track.start.assign(size, 0.0);
      track.start_velocity.assign(size, 0.0);
      track.positions.assign(size, 0.0);
      track.velocities.assign(size, 0.0);
      num_joints_ += size;
      tracks_.push_back(std::move(track));
    }

    if (!problems.empty()) {
      std::string message = "choreography '" + label + "' cannot be played as written:";
      for (const auto & problem : problems) {
        message += "\n  - " + problem;
      }
      throw std::invalid_argument(message);
    }

    configure_collision(context, choreography, label);

    duration_ = choreography.duration();
    rate_hz_ = context.rate_hz;
    time_ = 0.0;
    finished_ = false;
    interruptible_ = choreography.interruptible;
    context.log(locomotion::LogLevel::kInfo,
      "Choreography '" + label + "': " + std::to_string(tracks_.size()) + " tracks, " +
      std::to_string(num_joints_) + " joints, " + num(duration_, 2) + " s" +
      (interruptible_ ? "." : ", not interruptible."));
  }

  void on_enter(const locomotion::RobotState & state) override {
    time_ = 0.0;
    finished_ = false;
    const bool has_velocities = state.commanded_velocities.size() == state.commanded_positions.size();
    for (auto & track : tracks_) {
      track.cursor = 0;
      const auto & first = track.track.keyframes.front();
      // 第一個 keyframe 在 t = 0 或第一段是 linear 時無法帶入起點速度
      const bool carry = entry_velocity_ && has_velocities && first.time > 0.0 &&
        supports_entry_velocity(first.interpolation);
      for (size_t j = 0; j < track.indices.size(); ++j) {
        const size_t index = track.indices[j];
        track.start[j] = state.commanded_positions[index];
        double velocity = carry ? state.commanded_velocities[index] : 0.0;
        if (!std::isfinite(velocity)) {
          velocity = 0.0;
        }
        // 限制在 joint 的速度上限與 max_steps 之內：前一個動作可能有逐幀跳動造成的尖峰
        const double cap = std::min(track.velocity_limits[j], track.step_limits[j] * rate_hz_);
        track.start_velocity[j] = std::clamp(velocity, -cap, cap);
      }
    }
    // 全部軌道的起點都設好之後才檢查：丟出例外時這個模式改為保持，不會只動一部分
    for (const auto & track : tracks_) {
      check_entry(track);
    }
    check_entry_collision(state);
  }

  void update(
    double dt, const locomotion::RobotState & state, locomotion::JointCommand & command) override
  {
    time_ += dt;
    bool reached = time_ >= duration_;
    for (auto & track : tracks_) {
      sample_track(track.track, track.start, track.start_velocity, time_, track.cursor,
        track.positions, track.velocities);
      for (size_t j = 0; j < track.indices.size(); ++j) {
        const size_t index = track.indices[j];
        command.positions[index] = track.positions[j];
        command.velocities[index] = track.velocities[j];
        // 時間到了之後 positions 就是最後的位置；等 planner 的輸出也到達才算播完
        reached = reached &&
          std::abs(state.commanded_positions[index] - track.positions[j]) <= kReachedTolerance;
      }
    }
    finished_ = reached;
  }

  bool finished() const override {return finished_;}

  bool interruptible() const override {return interruptible_;}

private:
  struct TrackState {
    Track track;
    std::vector<size_t> indices;           // 各 joint 在 RobotState / JointCommand 中的 index
    std::vector<double> velocity_limits;   // URDF 速度上限 × scale（rad/s），沒有時為 inf
    std::vector<double> step_limits;       // max_steps（rad/週期），沒有時為 inf
    std::vector<double> lower_limits;      // URDF 位置下限，沒有時為 -inf
    std::vector<double> upper_limits;      // URDF 位置上限，沒有時為 inf
    std::optional<size_t> first_pose;      // 第一個有姿態的 keyframe；全部都是 hold 時沒有
    // 以下每次進入時重設
    std::vector<double> start;             // 進入時的位置
    std::vector<double> start_velocity;    // 進入時帶入的速度（不帶入時為 0）
    size_t cursor{0};
    std::vector<double> positions;         // 取樣用的緩衝
    std::vector<double> velocities;
  };

  // 檢查從進入時的位置開始、到第一個姿態為止的段落；這些段落取決於當下的位置與速度，只能在進入時檢查。
  //   第 0 段（起點 → keyframes[0]）：可能帶起點速度，曲線不單調，以取樣求出峰值速度與範圍
  //   第一個姿態在更後面時（前面是 hold）：從起點靜止出發的一段，峰值速度用公式
  void check_entry(const TrackState & state) const {
    const auto & track = state.track;
    const auto & first = track.keyframes.front();
    for (size_t j = 0; j < track.joints.size(); ++j) {
      const std::string who = "track '" + track.name + "' joint '" + track.joints[j] + "': ";
      // positions 為空（hold）表示維持起點
      const double first_target = first.positions.empty() ? state.start[j] : first.positions[j];

      if (!(first.time > 0.0)) {
        // 第一個 keyframe 在 t = 0：必須已經在這個姿態
        const double distance = std::abs(first_target - state.start[j]);
        if (distance > start_tolerance_) {
          throw std::runtime_error(
                  who + "is " + num(distance) + " rad away from the first keyframe at t = 0 "
                  "(start_tolerance " + num(start_tolerance_) +
                  "); move there first or give the keyframe a time");
        }
      } else {
        double peak_velocity = 0.0;
        double min_position = state.start[j];
        double max_position = state.start[j];
        for (int i = 0; i <= kEntrySamples; ++i) {
          const double u = static_cast<double>(i) / kEntrySamples;
          const auto sample = sample_entry(
            first.interpolation, state.start[j], first_target, state.start_velocity[j], first.time,
            u);
          peak_velocity = std::max(peak_velocity, std::abs(sample.velocity));
          min_position = std::min(min_position, sample.position);
          max_position = std::max(max_position, sample.position);
        }
        check_segment(state, j, who, first.time, peak_velocity);
        if (min_position < state.lower_limits[j] || max_position > state.upper_limits[j]) {
          throw std::runtime_error(
                  who + "the move to the first keyframe (carrying " +
                  num(state.start_velocity[j]) + " rad/s from the previous motion) reaches [" +
                  num(min_position) + ", " + num(max_position) + "], outside the limits [" +
                  num(state.lower_limits[j]) + ", " + num(state.upper_limits[j]) + "]");
        }
      }

      // 第一個姿態在 hold 之後：從起點（靜止）出發
      if (state.first_pose && *state.first_pose > 0) {
        const size_t k = *state.first_pose;
        const auto & keyframe = track.keyframes[k];
        const double duration = keyframe.time - track.keyframes[k - 1].time;
        const double distance = std::abs(keyframe.positions[j] - state.start[j]);
        check_segment(state, j, who, keyframe.time,
          distance / duration * peak_velocity_factor(keyframe.interpolation));
      }
    }
  }

  // 一段的峰值速度不能超過速度上限，也不能被 max_steps 限速
  void check_segment(
    const TrackState & state, size_t j, const std::string & who, double end_time,
    double peak_velocity) const
  {
    if (peak_velocity > state.velocity_limits[j]) {
      throw std::runtime_error(
              who + "reaching the keyframe at t = " + num(end_time, 2) + " s from here needs " +
              num(peak_velocity) + " rad/s, above the limit " + num(state.velocity_limits[j]) +
              " rad/s");
    }
    if (peak_velocity / rate_hz_ > state.step_limits[j]) {
      throw std::runtime_error(
              who + "reaching the keyframe at t = " + num(end_time, 2) + " s from here needs " +
              num(peak_velocity / rate_hz_, 5) + " rad per cycle, but max_steps allows " +
              num(state.step_limits[j], 5));
    }
  }

  // 設定 arm_ik_config 時：載入模型並檢查整條路徑；有碰撞時丟出例外
  void configure_collision(
    const locomotion::Context & context, const Choreography & choreography, const std::string & label)
  {
    collision_.reset();
    const auto config = context.params.get_string("arm_ik_config").value_or("");
    if (config.empty()) {
      return;
    }
    const double sample_rate = context.params.get_double("collision.sample_rate").value_or(context.rate_hz);
    if (!(sample_rate > 0.0)) {
      throw std::invalid_argument("'collision.sample_rate' must be greater than zero");
    }
    const auto entry_samples =
      context.params.get_int("collision.entry_samples").value_or(kDefaultCollisionEntrySamples);
    if (entry_samples < 1) {
      throw std::invalid_argument("'collision.entry_samples' must be at least 1");
    }
    const auto nominal_joints =
      context.params.get_string_array("collision.nominal_joints").value_or(std::vector<std::string>{});
    const auto nominal_positions =
      context.params.get_double_array("collision.nominal_positions").value_or(std::vector<double>{});
    if (nominal_joints.size() != nominal_positions.size()) {
      throw std::invalid_argument(
              "'collision.nominal_joints' and 'collision.nominal_positions' must have the same length");
    }

    std::shared_ptr<const arm_ik::ArmModel> model;
    try {
      model = arm_ik::ArmModel::load_cached(config);
    } catch (const std::exception & e) {
      throw std::invalid_argument("arm_ik_config '" + config + "': " + e.what());
    }
    auto checker = std::make_unique<PathCollisionChecker>(model, choreography);
    for (size_t i = 0; i < nominal_joints.size(); ++i) {
      if (!checker->set_base_position(nominal_joints[i], nominal_positions[i])) {
        throw std::invalid_argument(
                "'collision.nominal_joints': '" + nominal_joints[i] + "' is not a joint of the IK model");
      }
    }
    const auto start = std::chrono::steady_clock::now();
    const auto spans = checker->check_script(sample_rate);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (!spans.empty()) {
      std::string message = "choreography '" + label + "' makes the arms collide:";
      for (const auto & span : spans) {
        message += "\n  - " + span.describe();
      }
      throw std::invalid_argument(message);
    }
    context.log(locomotion::LogLevel::kInfo,
      "Choreography '" + label + "': no arm collision along the path (" + num(sample_rate, 0) +
      " Hz, " + num(ms, 0) + " ms).");

    collision_ = std::move(checker);
    collision_entry_samples_ = static_cast<int>(entry_samples);
    collision_joint_names_ = context.joint_names;
    entry_starts_.clear();
    entry_velocities_.clear();
    for (const auto & track : tracks_) {
      entry_starts_.emplace_back(track.indices.size(), 0.0);
      entry_velocities_.emplace_back(track.indices.size(), 0.0);
    }
  }

  // 進入時：起點 → 第一個 keyframe 那一段（取決於當下的位置）是否碰撞；其他 joint 用量測值
  void check_entry_collision(const locomotion::RobotState & state) {
    if (!collision_) {
      return;
    }
    for (size_t i = 0; i < collision_joint_names_.size() && i < state.measured_positions.size(); ++i) {
      collision_->set_base_position(collision_joint_names_[i], state.measured_positions[i]);
    }
    for (size_t t = 0; t < tracks_.size(); ++t) {
      std::copy(tracks_[t].start.begin(), tracks_[t].start.end(), entry_starts_[t].begin());
      std::copy(tracks_[t].start_velocity.begin(), tracks_[t].start_velocity.end(),
        entry_velocities_[t].begin());
    }
    const auto spans = collision_->check_entry(entry_starts_, entry_velocities_, collision_entry_samples_);
    if (!spans.empty()) {
      throw std::runtime_error("the move from here to the first keyframes makes the arms collide: " +
              spans.front().describe());
    }
  }

  std::vector<TrackState> tracks_;
  size_t num_joints_{0};
  double duration_{0.0};
  double rate_hz_{0.0};
  double start_tolerance_{0.05};
  bool entry_velocity_{true};
  bool interruptible_{true};  // 腳本頂層的 interruptible

  // 路徑碰撞檢查（沒有設定 arm_ik_config 時為 nullptr）
  std::unique_ptr<PathCollisionChecker> collision_;
  int collision_entry_samples_{static_cast<int>(kDefaultCollisionEntrySamples)};
  std::vector<std::string> collision_joint_names_;
  std::vector<std::vector<double>> entry_starts_;
  std::vector<std::vector<double>> entry_velocities_;

  double time_{0.0};  // 進入模式後經過的時間
  bool finished_{false};
};

}  // namespace graphen::choreographer

PLUGINLIB_EXPORT_CLASS(
  graphen::choreographer::ChoreographerAlgorithm,
  graphen::locomotion::LocomotionAlgorithm)
