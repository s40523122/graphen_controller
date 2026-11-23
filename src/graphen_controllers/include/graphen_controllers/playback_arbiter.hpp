#ifndef GRAPHEN_CONTROLLERS__PLAYBACK_ARBITER_HPP_
#define GRAPHEN_CONTROLLERS__PLAYBACK_ARBITER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "graphen_controllers/locomotion_planner.hpp"
#include "graphen_controllers/motion.hpp"
#include "rclcpp/logger.hpp"

namespace graphen::control {

// 播放的中斷設定
struct PlaybackPolicy {
  // false：播放中（尚未結束）拒絕被其他播放取代或重播，只有 preempt 的可以切入
  bool interruptible{true};
  // true：可以切入不可中斷的播放（停止類的系統模式）
  bool preempt{false};
};

// 一次播放要求：所有入口（SwitchMode / PlayChoreo / PlayBvh / RunPolicy / ~/mode）都轉成這個形式
struct PlaybackRequest {
  // 類型（choreo / bvh / policy）或系統模式名稱
  std::string mode;
  // 項目名稱（舞曲、BVH 檔、policy）；系統模式為空
  std::string item;
  // 判斷「同一個」的鍵：相同時套用 on_same，不同時視為切換。
  // 內容會變的類型要包含內容的版本（例如 sha256），讓重新發佈後的同名項目被視為不同
  std::string identity;
  // 只用在 log 中的補充說明（例如內容的 sha256）
  std::string detail;
  std::shared_ptr<Motion> motion;
  PlaybackPolicy policy;
  // 內容的 sha256（影子試播的憑證以它比對內容）；沒有內容的類型為空
  std::string content_sha256;
};

// 目前在播的就是同一個時的處理方式（對應 PlaybackOptions.msg 的 ON_SAME_*）
enum class OnSame {
  kIgnore,             // 不重新開始
  kRestart,            // 從頭重播（等同打斷目前的播放）
  kRestartIfFinished,  // 這次播放已經結束才重播
};

struct PlaybackOutcome {
  enum class Status {
    kSwitched,   // 已切換（或重播）
    kUnchanged,  // 已經在播同一個，依 on_same 不重新開始
    kRejected,   // 目前在播的不可中斷（或其他原因：內容有誤、試播憑證不符…）
    kDryRun,     // 影子試播通過（沒有切換）
  };
  Status status{Status::kRejected};
  uint64_t sequence{0};  // 目前這次播放在 planner 中的序號（kSwitched / kUnchanged 時有效）
  std::string message;   // kRejected 的原因；kDryRun 的結果說明
  std::string shadow_token;  // kDryRun 時的試播憑證
};

// "<mode>" 或 "<mode>:<item>"，用在訊息中
std::string playback_label(const std::string & mode, const std::string & item);

// 集中決定「這個播放要求能不能取代目前在播的」，並把通過的 Motion 交給 planner。
// 所有入口共用同一套規則，所以它們可以互相取代，不可中斷與 preempt 也對所有類型一致：
//   1. 和目前在播的是同一個（identity 相同）：依 on_same 決定重播或維持
//   2. 目前在播的不可中斷、還沒結束，而且要求的不是 preempt：拒絕（重播同一個也算中斷）
//   3. 其餘：切換
// 可以在任何 non-RT thread 呼叫（內部以 mutex 保護）；planner 必須比這個物件活得久。
class PlaybackArbiter {
public:
  struct Current {
    std::string mode;  // 尚未進入任何模式（保持）時為空
    std::string item;
    uint64_t sequence{0};
  };

  PlaybackArbiter(LocomotionPlanner * planner, rclcpp::Logger logger);

  PlaybackOutcome request(PlaybackRequest request, OnSame on_same);

  // 不套用規則，直接開始播放（controller activate 時的初始動作）。
  // request.mode 為空表示「尚未進入任何模式」：只是保持，之後任何要求都能切入
  uint64_t force(PlaybackRequest request);

  Current current() const;

  // 目前這次播放的進度（planner 的序號可能還停在上一次，見 LocomotionPlanner::Progress）
  LocomotionPlanner::Progress progress() const {return planner_->progress();}

private:
  // 目前這次播放是否已經結束（完成或失敗）。呼叫端需持有 mutex_
  bool current_ended() const;

  LocomotionPlanner * const planner_;
  const rclcpp::Logger logger_;

  mutable std::mutex mutex_;
  std::string mode_;
  std::string item_;
  std::string identity_;
  PlaybackPolicy policy_;
  // 目前這次播放在 planner 中的序號；與等待中 goal 的序號不同表示已被取代
  uint64_t sequence_{0};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__PLAYBACK_ARBITER_HPP_
