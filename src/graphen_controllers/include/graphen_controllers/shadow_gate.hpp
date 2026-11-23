#ifndef GRAPHEN_CONTROLLERS__SHADOW_GATE_HPP_
#define GRAPHEN_CONTROLLERS__SHADOW_GATE_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace graphen::control {

// 影子試播的憑證：試播通過時發出 token，正式播放時以 token 確認
//   1. token 是這個 controller 發出的、還沒過期（token_ttl）
//   2. 內容相同（sha256）：試播與正式播放可以一個用 content、一個用舞曲庫的名稱
//   3. 起點相同：目前 planner 的輸出位置和試播時的起點，每個介面的差都在 start_tolerance 以內
// token 在有效期間可以重複使用（每次都重新檢查起點）。可以在任何 non-RT thread 呼叫（內部以 mutex 保護）。
class ShadowGate {
public:
  struct Config {
    bool require{false};        // true：正式播放一定要帶 token
    double start_tolerance{0.01};  // rad
    double token_ttl{300.0};    // s
  };

  // 設定不合法（start_tolerance < 0、token_ttl <= 0）時丟出 std::invalid_argument
  explicit ShadowGate(Config config);

  bool required() const {return config_.require;}

  // 發出 token：sha256 為內容的 hash，start_positions 為試播的起點（planner 的輸出位置）
  std::string issue(const std::string & sha256, const std::vector<double> & start_positions);

  // 正式播放前的檢查；通過時回傳空字串，否則回傳原因（可以直接回給使用者）。
  // token 為空時：required() 為 false 則通過，否則拒絕。names 為各介面的名稱（只用在訊息中）
  std::string verify(
    const std::string & token, const std::string & sha256,
    const std::vector<double> & current_positions, const std::vector<std::string> & names) const;

private:
  using Clock = std::chrono::steady_clock;
  struct Entry {
    std::string token;
    std::string sha256;
    std::vector<double> start_positions;
    Clock::time_point issued;
  };

  // 移除過期的 token。呼叫端需持有 mutex_
  void purge(Clock::time_point now);

  const Config config_;
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
  uint64_t counter_{0};
  uint64_t salt_{0};  // 每個實例不同，避免 controller 重新 configure 後舊的 token 碰巧有效
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SHADOW_GATE_HPP_
