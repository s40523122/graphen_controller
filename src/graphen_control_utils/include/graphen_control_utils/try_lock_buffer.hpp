#ifndef GRAPHEN_CONTROL_UTILS__TRY_LOCK_BUFFER_HPP_
#define GRAPHEN_CONTROL_UTILS__TRY_LOCK_BUFFER_HPP_

#include <mutex>
#include <utility>

namespace graphen::control {

// 由 RT thread 寫入、non-RT thread 讀取的單一資料緩衝（方向與 realtime_tools::RealtimeBuffer 相反）。
//
// RT 端以 try_lock 寫入，拿不到 lock（non-RT 端正在讀）就跳過這一次，不會阻塞；
// non-RT 端加 lock 複製出最新值。T 的大小固定時（例如預先配置好長度的 vector），
// 寫入與複製都不會配置記憶體。
template<typename T>
class TryLockBuffer {
public:
  TryLockBuffer() = default;
  explicit TryLockBuffer(T initial)
  : data_(std::move(initial)) {}

  // non-RT：重設內容（例如 configure 時預先配置大小）
  void reset(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = std::move(value);
  }

  // RT：以 writer(T &) 就地更新；拿不到 lock 時不呼叫 writer，回傳 false
  template<typename Writer>
  bool try_write(Writer && writer) {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return false;
    }
    std::forward<Writer>(writer)(data_);
    return true;
  }

  // non-RT：複製最新值到 out
  void read(T & out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    out = data_;
  }

private:
  mutable std::mutex mutex_;
  T data_{};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROL_UTILS__TRY_LOCK_BUFFER_HPP_
