#ifndef IMU16460_CORE__DRIVER_HPP_
#define IMU16460_CORE__DRIVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "imu16460_core/frame_parser.hpp"
#include "imu16460_core/protocol.hpp"
#include "imu16460_core/serial_port.hpp"

namespace imu16460 {

// IMU 驅動：背景讀取 thread 依裝置自己的輸出頻率持續接收、解析，並更新「最新樣本」。
// 使用端（例如 ros2_control 的 read()）只需要取最新一筆，不做任何 serial I/O。
//
// - get_latest() 以 try_lock 交接，拿不到 lock 時直接回傳 false，呼叫端不會被阻塞（RT 安全）。
// - 命令 API 會等待對應 ID 的回應（condition_variable），只能在 non-RT thread 呼叫，且需要先 start()。
// - 所有錯誤以回傳值表示，不丟例外。
class Driver {
public:
  // poll 逾時決定 stop() 的反應時間（必須 < 100 ms）
  static constexpr int kPollTimeoutMs = 20;
  static constexpr size_t kReadChunkSize = 4096;
  static constexpr std::chrono::milliseconds kRateWindow{1000};
  static constexpr std::chrono::milliseconds kDefaultCommandTimeout{500};

  Driver();
  ~Driver();

  Driver(const Driver &) = delete;
  Driver & operator=(const Driver &) = delete;

  // 開啟 port；已開啟時會先關閉。失敗原因見 last_error()
  bool open(const std::string & port, int baudrate);
  // 停止讀取 thread 並關閉 port
  void close();
  bool is_open() const {return port_.is_open();}
  const std::string & last_error() const {return last_error_;}

  // 啟動 / 停止背景讀取 thread
  bool start();
  void stop();
  bool is_running() const {return running_.load();}

  // 複製最新樣本；還沒有資料或 lock 正被讀取 thread 持有時回傳 false。不配置記憶體
  bool get_latest(ImuSample & out) const;

  // 上次呼叫以來所有樣本的平均。姿態角取最新一筆（yaw 有 0/360 跳變，不能平均）。
  // 這段期間沒有新樣本時回傳 false
  bool get_average_since_last(ImuSample & out);

  // 送出命令並等待對應 ID 的回應，回傳回應 payload；逾時或失敗回傳 nullopt。
  // 沒有回應的命令（重啟）送出後立即回傳空 payload
  std::optional<std::vector<uint8_t>> send_command(
    uint16_t command, float param1, uint32_t param3,
    std::chrono::milliseconds timeout = kDefaultCommandTimeout);

  bool set_stream_mode(bool enable, std::chrono::milliseconds timeout = kDefaultCommandTimeout);
  // 讀取參數時 IMU 會關閉數據流，結束後（不論成功與否）會自動送出 CMD 3 恢復
  std::optional<float> read_param(
    uint32_t index, std::chrono::milliseconds timeout = kDefaultCommandTimeout);
  // 以下會改變裝置設定；set_param 需搭配 save_to_flash 才會永久保存
  bool set_param(
    uint32_t index, float value, std::chrono::milliseconds timeout = kDefaultCommandTimeout);
  bool save_to_flash(std::chrono::milliseconds timeout = kDefaultCommandTimeout);
  bool reboot();
  // 回傳 S1 原始 payload（長度依型號而異，尚待實機確認）
  std::optional<std::vector<uint8_t>> get_system_status(
    std::chrono::milliseconds timeout = kDefaultCommandTimeout);

  Stats get_stats() const;
  // 距離最後一筆 A1 樣本的時間；還沒有任何樣本時回傳 nullopt
  std::optional<double> ms_since_last_sample() const;

private:
  void read_loop();
  void on_frame(uint16_t id, const uint8_t * payload, size_t len);
  void update_rate(std::chrono::steady_clock::time_point now);

  SerialPort port_;
  FrameParser parser_;
  std::string last_error_;

  std::thread thread_;
  std::atomic<bool> running_{false};
  // 只在讀取 thread 中使用
  std::array<uint8_t, kReadChunkSize> read_buffer_{};

  // 最新樣本與平均用的累加值
  mutable std::mutex sample_mutex_;
  ImuSample latest_;
  bool has_latest_{false};
  std::array<double, 3> acc_sum_{};
  std::array<double, 3> gyro_sum_{};
  double temp_sum_{0.0};
  uint64_t sum_count_{0};

  // 命令 / 回應交接；command_mutex_ 保證同時只有一個命令在等待回應
  std::mutex command_mutex_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  static constexpr int32_t kNoPending = -1;
  std::atomic<int32_t> pending_id_{kNoPending};
  bool response_ready_{false};
  std::vector<uint8_t> response_;

  // 統計（讀取 thread 寫入，其他 thread 讀取）
  std::atomic<uint64_t> frames_ok_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> bytes_dropped_{0};
  std::atomic<double> rate_hz_{0.0};
  std::atomic<int64_t> last_sample_ns_{0};  // steady_clock，0 表示沒有樣本
  // 頻率計算視窗（只在讀取 thread 中使用）
  std::chrono::steady_clock::time_point rate_window_start_{};
  uint64_t rate_window_count_{0};
};

}  // namespace imu16460

#endif  // IMU16460_CORE__DRIVER_HPP_
