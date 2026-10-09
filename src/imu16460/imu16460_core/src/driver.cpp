#include "imu16460_core/driver.hpp"

#include <algorithm>

namespace imu16460 {

using Clock = std::chrono::steady_clock;

Driver::Driver()
: parser_([this](uint16_t id, const uint8_t * payload, size_t len) {on_frame(id, payload, len);})
{
  // 預先配置，讀取 thread 複製回應時不會再配置記憶體
  response_.reserve(kMaxPayloadSize);
}

Driver::~Driver()
{
  close();
}

bool Driver::open(const std::string & port, int baudrate)
{
  close();
  if (!port_.open(port, baudrate)) {
    last_error_ = port_.last_error();
    return false;
  }
  parser_.reset();
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    has_latest_ = false;
    acc_sum_ = {};
    gyro_sum_ = {};
    temp_sum_ = 0.0;
    sum_count_ = 0;
  }
  last_sample_ns_ = 0;
  rate_hz_ = 0.0;
  last_error_.clear();
  return true;
}

void Driver::close()
{
  stop();
  port_.close();
}

bool Driver::start()
{
  if (running_) {
    return true;
  }
  if (!port_.is_open()) {
    last_error_ = "port is not open";
    return false;
  }
  rate_window_start_ = Clock::now();
  rate_window_count_ = 0;
  running_ = true;
  thread_ = std::thread(&Driver::read_loop, this);
  return true;
}

void Driver::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  rate_hz_ = 0.0;
}

void Driver::read_loop()
{
  while (running_) {
    const ssize_t n = port_.read(read_buffer_.data(), read_buffer_.size(), kPollTimeoutMs);
    if (n > 0) {
      parser_.feed(read_buffer_.data(), static_cast<size_t>(n));
      frames_ok_ = parser_.frames_ok();
      crc_errors_ = parser_.crc_errors();
      bytes_dropped_ = parser_.bytes_dropped();
    } else if (n < 0) {
      // 裝置錯誤（例如拔除）：避免 busy loop，等一個 poll 週期後重試
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollTimeoutMs));
    }
    update_rate(Clock::now());
  }
}

void Driver::update_rate(Clock::time_point now)
{
  const auto elapsed = now - rate_window_start_;
  if (elapsed >= kRateWindow) {
    rate_hz_ = static_cast<double>(rate_window_count_) /
      std::chrono::duration<double>(elapsed).count();
    rate_window_start_ = now;
    rate_window_count_ = 0;
  }
}

void Driver::on_frame(uint16_t id, const uint8_t * payload, size_t len)
{
  const auto now = Clock::now();

  if (id == frame_id::kAhrs) {
    if (auto sample = decode_ahrs(payload, len)) {
      sample->host_rx_time = now;
      {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        latest_ = *sample;
        has_latest_ = true;
        for (size_t i = 0; i < 3; ++i) {
          acc_sum_[i] += sample->acc_g[i];
          gyro_sum_[i] += sample->gyro_dps[i];
        }
        temp_sum_ += sample->temp_c;
        ++sum_count_;
      }
      last_sample_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      ++rate_window_count_;
    }
  }

  // 命令回應會和數據流交錯出現，依 ID 分派
  if (pending_id_.load() == static_cast<int32_t>(id)) {
    std::lock_guard<std::mutex> lock(response_mutex_);
    if (!response_ready_ && pending_id_.load() == static_cast<int32_t>(id)) {
      response_.assign(payload, payload + std::min(len, kMaxPayloadSize));
      response_ready_ = true;
      response_cv_.notify_all();
    }
  }
}

bool Driver::get_latest(ImuSample & out) const
{
  std::unique_lock<std::mutex> lock(sample_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !has_latest_) {
    return false;
  }
  out = latest_;
  return true;
}

bool Driver::get_average_since_last(ImuSample & out)
{
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (sum_count_ == 0) {
    return false;
  }
  out = latest_;
  const double n = static_cast<double>(sum_count_);
  for (size_t i = 0; i < 3; ++i) {
    out.acc_g[i] = static_cast<float>(acc_sum_[i] / n);
    out.gyro_dps[i] = static_cast<float>(gyro_sum_[i] / n);
  }
  out.temp_c = static_cast<float>(temp_sum_ / n);
  acc_sum_ = {};
  gyro_sum_ = {};
  temp_sum_ = 0.0;
  sum_count_ = 0;
  return true;
}

std::optional<std::vector<uint8_t>> Driver::send_command(
  uint16_t command, float param1, uint32_t param3, std::chrono::milliseconds timeout)
{
  // 回應由讀取 thread 收取，沒有啟動就收不到
  if (!running_) {
    last_error_ = "driver is not running";
    return std::nullopt;
  }
  std::lock_guard<std::mutex> command_lock(command_mutex_);

  const auto expected_id = response_id_for(command);
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    response_ready_ = false;
    response_.clear();
    pending_id_ = expected_id ? static_cast<int32_t>(*expected_id) : kNoPending;
  }

  const auto frame = encode_command(command, param1, param3);
  if (!port_.write_all(frame.data(), frame.size())) {
    pending_id_ = kNoPending;
    last_error_ = "failed to write command " + std::to_string(command);
    return std::nullopt;
  }
  if (!expected_id) {
    return std::vector<uint8_t>{};
  }

  std::unique_lock<std::mutex> lock(response_mutex_);
  const bool ok = response_cv_.wait_for(lock, timeout, [this] {return response_ready_;});
  pending_id_ = kNoPending;
  if (!ok) {
    last_error_ = "timeout waiting for response to command " + std::to_string(command);
    return std::nullopt;
  }
  return response_;
}

bool Driver::set_stream_mode(bool enable, std::chrono::milliseconds timeout)
{
  const auto resp = send_command(cmd::kSetMode, enable ? kModeStream : kModeCommand, 0, timeout);
  if (!resp) {
    return false;
  }
  const auto ack = decode_ack(resp->data(), resp->size());
  return ack && ack->first == cmd::kSetMode && ack->second == kAckResultOk;
}

std::optional<float> Driver::read_param(uint32_t index, std::chrono::milliseconds timeout)
{
  const auto resp = send_command(cmd::kReadParam, 0.0f, index, timeout);
  // 讀取參數時 IMU 會關閉數據流，不論結果如何都恢復
  set_stream_mode(true, timeout);
  if (!resp) {
    return std::nullopt;
  }
  const auto value = decode_param_value(resp->data(), resp->size());
  if (!value || value->index != index) {
    last_error_ = "unexpected response to read_param " + std::to_string(index);
    return std::nullopt;
  }
  return value->value;
}

bool Driver::set_param(uint32_t index, float value, std::chrono::milliseconds timeout)
{
  const auto resp = send_command(cmd::kSetParam, value, index, timeout);
  if (!resp) {
    return false;
  }
  const auto ack = decode_ack(resp->data(), resp->size());
  return ack && ack->first == kSetParamAckTag && ack->second == index;
}

bool Driver::save_to_flash(std::chrono::milliseconds timeout)
{
  const auto resp = send_command(cmd::kSaveToFlash, 0.0f, 0, timeout);
  if (!resp) {
    return false;
  }
  const auto ack = decode_ack(resp->data(), resp->size());
  return ack && ack->first == cmd::kSaveToFlash && ack->second == kAckResultOk;
}

bool Driver::reboot()
{
  return send_command(cmd::kReboot, 0.0f, 0).has_value();
}

std::optional<std::vector<uint8_t>> Driver::get_system_status(std::chrono::milliseconds timeout)
{
  // TODO(verify-on-hw): S1 的實際長度與欄位依型號而異，目前只回傳原始 payload
  return send_command(cmd::kGetSystemStatus, 0.0f, 0, timeout);
}

Stats Driver::get_stats() const
{
  return Stats{frames_ok_.load(), crc_errors_.load(), bytes_dropped_.load(), rate_hz_.load()};
}

std::optional<double> Driver::ms_since_last_sample() const
{
  const int64_t last_ns = last_sample_ns_.load();
  if (last_ns == 0) {
    return std::nullopt;
  }
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
  return static_cast<double>(now_ns - last_ns) / 1e6;
}

}  // namespace imu16460
