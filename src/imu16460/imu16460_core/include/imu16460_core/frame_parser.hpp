#ifndef IMU16460_CORE__FRAME_PARSER_HPP_
#define IMU16460_CORE__FRAME_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "imu16460_core/protocol.hpp"

namespace imu16460 {

// 串流式解析器：從任意切段的 byte 串流中找出 IMU → 主機的幀，CRC 正確才回報。
//
// - 長度超過 max_payload 視為偽幀頭，跳過 2 bytes 重新同步。
// - CRC 錯誤時同樣跳過 2 bytes，讓錯誤幀內部可能存在的真正幀頭有機會被找到。
// - 內部 buffer 在建構時配置一次，feed() 不配置記憶體。
// - callback 收到的 payload 指標只在 callback 執行期間有效；callback 內不可再呼叫 feed()。
// - 非 thread-safe，應由單一 thread 使用。
class FrameParser {
public:
  using FrameCallback = std::function<void (uint16_t id, const uint8_t * payload, size_t len)>;

  explicit FrameParser(FrameCallback callback, size_t max_payload = kMaxPayloadSize);

  void feed(const uint8_t * data, size_t len);

  // 丟棄尚未解析完成的資料（例如重新開啟 port 時）；統計值不變
  void reset() {size_ = 0;}

  uint64_t frames_ok() const {return frames_ok_;}
  uint64_t crc_errors() const {return crc_errors_;}
  uint64_t bytes_dropped() const {return bytes_dropped_;}

private:
  void process();

  FrameCallback callback_;
  const size_t max_payload_;
  std::vector<uint8_t> buffer_;  // 容量 = 最長一幀
  size_t size_{0};

  uint64_t frames_ok_{0};
  uint64_t crc_errors_{0};
  uint64_t bytes_dropped_{0};
};

}  // namespace imu16460

#endif  // IMU16460_CORE__FRAME_PARSER_HPP_
