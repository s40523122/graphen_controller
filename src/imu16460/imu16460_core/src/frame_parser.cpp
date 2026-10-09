#include "imu16460_core/frame_parser.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include "imu16460_core/crc32.hpp"

namespace imu16460 {

FrameParser::FrameParser(FrameCallback callback, size_t max_payload)
: callback_(std::move(callback)),
  max_payload_(max_payload),
  buffer_(kFrameOverhead + max_payload)
{
}

void FrameParser::feed(const uint8_t * data, size_t len)
{
  // 分批複製進 buffer：process() 之後剩下的一定是不完整的一幀，必定小於容量，所以每輪都有進展
  while (len > 0) {
    const size_t n = std::min(len, buffer_.size() - size_);
    std::memcpy(buffer_.data() + size_, data, n);
    size_ += n;
    data += n;
    len -= n;
    process();
  }
}

void FrameParser::process()
{
  const uint8_t * buf = buffer_.data();
  size_t pos = 0;

  while (true) {
    // 尋找幀頭
    while (pos + 1 < size_ && !(buf[pos] == kDeviceHeader[0] && buf[pos + 1] == kDeviceHeader[1])) {
      ++pos;
      ++bytes_dropped_;
    }
    if (pos + 1 >= size_) {
      // 只剩 0 或 1 byte：若最後一個 byte 可能是幀頭的開頭就保留，否則丟掉
      if (pos < size_ && buf[pos] != kDeviceHeader[0]) {
        ++pos;
        ++bytes_dropped_;
      }
      break;
    }
    if (size_ - pos < kHeaderSize) {
      break;
    }

    const size_t payload_len = read_u16_le(buf + pos + kLengthOffset);
    if (payload_len > max_payload_) {
      // 偽幀頭
      pos += kDeviceHeader.size();
      bytes_dropped_ += kDeviceHeader.size();
      continue;
    }
    const size_t frame_len = kFrameOverhead + payload_len;
    if (size_ - pos < frame_len) {
      break;
    }

    const size_t crc_offset = pos + kHeaderSize + payload_len;
    if (crc32(buf + pos, kHeaderSize + payload_len) != read_u32_le(buf + crc_offset)) {
      ++crc_errors_;
      pos += kDeviceHeader.size();
      bytes_dropped_ += kDeviceHeader.size();
      continue;
    }

    ++frames_ok_;
    if (callback_) {
      callback_(read_u16_le(buf + pos + kIdOffset), buf + pos + kHeaderSize, payload_len);
    }
    pos += frame_len;
  }

  // 把未處理的資料移到 buffer 開頭
  if (pos > 0) {
    std::memmove(buffer_.data(), buffer_.data() + pos, size_ - pos);
    size_ -= pos;
  }
}

}  // namespace imu16460
