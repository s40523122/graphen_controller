#ifndef IMU16460_CORE__CRC32_HPP_
#define IMU16460_CORE__CRC32_HPP_

#include <cstddef>
#include <cstdint>

#include "imu16460_core/protocol.hpp"

namespace imu16460 {

// 計算 IMU 協議使用的 CRC32：範圍是幀頭第一個 byte 到 payload 最後一個 byte。
// init 可以傳入前一段的結果，分段計算。
uint32_t crc32(const uint8_t * data, size_t len, uint32_t init = kCrcInit);

}  // namespace imu16460

#endif  // IMU16460_CORE__CRC32_HPP_
