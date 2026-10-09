#include "imu16460_core/crc32.hpp"

#include <array>

namespace imu16460 {

namespace {

// 編譯期產生查表（等同手冊附錄的 crc32_tab）
constexpr std::array<uint32_t, 256> make_table()
{
  std::array<uint32_t, 256> table{};
  for (uint32_t i = 0; i < table.size(); ++i) {
    uint32_t c = i;
    for (int bit = 0; bit < 8; ++bit) {
      c = (c & 1u) ? (kCrcPolynomial ^ (c >> 1)) : (c >> 1);
    }
    table[i] = c;
  }
  return table;
}

constexpr auto kTable = make_table();

}  // namespace

uint32_t crc32(const uint8_t * data, size_t len, uint32_t init)
{
  uint32_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc = kTable[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return crc;
}

}  // namespace imu16460
