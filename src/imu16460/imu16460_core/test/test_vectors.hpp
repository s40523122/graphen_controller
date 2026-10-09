#ifndef TEST_VECTORS_HPP_
#define TEST_VECTORS_HPP_

#include <cstdint>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

// 手冊中的測試向量（CRC 已驗證正確），見 docs/IMU16460.md

namespace test_vectors {

using Bytes = std::vector<uint8_t>;

// "AA 55 02" → {0xAA, 0x55, 0x02}
inline Bytes hex(const std::string & s)
{
  Bytes out;
  std::istringstream in(s);
  std::string token;
  while (in >> token) {
    out.push_back(static_cast<uint8_t>(std::stoul(token, nullptr, 16)));
  }
  return out;
}

inline Bytes zeros(size_t n)
{
  return Bytes(n, 0);
}

inline Bytes cat(std::initializer_list<Bytes> parts)
{
  Bytes out;
  for (const auto & p : parts) {
    out.insert(out.end(), p.begin(), p.end());
  }
  return out;
}

// A1 數據流幀
inline Bytes a1_frame()
{
  return hex(
    "AA 55 02 00 2C 00 6D 89 16 05 8F C2 65 40 14 AE 07 BF 5C 0F B2 43 25 06 81 3D "
    "BC 74 13 3C 60 E5 80 BF EC 51 38 BD 0A D7 A3 BB CD CC CC BC D7 A3 EE 41 0C BF 84 80");
}

// ---- 主機 → IMU 命令 ----
inline Bytes cmd1_status() {return cat({hex("55 AA 01 00 18 00"), zeros(24), hex("BD DB 31 34")});}
inline Bytes cmd3_stream()
{
  return cat({hex("55 AA 03 00 18 00 00 00 80 3F"), zeros(20), hex("52 D8 8E E8")});
}
inline Bytes cmd5_save() {return cat({hex("55 AA 05 00 18 00"), zeros(24), hex("C9 2F E6 32")});}
inline Bytes cmd6_read21()
{
  return cat({hex("55 AA 06 00 18 00"), zeros(8), hex("15 00 00 00"), zeros(12),
      hex("66 CB 46 AC")});
}
inline Bytes cmd6_read4()
{
  return cat({hex("55 AA 06 00 18 00"), zeros(8), hex("04 00 00 00"), zeros(12),
      hex("69 64 09 E4")});
}
inline Bytes cmd14_baud115200()
{
  return cat({hex("55 AA 0E 00 18 00 00 00 E1 47"), zeros(4), hex("03 00 00 00"), zeros(12),
      hex("56 2B 4D 93")});
}
inline Bytes cmd14_rate100()
{
  return cat({hex("55 AA 0E 00 18 00 00 00 C8 42"), zeros(4), hex("15 00 00 00"), zeros(12),
      hex("0A 2B 2C 8D")});
}
inline Bytes cmd14_orient115()
{
  return cat({hex("55 AA 0E 00 18 00 00 00 E6 42"), zeros(4), hex("04 00 00 00"), zeros(12),
      hex("46 6A 4E 86")});
}

// ---- IMU → 主機 回應 ----
inline Bytes resp_cmd3() {return hex("AA 55 64 00 04 00 03 00 01 00 E7 87 E3 AD");}
inline Bytes resp_save() {return hex("AA 55 3D 75 04 00 05 00 01 00 5A CF B1 7C");}
inline Bytes resp_read21()  // 值 50.0f
{
  return cat({hex("AA 55 30 75 18 00 00 00 48 42"), zeros(4), hex("15 00 00 00"), zeros(12),
      hex("31 2F A2 0A")});
}
inline Bytes resp_read4()  // 值 115.0f
{
  return cat({hex("AA 55 30 75 18 00 00 00 E6 42"), zeros(4), hex("04 00 00 00"), zeros(12),
      hex("B2 2F 2D 4E")});
}
inline Bytes resp_set_baud() {return hex("AA 55 3D 75 04 00 34 75 03 00 A7 98 2A 54");}
inline Bytes resp_set_rate() {return hex("AA 55 3D 75 04 00 34 75 15 00 70 2D B2 48");}
inline Bytes resp_set_orient() {return hex("AA 55 3D 75 04 00 34 75 04 00 60 0E 6B 1B");}

inline std::vector<Bytes> all_frames()
{
  return {a1_frame(), cmd1_status(), cmd3_stream(), cmd5_save(), cmd6_read21(), cmd6_read4(),
    cmd14_baud115200(), cmd14_rate100(), cmd14_orient115(), resp_cmd3(), resp_save(),
    resp_read21(), resp_read4(), resp_set_baud(), resp_set_rate(), resp_set_orient()};
}

}  // namespace test_vectors

#endif  // TEST_VECTORS_HPP_
