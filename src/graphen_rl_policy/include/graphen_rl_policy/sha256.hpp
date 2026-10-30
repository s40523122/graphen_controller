#ifndef GRAPHEN_RL_POLICY__SHA256_HPP_
#define GRAPHEN_RL_POLICY__SHA256_HPP_

#include <array>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace graphen::rl_policy {

// SHA-256（FIPS 180-4），用來確認載入的 onnx 與 YAML 綁定的是同一個檔案。
// 自行實作以免為了一個雜湊引入 OpenSSL 依賴；只在 configure 時使用，不追求速度
class Sha256 {
public:
  Sha256() {reset();}

  void reset() {
    state_ = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    buffer_size_ = 0;
    total_bytes_ = 0;
  }

  void update(const unsigned char * data, size_t size) {
    total_bytes_ += size;
    for (size_t i = 0; i < size; ++i) {
      buffer_[buffer_size_++] = data[i];
      if (buffer_size_ == buffer_.size()) {
        transform();
        buffer_size_ = 0;
      }
    }
  }

  // 小寫十六進位字串；呼叫後物件需 reset() 才能再使用
  std::string hex_digest() {
    const uint64_t total_bits = total_bytes_ * 8;
    const unsigned char pad_start = 0x80;
    update(&pad_start, 1);
    const unsigned char zero = 0x00;
    while (buffer_size_ != 56) {
      update(&zero, 1);
    }
    std::array<unsigned char, 8> length{};
    for (size_t i = 0; i < length.size(); ++i) {
      length[i] = static_cast<unsigned char>(total_bits >> (56 - 8 * i));
    }
    update(length.data(), length.size());

    std::string hex;
    hex.reserve(64);
    char byte[3];
    for (const uint32_t word : state_) {
      for (int shift = 24; shift >= 0; shift -= 8) {
        std::snprintf(byte, sizeof(byte), "%02x", (word >> shift) & 0xffu);
        hex += byte;
      }
    }
    return hex;
  }

  static std::string file_hex_digest(const std::string & path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("cannot open '" + path + "'");
    }
    Sha256 sha;
    std::array<char, 65536> chunk{};
    while (file) {
      file.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
      sha.update(reinterpret_cast<const unsigned char *>(chunk.data()),
        static_cast<size_t>(file.gcount()));
    }
    return sha.hex_digest();
  }

private:
  static uint32_t rotr(uint32_t x, int n) {return (x >> n) | (x << (32 - n));}

  void transform() {
    static constexpr std::array<uint32_t, 64> k{
      0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u,
      0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu,
      0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu,
      0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
      0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
      0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
      0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u,
      0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
      0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u,
      0xc67178f2u};
    std::array<uint32_t, 64> w{};
    for (size_t i = 0; i < 16; ++i) {
      w[i] = static_cast<uint32_t>(buffer_[4 * i]) << 24 |
        static_cast<uint32_t>(buffer_[4 * i + 1]) << 16 |
        static_cast<uint32_t>(buffer_[4 * i + 2]) << 8 |
        static_cast<uint32_t>(buffer_[4 * i + 3]);
    }
    for (size_t i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    auto [a, b, c, d, e, f, g, h] = state_;
    for (size_t i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = h + s1 + ch + k[i] + w[i];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = s0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    const std::array<uint32_t, 8> add{a, b, c, d, e, f, g, h};
    for (size_t i = 0; i < state_.size(); ++i) {
      state_[i] += add[i];
    }
  }

  std::array<uint32_t, 8> state_{};
  std::array<unsigned char, 64> buffer_{};
  size_t buffer_size_{0};
  uint64_t total_bytes_{0};
};

}  // namespace graphen::rl_policy

#endif  // GRAPHEN_RL_POLICY__SHA256_HPP_
