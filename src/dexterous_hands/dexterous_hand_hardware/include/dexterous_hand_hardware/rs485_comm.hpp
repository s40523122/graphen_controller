#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace dexterous_hand_hardware
{

class RS485Comm
{
public:
  RS485Comm() = default;
  ~RS485Comm();

  bool connect(const std::string & port, int baudrate = 115200);
  void disconnect();
  bool is_connected() const;

  // 送出 13 軸角度（單位：靈巧手原始值）
  bool write_angles(uint8_t hand_id, const std::vector<int16_t> & angles);

  // 讀取 13 軸實際角度(可省略)
  bool read_angles(uint8_t hand_id, std::vector<int16_t> & out_angles);

  // 設定速度（所有軸相同）
  bool set_speed(uint8_t hand_id, int16_t speed);

  // 連續讀取有號整數 (用於關節狀態、受力、電流、溫度等)
  bool read_registers(
    uint8_t hand_id, uint16_t start_addr, uint16_t length,
    std::vector<int16_t> & out_data);

  // 連續讀取無號整數 (專用於觸覺感測器)
  bool read_registers_uint(
    uint8_t hand_id, uint16_t start_addr, uint16_t length,
    std::vector<uint16_t> & out_data);

private:
  int fd_ = -1;  // 串口 file descriptor

  static uint8_t checksum(const std::vector<uint8_t> & payload);

  std::vector<uint8_t> build_write_frame(
    uint8_t hand_id, uint16_t address, const std::vector<uint8_t> & data);

  std::vector<uint8_t> build_read_frame(
    uint8_t hand_id, uint16_t address, uint8_t byte_len);

  bool write_registers(
    uint8_t hand_id, uint16_t address, const std::vector<uint8_t> & data);

  bool read_registers(
    uint8_t hand_id, uint16_t address, uint8_t byte_len,
    std::vector<uint8_t> & response);

  ssize_t read_exact(uint8_t * buf, size_t len, int timeout_ms = 200);
};

}  // namespace dexterous_hand_hardware
