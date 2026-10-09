#include "dexterous_hand_hardware/rs485_comm.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <stdexcept>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace dexterous_hand_hardware
{

static constexpr uint16_t REG_ANGLE_SET = 0x0438;
static constexpr uint16_t REG_ANGLE_ACT = 0x0470;
static constexpr uint16_t REG_SPEED_SET = 0x0454;

// ── 連線 / 斷線 ──────────────────────────────────────────────

bool RS485Comm::connect(const std::string & port, int baudrate)
{
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RS485Comm"),
      "無法開啟串口 %s: %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  tcgetattr(fd_, &tty);

  // 設定波特率
  speed_t baud;
  switch (baudrate) {
    case 115200: baud = B115200; break;
    case 57600:  baud = B57600;  break;
    default:     baud = B115200; break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_iflag = IGNPAR;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  // 非阻塞讀取
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &tty);

  RCLCPP_INFO(rclcpp::get_logger("RS485Comm"),
    "串口已開啟：%s @ %d baud", port.c_str(), baudrate);
  return true;
}

void RS485Comm::disconnect()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("RS485Comm"), "串口已關閉");
  }
}

bool RS485Comm::is_connected() const { return fd_ >= 0; }

RS485Comm::~RS485Comm() { disconnect(); }

// ── 工具函式 ─────────────────────────────────────────────────

uint8_t RS485Comm::checksum(const std::vector<uint8_t> & payload)
{
  uint32_t sum = 0;
  for (auto b : payload) sum += b;
  return static_cast<uint8_t>(sum & 0xFF);
}

std::vector<uint8_t> RS485Comm::build_write_frame(
  uint8_t hand_id, uint16_t address, const std::vector<uint8_t> & data)
{
  uint8_t addr_l = address & 0xFF;
  uint8_t addr_h = (address >> 8) & 0xFF;

  std::vector<uint8_t> payload = {
    hand_id,
    static_cast<uint8_t>(data.size() + 3),
    0x12, addr_l, addr_h
  };
  payload.insert(payload.end(), data.begin(), data.end());

  std::vector<uint8_t> frame = {0xEB, 0x90};
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(checksum(payload));
  return frame;
}

std::vector<uint8_t> RS485Comm::build_read_frame(
  uint8_t hand_id, uint16_t address, uint8_t byte_len)
{
  uint8_t addr_l = address & 0xFF;
  uint8_t addr_h = (address >> 8) & 0xFF;

  std::vector<uint8_t> payload = {
    hand_id, 0x04, 0x11, addr_l, addr_h, byte_len
  };

  std::vector<uint8_t> frame = {0xEB, 0x90};
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(checksum(payload));
  return frame;
}

// ── 帶 timeout 的精確讀取 ─────────────────────────────────────

ssize_t RS485Comm::read_exact(uint8_t * buf, size_t len, int timeout_ms)
{
  size_t received = 0;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (received < len) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - now).count();

    struct timeval tv;
    tv.tv_sec  = remaining / 1000000;
    tv.tv_usec = remaining % 1000000;

    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) break;

    ssize_t n = read(fd_, buf + received, len - received);
    if (n > 0) received += n;
  }
  return static_cast<ssize_t>(received);
}

// ── 底層讀寫 ─────────────────────────────────────────────────

bool RS485Comm::write_registers(
  uint8_t hand_id, uint16_t address, const std::vector<uint8_t> & data)
{
  if (!is_connected()) return false;

  auto frame = build_write_frame(hand_id, address, data);

  tcflush(fd_, TCIFLUSH);
  write(fd_, frame.data(), frame.size());

  uint8_t resp[9];
  ssize_t n = read_exact(resp, 9, 200);

  if (n < 9) {
    RCLCPP_WARN(rclcpp::get_logger("RS485Comm"),
      "write_registers: 回覆不足 %zd/9 bytes", n);
    return false;
  }

  bool ok = (resp[0] == 0x90 && resp[4] == 0x12 && resp[7] == 0x01);
  if (!ok) {
    RCLCPP_WARN(rclcpp::get_logger("RS485Comm"), "write_registers: 回覆異常");
  }
  return ok;
}

bool RS485Comm::read_registers(
  uint8_t hand_id, uint16_t address, uint8_t byte_len,
  std::vector<uint8_t> & response)
{
  if (!is_connected()) return false;

  auto frame = build_read_frame(hand_id, address, byte_len);

  tcflush(fd_, TCIFLUSH);
  write(fd_, frame.data(), frame.size());

  size_t total = 7 + byte_len + 1;  // header(7) + data + checksum
  response.resize(total);
  ssize_t n = read_exact(response.data(), total, 200);

  if (n < static_cast<ssize_t>(total)) {
    RCLCPP_WARN(rclcpp::get_logger("RS485Comm"),
      "read_registers: 回覆不足 %zd/%zu bytes", n, total);
    return false;
  }

  return (response[0] == 0x90 && response[4] == 0x11);
}

// ── 公開介面 ─────────────────────────────────────────────────

bool RS485Comm::write_angles(uint8_t hand_id, const std::vector<int16_t> & angles)
{
  if (angles.size() != 13) return false;

  std::vector<uint8_t> data;
  data.reserve(26);
  for (auto v : angles) {
    data.push_back(static_cast<uint8_t>(v & 0xFF));
    data.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  }
  return write_registers(hand_id, REG_ANGLE_SET, data);
}

bool RS485Comm::read_angles(uint8_t hand_id, std::vector<int16_t> & out_angles)
{
  std::vector<uint8_t> resp;
  if (!read_registers(hand_id, REG_ANGLE_ACT, 26, resp)) return false;

  out_angles.resize(13);
  for (int i = 0; i < 13; i++) {
    out_angles[i] = static_cast<int16_t>(resp[7 + i * 2] | (resp[8 + i * 2] << 8));
  }
  return true;
}

bool RS485Comm::set_speed(uint8_t hand_id, int16_t speed)
{
  std::vector<uint8_t> data;
  data.reserve(26);
  for (int i = 0; i < 13; i++) {
    data.push_back(static_cast<uint8_t>(speed & 0xFF));
    data.push_back(static_cast<uint8_t>((speed >> 8) & 0xFF));
  }
  return write_registers(hand_id, REG_SPEED_SET, data);
}

// 新的
bool RS485Comm::read_registers(uint8_t hand_id, uint16_t start_addr, 
  uint16_t length, std::vector<int16_t> & out_data)
{
  std::vector<uint8_t> resp;
  uint8_t byte_len = static_cast<uint8_t>(length * 2);

  if (!read_registers(hand_id, start_addr, byte_len, resp)) {
    return false;
  }

  out_data.resize(length);
  for (size_t i = 0; i < length; i++) {
    out_data[i] = static_cast<int16_t>(resp[7 + i * 2] | (resp[8 + i * 2] << 8));
  }
  return true;
}

bool RS485Comm::read_registers_uint(
  uint8_t hand_id, uint16_t start_addr, uint16_t length,
  std::vector<uint16_t> & out_data)
{
  std::vector<uint8_t> resp;
  uint8_t byte_len = static_cast<uint8_t>(length * 2);
  
  if (!read_registers(hand_id, start_addr, byte_len, resp)) {
    return false;
  }

  out_data.resize(length);
  for (size_t i = 0; i < length; i++) {
    out_data[i] = static_cast<uint16_t>(resp[7 + i * 2] | (resp[8 + i * 2] << 8));
  }
  return true;
}


}  // namespace dexterous_hand_hardware
