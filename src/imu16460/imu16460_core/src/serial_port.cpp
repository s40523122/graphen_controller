#include "imu16460_core/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>

namespace imu16460 {

namespace {

// 只使用標準的 Bxxxx 常數，不依賴非標準的自訂鮑率 ioctl
std::optional<speed_t> to_speed(int baudrate)
{
  switch (baudrate) {
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    case 1500000:
      return B1500000;
    default:
      return std::nullopt;
  }
}

std::string errno_message(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baudrate)
{
  close();

  const auto speed = to_speed(baudrate);
  if (!speed) {
    last_error_ = "unsupported baudrate " + std::to_string(baudrate);
    return false;
  }

  // O_NONBLOCK 避免在等待 modem 控制線時卡在 open()，設定好 CLOCAL 後再改回 blocking
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    last_error_ = errno_message("open " + device);
    return false;
  }

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    last_error_ = errno_message("tcgetattr");
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
  tio.c_cflag |= CS8;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  // 等待交給 poll()，read() 有多少讀多少
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (cfsetispeed(&tio, *speed) != 0 || cfsetospeed(&tio, *speed) != 0 ||
    tcsetattr(fd, TCSANOW, &tio) != 0)
  {
    last_error_ = errno_message("configure " + device);
    ::close(fd);
    return false;
  }

  const int flags = fcntl(fd, F_GETFL);
  if (flags < 0 || fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) != 0) {
    last_error_ = errno_message("fcntl");
    ::close(fd);
    return false;
  }

  // 丟棄開啟前累積的舊資料
  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  last_error_.clear();
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::read(uint8_t * buffer, size_t size, int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }
  pollfd pfd{fd_, POLLIN, 0};
  const int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret == 0) {
    return 0;
  }
  if (ret < 0) {
    // 被 signal 打斷視為逾時，由呼叫端重試
    return errno == EINTR ? 0 : -1;
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return -1;
  }
  const ssize_t n = ::read(fd_, buffer, size);
  if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
    return 0;
  }
  return n;
}

bool SerialPort::write_all(const uint8_t * data, size_t size)
{
  if (fd_ < 0) {
    return false;
  }
  size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd_, data + written, size - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace imu16460
