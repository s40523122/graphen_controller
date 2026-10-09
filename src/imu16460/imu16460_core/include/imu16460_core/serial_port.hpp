#ifndef IMU16460_CORE__SERIAL_PORT_HPP_
#define IMU16460_CORE__SERIAL_PORT_HPP_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace imu16460 {

// termios serial port 的 RAII 包裝：raw 模式、8N1、無流量控制。
// read() 以 poll() 等待資料並帶逾時，讓讀取 thread 可以定期檢查是否該結束。
// read() 與 write_all() 可以在不同 thread 同時呼叫；open() / close() 必須在沒有其他 thread 使用時呼叫。
class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // baudrate 必須是 kSupportedBaudrates 之一；失敗時回傳 false，原因見 last_error()
  bool open(const std::string & device, int baudrate);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // 回傳讀到的 byte 數；逾時回傳 0；錯誤回傳 -1
  ssize_t read(uint8_t * buffer, size_t size, int timeout_ms);

  // 寫入全部資料；失敗回傳 false
  bool write_all(const uint8_t * data, size_t size);

  const std::string & last_error() const {return last_error_;}

private:
  int fd_{-1};
  std::string last_error_;
};

}  // namespace imu16460

#endif  // IMU16460_CORE__SERIAL_PORT_HPP_
