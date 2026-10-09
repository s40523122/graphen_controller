#ifndef IMU16460_CORE__PROTOCOL_HPP_
#define IMU16460_CORE__PROTOCOL_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

// FSS-IMU16460-C 通訊協議的常數、資料結構與編解碼。
// 協議細節與測試向量見 docs/IMU16460.md；所有欄位皆為 little-endian。

namespace imu16460 {

// ---- 幀格式 ----
// [幀頭 2][ID u16][payload 長度 u16][payload n][CRC32 u32]
constexpr std::array<uint8_t, 2> kDeviceHeader{0xAA, 0x55};  // IMU → 主機
constexpr std::array<uint8_t, 2> kHostHeader{0x55, 0xAA};    // 主機 → IMU
constexpr size_t kHeaderSize = 6;           // 幀頭 + ID + 長度
constexpr size_t kIdOffset = 2;
constexpr size_t kLengthOffset = 4;
constexpr size_t kCrcSize = 4;
constexpr size_t kFrameOverhead = kHeaderSize + kCrcSize;
// 超過此長度視為偽幀頭（實際最長的 payload 只有數十 bytes）
constexpr size_t kMaxPayloadSize = 512;

// ---- CRC32 ----
// 反射式 CRC32，但初值為 1、無最終 XOR（與一般的 CRC32 不同）
constexpr uint32_t kCrcPolynomial = 0xEDB88320u;
constexpr uint32_t kCrcInit = 1u;

// ---- IMU 輸出幀 ID ----
namespace frame_id {
constexpr uint16_t kAhrs = 0x0002;          // A1：AHRS 數據流
constexpr uint16_t kSystemStatus = 0x00FF;  // S1：系統狀態
constexpr uint16_t kParamValue = 0x7530;    // P1：讀取參數的回應
constexpr uint16_t kParamAck = 0x753D;      // 設定參數 / 存 FLASH 的回應
constexpr uint16_t kCommandAck = 0x0064;    // 一般命令回應（如 CMD 3）
}  // namespace frame_id

constexpr size_t kAhrsPayloadSize = 44;
constexpr size_t kParamValuePayloadSize = 24;
constexpr size_t kAckPayloadSize = 4;

// A1 payload 欄位位置
namespace ahrs_offset {
constexpr size_t kTimer = 0;
constexpr size_t kPitch = 4;
constexpr size_t kRoll = 8;
constexpr size_t kYaw = 12;
constexpr size_t kAcc = 16;   // ax, ay, az
constexpr size_t kGyro = 28;  // gx, gy, gz
constexpr size_t kTemp = 40;
}  // namespace ahrs_offset

// ---- 主機 → IMU 命令 ----
namespace cmd {
constexpr uint16_t kGetSystemStatus = 1;
constexpr uint16_t kGetAhrs = 2;
constexpr uint16_t kSetMode = 3;
constexpr uint16_t kSaveToFlash = 5;
constexpr uint16_t kReadParam = 6;
constexpr uint16_t kReboot = 9;
constexpr uint16_t kSetParam = 14;
}  // namespace cmd

constexpr size_t kCommandPayloadSize = 24;
constexpr size_t kCommandFrameSize = kHeaderSize + kCommandPayloadSize + kCrcSize;

// 命令 payload 與 P1 回應 payload 的欄位位置（兩者格式相同）
namespace command_offset {
constexpr size_t kParam1 = 0;  // float，設定值
constexpr size_t kParam3 = 8;  // uint32，參數索引
}  // namespace command_offset

// CMD 3 的 Param1
constexpr float kModeStream = 1.0f;
constexpr float kModeCommand = 100.0f;

// 回應 payload 中的固定值
constexpr uint16_t kAckResultOk = 1;          // 0x0064 / 存 FLASH 回應的 result
constexpr uint16_t kSetParamAckTag = 0x7534;  // 設定參數回應的第一個 u16

// ---- 參數索引（CMD 6 / CMD 14 的 Param3）----
namespace param {
constexpr uint32_t kBaudrate = 3;      // 需存 FLASH + 重啟才生效
constexpr uint32_t kOrientation = 4;   // 坐標系朝向 101–124
constexpr uint32_t kGyroBiasX = 8;
constexpr uint32_t kGyroBiasY = 9;
constexpr uint32_t kGyroBiasZ = 10;
constexpr uint32_t kOutputRate = 21;   // AHRS 輸出頻率 Hz
constexpr uint32_t kFilter = 31;       // 內部濾波器設定
constexpr uint32_t kOrientationMin = 101;
constexpr uint32_t kOrientationMax = 124;
}  // namespace param

constexpr std::array<int, 5> kSupportedBaudrates{115200, 230400, 460800, 921600, 1500000};

bool is_supported_baudrate(int baudrate);

// ---- 資料結構 ----
struct ImuSample
{
  uint32_t device_timer_us{0};
  float pitch_deg{0.0f};
  float roll_deg{0.0f};
  float yaw_deg{0.0f};
  std::array<float, 3> acc_g{};      // 含重力
  std::array<float, 3> gyro_dps{};
  float temp_c{0.0f};
  std::chrono::steady_clock::time_point host_rx_time{};
};

struct Stats
{
  uint64_t frames_ok{0};
  uint64_t crc_errors{0};
  uint64_t bytes_dropped{0};
  double rate_hz{0.0};  // A1 幀的接收頻率
};

// P1（讀取參數回應）
struct ParamValue
{
  float value{0.0f};
  uint32_t index{0};
};

// 4 bytes 的回應 payload：[u16][u16]
struct Ack
{
  uint16_t first{0};
  uint16_t second{0};
};

// ---- 編解碼 ----
std::array<uint8_t, kCommandFrameSize> encode_command(uint16_t command, float param1, uint32_t param3);

// 長度不符時回傳 nullopt；host_rx_time 不會設定
std::optional<ImuSample> decode_ahrs(const uint8_t * payload, size_t len);
std::optional<ParamValue> decode_param_value(const uint8_t * payload, size_t len);
std::optional<Ack> decode_ack(const uint8_t * payload, size_t len);

// 命令對應的回應幀 ID；沒有回應的命令（重啟）回傳 nullopt
std::optional<uint16_t> response_id_for(uint16_t command);

// ---- little-endian 讀寫 ----
uint16_t read_u16_le(const uint8_t * p);
uint32_t read_u32_le(const uint8_t * p);
float read_f32_le(const uint8_t * p);
void write_u16_le(uint8_t * p, uint16_t v);
void write_u32_le(uint8_t * p, uint32_t v);
void write_f32_le(uint8_t * p, float v);

}  // namespace imu16460

#endif  // IMU16460_CORE__PROTOCOL_HPP_
