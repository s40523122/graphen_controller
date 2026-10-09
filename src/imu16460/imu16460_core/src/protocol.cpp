#include "imu16460_core/protocol.hpp"

#include <algorithm>
#include <cstring>

#include "imu16460_core/crc32.hpp"

namespace imu16460 {

bool is_supported_baudrate(int baudrate)
{
  return std::find(kSupportedBaudrates.begin(), kSupportedBaudrates.end(), baudrate) !=
         kSupportedBaudrates.end();
}

uint16_t read_u16_le(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t read_u32_le(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float read_f32_le(const uint8_t * p)
{
  // 先組成 u32 再 memcpy，與主機的 byte order 無關，也不違反 strict aliasing
  const uint32_t bits = read_u32_le(p);
  float v;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

void write_u16_le(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>(v >> 8);
}

void write_u32_le(uint8_t * p, uint32_t v)
{
  for (int i = 0; i < 4; ++i) {
    p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFu);
  }
}

void write_f32_le(uint8_t * p, float v)
{
  uint32_t bits;
  std::memcpy(&bits, &v, sizeof(bits));
  write_u32_le(p, bits);
}

std::array<uint8_t, kCommandFrameSize> encode_command(
  uint16_t command, float param1, uint32_t param3)
{
  std::array<uint8_t, kCommandFrameSize> frame{};
  frame[0] = kHostHeader[0];
  frame[1] = kHostHeader[1];
  write_u16_le(&frame[kIdOffset], command);
  write_u16_le(&frame[kLengthOffset], static_cast<uint16_t>(kCommandPayloadSize));
  // Param2、Param4–6 保留為 0
  write_f32_le(&frame[kHeaderSize + command_offset::kParam1], param1);
  write_u32_le(&frame[kHeaderSize + command_offset::kParam3], param3);
  const size_t crc_offset = kHeaderSize + kCommandPayloadSize;
  write_u32_le(&frame[crc_offset], crc32(frame.data(), crc_offset));
  return frame;
}

std::optional<ImuSample> decode_ahrs(const uint8_t * payload, size_t len)
{
  if (len != kAhrsPayloadSize) {
    return std::nullopt;
  }
  ImuSample s;
  s.device_timer_us = read_u32_le(payload + ahrs_offset::kTimer);
  s.pitch_deg = read_f32_le(payload + ahrs_offset::kPitch);
  s.roll_deg = read_f32_le(payload + ahrs_offset::kRoll);
  s.yaw_deg = read_f32_le(payload + ahrs_offset::kYaw);
  for (size_t i = 0; i < 3; ++i) {
    s.acc_g[i] = read_f32_le(payload + ahrs_offset::kAcc + 4 * i);
    s.gyro_dps[i] = read_f32_le(payload + ahrs_offset::kGyro + 4 * i);
  }
  s.temp_c = read_f32_le(payload + ahrs_offset::kTemp);
  return s;
}

std::optional<ParamValue> decode_param_value(const uint8_t * payload, size_t len)
{
  if (len != kParamValuePayloadSize) {
    return std::nullopt;
  }
  return ParamValue{
    read_f32_le(payload + command_offset::kParam1),
    read_u32_le(payload + command_offset::kParam3)};
}

std::optional<Ack> decode_ack(const uint8_t * payload, size_t len)
{
  if (len != kAckPayloadSize) {
    return std::nullopt;
  }
  return Ack{read_u16_le(payload), read_u16_le(payload + 2)};
}

std::optional<uint16_t> response_id_for(uint16_t command)
{
  switch (command) {
    case cmd::kGetSystemStatus:
      return frame_id::kSystemStatus;
    case cmd::kGetAhrs:
      return frame_id::kAhrs;
    case cmd::kSetMode:
      return frame_id::kCommandAck;
    case cmd::kSaveToFlash:
    case cmd::kSetParam:
      return frame_id::kParamAck;
    case cmd::kReadParam:
      return frame_id::kParamValue;
    default:
      return std::nullopt;
  }
}

}  // namespace imu16460
