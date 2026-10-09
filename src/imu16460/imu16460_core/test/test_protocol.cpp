#include <gtest/gtest.h>

#include <vector>

#include "imu16460_core/crc32.hpp"
#include "imu16460_core/protocol.hpp"
#include "test_vectors.hpp"

namespace tv = test_vectors;
using imu16460::read_u32_le;

TEST(Crc32, MatchesAllManualVectors)
{
  for (const auto & frame : tv::all_frames()) {
    ASSERT_GT(frame.size(), imu16460::kCrcSize);
    const size_t body = frame.size() - imu16460::kCrcSize;
    EXPECT_EQ(imu16460::crc32(frame.data(), body), read_u32_le(frame.data() + body));
  }
}

TEST(Crc32, IncrementalEqualsOneShot)
{
  const auto frame = tv::a1_frame();
  const size_t body = frame.size() - imu16460::kCrcSize;
  const uint32_t first = imu16460::crc32(frame.data(), 10);
  EXPECT_EQ(imu16460::crc32(frame.data() + 10, body - 10, first),
    imu16460::crc32(frame.data(), body));
}

void expect_encoded(uint16_t command, float p1, uint32_t p3, const tv::Bytes & expected)
{
  const auto frame = imu16460::encode_command(command, p1, p3);
  EXPECT_EQ(std::vector<uint8_t>(frame.begin(), frame.end()), expected) << "command " << command;
}

TEST(EncodeCommand, MatchesManualVectors)
{
  namespace cmd = imu16460::cmd;
  namespace param = imu16460::param;
  expect_encoded(cmd::kGetSystemStatus, 0.0f, 0, tv::cmd1_status());
  expect_encoded(cmd::kSetMode, imu16460::kModeStream, 0, tv::cmd3_stream());
  expect_encoded(cmd::kSaveToFlash, 0.0f, 0, tv::cmd5_save());
  expect_encoded(cmd::kReadParam, 0.0f, param::kOutputRate, tv::cmd6_read21());
  expect_encoded(cmd::kReadParam, 0.0f, param::kOrientation, tv::cmd6_read4());
  expect_encoded(cmd::kSetParam, 115200.0f, param::kBaudrate, tv::cmd14_baud115200());
  expect_encoded(cmd::kSetParam, 100.0f, param::kOutputRate, tv::cmd14_rate100());
  expect_encoded(cmd::kSetParam, 115.0f, param::kOrientation, tv::cmd14_orient115());
}

TEST(DecodeAhrs, ManualVector)
{
  const auto frame = tv::a1_frame();
  const auto s = imu16460::decode_ahrs(
    frame.data() + imu16460::kHeaderSize, imu16460::kAhrsPayloadSize);
  ASSERT_TRUE(s.has_value());
  constexpr float kTol = 1e-3f;
  EXPECT_EQ(s->device_timer_us, 85363053u);
  EXPECT_NEAR(s->pitch_deg, 3.59f, kTol);
  EXPECT_NEAR(s->roll_deg, -0.53f, kTol);
  EXPECT_NEAR(s->yaw_deg, 356.12f, kTol);
  EXPECT_NEAR(s->acc_g[0], 0.063f, kTol);
  EXPECT_NEAR(s->acc_g[1], 0.009f, kTol);
  EXPECT_NEAR(s->acc_g[2], -1.007f, kTol);
  EXPECT_NEAR(s->gyro_dps[0], -0.045f, kTol);
  EXPECT_NEAR(s->gyro_dps[1], -0.005f, kTol);
  EXPECT_NEAR(s->gyro_dps[2], -0.025f, kTol);
  EXPECT_NEAR(s->temp_c, 29.83f, kTol);
}

TEST(DecodeAhrs, RejectsWrongLength)
{
  const auto frame = tv::a1_frame();
  EXPECT_FALSE(imu16460::decode_ahrs(frame.data() + imu16460::kHeaderSize, 43).has_value());
}

TEST(DecodeResponses, ParamValueAndAck)
{
  const auto read = tv::resp_read21();
  const auto value = imu16460::decode_param_value(
    read.data() + imu16460::kHeaderSize, imu16460::kParamValuePayloadSize);
  ASSERT_TRUE(value.has_value());
  EXPECT_FLOAT_EQ(value->value, 50.0f);
  EXPECT_EQ(value->index, imu16460::param::kOutputRate);

  const auto set = tv::resp_set_orient();
  const auto ack = imu16460::decode_ack(set.data() + imu16460::kHeaderSize, 4);
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->first, imu16460::kSetParamAckTag);
  EXPECT_EQ(ack->second, imu16460::param::kOrientation);
}
