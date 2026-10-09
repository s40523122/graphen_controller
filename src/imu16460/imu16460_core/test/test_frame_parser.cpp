#include <gtest/gtest.h>

#include <vector>

#include "imu16460_core/frame_parser.hpp"
#include "test_vectors.hpp"

namespace tv = test_vectors;

namespace {

struct Received
{
  uint16_t id;
  std::vector<uint8_t> payload;
};

class FrameParserTest : public ::testing::Test {
protected:
  imu16460::FrameParser parser{[this](uint16_t id, const uint8_t * p, size_t len) {
      received.push_back({id, std::vector<uint8_t>(p, p + len)});
    }};
  std::vector<Received> received;

  void feed(const tv::Bytes & data) {parser.feed(data.data(), data.size());}
};

}  // namespace

TEST_F(FrameParserTest, WholeFrame)
{
  feed(tv::a1_frame());
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].id, imu16460::frame_id::kAhrs);
  EXPECT_EQ(received[0].payload.size(), imu16460::kAhrsPayloadSize);
  EXPECT_EQ(parser.frames_ok(), 1u);
  EXPECT_EQ(parser.crc_errors(), 0u);
  EXPECT_EQ(parser.bytes_dropped(), 0u);
}

TEST_F(FrameParserTest, ByteByByte)
{
  const auto frame = tv::a1_frame();
  for (const uint8_t b : frame) {
    parser.feed(&b, 1);
  }
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].id, imu16460::frame_id::kAhrs);
}

TEST_F(FrameParserTest, ConcatenatedFramesInArbitraryChunks)
{
  const auto stream = tv::cat({tv::a1_frame(), tv::resp_cmd3(), tv::a1_frame(),
      tv::resp_read21(), tv::a1_frame()});
  // 用不整齊的切段長度餵入
  size_t pos = 0;
  size_t chunk = 1;
  while (pos < stream.size()) {
    const size_t n = std::min(chunk, stream.size() - pos);
    parser.feed(stream.data() + pos, n);
    pos += n;
    chunk = chunk % 17 + 3;
  }
  ASSERT_EQ(received.size(), 5u);
  EXPECT_EQ(received[1].id, imu16460::frame_id::kCommandAck);
  EXPECT_EQ(received[3].id, imu16460::frame_id::kParamValue);
  EXPECT_EQ(parser.bytes_dropped(), 0u);
}

TEST_F(FrameParserTest, LeadingNoise)
{
  feed(tv::cat({tv::hex("00 13 AA 37 55 FF AA"), tv::a1_frame()}));
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(parser.bytes_dropped(), 7u);
}

TEST_F(FrameParserTest, FakeHeaderWithHugeLength)
{
  // 偽 AA 55，長度 0xFFFF 超過上限 → 跳過 2 bytes 重新同步
  feed(tv::cat({tv::hex("AA 55 02 00 FF FF"), tv::a1_frame()}));
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].id, imu16460::frame_id::kAhrs);
}

TEST_F(FrameParserTest, FakeHeaderWithPlausibleLength)
{
  // 偽 AA 55，長度合理但 CRC 不對 → CRC 錯誤計數，之後的真幀仍能解析
  feed(tv::cat({tv::hex("AA 55 02 00 04 00 01 02"), tv::a1_frame(), tv::a1_frame()}));
  EXPECT_EQ(parser.crc_errors(), 1u);
  EXPECT_EQ(received.size(), 2u);
}

TEST_F(FrameParserTest, CorruptedFrameDroppedThenRecovers)
{
  auto bad = tv::a1_frame();
  bad[10] ^= 0x01;
  feed(tv::cat({bad, tv::a1_frame(), tv::resp_save()}));
  EXPECT_EQ(parser.crc_errors(), 1u);
  ASSERT_EQ(received.size(), 2u);
  EXPECT_EQ(received[0].id, imu16460::frame_id::kAhrs);
  EXPECT_EQ(received[1].id, imu16460::frame_id::kParamAck);
}

TEST_F(FrameParserTest, TruncatedFrameThenValid)
{
  // 前一幀被截斷（例如開啟 port 時只收到後半段），之後的完整幀仍能解析
  const auto frame = tv::a1_frame();
  const tv::Bytes tail(frame.begin() + 20, frame.end());
  feed(tv::cat({tail, frame}));
  ASSERT_EQ(received.size(), 1u);
}
