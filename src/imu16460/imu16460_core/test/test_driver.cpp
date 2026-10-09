#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "imu16460_core/driver.hpp"
#include "test_vectors.hpp"

namespace tv = test_vectors;
using namespace std::chrono_literals;

namespace {

// 以 pty 模擬 IMU：master 端扮演裝置，Driver 開啟 slave 端。
// 背景 thread 讀取主機送來的命令幀（固定 34 bytes），依 ID 回應手冊中的回應幀；
// stream 為 true 時以約 1 kHz 持續送出 A1 幀。
class FakeImu {
public:
  FakeImu()
  {
    char name[256] = {};
    if (openpty(&master_, &slave_, name, nullptr, nullptr) == 0) {
      slave_name_ = name;
    }
  }

  ~FakeImu()
  {
    stop();
    if (slave_ >= 0) {
      ::close(slave_);
    }
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  bool ok() const {return !slave_name_.empty();}
  const std::string & slave_name() const {return slave_name_;}

  void start()
  {
    running_ = true;
    thread_ = std::thread([this] {loop();});
  }

  void stop()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  std::atomic<bool> stream{false};
  std::atomic<bool> respond{true};

  std::vector<uint16_t> received_commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }

private:
  void write_bytes(const tv::Bytes & b)
  {
    size_t off = 0;
    while (off < b.size()) {
      const ssize_t n = ::write(master_, b.data() + off, b.size() - off);
      if (n <= 0) {
        return;
      }
      off += static_cast<size_t>(n);
    }
  }

  void handle_command(const tv::Bytes & frame)
  {
    const uint16_t id = imu16460::read_u16_le(frame.data() + imu16460::kIdOffset);
    const uint32_t index = imu16460::read_u32_le(frame.data() + imu16460::kHeaderSize + 8);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands_.push_back(id);
    }
    if (!respond) {
      return;
    }
    switch (id) {
      case imu16460::cmd::kSetMode:
        write_bytes(tv::resp_cmd3());
        break;
      case imu16460::cmd::kSaveToFlash:
        write_bytes(tv::resp_save());
        break;
      case imu16460::cmd::kReadParam:
        write_bytes(index == imu16460::param::kOutputRate ? tv::resp_read21() : tv::resp_read4());
        break;
      case imu16460::cmd::kSetParam:
        if (index == imu16460::param::kOrientation) {
          write_bytes(tv::resp_set_orient());
        }
        break;
      default:
        break;
    }
  }

  void loop()
  {
    tv::Bytes pending;
    auto next_sample = std::chrono::steady_clock::now();
    while (running_) {
      pollfd pfd{master_, POLLIN, 0};
      if (::poll(&pfd, 1, 1) > 0 && (pfd.revents & POLLIN)) {
        uint8_t buf[256];
        const ssize_t n = ::read(master_, buf, sizeof(buf));
        if (n > 0) {
          pending.insert(pending.end(), buf, buf + n);
        }
      }
      // 命令幀長度固定，依序切出處理
      while (pending.size() >= imu16460::kCommandFrameSize) {
        handle_command(tv::Bytes(pending.begin(), pending.begin() + imu16460::kCommandFrameSize));
        pending.erase(pending.begin(), pending.begin() + imu16460::kCommandFrameSize);
      }
      const auto now = std::chrono::steady_clock::now();
      if (stream && now >= next_sample) {
        write_bytes(tv::a1_frame());
        next_sample = now + 1ms;
      }
    }
  }

  int master_{-1};
  int slave_{-1};
  std::string slave_name_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::vector<uint16_t> commands_;
};

class DriverTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    ASSERT_TRUE(imu.ok());
    imu.start();
    ASSERT_TRUE(driver.open(imu.slave_name(), 921600)) << driver.last_error();
    ASSERT_TRUE(driver.start());
  }

  // 等到條件成立或逾時
  template<typename Pred>
  bool wait_for(Pred pred, std::chrono::milliseconds timeout = 1000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return pred();
  }

  FakeImu imu;
  imu16460::Driver driver;
};

}  // namespace

TEST_F(DriverTest, ReceivesStream)
{
  imu.stream = true;
  imu16460::ImuSample s;
  ASSERT_TRUE(wait_for([&] {return driver.get_latest(s);}));
  EXPECT_EQ(s.device_timer_us, 85363053u);
  EXPECT_NEAR(s.acc_g[2], -1.007f, 1e-3f);

  ASSERT_TRUE(wait_for([&] {return driver.get_stats().frames_ok > 10;}));
  EXPECT_EQ(driver.get_stats().crc_errors, 0u);
  const auto age = driver.ms_since_last_sample();
  ASSERT_TRUE(age.has_value());
  EXPECT_LT(*age, 100.0);
}

TEST_F(DriverTest, AverageSinceLast)
{
  imu16460::ImuSample s;
  EXPECT_FALSE(driver.get_average_since_last(s));
  imu.stream = true;
  ASSERT_TRUE(wait_for([&] {return driver.get_stats().frames_ok > 5;}));
  ASSERT_TRUE(driver.get_average_since_last(s));
  // 每筆都相同，平均值等於單筆
  EXPECT_NEAR(s.gyro_dps[0], -0.045f, 1e-3f);
  EXPECT_NEAR(s.yaw_deg, 356.12f, 1e-3f);
}

TEST_F(DriverTest, SetStreamModeAck)
{
  EXPECT_TRUE(driver.set_stream_mode(true));
}

TEST_F(DriverTest, CommandInterleavedWithStream)
{
  imu.stream = true;
  ASSERT_TRUE(wait_for([&] {return driver.get_stats().frames_ok > 5;}));
  EXPECT_TRUE(driver.save_to_flash());
  EXPECT_TRUE(driver.set_param(imu16460::param::kOrientation, 115.0f));
}

TEST_F(DriverTest, ReadParamResumesStream)
{
  const auto rate = driver.read_param(imu16460::param::kOutputRate);
  ASSERT_TRUE(rate.has_value());
  EXPECT_FLOAT_EQ(*rate, 50.0f);
  const auto orient = driver.read_param(imu16460::param::kOrientation);
  ASSERT_TRUE(orient.has_value());
  EXPECT_FLOAT_EQ(*orient, 115.0f);

  // 每次讀取後都要送出 CMD 3 恢復數據流
  ASSERT_TRUE(wait_for([&] {return imu.received_commands().size() >= 4;}));
  const std::vector<uint16_t> expected{
    imu16460::cmd::kReadParam, imu16460::cmd::kSetMode,
    imu16460::cmd::kReadParam, imu16460::cmd::kSetMode};
  EXPECT_EQ(imu.received_commands(), expected);
}

TEST_F(DriverTest, CommandTimeout)
{
  imu.respond = false;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(driver.set_stream_mode(true, 100ms));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GE(elapsed, 100ms);
  EXPECT_LT(elapsed, 500ms);
}

TEST_F(DriverTest, StopIsFast)
{
  imu.stream = true;
  ASSERT_TRUE(wait_for([&] {return driver.get_stats().frames_ok > 5;}));
  const auto start = std::chrono::steady_clock::now();
  driver.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  EXPECT_FALSE(driver.is_running());
}

TEST(DriverStandalone, CommandRequiresRunning)
{
  imu16460::Driver driver;
  EXPECT_FALSE(driver.send_command(imu16460::cmd::kSetMode, 1.0f, 0).has_value());
  EXPECT_FALSE(driver.open("/dev/nonexistent_imu16460", 921600));
  EXPECT_FALSE(driver.last_error().empty());
}
