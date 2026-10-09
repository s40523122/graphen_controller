// imu16460_cli：實機除錯工具。
// 唯讀指令（monitor / scan-baud / status / get）可直接執行；
// 會改變裝置設定的指令必須帶 --yes 才會實際送出。

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "imu16460_core/driver.hpp"
#include "imu16460_core/protocol.hpp"

namespace {

constexpr const char * kDefaultPort = "/dev/ttyTHS1";
constexpr int kDefaultBaudrate = 921600;
constexpr int kDefaultMonitorPeriodMs = 100;
constexpr int kDefaultScanDurationMs = 500;
// monitor 每次刷新輸出的行數，ANSI 游標上移時使用
constexpr int kMonitorLines = 7;

std::atomic<bool> g_stop{false};

void on_signal(int)
{
  g_stop = true;
}

struct Options
{
  std::string port{kDefaultPort};
  int baudrate{kDefaultBaudrate};
  bool yes{false};
  int period_ms{kDefaultMonitorPeriodMs};
  int duration_ms{kDefaultScanDurationMs};
  std::vector<std::string> args;  // 指令與位置參數
};

void print_usage()
{
  std::printf(
    "Usage: imu16460_cli [--port DEV] [--baud N] <command> [args] [--yes]\n"
    "\n"
    "Read-only commands:\n"
    "  monitor [--period-ms N]   live view of the latest sample, frame rate and CRC error rate\n"
    "  scan-baud [--duration-ms N]  try every supported baudrate and count valid frames\n"
    "  status                    request one system status frame (raw payload)\n"
    "  get <index>               read a parameter (stream is resumed afterwards)\n"
    "\n"
    "Commands that change the device (require --yes):\n"
    "  set-rate <hz>             AHRS output rate (param 21)\n"
    "  set-baud <baud>           baudrate (param 3, needs save + reboot)\n"
    "  set-orient <101-124>      coordinate orientation (param 4)\n"
    "  save                      save parameters to FLASH\n"
    "  reboot                    software reboot\n"
    "\n"
    "Defaults: --port %s --baud %d\n", kDefaultPort, kDefaultBaudrate);
}

bool parse_int(const std::string & s, int & out)
{
  try {
    size_t pos = 0;
    out = std::stoi(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

bool parse_float(const std::string & s, float & out)
{
  try {
    size_t pos = 0;
    out = std::stof(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

bool parse_options(int argc, char ** argv, Options & opt)
{
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has_value = i + 1 < argc;
    if (a == "--port" && has_value) {
      opt.port = argv[++i];
    } else if (a == "--baud" && has_value) {
      if (!parse_int(argv[++i], opt.baudrate)) {
        return false;
      }
    } else if (a == "--period-ms" && has_value) {
      if (!parse_int(argv[++i], opt.period_ms) || opt.period_ms <= 0) {
        return false;
      }
    } else if (a == "--duration-ms" && has_value) {
      if (!parse_int(argv[++i], opt.duration_ms) || opt.duration_ms <= 0) {
        return false;
      }
    } else if (a == "--yes") {
      opt.yes = true;
    } else if (a == "-h" || a == "--help") {
      return false;
    } else {
      opt.args.push_back(a);
    }
  }
  return !opt.args.empty();
}

bool open_and_start(imu16460::Driver & driver, const Options & opt)
{
  if (!driver.open(opt.port, opt.baudrate) || !driver.start()) {
    std::fprintf(stderr, "Failed to open %s @ %d: %s\n",
      opt.port.c_str(), opt.baudrate, driver.last_error().c_str());
    return false;
  }
  return true;
}

int cmd_monitor(const Options & opt)
{
  imu16460::Driver driver;
  if (!open_and_start(driver, opt)) {
    return 1;
  }
  bool first = true;
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(opt.period_ms));
    imu16460::ImuSample s;
    const bool has_sample = driver.get_latest(s);
    const auto stats = driver.get_stats();
    const auto age = driver.ms_since_last_sample();
    const uint64_t total = stats.frames_ok + stats.crc_errors;
    const double crc_rate = total ? 100.0 * static_cast<double>(stats.crc_errors) / total : 0.0;

    // 游標移回上次輸出的開頭，原地刷新
    if (!first) {
      std::printf("\033[%dA", kMonitorLines);
    }
    first = false;
    std::printf("\033[K%s @ %d   rate %8.1f Hz   last sample %s\n",
      opt.port.c_str(), opt.baudrate, stats.rate_hz,
      age ? (std::to_string(static_cast<int>(*age)) + " ms ago").c_str() : "none");
    std::printf("\033[Kframes %llu   crc errors %llu (%.3f %%)   dropped bytes %llu\n",
      static_cast<unsigned long long>(stats.frames_ok),
      static_cast<unsigned long long>(stats.crc_errors), crc_rate,
      static_cast<unsigned long long>(stats.bytes_dropped));
    if (has_sample) {
      std::printf("\033[Ktimer %10u us   temp %6.2f C\n", s.device_timer_us, s.temp_c);
      std::printf("\033[Kpitch %9.3f   roll %9.3f   yaw %9.3f  deg\n",
        s.pitch_deg, s.roll_deg, s.yaw_deg);
      std::printf("\033[Kacc   %9.4f  %9.4f  %9.4f  g\n", s.acc_g[0], s.acc_g[1], s.acc_g[2]);
      std::printf("\033[Kgyro  %9.4f  %9.4f  %9.4f  deg/s\n",
        s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2]);
    } else {
      for (int i = 0; i < 4; ++i) {
        std::printf("\033[K%s\n", i == 0 ? "(no sample)" : "");
      }
    }
    std::printf("\033[K(Ctrl+C to quit)\n");
    std::fflush(stdout);
  }
  return 0;
}

int cmd_scan_baud(const Options & opt)
{
  int best = 0;
  uint64_t best_frames = 0;
  for (const int baud : imu16460::kSupportedBaudrates) {
    if (g_stop) {
      break;
    }
    imu16460::Driver driver;
    Options o = opt;
    o.baudrate = baud;
    if (!open_and_start(driver, o)) {
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(opt.duration_ms));
    const auto stats = driver.get_stats();
    driver.close();
    std::printf("%8d: %llu valid frames, %llu crc errors\n", baud,
      static_cast<unsigned long long>(stats.frames_ok),
      static_cast<unsigned long long>(stats.crc_errors));
    if (stats.frames_ok > best_frames) {
      best_frames = stats.frames_ok;
      best = baud;
    }
  }
  if (best_frames > 0) {
    std::printf("Detected baudrate: %d\n", best);
    return 0;
  }
  std::printf("No valid frames at any baudrate\n");
  return 1;
}

int cmd_status(const Options & opt)
{
  imu16460::Driver driver;
  if (!open_and_start(driver, opt)) {
    return 1;
  }
  const auto payload = driver.get_system_status();
  if (!payload) {
    std::fprintf(stderr, "No status response: %s\n", driver.last_error().c_str());
    return 1;
  }
  std::printf("System status payload (%zu bytes):", payload->size());
  for (size_t i = 0; i < payload->size(); ++i) {
    std::printf("%s%02X", (i % 16 == 0) ? "\n  " : " ", (*payload)[i]);
  }
  std::printf("\n");
  return 0;
}

int cmd_get(const Options & opt)
{
  int index = 0;
  if (opt.args.size() < 2 || !parse_int(opt.args[1], index) || index < 0) {
    std::fprintf(stderr, "Usage: get <index>\n");
    return 2;
  }
  imu16460::Driver driver;
  if (!open_and_start(driver, opt)) {
    return 1;
  }
  const auto value = driver.read_param(static_cast<uint32_t>(index));
  if (!value) {
    std::fprintf(stderr, "Failed to read param %d: %s\n", index, driver.last_error().c_str());
    return 1;
  }
  std::printf("param %d = %g\n", index, *value);
  return 0;
}

// 會改變裝置設定的指令：沒有 --yes 時只說明會送出什麼
bool confirm(const Options & opt, const std::string & what)
{
  if (opt.yes) {
    return true;
  }
  std::fprintf(stderr, "Refusing to %s without --yes (this changes the device).\n", what.c_str());
  return false;
}

int cmd_set_param(const Options & opt, uint32_t index, const std::string & label)
{
  float value = 0.0f;
  if (opt.args.size() < 2 || !parse_float(opt.args[1], value)) {
    std::fprintf(stderr, "Usage: %s <value>\n", opt.args[0].c_str());
    return 2;
  }
  if (index == imu16460::param::kBaudrate &&
    !imu16460::is_supported_baudrate(static_cast<int>(value)))
  {
    std::fprintf(stderr, "Unsupported baudrate %g\n", value);
    return 2;
  }
  if (index == imu16460::param::kOrientation &&
    (value < imu16460::param::kOrientationMin || value > imu16460::param::kOrientationMax))
  {
    std::fprintf(stderr, "Orientation must be in [%u, %u]\n",
      imu16460::param::kOrientationMin, imu16460::param::kOrientationMax);
    return 2;
  }
  if (!confirm(opt, "set " + label + " to " + opt.args[1])) {
    return 2;
  }
  imu16460::Driver driver;
  if (!open_and_start(driver, opt)) {
    return 1;
  }
  if (!driver.set_param(index, value)) {
    std::fprintf(stderr, "Failed to set %s: %s\n", label.c_str(), driver.last_error().c_str());
    return 1;
  }
  std::printf("Set %s = %g (run 'save --yes' to persist", label.c_str(), value);
  std::printf(index == imu16460::param::kBaudrate ? ", then 'reboot --yes')\n" : ")\n");
  return 0;
}

int cmd_save(const Options & opt)
{
  if (!confirm(opt, "save parameters to FLASH")) {
    return 2;
  }
  imu16460::Driver driver;
  if (!open_and_start(driver, opt)) {
    return 1;
  }
  if (!driver.save_to_flash()) {
    std::fprintf(stderr, "Save failed: %s\n", driver.last_error().c_str());
    return 1;
  }
  std::printf("Saved to FLASH\n");
  return 0;
}

int cmd_reboot(const Options & opt)
{
  if (!confirm(opt, "reboot the device")) {
    return 2;
  }
  imu16460::Driver driver;
  if (!open_and_start(driver, opt)) {
    return 1;
  }
  if (!driver.reboot()) {
    std::fprintf(stderr, "Reboot failed: %s\n", driver.last_error().c_str());
    return 1;
  }
  std::printf("Reboot command sent\n");
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parse_options(argc, argv, opt)) {
    print_usage();
    return 2;
  }
  if (!imu16460::is_supported_baudrate(opt.baudrate)) {
    std::fprintf(stderr, "Unsupported baudrate %d\n", opt.baudrate);
    return 2;
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  const std::string & command = opt.args[0];
  if (command == "monitor") {
    return cmd_monitor(opt);
  } else if (command == "scan-baud") {
    return cmd_scan_baud(opt);
  } else if (command == "status") {
    return cmd_status(opt);
  } else if (command == "get") {
    return cmd_get(opt);
  } else if (command == "set-rate") {
    return cmd_set_param(opt, imu16460::param::kOutputRate, "output rate");
  } else if (command == "set-baud") {
    return cmd_set_param(opt, imu16460::param::kBaudrate, "baudrate");
  } else if (command == "set-orient") {
    return cmd_set_param(opt, imu16460::param::kOrientation, "orientation");
  } else if (command == "save") {
    return cmd_save(opt);
  } else if (command == "reboot") {
    return cmd_reboot(opt);
  }
  std::fprintf(stderr, "Unknown command '%s'\n", command.c_str());
  print_usage();
  return 2;
}
