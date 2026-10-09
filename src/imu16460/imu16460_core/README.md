# imu16460_core

FSS-IMU16460-C 的純 C++17 驅動，**不依賴 ROS**（`<build_type>cmake</build_type>`），可以在非 ROS 環境單獨建置。
協議與測試向量見 [docs/IMU16460.md](../../../docs/IMU16460.md)。

## 元件

| 檔案 | 內容 |
|---|---|
| `protocol.hpp` | 所有常數（幀頭、ID、命令、參數索引）、`ImuSample` / `Stats`、`encode_command()`、`decode_ahrs()` 等 |
| `crc32.hpp` | 協議用的 CRC32（初值 1、無最終 XOR） |
| `frame_parser.hpp` | 串流式解析器：任意切段、雜訊、偽幀頭重新同步、CRC 錯誤計數；不在每幀配置記憶體 |
| `serial_port.hpp` | termios raw 模式的 RAII 包裝，`poll()` 讀取帶逾時 |
| `driver.hpp` | 背景讀取 thread、最新樣本、命令 / 回應 |

### Driver

- 背景 thread 依裝置自己的頻率（約 1000 Hz）持續接收、解析，更新最新樣本。
- `get_latest()`：以 `try_lock` 複製最新樣本，拿不到 lock 時回傳 false，不會阻塞（可在 RT thread 使用）。
- `get_average_since_last()`：上次呼叫以來的平均；姿態角取最新一筆（yaw 有 0/360 跳變）。
- 命令 API：`send_command()` 會等待對應 ID 的回應。包裝成 `set_stream_mode()`、`read_param()`（結束後自動恢復數據流）、`set_param()`、`save_to_flash()`、`reboot()`、`get_system_status()`。需要先 `start()`，且只能在 non-RT thread 呼叫。
- `stop()` 在一個 poll 週期（20 ms）內結束。
- 錯誤以回傳值 / `std::optional` 表示，不丟例外；原因見 `last_error()`。

其他套件以 `find_package(imu16460_core)` + `imu16460_core::imu16460_core` 使用。

## imu16460_cli

`source install/setup.bash` 後可直接執行（裝在 `install/imu16460_core/bin`，純 CMake 套件不支援 `ros2 run`）。

```bash
imu16460_cli monitor                    # 原地刷新：最新樣本、幀率、CRC 錯誤率
imu16460_cli scan-baud                  # 依序嘗試所有支援的鮑率
imu16460_cli status                     # 系統狀態（原始 payload）
imu16460_cli get 21                     # 讀參數（例：輸出頻率）
imu16460_cli --port /dev/ttyUSB0 --baud 460800 monitor
```

以下指令會改變裝置設定，**必須帶 `--yes`** 才會送出：

```bash
imu16460_cli set-rate 500 --yes         # 參數 21，之後要 save 才會保存
imu16460_cli set-baud 460800 --yes      # 參數 3，需 save + reboot 才生效
imu16460_cli set-orient 101 --yes       # 參數 4（101–124）
imu16460_cli save --yes
imu16460_cli reboot --yes
```

## 測試

GoogleTest（工作區中只有這個套件有自動化測試）：

```bash
colcon build --symlink-install --packages-select imu16460_core
colcon test --packages-select imu16460_core && colcon test-result --verbose
```

涵蓋：CRC 對所有手冊向量、`encode_command` 逐 byte 比對、A1 解析、parser（整幀、逐 byte、多幀任意切段、雜訊、偽幀頭、CRC 錯誤後恢復）、以 `openpty` 模擬裝置測 Driver（數據流、命令 / 回應交錯、`read_param` 自動恢復數據流、逾時、`stop()` < 100 ms）。

## 待實機驗證

見 [docs/IMU16460.md §3](../../../docs/IMU16460.md#3-待實機驗證)。與本套件相關的是 S1 系統狀態的實際長度。
