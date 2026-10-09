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

## 已知問題：Jetson Orin 上約 1.2% 掉幀

在 Orin（JetPack R36.4.4，`/dev/ttyTHS1` @ 921600、1000 Hz）實測：

- `imu16460_cli monitor` 的頻率約 980 Hz，但 **CRC 錯誤率約 1.2–1.6%**，未達 < 0.1% 的驗收標準。
- 原始資料中，壞幀是「少了一段 bytes」或「多出 2 / 4 個 `0x00`」，裝置 timer 偶爾會跳 3000 µs（連掉 2 幀）。
- UART 驅動的錯誤計數（`TIOCGICOUNT`）：`frame`、`parity`、`brk` 都是 0，**`overrun` 次數約等於壞幀數**。

**原因**：主機端 UART 接收 FIFO overrun，與接線和 IMU 本身無關。

1. device tree 中 `serial@3100000`（ttyTHS1）沒有設定 `dmas`，以中斷搬 FIFO（PIO 模式）。
2. CPU idle 狀態 `c7` 的退出延遲是 5000 µs。921600 baud 下 5 ms 約進來 460 bytes，遠超過 FIFO 容量。CPU 在 c7 時中斷來不及處理，就會 overrun。

**驗證**：讓所有核心保持忙碌（不會進入 c7）時，5 秒內的 overrun 是 **0**；恢復閒置後又回到約 1.2%。

**目前的處理**：先接受。本套件的使用方式是「取最新一筆」，1 kHz 下偶爾少一幀影響不大，parser 會丟棄壞幀並重新同步，不會輸出錯誤資料。

**可行的修正（尚未採用）**：

| 方式 | 說明 |
|---|---|
| Driver 使用 PM QoS | 執行期間開啟 `/dev/cpu_dma_latency` 寫入延遲上限，只在 IMU 執行時禁止深度 idle；需要 udev 規則開放寫入權限（預設是 root 才能寫） |
| 系統層關閉 c7 | 開機時停用 `cpuidle/state1`；不需改程式，但會一直增加閒置功耗 |
| 為 UART 開啟 DMA | 以 device tree overlay 為 `serial@3100000` 加上 DMA；可以從根本解決，但需要修改 DT 並重新開機 |
| 降低資料量 | 改成 460800 / 500 Hz（需要修改 IMU 設定並存 FLASH，要經過同意） |

## 待實機驗證

見 [docs/IMU16460.md §3](../../../docs/IMU16460.md#3-待實機驗證)。與本套件相關的是 S1 系統狀態的實際長度。
