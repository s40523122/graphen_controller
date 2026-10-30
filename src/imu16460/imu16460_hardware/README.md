# imu16460_hardware

FSS-IMU16460-C 的 ros2_control（Humble）`SensorInterface` plugin：`imu16460_hardware/Imu16460SensorInterface`。
內部使用 [imu16460_core](../imu16460_core/) 的 `Driver`。

## 行為

- IMU 由 Driver 的背景 thread 依裝置自己的頻率（約 1000 Hz）接收，與 controller_manager 的 `update_rate` 無關。
- `read()` 每個週期只取最新一筆（`try_lock`，拿不到就沿用上一筆），換算後寫入 state；不做 serial I/O、不阻塞、不配置記憶體。
- 換算：角速度 °/s → rad/s；加速度 g × 9.80665 → m/s²；姿態 Euler → quaternion。
- 生命週期：`on_configure` 開 port；`on_activate` 啟動 thread，並確認 `activate_timeout_ms` 內收到資料，否則回傳 ERROR；`on_deactivate` 停止 thread；`on_cleanup` 關閉 port。
- 超過 `data_timeout_ms` 沒有新樣本時以 `RCLCPP_WARN_THROTTLE` 警告（每秒一次）；`error_on_timeout: true` 時 `read()` 回傳 ERROR。
- **不會送出任何命令、不修改 IMU 設定**。裝置設定請用 `imu16460_cli`。

## State interface

sensor 必須有以下 10 個 state interface（`imu_sensor_broadcaster` 需要），`temperature` 為選用（broadcaster 會忽略）：

`orientation.{x,y,z,w}`、`angular_velocity.{x,y,z}`、`linear_acceleration.{x,y,z}`、`temperature`

## 參數（URDF `<hardware>`）

| 參數 | 預設 | 說明 |
|---|---|---|
| `port` | `/dev/ttyTHS1` | serial port |
| `baudrate` | `921600` | 115200 / 230400 / 460800 / 921600 / 1500000 |
| `data_timeout_ms` | `100` | 超過此時間沒有新樣本就警告，必須 > 0 |
| `error_on_timeout` | `false` | 逾時時 `read()` 是否回傳 ERROR（Humble 對 read 錯誤的處理有限，預設不回傳） |
| `activate_timeout_ms` | `1000` | activate 時等待第一筆資料的時間，必須 > 0 |
| `axis_transform` | `none` | `none` 或 `frd_to_flu`（y、z 取負） |
| `euler_order` | `zyx` | `zyx`（q = Rz·Ry·Rx）或 `xyz`（q = Rx·Ry·Rz） |

## 使用方式

[urdf/imu16460.ros2_control.xacro](urdf/imu16460.ros2_control.xacro) 提供 macro，參數同上表（另有 `name`、`sensor_name`）：

```xml
<xacro:include filename="$(find imu16460_hardware)/urdf/imu16460.ros2_control.xacro"/>
<xacro:imu16460_ros2_control name="imu16460" sensor_name="imu_sensor"
                             port="/dev/ttyTHS1" baudrate="921600"/>
```

在 graphen 中由 `ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro` 以 `use_mock_imu:=false` 引入：

```bash
ros2 launch graphen_bringup bringup.launch.py use_mock_imu:=false imu_port:=/dev/ttyTHS1   # 只對 hardware:=mock / real 有效
ros2 topic echo /imu_sensor_broadcaster/imu
```

## 待實機驗證

見 [docs/IMU16460.md §3](../../../docs/IMU16460.md#3-待實機驗證)：坐標系方向（`axis_transform` 預設值）、Euler 順序與正方向、時間戳（broadcaster 用 controller_manager 時間）。程式中以 `// TODO(verify-on-hw):` 標記。

坐標系方向、陀螺正方向與 Euler 定義可以用 `ros2 run graphen_bringup imu_orientation_check` 在機器人上驗證，見 [IMU16460.md §3.1](../../../docs/IMU16460.md#31-方向性驗證工具imu_orientation_check)。
