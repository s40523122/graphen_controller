# ti5_hardware

Ti5 T170 關節實機的 ros2_control `SystemInterface` plugin（`ti5_hardware/Ti5Hardware`），透過 `can_driver` 以 SocketCAN 與鈦虎 C1 關節模組通訊。

## 指令模式

模式由 URDF 的 command interface 決定，所有 joint 必須一致：

| bringup `hardware` | command interface | 馬達模式 | 子群組 controller |
|---|---|---|---|
| `real` | `position` | `0x1E` 位置模式，指令以 `max_command_velocity` 限速 | `PositionController` |
| `real_pd` | `effort` | `0x1C` 電流模式，力矩依 KT 與減速比換算 | `PdController`（200 Hz） |

state interface：`position`、`velocity`、`current`（mA）、`error_code`；`real_pd` 另有 `effort`（由回饋電流估計）。

configure 時從馬達讀取減速比（速度回饋是電機端，需要換算），`real_pd` 另外讀取 KT；不寫入任何馬達參數。

effort 模式的流程、單位換算與待驗證項目見 [docs/TI5_CAN_TORQUE.md](../../docs/TI5_CAN_TORQUE.md)。

## 硬體參數

| 參數 | 說明 |
|---|---|
| `can_config_path` | CAN 通道與馬達對應（`graphen_bringup/config/can_driver_config.json`） |
| `hw_read_timeout_ms` | CAN 讀取逾時（real 預設 20、real_pd 預設 2，由 bringup launch 決定） |
| `initial_positions_file` | 位置模式開機時走向的初始姿態 |
| `max_command_velocity` | 位置模式送往馬達的指令最大變化速度（rad/s，0 為不限速） |
| `max_missed_cycles` | 只用於 effort 模式：容許連續回饋缺漏的週期數，超過就退回位置保持 |

關節位置限位取自 URDF `<limit lower/upper>`（超限命令 clamp），effort 模式的力矩上限取自 `<limit effort>`。
