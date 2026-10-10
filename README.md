# graphen_ws

graphen 的 ROS 2 控制工作區，分成兩層：

- **控制層（ros2_control）**：上層的 `SwitchGroupController` 依目前的模式設定目標位置，由內部的 `LocomotionPlanner` 以固定頻率漸進產生目標，再寫給下層各群組的 `SubGroupController`（chainable），最後由下層寫到硬體 command interface。
- **行為層（BehaviorTree.CPP v3）**：`bt_runner` 載入 XML 行為樹，並以 plugin 的形式載入自訂節點。

兩層透過 `SwitchMode` action（切換模式）和 `~/states` topic（joint 狀態）連動。詳細設計見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 環境

| 項目 | 版本 |
|---|---|
| ROS 2 | Humble |
| ros2_control / controller_interface | 2.53 |
| behaviortree_cpp_v3 | 3.8.7 |
| 機器人 | Ti5 T170（`ti5_170a_graphen`，32 joint）；預設 `mock_components/GenericSystem`，`use_mock_hardware:=false` 切換為 CAN 實機（`ti5_hardware`） |
| 靈巧手 | 預設不啟用，`use_hands:=true` 加入左右手（各 13 joint） |
| IMU | 預設 mock，`use_mock_imu:=false` 切換為實機 FSS-IMU16460-C |

## 套件一覽

| 套件 | 類型 | 職責 | 依賴（工作區內） |
|---|---|---|---|
| [graphen_interfaces](src/graphen_interfaces/) | 介面定義 | `SwitchMode` action | — |
| [graphen_control_utils](src/graphen_control_utils/) | header-only lib | 共用元件 `StateInterfacePublisher` | — |
| [graphen_motion](src/graphen_motion/) | C++ lib（不依賴 rclcpp） | BVH 解析、BVH → URDF 1-DOF joint 轉換（`BvhRetargeter`）、`bvh_retarget_cli` | — |
| [graphen_controllers](src/graphen_controllers/) | controller plugin | `SwitchGroupController`（含 `LocomotionPlanner` 與 `TargetMotion` / `BvhMotion`）、`SubGroupController` | graphen_control_utils、graphen_interfaces、graphen_motion |
| [graphen_bringup](src/graphen_bringup/) | 資源 | controller 設定檔、CAN 設定、BVH 動作檔（`motions/`）與 launch | graphen_controllers、ti5_170a_graphen |
| [graphen_bt_nodes](src/graphen_bt_nodes/) | BT plugin | 自訂 BT 節點（RateControl、IsOk、SysCount、SwitchMode…） | graphen_interfaces |
| [graphen_bt](src/graphen_bt/) | 執行檔 | `bt_runner`、tree XML、設定檔與 launch | graphen_bt_nodes（執行期載入） |
| [imu16460_core](src/imu16460/imu16460_core/) | C++ lib（純 CMake，無 ROS） | IMU 協議、parser、serial、`Driver`、`imu16460_cli` | — |
| [imu16460_hardware](src/imu16460/imu16460_hardware/) | hardware plugin | ros2_control `SensorInterface`，xacro macro | imu16460_core |
| [ti5_170a_graphen](src/ti5_170a_graphen/) | 資源（外部） | T170 URDF、ros2_control xacro、joint 分組 SRDF | imu16460_hardware、ti5_hardware、dexterous_hand_description |
| [ti5_hardware](src/ti5_hardware/) | hardware plugin（外部） | T170 關節 `SystemInterface`（`ti5_hardware/Ti5Hardware`） | can_driver |
| [can_driver](src/can_driver/) | C++ lib（外部） | SocketCAN 與 Ti5 馬達指令 | — |
| [dexterous_hand_description](src/dexterous_hands/dexterous_hand_description/) | 資源（外部） | 靈巧手 ros2_control xacro macro | dexterous_hand_hardware（執行期） |
| [dexterous_hand_hardware](src/dexterous_hands/dexterous_hand_hardware/) | hardware plugin（外部） | 靈巧手 `SystemInterface`（RS485） | — |

## 快速開始

```bash
cd ~/graphen_ws
colcon build --symlink-install                      # 只改一個套件時加 --packages-select <pkg>
source install/setup.bash

ros2 launch graphen_bringup bringup.launch.py       # 控制層
ros2 launch graphen_bt bt.launch.py                 # 行為層（另開終端機）
```

## 手動測試

### 控制層

```bash
# joint_state_broadcaster、sensor_forwarder、6 個子群組與 switch_group_controller 都應該是 active
ros2 control list_controllers

# 啟動後不進入任何模式，所有 joint 保持初始位置（default_mode: ""），imu/raw_data 為 1.23
ros2 topic echo --once /switch_group_controller/states

# 用 action 切換到 reverse 模式：result 附上切換當下的 state 快照，之後兩個手腕（L/R_WRIST_P）往 -1.5 移動，
# 其他 30 軸保持
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: reverse}"

# 不在 modes 中的名稱會被 reject（topic 收到時則忽略並 WARN）
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: unknown}"

# 也可以用 topic 切換（QoS 必須相容）
ros2 topic pub --once /switch_group_controller/mode std_msgs/msg/String "{data: forward}" \
  --qos-durability transient_local --qos-reliability reliable
ros2 topic echo --once /left_arm_group_controller/references

# 播放 BVH 步態（walk 模式）：先以約 0.5 rad/s 走向第 0 幀，再即時播放 32 軸，到結尾後接回起點循環。
# planner 依檔案頻率（24 fps）送出原始幀，幀間插補由子群組依 sample_period 進行（約晚一幀）。
# 已在 walk 時再送 walk 不會從頭播放；要重播請先切到其他模式
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: walk}"
```

> walk 是開環播放，沒有平衡控制；實機上只能在吊掛等安全條件下使用。

### BVH 轉換檢查

`bvh_retarget_cli` 把整段 BVH 轉成 CSV（time + 各 joint 角度，rad），並在 stderr 印出分組、每組最大殘差、各 joint 的範圍 / 限位 / 最大速度，用來檢查新的 BVH 或設定 `<mode>.max_steps`：

```bash
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_motion bvh_retarget_cli \
  $(ros2 pkg prefix graphen_bringup)/share/graphen_bringup/motions/t170wk_walk1_subject1.bvh /tmp/t170.urdf > /tmp/walk.csv
```

### 行為層

正常情況下，log 會持續印出 `[Normal]` 約 10 Hz、`[RateControl]` 約 1 Hz。

```bash
# 切到異常分支：[1000Hz] 以 gp_rate（約 100 Hz）執行，gp_sys_count 累加到約 3000 後停止
ros2 topic pub --once /is_ok std_msgs/msg/Bool "{data: false}"
ros2 param get /bt_runner gp_sys_count

# 執行中調整頻率
ros2 param set /bt_runner gp_rate 50.0

# 切回正常分支，計數歸零
ros2 param set /bt_runner gp_is_ok true
```

想縮短等待時間時，可以複製一份 tree，把 `CheckSysCount target` 改小，再用
`ros2 launch graphen_bt bt.launch.py tree_file:=/path/to/test_tree.xml` 執行。

### 兩層連動（joint 往返）

先啟動控制層，再用 `joint_reciprocate.xml` 啟動行為層：

```bash
ros2 launch graphen_bt bt.launch.py \
  tree_file:=$(ros2 pkg prefix graphen_bt)/share/graphen_bt/trees/joint_reciprocate.xml
```

tree 依目前方向分成 `Forward` / `Reverse` 兩個分支，跨過門檻時走 `TurnToReverse` / `TurnToForward` 換向。用 Groot v1 連線可以看到目前亮著的分支和換向的瞬間；log 會每隔約 4 秒交替印出 `SwitchMode: switched to mode [ forward ]` / `[ reverse ]`，`/switch_group_controller/states` 中的 L_WRIST_P 會在約 ±1.0 之間往返。門檻在 tree XML 中修改（必須落在兩個模式的目標 ±1.5 之間）。

模式可在執行中用 rqt_reconfigure 或 `ros2 param set` 切換（walk 優先，兩個都 false 時切到 `stop` 停在目前位置）：

```bash
ros2 param set /bt_runner gp_enable_walk true          # 切到 walk（BVH 步行）
ros2 param set /bt_runner gp_enable_walk false         # 回到往返（從 forward 開始）
ros2 param set /bt_runner gp_enable_reciprocate false  # 兩個都 false：停止
```

### 靈巧手（mock）

```bash
ros2 launch graphen_bringup bringup.launch.py use_hands:=true
ros2 control list_hardware_components               # 多出 left_hand_hardware、right_hand_hardware
```

目前沒有 controller 驅動手部 joint，只會發布狀態。

### T170 實機

> 會對馬達送出指令。未確認機器人狀態與安全措施前不要執行。

```bash
ros2 launch graphen_bringup bringup.launch.py use_mock_hardware:=false
```

CAN 通道與馬達對應在 [can_driver_config.json](src/graphen_bringup/config/can_driver_config.json)，實機初始位置在 [real_initial_positions.yaml](src/graphen_bringup/config/real_initial_positions.yaml)：開機時有列出的 joint 會以 `max_command_velocity`（預設 0.5 rad/s）限速走到該值，沒列的保持目前位置；之後要等 BT 或 action 下命令才會動。加上 `use_hands:=true` 時靈巧手也會改用實機（`left_hand_port`、`right_hand_port`、`hand_baudrate`）。

### 實機 IMU（FSS-IMU16460-C）

預設 `use_mock_imu:=true`，行為與上面相同（mock `imu/raw_data` + `sensor_forwarder`）。接上實機時（Orin）：

```bash
imu16460_cli monitor                                # 先確認資料：約 1000 Hz、CRC 錯誤率 < 0.1%
ros2 launch graphen_bringup bringup.launch.py use_mock_imu:=false imu_port:=/dev/ttyTHS1 imu_baudrate:=921600
ros2 topic echo /imu_sensor_broadcaster/imu         # 角速度 rad/s、加速度 m/s²
```

實機時 `sensor_forwarder` 不啟動，改為 `imu_sensor_broadcaster`，`switch_group_controller` 的 sensor 改讀 `imu_sensor/angular_velocity.z`。
協議、待實機驗證項目與 CLI 用法見 [docs/IMU16460.md](docs/IMU16460.md) 和 [imu16460_core/README.md](src/imu16460/imu16460_core/README.md)。

> 會改變 IMU 設定的 CLI 指令（`set-*`、`save`、`reboot`）必須帶 `--yes`，未經確認不要對實機送出。

### 自動化測試

```bash
colcon test --packages-select imu16460_core && colcon test-result --verbose
```

目前只有 `imu16460_core` 有單元測試（GoogleTest，含以 pty 模擬裝置的 Driver 測試）。

### 常見問題

- **`ros2 param` 顯示 "Node not found"**：WSL 下 daemon 探索偏慢，重試一次即可。仍然不行時執行 `ros2 daemon stop && ros2 daemon start`。
- **`bt_runner` 啟動時出現 "Address already in use"**：上一個 `bt_runner` 還在執行，佔用了 Groot 的 port。用 `pgrep -x bt_runner` 確認後結束它。
- **Groot**：v3 只能用 Groot v1 連線（port 1666/1667），Groot2 不支援。
