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
| 硬體 | `mock_components/GenericSystem`（模擬） |

## 套件一覽

| 套件 | 類型 | 職責 | 依賴（工作區內） |
|---|---|---|---|
| [graphen_interfaces](src/graphen_interfaces/) | 介面定義 | `SwitchMode` action | — |
| [graphen_control_utils](src/graphen_control_utils/) | header-only lib | 共用元件 `StateInterfacePublisher` | — |
| [graphen_controllers](src/graphen_controllers/) | controller plugin | `SwitchGroupController`（含 `LocomotionPlanner`）、`SubGroupController` | graphen_control_utils、graphen_interfaces |
| [graphen_description](src/graphen_description/) | 資源 | URDF / ros2_control 硬體描述 | — |
| [graphen_bringup](src/graphen_bringup/) | 資源 | controller 設定檔與 launch | graphen_controllers、graphen_description |
| [graphen_bt_nodes](src/graphen_bt_nodes/) | BT plugin | 自訂 BT 節點（RateControl、IsOk、SysCount、SwitchMode…） | graphen_interfaces |
| [graphen_bt](src/graphen_bt/) | 執行檔 | `bt_runner`、tree XML、設定檔與 launch | graphen_bt_nodes（執行期載入） |

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
# 五個 controller 都應該是 active
ros2 control list_controllers

# 預設 forward 模式：joint_arm1 以約 0.5 rad/s 往 1.5 移動，joint_gripper1 往 0.05，imu/raw_data 為 1.23
ros2 topic echo --once /switch_group_controller/states

# 用 action 切換到 reverse 模式：result 附上切換當下的 state 快照，之後 joint_arm1 往 -1.5 移動
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: reverse}"

# 不在 modes 中的名稱會被 reject
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: unknown}"

# 也可以用 topic 切換（QoS 必須相容）
ros2 topic pub --once /switch_group_controller/mode std_msgs/msg/String "{data: forward}" \
  --qos-durability transient_local --qos-reliability reliable
ros2 topic echo --once /arm_group_controller/references
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

tree 依目前方向分成 `Forward` / `Reverse` 兩個分支，跨過門檻時走 `TurnToReverse` / `TurnToForward` 換向。用 Groot v1 連線可以看到目前亮著的分支和換向的瞬間；log 會每隔約 4 秒交替印出 `SwitchMode: switched to mode [ forward ]` / `[ reverse ]`，`/switch_group_controller/states` 中的 joint_arm1 會在約 ±1.0 之間往返。門檻在 tree XML 中修改（必須落在兩個模式的 arm 目標 ±1.5 之間）。

### 常見問題

- **`ros2 param` 顯示 "Node not found"**：WSL 下 daemon 探索偏慢，重試一次即可。仍然不行時執行 `ros2 daemon stop && ros2 daemon start`。
- **`bt_runner` 啟動時出現 "Address already in use"**：上一個 `bt_runner` 還在執行，佔用了 Groot 的 port。用 `pgrep -x bt_runner` 確認後結束它。
- **Groot**：v3 只能用 Groot v1 連線（port 1666/1667），Groot2 不支援。
