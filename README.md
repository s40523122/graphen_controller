# graphen_ws

graphen 的 ROS 2 控制工作區，分成兩層：

- **控制層（ros2_control）**：上層的 `SwitchGroupController` 依目前的模式，把目標值寫給下層各群組的 `SubGroupController`（chainable），再由下層寫到硬體 command interface。
- **行為層（BehaviorTree.CPP v3）**：`bt_runner` 載入 XML 行為樹，並以 plugin 的形式載入自訂節點。

目前兩層獨立運作，尚未連動。詳細設計見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

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
| [graphen_control_utils](src/graphen_control_utils/) | header-only lib | 共用元件 `StateInterfacePublisher` | — |
| [graphen_controllers](src/graphen_controllers/) | controller plugin | `SwitchGroupController`、`SubGroupController` | graphen_control_utils |
| [graphen_description](src/graphen_description/) | 資源 | URDF / ros2_control 硬體描述 | — |
| [graphen_bringup](src/graphen_bringup/) | 資源 | controller 設定檔與 launch | graphen_controllers、graphen_description |
| [graphen_bt_nodes](src/graphen_bt_nodes/) | BT plugin | 自訂 BT 節點（RateControl、IsOk、SysCount…） | — |
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

# 預設 mode 1：joint_arm1 約 0.1、joint_gripper1 約 0.2，imu/raw_data 為 1.23
ros2 topic echo --once /switch_group_controller/states

# 切換到 mode 2：位置應該變成 0.3 / 0.4
ros2 topic pub --once /switch_group_controller/mode std_msgs/msg/Int32 "{data: 2}" \
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

### 常見問題

- **`ros2 param` 顯示 "Node not found"**：WSL 下 daemon 探索偏慢，重試一次即可。仍然不行時執行 `ros2 daemon stop && ros2 daemon start`。
- **`bt_runner` 啟動時出現 "Address already in use"**：上一個 `bt_runner` 還在執行，佔用了 Groot 的 port。用 `pgrep -x bt_runner` 確認後結束它。
- **Groot**：v3 只能用 Groot v1 連線（port 1666/1667），Groot2 不支援。
