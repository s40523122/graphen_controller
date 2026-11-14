# graphen_ws

graphen 的 ROS 2 控制工作區，分成兩層：

- **控制層（ros2_control）**：上層的 `SwitchGroupController` 依目前的模式設定目標位置，由內部的 `LocomotionPlanner` 以固定頻率漸進產生目標，再寫給下層各群組的子群組 controller（chainable 的 `PositionController`；MuJoCo effort 模式為 `PdController`），最後由下層寫到硬體 command interface。
- **行為層（BehaviorTree.CPP v3）**：`bt_runner` 載入 XML 行為樹，並以 plugin 的形式載入自訂節點。

兩層透過 `SwitchMode` action（切換模式）和 `~/states` topic（joint 狀態）連動。詳細設計見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 環境

| 項目 | 版本 |
|---|---|
| ROS 2 | Humble |
| ros2_control / controller_interface | 2.53 |
| behaviortree_cpp_v3 | 3.8.7 |
| 機器人 | Ti5 T170（`ti5_170a_graphen`，32 joint）；預設 `mock_components/GenericSystem`，`hardware:=real`（位置模式）/ `real_pd`（電流模式 + 主機端 PD）切換為 CAN 實機（`ti5_hardware`） |
| 靈巧手 | 預設不啟用，`use_hands:=true` 加入左右手（各 13 joint） |
| IMU | 預設 mock，`use_mock_imu:=false` 切換為實機 FSS-IMU16460-C |
| 模擬 | `hardware:=mujoco` / `mujoco_pd` 時本體與 IMU 改由 MuJoCo 模擬（`graphen_mujoco`）；需要另外安裝 MuJoCo 3.11.0，見[MuJoCo 模擬與 RL policy](#mujoco-模擬與-rl-policy) |

## 套件一覽

| 套件 | 類型 | 職責 | 依賴（工作區內） |
|---|---|---|---|
| [graphen_interfaces](src/graphen_interfaces/) | 介面定義 | `SwitchMode` action | — |
| [graphen_control_utils](src/graphen_control_utils/) | header-only lib | 共用元件 `StateInterfacePublisher` | — |
| [graphen_motion](src/graphen_motion/) | C++ lib（不依賴 rclcpp） | BVH 解析、BVH → URDF 1-DOF joint 轉換（`BvhRetargeter`）、`bvh_retarget_cli` | — |
| [graphen_controllers](src/graphen_controllers/) | controller plugin | `SwitchGroupController`（含 `LocomotionPlanner` 與 `TargetMotion` / `BvhMotion`）、子群組 `PositionController` / `PdController` | graphen_control_utils、graphen_interfaces、graphen_motion |
| [graphen_bringup](src/graphen_bringup/) | 資源 | controller 設定檔、CAN 設定、BVH 動作檔（`motions/`）與 launch | graphen_controllers、ti5_170a_graphen |
| [graphen_bt_nodes](src/graphen_bt_nodes/) | BT plugin | 自訂 BT 節點（RateControl、IsOk、SysCount、SwitchMode…） | graphen_interfaces |
| [graphen_bt](src/graphen_bt/) | 執行檔 | `bt_runner`、tree XML、設定檔與 launch | graphen_bt_nodes（執行期載入） |
| [imu16460_core](src/imu16460/imu16460_core/) | C++ lib（純 CMake，無 ROS） | IMU 協議、parser、serial、`Driver`、`imu16460_cli` | — |
| [imu16460_hardware](src/imu16460/imu16460_hardware/) | hardware plugin | ros2_control `SensorInterface`，xacro macro | imu16460_core |
| [ti5_170a_graphen](src/robot_description/ti5_170a_graphen/) | 資源（外部） | T170 URDF、ros2_control xacro、joint 分組 SRDF | imu16460_hardware、ti5_hardware、dexterous_hand_description |
| [ti5_hardware](src/ti5_hardware/) | hardware plugin（外部） | T170 關節 `SystemInterface`（`ti5_hardware/Ti5Hardware`） | can_driver |
| [can_driver](src/can_driver/) | C++ lib（外部） | SocketCAN 與 Ti5 馬達指令 | — |
| [dexterous_hand_description](src/dexterous_hands/dexterous_hand_description/) | 資源（外部） | 靈巧手 ros2_control xacro macro | dexterous_hand_hardware（執行期） |
| [dexterous_hand_hardware](src/dexterous_hands/dexterous_hand_hardware/) | hardware plugin（外部） | 靈巧手 `SystemInterface`（RS485） | — |
| [graphen_mujoco](src/graphen_mujoco/) | hardware plugin | MuJoCo 模擬的 T170 本體 + IMU（`graphen_mujoco/MujocoSystem`）、MJCF 匯出與主機端 viewer（`tools/`） | — |
| [t170a_description](src/robot_description/t170a/) | 資源（外部） | robolab 訓練用的 T170 A0 模型（URDF、MJCF、mesh），MuJoCo 驗證控制架構用，不代表實機 | — |
| [ti5_t170_description](src/robot_description/ti5_t170_description/) | 資源（外部） | T170-V2.1-A0 模型（URDF xacro、MJCF、mesh），robolab `Ti5T170-Flat`（tc_new_025）的訓練模型 | — |
| [graphen_choreographer](src/graphen_choreographer/) | locomotion plugin（不依賴 rclcpp） | 編舞（多軌 keyframe 的組合動作）：`ChoreographerAlgorithm`、腳本解析、`choreo_check` | graphen_locomotion |
| [graphen_rl_policy](src/graphen_rl_policy/) | locomotion plugin | RL policy 共用基類 `PolicyRunner`（ONNX Runtime）與 policy plugin：`T170VelocityPolicy`（FullBodyT70）、`Ti5FlatPolicy`（robolab） | graphen_locomotion |

## 快速開始

首次使用時，先從範本複製本機設定檔（這三個檔案不進版控，各機器可自行修改，pull 時不會被覆蓋）：

```bash
cd ~/graphen_ws
cp src/graphen_bringup/config/controllers.yaml.example src/graphen_bringup/config/controllers.yaml
cp src/graphen_bringup/config/real_initial_positions.yaml.example src/graphen_bringup/config/real_initial_positions.yaml
cp src/graphen_bt/config/bt_runner.yaml.example src/graphen_bt/config/bt_runner.yaml
```

```bash
cd ~/graphen_ws
colcon build --symlink-install                      # 只改一個套件時加 --packages-select <pkg>
source install/setup.bash

ros2 launch graphen_bringup bringup.launch.py       # 控制層（預設 hardware:=mock，見下方「硬體設定」）
ros2 launch graphen_bt bt.launch.py                 # 行為層（另開終端機）
```


### 硬體設定

bringup 以 `hardware:=` 選擇硬體（預設 `mock`）。controller 設定分成兩部分，launch 依序疊加：

| 檔案（`graphen_bringup/config/`） | 內容 |
|---|---|
| `controllers.yaml`（本機檔，範本 `.example`） | 與硬體無關：模式、switch 接線、BVH；本機自訂（例如子群組的 `output_enabled`） |
| `hardware/pd_common.yaml` | （`mujoco_pd` / `real_pd`）`update_rate` 200、`PdController` 子群組與 switch 的速度前饋 / kp / kd 接線 |
| `hardware/<hardware>.yaml` | `update_rate`、6 個子群組的 type（`PositionController` / `PdController`）與介面接線；`real_pd` 為 PD 預設增益與力矩上限 |
| `hardware/mujoco_pd_<mujoco_robot>.yaml` | （`mujoco_pd`）依模型的 PD 預設增益與力矩上限 |
| `imu/mock.yaml` 或 `imu/imu16460.yaml` | （`mock` / `real` / `real_pd`）依 `use_mock_imu` 選擇 |

| `hardware` | 本體 | IMU |
|---|---|---|
| `mock`（預設） | `mock_components/GenericSystem` | 依 `use_mock_imu` |
| `mujoco` / `mujoco_pd` | MuJoCo（見下方 MuJoCo 段落） | MuJoCo 內建 |
| `real` | `ti5_hardware`（CAN 實機位置模式，會對馬達送出指令） | 依 `use_mock_imu` |
| `real_pd` | `ti5_hardware`（CAN 實機電流模式 + `PdController`，**尚未在實機驗證**，見 [TI5_CAN_TORQUE.md](docs/TI5_CAN_TORQUE.md)） | 依 `use_mock_imu` |

launch 會在 log 印出這次載入的參數檔清單（`controller parameter files: ...`）。

## 手動測試

### 控制層

```bash
# joint_state_broadcaster、sensor_forwarder、6 個子群組與 switch_group_controller 都應該是 active
ros2 control list_controllers

# 啟動後不進入任何模式，所有 joint 保持初始位置（default_mode: ""）。
# states 含 32 軸的 position / velocity 與 mock IMU（imu_sensor/*，初始為靜止水平：orientation.w = 1、
# linear_acceleration.z = 9.81）
ros2 topic echo --once /switch_group_controller/states

# 寫入 mock IMU 的模擬值，10 個值依序為 orientation.xyzw、angular_velocity.xyz、linear_acceleration.xyz
# （用 -t 多送幾次：--once 可能在 subscriber 完成 discovery 前就送出而遺失）
ros2 topic pub -t 3 -w 1 /sensor_forwarder/commands std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 1,  0, 0, 0.5,  0, 0, 9.81]}"

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

# 外部演算法 plugin 的範例（sine 模式，graphen_locomotion/SineWaveAlgorithm）：NECK_Y 以進入時的位置為中心
# 擺動（±0.2 rad、0.25 Hz），controller 的 log 每秒印出演算法收到的量測角度 / 速度與 IMU
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: sine}"

# 機身狀態估測（ImuLeakyEstimator）：寫入 x 方向 1 m/s² 的加速度，sine 的 log 中 base lin_vel x 會趨近
# a × leak_time_constant = 2.0 m/s；改回 0 後以時間常數 2 s 衰減
ros2 topic pub -t 3 -w 1 /sensor_forwarder/commands std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 1,  0, 0, 0,  1.0, 0, 9.81]}"

# 速度指令：發布期間 log 中 command active 為 true；停止發布 0.5 s（command_timeout）後歸零
ros2 topic pub -r 10 -w 1 /switch_group_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.2}}"
```

> walk 是開環播放，沒有平衡控制；實機上只能在吊掛等安全條件下使用。

### BVH 轉換檢查

`bvh_retarget_cli` 把整段 BVH 轉成 CSV（time + 各 joint 角度，rad），並在 stderr 印出分組、每組最大殘差、各 joint 的範圍 / 限位 / 最大速度，用來檢查新的 BVH 或設定 `<mode>.max_steps`：

```bash
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_motion bvh_retarget_cli \
  $(ros2 pkg prefix graphen_bringup)/share/graphen_bringup/motions/t170wk_walk1_subject1.bvh /tmp/t170.urdf > /tmp/walk.csv
```

### 編舞（組合動作）

多個部位各自依自己的時間動作、時間可以重疊，寫成一份腳本（`graphen_bringup/motions/*.choreo.yaml`）、對應一個 `type: plugin` 的模式。腳本格式、檢查項目與限制見 [ARCHITECTURE §2.12](docs/ARCHITECTURE.md#212-編舞graphen_choreographer)。

範例 `arm_demo`（右臂 0 ~ 3 s、左臂 1 ~ 5 s）與 `arm_demo_return`（回到下垂姿態）的模式設定在 `controllers.yaml.example`，要用時複製到本機的 `controllers.yaml`（`modes` 也要加上名稱）。

上機前先離線檢查腳本（範圍、限位、峰值速度、建議的 `max_steps`；`--csv` 可輸出取樣後的整份編舞）：

```bash
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_choreographer choreo_check \
  $(ros2 pkg prefix graphen_bringup)/share/graphen_bringup/motions/arm_demo.choreo.yaml \
  --urdf /tmp/t170.urdf --csv /tmp/arm_demo.csv
```

播放並等它播完（`wait_until_finished`；feedback 為進入模式後經過的秒數）。`arm_demo` 的第一個 keyframe 在 `t: 0`，必須從下垂姿態開始，否則不執行並 abort：

```bash
timeout 15 ros2 action send_goal --feedback /switch_group_controller/switch_mode \
  graphen_interfaces/action/SwitchMode "{mode: arm_demo, wait_until_finished: true}"
# 回到下垂姿態（從任何位置都可以）。on_same_mode：0 = ignore（預設）、1 = restart、2 = restart_if_finished
timeout 15 ros2 action send_goal /switch_group_controller/switch_mode \
  graphen_interfaces/action/SwitchMode "{mode: arm_demo_return, wait_until_finished: true, on_same_mode: 2}"
```

- 等待中被其他模式取代（action 或 `~/mode`）時 goal 會 abort（`Preempted by mode [ X ]`）；Ctrl-C（cancel）只是不再等待，**動作會繼續執行**，要停下來請切到 `stop`。
- 已經在同一個模式時：`on_same_mode: 0` 不重新開始；`1` 一律重播；`2` 播完才重播（播放中重複送出的指令被吃掉）。
- 範例的 `arm_demo` 設了 `interruptible: false`：播放中切到其他模式或重播都會被拒絕（`success: false`），只有 `stop`（hold，預設 `preempt`）可以切入。
- 播放途中切到另一個編舞模式時，新模式會從當下的速度平順地接上（`entry_velocity`），不會頓一下。
- 用 BT 依序播放兩段並重複：

  ```bash
  ros2 launch graphen_bt bt.launch.py \
    tree_file:=$(ros2 pkg prefix graphen_bt)/share/graphen_bt/trees/choreo_demo.xml
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
ros2 launch graphen_bringup bringup.launch.py hardware:=real
```

CAN 通道與馬達對應在 [can_driver_config.json](src/graphen_bringup/config/can_driver_config.json)，實機初始位置在 [real_initial_positions.yaml](src/graphen_bringup/config/real_initial_positions.yaml)：開機時有列出的 joint 會以 `max_command_velocity`（預設 0.5 rad/s）限速走到該值，沒列的保持目前位置；之後要等 BT 或 action 下命令才會動。加上 `use_hands:=true` 時靈巧手也會改用實機（`left_hand_port`、`right_hand_port`、`hand_baudrate`）。

`hardware:=real_pd` 改以電流模式輸出力矩，PD 由 `PdController` 在主機端以 200 Hz 計算（與訓練相同）。開機時先以位置模式保持目前姿態，等 controller 寫入力矩後才切換；回饋中斷或指令異常時退回位置保持。**尚未在實機驗證**，流程、單位換算與待驗證項目見 [TI5_CAN_TORQUE.md](docs/TI5_CAN_TORQUE.md)。

### MuJoCo 模擬與 RL policy

本體和 IMU 改由 MuJoCo 模擬（物理 200 Hz），controller 設定不變。

**安裝 MuJoCo 3.11.0**（每台機器一次，依架構選 `x86_64` 或 `aarch64`；`graphen_mujoco` 建置時尋找 `MUJOCO_DIR` → `~/.local/mujoco-3.11.0` → `/opt/mujoco-3.11.0`，版本不符會 configure 失敗）：

```bash
ARCH=$(uname -m)   # x86_64 或 aarch64
curl -fLO https://github.com/google-deepmind/mujoco/releases/download/3.11.0/mujoco-3.11.0-linux-${ARCH}.tar.gz
mkdir -p ~/.local/mujoco-3.11.0
tar xzf mujoco-3.11.0-linux-${ARCH}.tar.gz -C ~/.local/mujoco-3.11.0 --strip-components=1
colcon build --symlink-install --packages-select graphen_mujoco   # 裝在其他位置時加 --cmake-args -DMUJOCO_DIR=<路徑>
```

控制方式由 `hardware` 決定，模型由 `mujoco_robot` 決定：

| `hardware` | 致動器 | 子群組 | 預設增益 |
|---|---|---|---|
| `mujoco` | MJCF 的位置伺服 | `PositionController` | MJCF 內的值，所有模式相同 |
| `mujoco_pd` | 力矩馬達 | `PdController`（`update_rate` 200） | `hardware/mujoco_pd_<mujoco_robot>.yaml`；模式有設定 `kp` / `kd` 時（RL policy）改用模式的值 |

| `mujoco_robot` | 模型 | 可用的 hardware | 說明 |
|---|---|---|---|
| `fb70`（預設） | `graphen_mujoco/models/t170_{free,fixed}.xml`（FullBodyT70 匯出，wtz 的訓練模型） | `mujoco`、`mujoco_pd` | mesh 指向 Orin 上的 `/devtest/FullBodyT70/...`，只能在 Orin 上載入 |
| `t170a` | `t170a_description/mjcf/t170a.xml`（robolab，tc_flat 的訓練模型） | `mujoco_pd` | mesh 用相對路徑，WSL 也能跑；沒有頸部（三軸為虛擬關節） |
| `ti5_t170` | `ti5_t170_description/mjcf/ti5_t170.xml`（T170-V2.1-A0 T-pose，tc_new_025 的訓練模型） | `mujoco_pd` | mesh 用相對路徑，WSL 也能跑；頸部是真正的關節；初始基座高度 1.07 m |

policy 只在自己的訓練模型上才站得穩（例如 tc_new_025 在 t170a 上會倒），模型與 policy 要對應。各模型的浮動基座初始高度不同（訓練初始姿態下腳底貼地），由 launch 依 `mujoco_robot` 帶入 MujocoSystem 的 `initial_base_height`。

`mujoco_fixed_base:=true`（預設）時基座固定在空中，相當於吊掛：fb70 改用 `t170_fixed.xml`，t170a / ti5_t170 每步把基座重設回初始位姿。`false` 時為自由站立。

```bash
# fb70（Orin 容器內）
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco mujoco_fixed_base:=false
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco_pd mujoco_fixed_base:=false
# t170a / ti5_t170（任何機器）
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco_pd mujoco_robot:=t170a mujoco_fixed_base:=true
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco_pd mujoco_robot:=ti5_t170 mujoco_fixed_base:=true

# 主機上另開視窗（只顯示，不需要 ROS；視窗內按 End 重置模擬）。模型要與 launch 一致
bash src/graphen_mujoco/tools/mujoco_viewer.sh                                  # fb70 自由站立
bash src/graphen_mujoco/tools/mujoco_viewer.sh --model ../models/t170_fixed.xml # fb70 吊掛
bash src/graphen_mujoco/tools/mujoco_viewer.sh --model "$PWD/src/robot_description/t170a/mjcf/t170a.xml"
bash src/graphen_mujoco/tools/mujoco_viewer.sh --model "$PWD/src/robot_description/ti5_t170_description/mjcf/ti5_t170.xml"  # sim_rl 預設

# 切到 policy：進入時會先以 entry_speed 走到 policy 的預設姿態，再開始推論
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: rl_walk}"

# 速度指令（vx、vy m/s，wz rad/s）；停止發布 0.5 s 後回到原地站立
ros2 topic pub -r 20 /switch_group_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

- `mujoco_pd` 下，所有子群組都啟動之前物理是暫停的，controller 啟動前不會倒下。
- **自由站立測 RL policy 要用 `sim_rl.launch.py`**：PD 只保持姿態、沒有平衡能力，`bringup.launch.py` 自由站立啟動後幾秒內就會倒下（姿態約 90°），之後才切到 RL 模式已經來不及。`sim_rl.launch.py` 會在最後疊加一份情境參數（`default_mode` 設為 `mode`、該模式的 `entry_speed` 設為 0），讓 policy 從模擬開始就接手；本機 `controllers.yaml` 不用改，但 `mode` 必須已經定義在其中：

  ```bash
  ros2 launch graphen_bringup sim_rl.launch.py                        # ti5_t170 + rl_flat（預設）
  ros2 launch graphen_bringup sim_rl.launch.py mujoco_robot:=t170a    # tc_flat_policy*（t170a 訓練）
  ros2 launch graphen_bringup sim_rl.launch.py mode:=rl_walk mujoco_robot:=fb70   # Orin
  ```
- `controllers.yaml.example` 有兩個 RL 模式（本機的 `controllers.yaml` 要使用時，從範本複製並加到 `modes`）：
  - `rl_walk`：`T170VelocityPolicy` + `policy/wtz_policy.onnx`，用 fb70 訓練。實測（fb70、position、自由站立）：能穩定站立；vx=0.5 時實際約 0.23 m/s，這是 policy 本身的表現（直接跑 `play_mujoco.py` 也約 0.25 m/s）。
  - `rl_flat`：`Ti5FlatPolicy` + `policy/tc_flat_policy.onnx`，用 t170a 訓練。實測（t170a、`mujoco_pd`、自由站立、WSL）：能穩定站立（傾斜約 2.5°）；vx=0.5 時能邁步、不會倒下。關節順序與 obs 組法是推斷的，**仍待訓練端確認**。
- ti5_t170 的 MJCF 由 CAD 匯出的 T-pose 檔產生（補上 timestep、armature、ctrlrange、IMU、碰撞設定與 keyframe）；原檔或 `param.csv` 更新後重新產生：
  `python3 src/robot_description/ti5_t170_description/scripts/make_mjcf.py --base-height 1.07`
- fb70 的 MJCF 由 FullBodyT70 的訓練場景匯出，重新訓練或修改場景後要重新產生（需要主機的 `~/learnWalk_env`）：
  `~/learnWalk_env/bin/python src/graphen_mujoco/tools/export_mjcf.py`
- **onnx 與設定綁定**：每個 RL 模式以 `model_sha256` 綁定 onnx，不符時 controller configure 失敗。
  - **同一套訓練設定重新訓練**：放入新的 onnx，更新 `model_path`、`model_sha256`（`sha256sum <onnx>`）與改變的常數，不需要修改程式。
  - **obs / action 組法不同**：要新增 policy plugin（見 [ARCHITECTURE §5](docs/ARCHITECTURE.md#5-擴充指引)）。
- **ONNX Runtime**：`graphen_rl_policy` 需要 `third_party/onnxruntime`（1.31.0，不進版控）。Orin 使用 aarch64 版（取自 FullBodyT70/cpp_interface），x86_64 開發機使用 GitHub release 的 `onnxruntime-linux-x64-1.31.0.tgz`。
- **RL policy 只在 MuJoCo 驗證過**，不要在實機上使用（見 [ARCHITECTURE §2.11](docs/ARCHITECTURE.md#211-rl-policy-plugingraphen_rl_policy)）。實機的對應是 `hardware:=real_pd`（尚未驗證）。

### 實機 IMU（FSS-IMU16460-C）

預設 `use_mock_imu:=true`，行為與上面相同（mock `imu_sensor/*` + `sensor_forwarder`，介面名稱與實機相同）。接上實機時（Orin）：

```bash
imu16460_cli monitor                                # 先確認資料：約 1000 Hz、CRC 錯誤率 < 0.1%
ros2 launch graphen_bringup bringup.launch.py use_mock_imu:=false imu_port:=/dev/ttyTHS1 imu_baudrate:=921600   # 本體 mock；本體也用實機時加 hardware:=real
ros2 topic echo /imu_sensor_broadcaster/imu         # 角速度 rad/s、加速度 m/s²
```

實機時 `sensor_forwarder` 不啟動，改為 `imu_sensor_broadcaster`；`switch_group_controller` 的設定不需要改動（mock 與實機的 interface 名稱相同）。
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
