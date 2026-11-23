# graphen_ws

graphen 的 ROS 2 控制工作區，分成兩層：

- **控制層（ros2_control）**：上層的 `SwitchGroupController` 播放目前的動作（系統模式、編舞、BVH 或 policy），由內部的 `LocomotionPlanner` 以固定頻率漸進產生目標，再寫給下層各群組的子群組 controller（chainable 的 `PositionController`；MuJoCo effort 模式為 `PdController`），最後由下層寫到硬體 command interface。
- **行為層（BehaviorTree.CPP v3）**：`bt_runner` 載入 XML 行為樹，並以 plugin 的形式載入自訂節點。

兩層透過播放類 action（`SwitchMode` 切換系統模式、`PlayChoreo` 播放編舞、`PlayBvh` 播放 BVH、`RunPolicy` 執行 policy）和 `~/states` topic（joint 狀態）連動。詳細設計見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

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
| [graphen_interfaces](src/graphen_interfaces/) | 介面定義 | 播放類 action（`SwitchMode` / `PlayChoreo` / `PlayBvh` / `RunPolicy`）與共用的 `Playback*` msg、舞曲庫與預載的 service | — |
| [graphen_control_utils](src/graphen_control_utils/) | header-only lib | 共用元件 `StateInterfacePublisher` | — |
| [graphen_motion](src/graphen_motion/) | C++ lib（不依賴 rclcpp） | BVH 解析、BVH → URDF 1-DOF joint 轉換（`BvhRetargeter`）、`bvh_retarget_cli` | — |
| [graphen_controllers](src/graphen_controllers/) | controller plugin | `SwitchGroupController`（含 `LocomotionPlanner`、切換規則 `PlaybackArbiter`、內容來源 `ChoreoSource` / `BvhSource` / `PolicySource`）、子群組 `PositionController` / `PdController` | graphen_control_utils、graphen_interfaces、graphen_motion |
| [graphen_bringup](src/graphen_bringup/) | 資源 | controller 設定檔、CAN 設定、BVH 與範例編舞（`motions/`）、launch、工具腳本（`choreo_publish`、`playback_scenarios`） | graphen_controllers、ti5_170a_graphen |
| [graphen_bt_nodes](src/graphen_bt_nodes/) | BT plugin | 自訂 BT 節點（RateControl、IsOk、SysCount、播放節點 SwitchMode / PlayChoreo / PlayBvh / RunPolicy、速度命令 JointVelocity、視覺跟隨用的 GetPixelError / SendOnChange…） | graphen_interfaces |
| [graphen_bt](src/graphen_bt/) | 執行檔 | `bt_runner`、tree XML、設定檔與 launch | graphen_bt_nodes（執行期載入） |
| [graphen_vision](src/graphen_vision/) | 節點（Python） | `face_tracker`：相機影像（ROS topic，或直接讀 webcam / RealSense 彩色串流）→ YuNet 人臉偵測與鎖定 → `/face_tracker/target`（給 face_track.xml） | — （執行期選用：realsense2_camera） |
| [imu16460_core](src/imu16460/imu16460_core/) | C++ lib（純 CMake，無 ROS） | IMU 協議、parser、serial、`Driver`、`imu16460_cli` | — |
| [imu16460_hardware](src/imu16460/imu16460_hardware/) | hardware plugin | ros2_control `SensorInterface`，xacro macro | imu16460_core |
| [ti5_170a_graphen](src/robot_description/ti5_170a_graphen/) | 資源（外部） | T170 URDF、ros2_control xacro、joint 分組 SRDF | imu16460_hardware、ti5_hardware、dexterous_hand_description |
| [ti5_hardware](src/ti5_hardware/) | hardware plugin（外部） | T170 關節 `SystemInterface`（`ti5_hardware/Ti5Hardware`） | can_driver |
| [can_driver](src/can_driver/) | C++ lib（外部） | SocketCAN 與 Ti5 馬達指令 | — |
| [dexterous_hand_description](src/dexterous_hands/dexterous_hand_description/) | 資源（外部） | 靈巧手 ros2_control xacro macro | dexterous_hand_hardware（執行期） |
| [dexterous_hand_hardware](src/dexterous_hands/dexterous_hand_hardware/) | hardware plugin（外部） | 靈巧手 `SystemInterface`（RS485） | — |
| [graphen_mujoco](src/graphen_mujoco/) | hardware plugin | MuJoCo 模擬的 T170 本體 + IMU（`graphen_mujoco/MujocoSystem`）、MJCF 匯出與主機端 viewer（`tools/`） | — |
| [fb70_description](src/robot_description/fb70/) | 資源 | FullBodyT70 訓練場景匯出的 MJCF（位置伺服）與依關節的 PD 增益；沒有自己的 URDF | — |
| [ti5_t170_description](src/robot_description/ti5_t170_description/) | 資源（外部） | T170-V2.1-A0 模型（URDF xacro、MJCF、mesh、依關節的 PD 增益），robolab `Ti5T170-Flat`（tc_new_025）的訓練模型，實機 T170 對應的模型 | — |
| [graphen_aiterone_ny_description](src/robot_description/graphen_aiterone_ny_description/) | 資源（外部） | NY 機台的模型（`robot:=aiterone_ny`）：運動學同 ti5_170a、質量取自 ti5_t170、L 型手腕轉接板；URDF xacro、產生的 MJCF、依關節的 PD 增益、手臂 IK 設定與該機台的 CAN 配線 | dexterous_hand_description |
| [graphen_choreographer](src/graphen_choreographer/) | locomotion plugin（不依賴 rclcpp） | 編舞（多軌 keyframe 的組合動作）：`ChoreographerAlgorithm`、腳本解析、路徑碰撞檢查、`choreo_check` | graphen_locomotion、graphen_arm_ik |
| [graphen_arm_ik](src/graphen_arm_ik/) | C++ lib + node + locomotion plugin | 手臂 IK（手腕 6D + 手肘 swivel）與碰撞偵測（MuJoCo 3.11）、`arm_ik_server`（`~/solve`、`~/forward`、`~/tcp_jog`）、視覺伺服 plugin `ArmServoAlgorithm`、`arm_ik_check` | graphen_mujoco（CMake）、graphen_locomotion、graphen_interfaces |
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

bringup 以 `hardware:=` 選擇硬體（預設 `mock`），以 `robot:=` 選擇模型（預設 `ti5_170a`，也就是一直以來實機用的 URDF；另有 `ti5_t170`、`fb70`、`aiterone_ny`），所有 hardware 的預設相同。
`robot` 同時決定 URDF 本體、MuJoCo 的 MJCF 與 PD 預設增益。controller 設定由 launch 依序疊加：

| 檔案 | 內容 |
|---|---|
| `config/controllers.yaml`（本機檔，範本 `.example`） | 與硬體無關：系統模式、switch 接線、編舞 / BVH / policy 的庫位置與上限；本機自訂（例如子群組的 `output_enabled`）。表演內容與 policy 的設定不在這裡 |
| `config/hardware/position_common.yaml` | （`mock` / `mujoco` / `real`）`update_rate` 100、6 個 `PositionController` 子群組與介面接線 |
| `config/hardware/pd_common.yaml` | （`mujoco_pd` / `real_pd`）`update_rate` 200、6 個 `PdController` 子群組與 switch 的速度前饋 / kp / kd 接線 |
| `config/imu/common.yaml` | `imu_sensor_broadcaster`（所有硬體都啟動，發布 `/imu_sensor_broadcaster/imu`） |
| `config/imu/<mujoco\|mock\|imu16460>.yaml` | 依 IMU 來源：MuJoCo、`use_mock_imu:=true` 的 mock（加上 `sensor_forwarder`）、實機 |
| `<robot 套件>/config/joint_pd_gains.yaml` | （`mujoco_pd` / `real_pd`）模型的 PD 預設增益與力矩上限，以關節為 key（`joint_gains.<joint>.kp/kd/effort_limit`） |

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

# 啟動後不進入任何模式，所有 joint 保持初始位置（default_mode 與 default_policy 都是 ""）。
# states 含 32 軸的 position / velocity 與 mock IMU（imu_sensor/*，初始為靜止水平：orientation.w = 1、
# linear_acceleration.z = 9.81）
ros2 topic echo --once /switch_group_controller/states

# 寫入 mock IMU 的模擬值，10 個值依序為 orientation.xyzw、angular_velocity.xyz、linear_acceleration.xyz
# （用 -t 多送幾次：--once 可能在 subscriber 完成 discovery 前就送出而遺失）
ros2 topic pub -t 3 -w 1 /sensor_forwarder/commands std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 1,  0, 0, 0.5,  0, 0, 9.81]}"

# 用 action 切換到系統模式 reverse：result 附上目前的 mode / item 與切換當下的 state 快照，之後兩個手腕（L/R_WRIST_P）往 -1.5 移動，
# 其他 30 軸保持
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: reverse}"

# 所有播放類 action 的 goal 都有 options：on_same（0 = ignore 預設、1 = restart、2 = restart_if_finished）
# 與 wait_until_finished（等動作播完才回傳）。例如等 forward 到達目標：
ros2 action send_goal --feedback /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode \
  "{mode: forward, options: {wait_until_finished: true}}"

# 不在 modes 中的名稱會被 reject（topic 收到時則忽略並 WARN）
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: unknown}"

# 系統模式也可以用 topic 切換（QoS 必須相容；沒有回應，被拒絕時只有 controller 的 log）
ros2 topic pub --once /switch_group_controller/mode std_msgs/msg/String "{data: forward}" \
  --qos-durability transient_local --qos-reliability reliable
ros2 topic echo --once /left_arm_group_controller/references
# 各關節的追蹤誤差：expected（送進硬體的 command，PD 時為插補後的 q*）/ actual（量測）/ error（expected − actual）
ros2 topic echo --once /left_leg_group_controller/tracking

# 播放 BVH 步態（play_bvh：BVH 目錄 bvh.library_path 中的 <name>.bvh）：先以約 0.5 rad/s 走向第 0 幀，
# 再即時播放 32 軸，loop 時到結尾後接回起點循環。第一次播放要載入檔案（feedback 的 stage 為 0），之後使用快取。
# planner 依檔案頻率（24 fps）送出原始幀，幀間插補由子群組依 sample_period 進行（約晚一幀）。
# 同一個檔案、同樣的選項再送一次不會從頭播放；要重播用 options: {on_same: 1}
ros2 action send_goal --feedback /switch_group_controller/play_bvh graphen_interfaces/action/PlayBvh \
  "{name: t170wk_walk1_subject1, loop: true}"

# 停止：切到 hold 類的系統模式。不論目前在播什麼（包括不可中斷的編舞）都有效
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: stop}"

# 外部演算法 plugin 的範例（系統模式 sine，graphen_locomotion/SineWaveAlgorithm）：NECK_Y 以進入時的位置為中心
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

> BVH 步態是開環播放，沒有平衡控制；實機上只能在吊掛等安全條件下使用。

### BVH 轉換檢查

`bvh_retarget_cli` 把整段 BVH 轉成 CSV（time + 各 joint 角度，rad），並在 stderr 印出分組、每組最大殘差、各 joint 的範圍 / 限位 / 最大速度，用來檢查新的 BVH 或設定 `bvh.max_steps`：

```bash
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_motion bvh_retarget_cli \
  $(ros2 pkg prefix graphen_bringup)/share/graphen_bringup/motions/t170wk_walk1_subject1.bvh /tmp/t170.urdf > /tmp/walk.csv
```

### 編舞（組合動作）

多個部位各自依自己的時間動作、時間可以重疊，寫成一份腳本（`*.choreo.yaml`）。腳本不寫在 `controllers.yaml` 裡：編輯在外部工具進行，可以直接帶內容**試播**，確定之後**發佈**到機器人端的舞曲庫，表演（BT）依名稱播放。腳本格式與給 UI 開發的介面說明見 [docs/CHOREOGRAPHER.md](docs/CHOREOGRAPHER.md)，設計見 [ARCHITECTURE §2.12](docs/ARCHITECTURE.md#212-編舞graphen_choreographer)。

需要 `controllers.yaml` 有 `choreo` 區塊（見 `controllers.yaml.example`：plugin、舞曲庫位置、所有編舞共用的上限）。範例腳本在 `graphen_bringup/motions/`：`arm_demo`（右臂 0 ~ 3 s、左臂 1 ~ 5 s，標示為不可中斷）與 `arm_demo_return`（回到下垂姿態）、`greeting_demo`（揮手、前伸、轉腰、轉頭點頭四條軌道重疊，約 11.5 s；第一個 keyframe 都不在 `t: 0`，從任何姿態都能播，適合在 MuJoCo 上查看，用法見檔案開頭的註解）。

離線檢查腳本（範圍、限位、峰值速度、建議的 `max_steps`、姿態中的 joint 名稱；`--csv` 可輸出取樣後的整份編舞）：

```bash
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_choreographer choreo_check \
  $(ros2 pkg prefix graphen_bringup)/share/graphen_bringup/motions/arm_demo.choreo.yaml \
  --urdf /tmp/t170.urdf --csv /tmp/arm_demo.csv
```

發佈到舞曲庫（bringup 啟動之後；controller 會做和播放前相同的完整檢查，不通過就不寫入）：

```bash
MOTIONS=$(ros2 pkg prefix graphen_bringup)/share/graphen_bringup/motions
ros2 run graphen_bringup choreo_publish arm_demo $MOTIONS/arm_demo.choreo.yaml
ros2 run graphen_bringup choreo_publish arm_demo_return $MOTIONS/arm_demo_return.choreo.yaml
ros2 run graphen_bringup choreo_publish --list          # 已發佈的名稱與內容的 sha256
ros2 run graphen_bringup choreo_publish --get arm_demo  # 讀回內容
```

依名稱播放並等它播完（feedback 的 `elapsed` 為開始播放後經過的秒數）。`arm_demo` 的第一個 keyframe 在 `t: 0`，必須從下垂姿態開始，所以先播 `arm_demo_return`（從任何位置都可以）：

```bash
timeout 15 ros2 action send_goal --feedback /switch_group_controller/play_choreo \
  graphen_interfaces/action/PlayChoreo "{name: arm_demo_return, options: {wait_until_finished: true, on_same: 2}}"
timeout 15 ros2 action send_goal --feedback /switch_group_controller/play_choreo \
  graphen_interfaces/action/PlayChoreo "{name: arm_demo, options: {wait_until_finished: true}}"
```

- **試播**：把 `name` 換成 `content`（腳本的 YAML 字串），不存檔直接播放；檢查與發佈相同。這是給編輯工具用的；BT 的表演依名稱播放（`content` 只用於樹即時產生的短命令，例如視覺跟隨）。
- 內容有誤、名稱沒有發佈時 goal 為 aborted，`message` 說明原因，機器人繼續原本的動作。
- 等待中被其他播放取代（任何一個 action 或 `~/mode`）時 goal 會 abort（`Preempted by [ X ]`）；Ctrl-C（cancel）只是不再等待，**動作會繼續執行**，要停下來請切到 `stop`。
- 已經在播同一段時：`on_same: 0` 不重新開始；`1` 一律重播；`2` 播完才重播（播放中重複送出的指令被吃掉）。重新發佈後的同名舞曲視為不同的內容，會直接切換。
- `arm_demo` 的腳本寫了 `interruptible: false`：播放中其他播放或重播都會被拒絕（`success: false`），只有 `stop`（hold，預設 `preempt`）可以切入。
- 播放途中切到另一段編舞時，新的一段會從當下的速度平順地接上（`entry_velocity`），不會頓一下。
- 用 BT 依序播放兩段並重複（先發佈上面兩段）：

  ```bash
  ros2 launch graphen_bt bt.launch.py \
    tree_file:=$(ros2 pkg prefix graphen_bt)/share/graphen_bt/trees/choreo_demo.xml
  ```

### 手臂 IK（指令 / 視覺伺服 / 碰撞檢查）

bringup 依 `robot:=` 帶入 `<模型套件>/config/arm_ik.yaml`（launch 參數 `arm_ik_config`，`auto` 預設，`none` 停用；fb70 尚未驗證，`auto` 時停用），啟動 `arm_ik_server`，並設定編舞的路徑碰撞檢查與 `arm_servo` 模式。目標位姿一律以軀幹 `WAIST_P_S` 座標表示；swivel 為手肘繞「肩 → 腕」軸的絕對角度（0 = 軀幹 −z），`.nan` = 不指定。

```bash
# 設定檔的自我檢查（新增模型或修改 arm_ik.yaml 後）：收斂率、swivel、碰撞探測、伺服
ros2 run graphen_arm_ik arm_ik_check package://ti5_170a_graphen/config/arm_ik.yaml

# 指令：末端位姿 + elbow → 關節角（只計算，不送指令）；寫進編舞腳本的方式見 docs/CHOREOGRAPHER.md §7
ros2 service call /arm_ik_server/forward graphen_interfaces/srv/ArmForward "{arm: right}"
ros2 service call /arm_ik_server/solve graphen_interfaces/srv/SolveArmIk \
  "{arm: right, target: {header: {frame_id: WAIST_P_S}, pose: {position: {x: 0.35, y: -0.25, z: 0.05}, orientation: {w: 1.0}}}, swivel: .nan}"

# 末端單步（TCP jog）：以 reference（上一步的 positions；空 = 目前角度）沿一個軸走 delta，回傳新的關節角，
# 再以一個 keyframe 的編舞播放（docs/CHOREOGRAPHER.md §7.4）。axis 0-6 = X/Y/Z/ROLL/PITCH/YAW/ELBOW，frame 0 = 手腕、1 = 軀幹
ros2 service call /arm_ik_server/tcp_jog graphen_interfaces/srv/TcpJog "{arm: right, axis: 0, delta: 0.01, frame: 0}"

# 編舞的路徑碰撞（離線；播放與發佈時 controller 會做同樣的檢查）
ros2 run graphen_choreographer choreo_check <腳本> --arm-config package://ti5_170a_graphen/config/arm_ik.yaml

# 視覺伺服：進入 arm_servo，持續發布目標（超過 pose_targets.timeout 0.5 s 沒更新就保持）；下一步會碰撞時該手臂停住
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: arm_servo}"
ros2 topic pub -r 30 /switch_group_controller/pose_targets graphen_interfaces/msg/ArmPoseTarget \
  "{header: {frame_id: WAIST_P_S}, arm: right, pose: {position: {x: 0.35, y: -0.3, z: 0.05}, orientation: {x: 0.59, y: 0.01, z: 0.15, w: 0.79}}, swivel: .nan}"
ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: stop}"
```

本機 `controllers.yaml` 的手臂子群組若是 `output_enabled: false`，mock 上手臂不會動（伺服、編舞都一樣）。

### 播放介面的情境回歸

`playback_scenarios` 對著已啟動的 bringup 送出一連串指令並比對回應，涵蓋四個播放入口、舞曲庫的 service、切換規則（`on_same`、不可中斷、stop 搶占、互相取代）與影子試播（`dry_run`、`shadow_token`、`~/playback_state`，需要 hold 模式 `teach`），每一項印出 PASS / FAIL。**會讓機器人動作，只在 mock 或 MuJoCo 上執行。** 它會發佈名稱以 `scn_` 開頭的測試舞曲，建議用 `scenario_params` 把舞曲庫指到暫存目錄：

```bash
printf 'switch_group_controller:\n  ros__parameters:\n    choreo:\n      library_path: /tmp/choreo_test_library\n' \
  > /tmp/playback_test.yaml
ros2 launch graphen_bringup bringup.launch.py scenario_params:=/tmp/playback_test.yaml
# 另一個終端機
ros2 run graphen_bringup playback_scenarios                   # 全部；沒有啟用的類型自動略過
ros2 run graphen_bringup playback_scenarios --only choreo     # switch_mode / choreo / shadow / bvh / policy / cross / arm
```

`arm` 區段（手臂 IK service、編舞的路徑碰撞、`arm_servo` 伺服）會移動右手臂，需要手臂子群組 `output_enabled: true`（可以在 `scenario_params` 中覆寫 `left_arm_group_controller` / `right_arm_group_controller` 的 `output_enabled`）。

修改腳本（`scripts/*.py`）之後要重新建置 `graphen_bringup` 才會生效。

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

tree 依目前方向分成 `Forward` / `Reverse` 兩個分支，跨過門檻時走 `TurnToReverse` / `TurnToForward` 換向。用 Groot v1 連線可以看到目前亮著的分支和換向的瞬間；log 會每隔約 4 秒交替印出 `SwitchMode: switched to [ forward ]` / `[ reverse ]`，`/switch_group_controller/states` 中的 L_WRIST_P 會在約 ±1.0 之間往返。門檻在 tree XML 中修改（必須落在兩個模式的目標 ±1.5 之間）。

模式可在執行中用 rqt_reconfigure 或 `ros2 param set` 切換（walk 優先，兩個都 false 時切到 `stop` 停在目前位置）：

```bash
ros2 param set /bt_runner gp_enable_walk true          # 切到 walk（BVH 步行）
ros2 param set /bt_runner gp_enable_walk false         # 回到往返（從 forward 開始）
ros2 param set /bt_runner gp_enable_reciprocate false  # 兩個都 false：停止
```

### 視覺跟隨（頭部轉向人臉）

外部的人臉偵測節點把人臉中心發布成 `geometry_msgs/PointStamped`（`point.x` / `point.y` 為像素，原點在左上角；預設 topic `/face_tracker/target`，以 `gp_face_topic` 修改）。相機要裝在頭上，不需要校正。`face_track.xml` 依誤差以低頻送出頭部的速度命令（快 / 慢 / 停），人臉遺失時立即停止，超過 `gp_face_lost_hold` 秒回到正前方（系統模式 `head_center`）。設計見 [ARCHITECTURE §3.5](docs/ARCHITECTURE.md#35-與控制層的連動) 與 D47。

視覺端用 `graphen_vision` 的 `face_tracker`（YuNet 人臉偵測，多張臉時鎖定一張持續跟隨）。啟動順序：

```bash
# 1. 相機 + 視覺端，二擇一：
#    a. 直接讀 V4L2 裝置：webcam，或不裝 RealSense SDK、把 RealSense 的彩色串流當 webcam
#       （v4l2-ctl --list-devices 找出 RealSense 的節點，--list-formats-ext 中 YUYV 的是彩色；
#        建議用 /dev/v4l/by-id/... 的路徑，WSL 預設核心沒有 uvcvideo 讀不到）
ros2 launch graphen_vision face_tracker.launch.py source:=device device:=/dev/video4
#    b. realsense2_camera（要在機器人端安裝）發布影像，face_tracker 訂閱
ros2 launch graphen_vision face_tracker.launch.py start_camera:=true color_profile:=640x480x30
#    log 會印出影像尺寸，必須和 bt_runner 的 gp_face_image_width / gp_face_image_height 一致；
#    設定 publish_debug_image: true 後可以用 rqt_image_view 看 /face_tracker/debug_image 的框
# 2. 控制層（bringup）
# 3. 行為層
ros2 launch graphen_bt bt.launch.py \
  tree_file:=$(ros2 pkg prefix graphen_bt)/share/graphen_bt/trees/face_track.xml
ros2 param set /bt_runner gp_enable_face_track true    # 開始跟隨（預設 false）
ros2 param set /bt_runner gp_face_v_slow 3.0           # 執行中調整；小數要寫成 x.0
```

- 參數（速度 deg/s、門檻、正負號、範圍、命令時效）都在 `bt_runner.yaml` 的 `gp_face_*`，說明見 [bt_runner.yaml.example](src/graphen_bt/config/bt_runner.yaml.example)。本機的 `bt_runner.yaml` 與 `controllers.yaml`（`head_center` 模式）要和範本同步。
- **第一次上機先確認方向**：`gp_face_sign_yaw` / `gp_face_sign_pitch` 反了頭會越轉越遠，立即把 `gp_enable_face_track` 設為 false（頭最多再轉 `v × gp_face_horizon`）。
- 需要 `choreo.shadow.require: false`：速度命令是 PlayChoreo 帶內容的短編舞，強制試播時每個命令都會被拒絕。
- `face_tracker` 的參數（影像 topic、偵測門檻、鎖定、平滑）在 [face_tracker.yaml](src/graphen_vision/config/face_tracker.yaml)。收不到影像時（每 5 秒 WARN）先確認 `image_topic` 與 `image_reliable` 和發送端相符（`ros2 topic info -v <topic>`）。

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

CAN 通道與馬達對應在 [can_driver_config.json](src/graphen_bringup/config/can_driver_config.json)（`robot:=aiterone_ny` 時改用該機台的 [config/can_driver_config.json](src/robot_description/graphen_aiterone_ny_description/config/can_driver_config.json)，log 會印出 `can_config:`），實機初始位置在 [real_initial_positions.yaml](src/graphen_bringup/config/real_initial_positions.yaml)：開機時有列出的 joint 會以 `max_command_velocity`（預設 0.5 rad/s）限速走到該值，沒列的保持目前位置；之後要等 BT 或 action 下命令才會動。加上 `use_hands:=true` 時靈巧手也會改用實機（`left_hand_port`、`right_hand_port`、`hand_baudrate`）。

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

控制方式由 `hardware` 決定，模型由 `robot` 決定：

| `hardware` | 致動器 | 子群組 | 預設增益 |
|---|---|---|---|
| `mujoco` | MJCF 的位置伺服 | `PositionController` | MJCF 內的值，所有模式相同 |
| `mujoco_pd` | 力矩馬達 | `PdController`（`update_rate` 200） | `<robot 套件>/config/joint_pd_gains.yaml`；模式有設定 `kp` / `kd` 時（RL policy）改用模式的值 |

| `robot` | URDF 本體 | MJCF | 可用的 hardware | 說明 |
|---|---|---|---|---|
| `ti5_t170` | `ti5_t170_description`（+ 靈巧手） | `ti5_t170_description/mjcf/ti5_t170.xml`（T170-V2.1-A0 T-pose，tc_new_025 的訓練模型） | 全部（MuJoCo 只有 `mujoco_pd`） | mesh 用相對路徑，WSL 也能跑；頸部是真正的關節；初始基座高度 1.000 m（直膝；MJCF 的 keyframe 為屈膝的 0.985 m） |
| `ti5_170a`（預設） | `ti5_170a_graphen` 的 `robot.urdf.xacro` | `ti5_170a_graphen/mjcf/ti5_170a.xml`（由 URDF 產生，手固定、armature 借用 ti5_t170） | 全部（MuJoCo 只有 `mujoco_pd`） | 質量分布可疑（`WAIST_R_S` 49 kg、本體 93.7 kg），模擬不代表實機；初始基座高度 0.899 m |
| `fb70` | `ti5_170a_graphen` 的 `robot.urdf.xacro` | `fb70_description/mjcf/t170_{free,fixed}.xml`（FullBodyT70 匯出，wtz 的訓練模型） | 全部 | mesh 指向 Orin 上的 `/devtest/FullBodyT70/...`，只能在 Orin 上載入 |
| `aiterone_ny` | `graphen_aiterone_ny_description`（+ 靈巧手） | `graphen_aiterone_ny_description/mjcf/graphen_aiterone_ny.xml`（由 URDF 產生，手固定、位置伺服） | 全部 | 外觀（group 2）為原始網格、碰撞（group 3，viewer 按 `3` 顯示）為凸包，靈巧手的外觀也是凸包；`mujoco` 的伺服增益依頻寬計算（預設 10 Hz、阻尼比 1），近似實機位置模式（驅動器三環）、尚未以實機校正；`mujoco_pd` 時自動改成力矩馬達；初始基座高度 0.899 m |

policy 只在自己的訓練模型上才站得穩，模型與 policy 要對應（`rl_flat` / `tc_flat_policy*` 的訓練模型 t170a 已移出工作區，目前沒有對應的模擬模型）。各模型的浮動基座初始高度不同（直膝下腳底離地 5 mm），由 launch 依 `robot` 帶入 MujocoSystem 的 `initial_base_height`。

`mujoco_fixed_base:=true`（預設）時基座固定在空中，相當於吊掛：fb70 改用 `t170_fixed.xml`，ti5_170a / ti5_t170 每步把基座重設回初始位姿。`false` 時為自由站立。

```bash
# fb70（Orin 容器內）
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco robot:=fb70 mujoco_fixed_base:=false
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco_pd robot:=fb70 mujoco_fixed_base:=false
# ti5_t170（任何機器）
ros2 launch graphen_bringup bringup.launch.py hardware:=mujoco_pd mujoco_fixed_base:=true

# 主機上另開視窗（只顯示，不需要 ROS；視窗內按 End 重置模擬）。不指定 --model 時向 MujocoSystem 查詢
# 它載入的 MJCF（先開 viewer 會等到 bringup 啟動）；bringup 在容器內、路徑在主機上不存在時要自己指定
bash src/graphen_mujoco/tools/mujoco_viewer.sh                                  # 模型跟著 bringup
bash src/graphen_mujoco/tools/mujoco_viewer.sh --model "$PWD/src/robot_description/fb70/mjcf/t170_fixed.xml" # fb70 吊掛
bash src/graphen_mujoco/tools/mujoco_viewer.sh --model "$PWD/src/robot_description/ti5_t170_description/mjcf/ti5_t170.xml"  # sim_rl 預設
# 兩手手腕（arm_ik.yaml 的 tip_body）的座標軸以箭頭顯示（x 紅、y 綠、z 藍，也就是 tcp_jog FRAME_TOOL 的軸），
# Pause 鍵切換；--no-wrist-axes 預設隱藏，--wrist-bodies / --wrist-axis-length / --wrist-axis-width 調整

# mock / 實機：--source ros 訂閱 /joint_states 顯示機器人的姿態（要能 import rclpy，見下方的 venv；必須指定 --model）
PY=~/teach_env/bin/python bash src/graphen_mujoco/tools/mujoco_viewer.sh --source ros \
    --model "$PWD/src/robot_description/ti5_170a_graphen/mjcf/ti5_170a.xml" \
    --ghost-topic /switch_group_controller/shadow/joint_states        # 影子試播的軌跡以半透明的機器人疊加

# 手臂示教（只改畫面、不送指令）：controller 切到 teach 模式（hold，機器人不動）時 viewer 自動進入示教，
# 離開該模式就結束（--teach-always：不看 controller）。雙擊手腕目標（紅 = 右、藍 = 左），
# Ctrl+右鍵拖曳平移、Ctrl+左鍵拖曳旋轉，手臂跟隨 wrist 位置 + 姿態；F9 / F10 調整手肘的 swivel、wrist 不動；
# 解的姿態有碰撞時目標變黃並列出 body 對；PageDown 切換要調整 swivel 的手臂。
# 也可以直接指定關節角（只填要改的 joint；超出限位截斷）。不要用 --once（viewer 常常還沒配對到就送完了）：
#   timeout 10 ros2 topic pub -r 5 --times 3 -w 1 /mujoco_viewer_teach/set_pose sensor_msgs/msg/JointState \
#     '{name: [R_SHOULDER_P, R_ELBOW_Y], position: [-0.5, 1.2]}' 
# viewer 不記錄、不匯出：全身的姿態發布在 /mujoco_viewer_teach/pose（格式同 /joint_states：joint 與順序相同，
# velocity / effort 為 0；sensor_msgs/JointState，transient_local），
# 由外部程式擷取成 keyframe、組成腳本，再以 PlayChoreo 的影子試播（dry_run）檢查後播放（見 docs/CHOREOGRAPHER.md §3.4、§7.5）。
# IK 由 arm_ik_server 計算（bringup 預設啟動）：viewer 要能 import rclpy（先 source install/setup.bash），
# 沒有時示教停用。建議用 ROS 的 python3（3.10）另建 venv，避免 pip 升級系統的 numpy 影響 ROS 套件：
#   python3 -m venv ~/teach_env && ~/teach_env/bin/pip install mujoco==3.11.0 glfw pyyaml "numpy<2"
timeout 10 ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: teach}"
timeout 5 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /mujoco_viewer_teach/pose sensor_msgs/msg/JointState                                  # 擷取目前示教的姿態
# 影子試播：teach（shadow_playback: true）中的 PlayChoreo 一律只在 controller 內部空跑（不動），回傳 shadow_token；
# 其他模式中要加 options.dry_run。content 的換行寫成 \n（YAML 單引號跨行會把換行摺成空格）
timeout 30 ros2 action send_goal /switch_group_controller/play_choreo graphen_interfaces/action/PlayChoreo \
    '{content: "<腳本，換行寫成 \n>"}'
# 上機：先離開 teach（stop 同樣不動、起點不變），再帶 token 正式播放
timeout 10 ros2 action send_goal /switch_group_controller/switch_mode graphen_interfaces/action/SwitchMode "{mode: stop}"
timeout 30 ros2 action send_goal /switch_group_controller/play_choreo graphen_interfaces/action/PlayChoreo \
    '{content: "<同一份腳本>", options: {shadow_token: "<token>", wait_until_finished: true}}'
# 不開 bringup 時自己啟動 server（模型要和 viewer 的一致），viewer 加 --teach-always：
ros2 run graphen_arm_ik arm_ik_server --ros-args -p arm_ik_config:=package://ti5_170a_graphen/config/arm_ik.yaml
PY=~/teach_env/bin/python bash src/graphen_mujoco/tools/mujoco_viewer.sh --teach-always \
    --model "$PWD/src/robot_description/ti5_170a_graphen/mjcf/ti5_170a.xml"

# 執行 policy（run_policy：policy 庫中的 <name>.policy.yaml）：第一次要載入（建立 ONNX session），
# 進入時會先以 entry_speed 走到 policy 的預設姿態，再開始推論
ros2 action send_goal --feedback /switch_group_controller/run_policy graphen_interfaces/action/RunPolicy "{name: rl_walk}"

# 表演前可以先把要用的 policy / BVH 載入快取，之後第一次播放就沒有載入延遲
ros2 service call /switch_group_controller/preload graphen_interfaces/srv/Preload "{mode: policy, items: [rl_walk]}"

# 速度指令（vx、vy m/s，wz rad/s）；停止發布 0.5 s 後回到原地站立
ros2 topic pub -r 20 /switch_group_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

- `mujoco_pd` 下，所有子群組都啟動之前物理是暫停的，controller 啟動前不會倒下。
- **自由站立測 RL policy 要用 `sim_rl.launch.py`**：PD 只保持姿態、沒有平衡能力，`bringup.launch.py` 自由站立啟動後幾秒內就會倒下（姿態約 90°），之後才切到 RL 模式已經來不及。`sim_rl.launch.py` 會在最後疊加一份情境參數（`default_policy` 設為 `policy`、以 `<policy>.entry_speed: 0` 覆寫設定檔的值），讓 policy 從模擬開始就接手；本機 `controllers.yaml` 不用改，policy 必須在 policy 庫中：

  ```bash
  ros2 launch graphen_bringup sim_rl.launch.py                        # ti5_170a + rl_flat（預設）
  ros2 launch graphen_bringup sim_rl.launch.py policy:=rl_walk robot:=fb70   # Orin
  ros2 launch graphen_bringup sim_rl.launch.py policy:=tc_new_nowave_land_2        # 指定另一個 policy 設定檔
  ```
- policy 的設定不在 `controllers.yaml`：每個 policy 是 policy 庫（`policy.library_path`，預設 `graphen_rl_policy/policy/`）中的一個 `<名稱>.policy.yaml`，和 onnx 放在一起，以檔名選擇。要調整某個 policy 的單一設定時，在 `controllers.yaml` 寫 `<policy 名稱>.<鍵>`（會覆寫檔案中的同名鍵）。目前有：
  - `rl_walk`：`T170VelocityPolicy` + `policy/wtz_policy.onnx`，用 fb70 訓練。實測（fb70、position、自由站立）：能穩定站立；vx=0.5 時實際約 0.23 m/s，這是 policy 本身的表現（直接跑 `play_mujoco.py` 也約 0.25 m/s）。
  - `tc_new_nowave_land_2`：`Ti5FlatPolicy` + `policy/tc_new_nowave_land_2.onnx`，PD 增益為 t170a 參數表的值（由原本本機 `controllers.yaml` 的 `rl_flat` 搬過來）。
  - `rl_flat`：`Ti5FlatPolicy` + `policy/tc_flat_policy.onnx`，用 t170a 訓練（t170a 模型已移出工作區）。當時的實測（t170a、`mujoco_pd`、自由站立、WSL）：能穩定站立（傾斜約 2.5°）；vx=0.5 時能邁步、不會倒下。關節順序與 obs 組法是推斷的，**仍待訓練端確認**。
- ti5_170a 的 MJCF 由 URDF 產生，URDF 或 mesh 更新後重新產生，並把印出的基座高度同步到 `bringup.launch.py` 的 `robots['ti5_170a']`：
  `python3 src/robot_description/ti5_170a_graphen/scripts/make_mjcf.py`
- aiterone_ny 的 MJCF 同樣由 URDF 產生（手部碰撞凸包產生在該套件的 `meshes/hulls/dexterous_hand/`），基座高度同步到 `robots['aiterone_ny']`；本體 mesh 更新時先跑同目錄的 `make_hulls.py`：
  `python3 src/robot_description/graphen_aiterone_ny_description/scripts/make_mjcf.py`
  外觀與碰撞是兩組 geom：class `visual`（group 2，只顯示、不碰撞、不算質量，本體用原始 STL）與 class `collision`（group 3，URDF collision 的凸包，viewer 預設隱藏）。`--visual-mesh hull` 時外觀也用凸包（載入約 0.14 s，原始網格約 0.46 s）；靈巧手的外觀一律沿用碰撞網格（要原始外觀用 `--hand-mesh-type original`，MJCF 會改用絕對路徑）。
  致動器是位置伺服：`kp = I·ω²`、`kv = 2ζ·I·ω`，`I` 為 home 姿態下質量矩陣的對角項（含 armature），`ω = 2π·--servo-bandwidth`（預設 10 Hz），`ζ` 為 `--servo-damping-ratio`（預設 1）；腳本會列出各關節的 I / kp / kv。頻寬越高越硬，但 kp 是顯式積分，`ω·timestep` 超過約 0.5（約 16 Hz）就容易不穩。有實機位置模式的階躍響應後，以它校正頻寬與阻尼比
- ti5_t170 的 MJCF 由 CAD 匯出的 T-pose 檔產生（補上 timestep、armature、ctrlrange、IMU、碰撞設定與 keyframe）；原檔或 `param.csv` 更新後重新產生：
  `python3 src/robot_description/ti5_t170_description/scripts/make_mjcf.py --base-height 0.985`
- fb70 的 MJCF 由 FullBodyT70 的訓練場景匯出，重新訓練或修改場景後要重新產生（需要主機的 `~/learnWalk_env`）：
  `~/learnWalk_env/bin/python src/graphen_mujoco/tools/export_mjcf.py`
- **onnx 與設定綁定**：每個 policy 設定檔以 `model_sha256` 綁定 onnx，不符時載入失敗（`run_policy` 的 goal 為 aborted；`default_policy` 則是 controller configure 失敗）。
  - **同一套訓練設定重新訓練**：放入新的 onnx，複製一份 `<名稱>.policy.yaml`，更新 `model_path`、`model_sha256`（`sha256sum <onnx>`）與改變的常數，不需要修改程式。
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

實機時 `sensor_forwarder` 不啟動（`imu_sensor_broadcaster` 在所有硬體都會啟動）；`switch_group_controller` 的設定不需要改動（mock 與實機的 interface 名稱相同）。
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
