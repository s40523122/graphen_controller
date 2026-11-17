# 編舞（graphen_choreographer）開發參考

給「編舞編輯 / 播放 UI」開發用的參考文件，涵蓋以下內容：

- 腳本格式與語意
- 執行時的規則與檢查
- 控制介面：action、topic、BT
- 指令範例
- 注意事項

設計背景見 [ARCHITECTURE.md](ARCHITECTURE.md) §2.12；程式的權威定義在 `graphen_choreographer/choreography.hpp`、`interpolation.hpp`、`choreographer_algorithm.cpp`。

---

## 1. 整體運作

```
腳本檔 (*.choreo.yaml)
   │  controller configure 時載入、檢查（之後不會重讀）
   ▼
controllers.yaml 的模式（type: plugin, plugin: graphen_choreographer/ChoreographerAlgorithm, file: ...）
   │  SwitchMode action / ~/mode topic / BT 切換到這個模式
   ▼
LocomotionPlanner（100 Hz）每週期取樣編舞 → 位置 + 速度前饋 → 子群組 controller → 馬達
```

- 一個腳本對應一個**模式**。UI 播放腳本，其實就是「切換到那個模式」。
- 腳本裡有多條**軌道（track）**，每條軌道控制自己的一組 joint，並有自己的 keyframe。
- 所有軌道共用同一個時鐘，時間可以重疊。
- 編舞是**開環**的：只照時鐘輸出，不等待實際位置。某個 joint 跟不上，也不會拖慢其他軌道。
- 播完之後保持最後的姿態，不支援 loop。要重播必須重新進入模式，見 §4.2 的 `on_same_mode`。

---

## 2. 腳本格式

檔案放在 `src/graphen_bringup/motions/<名稱>.choreo.yaml`，單位一律為 rad 與秒。

```yaml
version: 1                       # 必填，目前只有 1
default_interp: quintic          # 選填：linear / cubic / quintic，預設 quintic

poses:                           # 選填：命名姿態庫，joint → 角度
  A: {R_SHOULDER_P: 0.0, R_SHOULDER_R: 1.3, ..., L_SHOULDER_P: 0.0, ...}   # 可以是全身 32 軸
  B: {...}

tracks:                          # 必填，至少一條
  right_arm:                     # 軌道名稱（自訂）
    joints: [R_SHOULDER_P, R_SHOULDER_R, R_SHOULDER_Y, R_ELBOW_Y, R_WRIST_P, R_WRIST_Y, R_WRIST_R]
    keyframes:                   # 至少一個
      - {t: 0.0, pose: A}                       # 引用命名姿態
      - {t: 3.0, pose: B, interp: cubic}        # interp 決定「到達這個 keyframe」那一段的插值
      - {t: 4.0, hold: true}                    # 停留到 4.0 s
      - {t: 6.0, positions: {R_SHOULDER_P: 0.0, R_SHOULDER_R: 1.3, R_SHOULDER_Y: 0.0,
                             R_ELBOW_Y: 0.3, R_WRIST_P: 0.0, R_WRIST_Y: 0.0, R_WRIST_R: 0.0}}
```

### 2.1 欄位

| 位置 | 欄位 | 必填 | 說明 |
|---|---|---|---|
| 頂層 | `version` | ✔ | 整數 `1` |
| 頂層 | `default_interp` | | 沒寫 `interp` 的 keyframe 用這個值 |
| 頂層 | `poses` | | `{姿態名稱: {joint: 角度}}` |
| 頂層 | `tracks` | ✔ | `{軌道名稱: {joints, keyframes}}`，依檔案中的順序 |
| track | `joints` | ✔ | 這條軌道控制的 joint 名稱 |
| track | `keyframes` | ✔ | keyframe 陣列 |
| keyframe | `t` | ✔ | 從開始播放起算的時間（s），≥ 0，同一條軌道內嚴格遞增 |
| keyframe | `pose` / `positions` / `hold: true` | 三選一 | 見下表 |
| keyframe | `interp` | | `linear` / `cubic` / `quintic` |

| keyframe 來源 | 規則 |
|---|---|
| `pose: <名稱>` | 姿態必須**包含**這條軌道的全部 joint，**可以多出其他 joint**，多出的部分會被忽略。用這個方式可以讓「全身姿態」只取一部分 |
| `positions: {...}` | 必須**剛好是**這條軌道的全部 joint，不能缺也不能多 |
| `hold: true` | 維持上一個 keyframe 的位置；前面沒有姿態時，維持起點 |

**格式檢查很嚴格**，以下情況都會報錯，訊息會指出位置，例如 `tracks.a.keyframes[1]: 't' must be greater than the previous keyframe's`：

- 未知的 key
- 數值不是有限值
- 引用不存在的姿態名稱
- 同一個 joint 出現在兩條軌道
- 姿態名稱重複

### 2.2 播放語意

- **隱含的起點：** 每條軌道在 t = 0 都有一個起點，就是進入模式當下的命令位置。軌道會從起點插值到第一個 keyframe，所以第一個 keyframe 的 `t` 就是走過去花的時間。
- **第一個 keyframe 在 `t: 0`** 代表「開始時必須已經在這個姿態」。如果目前位置和它的差距超過 `start_tolerance`，就不會執行，見 §3.3。表演時，想確保從固定姿態開始就用這個寫法；想從任何位置平順地接過去，就把第一個 keyframe 的 `t` 設成大於 0。
- **一段的插值方式由「終點」keyframe 的 `interp` 決定。**
- **軌道播完後保持最後的位置。** 整個模式的長度等於最長那條軌道的最後一個 `t`。
- **不在任何軌道中的 joint 不會被控制**，會保持進入模式時的位置。

### 2.3 插值

`u = (t - t_段起點) / T`，其中 `T` 是這一段的長度，`Δ` 是兩端的差。

| interp | s(u) | 兩端 | 峰值速度 |
|---|---|---|---|
| `linear` | u | 速度不連續 | \|Δ\|/T × **1.0** |
| `cubic` | 3u² − 2u³ | 速度為 0 | \|Δ\|/T × **1.5** |
| `quintic` | 10u³ − 15u⁴ + 6u⁵ | 速度、加速度都為 0（對實機最溫和） | \|Δ\|/T × **1.875** |

三種插值都是單調的，所以 keyframe 的範圍就是整條軌跡的範圍。UI 可以直接用 `|Δ|/T × 係數` 算出每一段的峰值速度並即時顯示警告，結果和 controller 的檢查一致。

**速度平滑化（`entry_velocity`）：** 播放途中從另一個動作切過來時，第一段會帶入切換當下的速度：

`p(u) = p0 + Δ·s(u) + v0·T·h(u)`，其中 `h(u) = u − 6u³ + 8u⁴ − 3u⁵`

效果是曲線先順著原本的方向減速，再轉向往第一個 keyframe 走，不會瞬間煞停。帶入速度時，這一段**不再是單調的**，可能超出兩端的值；進入模式時會檢查位置限位。

以下情況不會帶入速度：

- 第一段是 `linear`
- 第一個 keyframe 在 `t: 0`
- 參數 `entry_velocity: false`

### 2.4 Joint 名稱（T170，32 軸）

順序和 `controllers.yaml` 的 `max_steps` 相同：

```
L_SHOULDER_P L_SHOULDER_R L_SHOULDER_Y L_ELBOW_Y L_WRIST_P L_WRIST_Y L_WRIST_R      # 0-6   左臂
R_SHOULDER_P R_SHOULDER_R R_SHOULDER_Y R_ELBOW_Y R_WRIST_P R_WRIST_Y R_WRIST_R      # 7-13  右臂
WAIST_Y WAIST_R WAIST_P                                                             # 14-16 腰
NECK_Y NECK_P NECK_R                                                                # 17-19 頭
L_HIP_P L_HIP_R L_HIP_Y L_KNEE_P L_ANKLE_P L_ANKLE_R                                # 20-25 左腿
R_HIP_P R_HIP_R R_HIP_Y R_KNEE_P R_ANKLE_P R_ANKLE_R                                # 26-31 右腿
```

位置與速度限位以 URDF 為準（`ti5_170a_graphen`），UI 可以讀取 `/robot_description` 取得。

---

## 3. 模式設定與檢查

### 3.1 在 controllers.yaml 註冊模式

```yaml
switch_group_controller:
  ros__parameters:
    modes: [..., arm_demo]
    arm_demo:
      type: plugin
      plugin: graphen_choreographer/ChoreographerAlgorithm
      file: package://graphen_bringup/motions/arm_demo.choreo.yaml   # 或絕對路徑
      start_tolerance: 0.05        # rad，預設 0.05
      velocity_limit_scale: 0.25   # 速度上限 = URDF velocity × 這個值，預設 1.0
      entry_velocity: true         # 預設 true
      interruptible: false         # 預設 true，見 §4.3
      preempt: false               # 預設：只有 type: hold 為 true
      max_steps: [0.02 ×14 (手臂), 0.005 ×18 (其他)]   # 32 個值，每週期最大移動量（rad）
```

`max_steps` 必須容得下腳本的峰值速度：`峰值速度 / planner_rate ≤ max_steps`，`planner_rate` 預設為 100 Hz。

- 編舞是開環的，一旦被限速，時間就會被拉長，所以超過時直接拒絕。
- 沒寫 `max_steps` 時，使用 `planner_max_steps`（預設 0.005，等於 0.5 rad/s）。這對大部分動作都太小。
- 建議值可以用 `choreo_check` 算出。

### 3.2 configure 時的檢查（任何一項失敗，controller 就無法啟動）

- 腳本格式（§2.1）
- 腳本的 joint 都是受控的 joint
- 每個 keyframe 都在 URDF 的位置限位內
- 每一段的峰值速度不超過 URDF velocity × `velocity_limit_scale`
- 每一段的峰值速度不會被 `max_steps` 限速

錯誤訊息範例：

```
track 'right_arm' joint 'R_ELBOW_Y': peak velocity 0.438 rad/s needs 0.00438 rad per cycle,
but max_steps allows 0.00500 (set it to at least 0.00525, or slow the choreography down)
```

### 3.3 進入模式時的檢查（失敗 → 不執行、保持不動、回報失敗）

「起點 → 第一個 keyframe」這一段取決於當下的位置，所以只能在進入模式時檢查：

| 情況 | 訊息（節錄） |
|---|---|
| 第一個 keyframe 在 `t: 0`，但距離超過 `start_tolerance` | `is 0.6000 rad away from the first keyframe at t = 0 (start_tolerance 0.0500); move there first or give the keyframe a time` |
| 走過去的速度超過上限 | `reaching the keyframe at t = 1.00 s from here needs ... rad/s, above the limit ...` |
| 走過去會被 `max_steps` 限速 | `... needs ... rad per cycle, but max_steps allows ...` |
| 帶入速度的曲線超出位置限位 | `the move to the first keyframe (carrying ... rad/s ...) reaches [...], outside the limits [...]` |

失敗時：

- 如果是用 `wait_until_finished` 送出的 action，goal 會 abort，訊息為 `Mode [ X ] failed, see the controller log`。
- 詳細原因只會出現在 controller 的 log。

### 3.4 什麼時候算「播完」

同時滿足以下兩個條件時算播完：

- 時間 ≥ 最長軌道的最後一個 `t`
- 每個受控 joint 的**命令位置**都已經等於最後的值

判斷依據是命令位置，不是量測位置。所以在 MuJoCo 或實機上即使有穩態誤差，也會正常回報播完。

---

## 4. 控制介面

controller 名稱為 `switch_group_controller`，以下都在它的 namespace 下。

| 名稱 | 類型 | 用途 |
|---|---|---|
| `/switch_group_controller/switch_mode` | action `graphen_interfaces/action/SwitchMode` | **UI 主要使用的介面**：播放、重播、等待播完 |
| `/switch_group_controller/mode` | topic `std_msgs/msg/String`（reliable + transient_local） | 簡單切換模式，沒有回應；固定為 ignore 語意 |
| `/switch_group_controller/states` | topic `control_msgs/msg/DynamicJointState`（50 Hz） | 各關節的 state interface，用來顯示即時姿態 |
| `/joint_states` | topic `sensor_msgs/msg/JointState` | 同上，標準格式 |
| `/robot_description` | topic `std_msgs/msg/String` | URDF：關節限位 |

### 4.1 SwitchMode action

```
# Goal
string mode                      # controllers.yaml 的模式名稱
int8   on_same_mode 0            # 0 IGNORE / 1 RESTART / 2 RESTART_IF_FINISHED
bool   wait_until_finished false
---
# Result
bool     success
string   message
string   mode                    # 回傳當下實際生效的模式
string[] interface_names         # 當下的 state 快照
float64[] values
---
# Feedback（只在 wait_until_finished 時發布，頻率為 switch_mode_poll_rate，預設 20 Hz）
float64 elapsed                  # 進入模式後經過的秒數
```

**goal 被拒絕（REJECTED）的情況：**

- controller 不在 active 狀態
- 模式名稱不存在
- `on_same_mode` 超出 0–2

**結果一覽：**

| 情況 | 狀態 | `message` |
|---|---|---|
| 切換成功，不等待 | SUCCEEDED | `Mode switched` |
| 已經在這個模式，沒有重播 | SUCCEEDED | `Already in this mode, not restarted` |
| 等待後播完 | SUCCEEDED | `Mode finished` |
| 目前的模式不可中斷 | ABORTED | `Mode [ X ] is not interruptible until it finishes; [ Y ] rejected.` |
| 等待中被其他模式取代 | ABORTED | `Preempted by mode [ Z ]` |
| 動作失敗（例如 §3.3 的情況） | ABORTED | `Mode [ X ] failed, see the controller log` |
| 等待中 controller 被停用 | ABORTED | |
| 客戶端 cancel | CANCELED | `Stopped waiting; the mode keeps running` |

> **cancel 只會停止等待，不會停止動作。** 要停止動作，請切換到 `stop`（hold）模式。

**進度條：** `elapsed / duration`。`duration` 由 UI 從腳本自行算出（所有軌道最後一個 `t` 的最大值），action 不會回傳 duration。

### 4.2 `on_same_mode`：已經在同一個模式時的處理

這個設定**只影響這一次指令**，不會留給之後的指令。

| 值 | 名稱 | 播放中 | 已播完 |
|---|---|---|---|
| 0 | IGNORE（預設） | 不重播 | 不重播 |
| 1 | RESTART | 從頭重播（算是中斷，受 `interruptible` 限制） | 重播 |
| 2 | RESTART_IF_FINISHED | 不重播（吃掉重複送出的指令） | 重播 |

UI 的建議：

- 「播放」按鈕用 **2**，按很多次也不會讓動作從頭開始抖動。
- 「重新播放」按鈕用 **1**。

`wait_until_finished: true` 搭配「不重播」時，等待的是目前正在播放的那一次。如果那一次已經播完，會立即回傳成功。

### 4.3 中斷規則（`interruptible` / `preempt`）

| 目前的模式 | 要切換到 | 結果 |
|---|---|---|
| `interruptible: true`（預設） | 任何模式 | 切換 |
| `interruptible: false`，**播放中** | `preempt: true` 的模式（`stop` 等 hold） | 切換：停止永遠有效 |
| `interruptible: false`，**播放中** | 其他模式（包括自己帶 RESTART） | **拒絕** |
| `interruptible: false`，已播完或已失敗 | 任何模式 | 切換 |

- 透過 `~/mode` topic 被拒絕時，只會在 controller log 印出 WARN，送出端收不到任何回應。所以 UI 應該用 action。
- 被其他模式取代時，原本在等待的 goal 會被 abort（`Preempted by mode [ Z ]`）。

### 4.4 BT 節點 `SwitchMode`

| port | 預設 | 說明 |
|---|---|---|
| `action_name` | `/switch_group_controller/switch_mode` | |
| `mode` | — | 模式名稱 |
| `timeout_ms` | 1000 | 等待 server 與接受 goal 的時間上限 |
| `wait_finished` | false | 播完才回傳 SUCCESS |
| `on_same_mode` | `ignore` | `ignore` / `restart` / `restart_if_finished` |
| `finish_timeout_ms` | 0 | `wait_finished` 時等待播完的上限，0 表示不限。超過時回傳 FAILURE，但動作不會停 |

halt 時會 cancel goal，只是停止等待，動作不會停。範例見 `graphen_bt/trees/choreo_demo.xml`。

---

## 5. 指令範例

```bash
source install/setup.bash

# 離線檢查：範圍、峰值速度、建議的 max_steps、限位。
# 回傳值：0 通過、1 有誤、2 用法錯誤
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_choreographer choreo_check src/graphen_bringup/motions/arm_demo.choreo.yaml \
  --urdf /tmp/t170.urdf --velocity-scale 0.25 --rate 100 --csv /tmp/arm_demo.csv
# CSV：time + 每個 joint 一欄。假設從第一個姿態、靜止開始，可以拿來預覽曲線

# 播放（等待播完、顯示進度）
timeout 30 ros2 action send_goal --feedback /switch_group_controller/switch_mode \
  graphen_interfaces/action/SwitchMode "{mode: arm_demo, on_same_mode: 2, wait_until_finished: true}"

# 從頭重播
timeout 10 ros2 action send_goal /switch_group_controller/switch_mode \
  graphen_interfaces/action/SwitchMode "{mode: arm_demo, on_same_mode: 1}"

# 停止（任何時候都有效）
timeout 10 ros2 action send_goal /switch_group_controller/switch_mode \
  graphen_interfaces/action/SwitchMode "{mode: stop}"

# 簡易切換（沒有回應）
timeout 5 ros2 topic pub --once -w 1 /switch_group_controller/mode std_msgs/msg/String "{data: arm_demo}" \
  --qos-reliability reliable --qos-durability transient_local

# 讀取目前的姿態（例如 UI 的「擷取目前姿態成 pose」功能）
timeout 5 ros2 topic echo --once /joint_states

# 查看模式清單與參數
timeout 5 ros2 param get /switch_group_controller modes
timeout 5 ros2 param get /switch_group_controller arm_demo.file
```

用 Python（rclpy）送 action 的範例：

```python
from rclpy.action import ActionClient
from graphen_interfaces.action import SwitchMode

client = ActionClient(node, SwitchMode, '/switch_group_controller/switch_mode')
goal = SwitchMode.Goal(mode='arm_demo',
                       on_same_mode=SwitchMode.Goal.ON_SAME_MODE_RESTART_IF_FINISHED,
                       wait_until_finished=True)
future = client.send_goal_async(goal, feedback_callback=lambda fb: print(fb.feedback.elapsed))
# goal_handle.accepted == False → 模式不存在或 controller 未啟動
# result.success == False → 看 result.message（不可中斷、被取代、失敗）
```

---

## 6. 開發 UI 時的注意事項

### 6.1 目前架構的限制

1. **腳本只在 configure 時載入。** 修改腳本檔之後，必須重新載入 controller（目前做法是重啟 bringup），新內容才會生效。播放中改檔不會有任何影響。
2. **模式必須事先寫在 controllers.yaml 裡。** UI 不能在執行時動態新增一個腳本來播放。

如果 UI 需要「編輯後立即試播」，可以考慮以下擴充方向（都需要另外改 controller，可以再討論）：

- 加一個「重新載入腳本」的 service
- 加一個通用的編舞模式，在 goal 中指定腳本路徑

### 6.2 建議 UI 實作的檢查（與 controller 一致，可以在存檔前就提示）

- 同一條軌道內 `t` 嚴格遞增，而且 ≥ 0
- 一個 joint 只能屬於一條軌道
- `pose` 必須包含軌道的全部 joint；`positions` 必須剛好是軌道的全部 joint
- 每個值都在 URDF 的位置限位內
- 每一段的峰值速度 `|Δ|/T × 係數（1 / 1.5 / 1.875）`：
  - ≤ URDF velocity × `velocity_limit_scale`
  - ≤ `max_steps × planner_rate`
- 最後再呼叫 `choreo_check` 做一次權威檢查，用回傳值判斷是否通過

### 6.3 行為上容易誤會的地方

- 第一個 keyframe 寫在 `t: 0` 時，必須已經在那個姿態，否則不會動。接續上一個動作的腳本，要嘛先播一個「回到起始姿態」的腳本，要嘛把第一個 keyframe 的 `t` 設成大於 0。
- `poses` 裡沒有被任何軌道使用的 joint **不會被檢查**，打錯字也不會報錯。
- 不在任何軌道中的 joint 保持不動，**不會**被帶到姿態裡寫的值。
- `positions` 和 `pose` 的規則不同：前者必須剛好是軌道的 joint，後者可以多出其他 joint。
- action 的 cancel 只會停止等待，不會停止動作。UI 的「停止」按鈕要送出 `stop` 模式。
- `~/mode` topic 收不到成功或失敗的回應，UI 請用 action。
- 速度平滑化只作用在第一段，而且只對 cubic / quintic 有效。如果想要從移動中平順地接過來，第一段請不要用 linear，也不要放在 `t: 0`。
- 「播完」的判斷依據是命令位置，不代表實機已經到位。

### 6.4 安全

- 新的腳本請依序在 mock → MuJoCo（`hardware:=mujoco_pd mujoco_fixed_base:=true`，`robot` 預設 ti5_170a）上驗證，最後才上實機。
- 上實機前用 `velocity_limit_scale`（例如 0.25）壓低速度上限。
- 姿態的方向與碰撞未經實機確認的腳本，加上 `TODO(verify-on-hw)` 註解。
- 表演中不可以被打斷的段落設定 `interruptible: false`；`stop` 仍然可以隨時切入。
