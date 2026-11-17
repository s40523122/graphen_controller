# 編舞（graphen_choreographer）開發參考

給「編舞編輯 / 播放 UI」開發用的參考文件，涵蓋以下內容：

- 整體流程：編輯、試播、發佈、表演
- 腳本格式與語意
- 機器人端的設定與檢查
- 控制介面：action、service、topic、BT
- 指令範例
- 注意事項

設計背景見 [ARCHITECTURE.md](ARCHITECTURE.md) §2.2 與 §2.12；程式的權威定義在 `graphen_choreographer/choreography.hpp`、`interpolation.hpp`、`choreographer_algorithm.cpp`，介面定義在 `graphen_interfaces`（`action/PlayChoreo.action`、`msg/Playback*.msg`、`srv/*Choreo*.srv`）。

---

## 1. 整體運作

```
UI（編輯區：腳本的來源）
  ├─ 編輯 poses / tracks，自己保存
  ├─ 試播：play_choreo {content: <YAML>}        機器人檢查後播放，不存檔
  └─ 發佈：publish_choreo {name, content}       機器人檢查後寫入舞曲庫（同名覆寫）
                                                      │
機器人（正式區）                                       ▼
  舞曲庫 <choreo.library_path>/<name>.choreo.yaml ──→ play_choreo {name}（BT / 表演只用名稱）
                                                      │
  LocomotionPlanner（100 Hz）每週期取樣編舞 → 位置 + 速度前饋 → 子群組 controller → 馬達
```

- **編輯以 UI 為主**。機器人只保存「發佈過的版本」，不在機器人端修改；方向固定是 UI → 機器人。
- **試播和正式播放分開**：試播帶的是內容本身，播完就丟掉；表演（BT）只能依名稱播放已發佈的舞曲，所以表演不依賴 UI 在線。
- **試播和發佈用同一套檢查**（§3.2）。不通過時不播放、不寫入，原因直接回傳。
- 腳本裡有多條**軌道（track）**，每條軌道控制自己的一組 joint，並有自己的 keyframe；所有軌道共用同一個時鐘，時間可以重疊。
- 編舞是**開環**的：只照時鐘輸出，不等待實際位置。某個 joint 跟不上，也不會拖慢其他軌道。
- 播完之後保持最後的姿態，不支援 loop。要重播就再送一次指令，見 §4.3 的 `on_same`。
- 編舞只是四種可以播放的東西之一（另外有系統模式、BVH、policy），它們共用同一套切換規則、可以互相取代；停止用系統模式 `stop`。

---

## 2. 腳本格式

腳本是一份 YAML 文字，單位一律為 rad 與秒。UI 自己保存編輯中的腳本；交給機器人時整份當成字串傳送（試播或發佈，見 §4）。範例在 `src/graphen_bringup/motions/*.choreo.yaml`。

```yaml
version: 1                       # 必填，目前只有 1
default_interp: quintic          # 選填：linear / cubic / quintic，預設 quintic
interruptible: false             # 選填：播放中不可被其他播放打斷（stop 除外），預設 true

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
| 頂層 | `interruptible` | | 只接受 `true` / `false`，預設 `true`。`false` 時這段編舞播放中不可被打斷，見 §4.4 |
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
- `interruptible` 不是 `true` / `false`

另外，**`poses` 裡出現的每個 joint 名稱都必須是機器人的 joint**，即使沒有被任何軌道使用。這項檢查在機器人端進行（播放或發佈時），`choreo_check --urdf` 也會檢查；全身姿態只取一部分時，其餘名稱打錯字也會被發現。

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

位置與速度限位以 URDF 為準（`ti5_170a_graphen`），UI 可以讀取 `/robot_description` 取得。軌道可以只包含一部分 joint（例如只有 `NECK_Y`）。

---

## 3. 機器人端的設定與檢查

### 3.1 controllers.yaml 的 `choreo` 區塊

編舞的內容不寫在 `controllers.yaml`；這裡只有機器人端的設定，**所有編舞共用，腳本無法放寬**：

```yaml
switch_group_controller:
  ros__parameters:
    choreo:
      plugin: graphen_choreographer/ChoreographerAlgorithm   # 空字串：不啟用編舞
      library_path: ~/.local/share/graphen/choreo            # 舞曲庫目錄
      start_tolerance: 0.05        # rad，預設 0.05
      velocity_limit_scale: 0.25   # 速度上限 = URDF velocity × 這個值，預設 1.0
      entry_velocity: true         # 預設 true，見 §2.3
      max_steps: [0.02 ×14 (手臂), 0.005 ×18 (其他)]   # 32 個值，每週期最大移動量（rad）
```

`max_steps` 必須容得下腳本的峰值速度：`峰值速度 / planner_rate ≤ max_steps`，`planner_rate` 預設為 100 Hz。

- 編舞是開環的，一旦被限速，時間就會被拉長，所以超過時直接拒絕。
- 沒寫 `choreo.max_steps` 時，使用 `planner_max_steps`（預設 0.005，等於 0.5 rad/s）。這對大部分動作都太小。
- UI 可以用 `ros2 param get /switch_group_controller choreo.max_steps` 等指令讀到這些值，據此在編輯時就提示。

### 3.2 播放前 / 發佈前的檢查（任何一項失敗 → 不播放、不寫入）

- 腳本格式（§2.1）
- 腳本的 joint 都是受控的 joint；`poses` 裡的 joint 名稱也都存在
- 每個 keyframe 都在 URDF 的位置限位內
- 每一段的峰值速度不超過 URDF velocity × `velocity_limit_scale`
- 每一段的峰值速度不會被 `max_steps` 限速

失敗時 `play_choreo` 的 goal 為 aborted、`publish_choreo` 的 `success` 為 false，`message` 就是原因，例如：

```
choreography 'show_1' cannot be played as written:
  - track 'right_arm' joint 'R_ELBOW_Y': peak velocity 0.438 rad/s needs 0.00438 rad per cycle,
    but max_steps allows 0.00500 (set it to at least 0.00525, or slow the choreography down)
```

```
choreography 'show_1': poses use 'R_ELBOW_YY', which is not a commanded joint
```

這時還沒有切換，機器人繼續原本的動作。

### 3.3 進入時的檢查（失敗 → 已經切換，但不執行、保持不動）

「起點 → 第一個 keyframe」這一段取決於當下的位置，所以只能在開始播放的那一刻檢查，**發佈時檢查不到**：

| 情況 | controller log 的訊息（節錄） |
|---|---|
| 第一個 keyframe 在 `t: 0`，但距離超過 `start_tolerance` | `is 0.6000 rad away from the first keyframe at t = 0 (start_tolerance 0.0500); move there first or give the keyframe a time` |
| 走過去的速度超過上限 | `reaching the keyframe at t = 1.00 s from here needs ... rad/s, above the limit ...` |
| 走過去會被 `max_steps` 限速 | `... needs ... rad per cycle, but max_steps allows ...` |
| 帶入速度的曲線超出位置限位 | `the move to the first keyframe (carrying ... rad/s ...) reaches [...], outside the limits [...]` |

失敗時：

- 編舞已經取代了原本的動作，但所有 joint 保持不動。
- 用 `wait_until_finished` 送出的 goal 會 abort，訊息為 `[ choreo:<name> ] failed, see the controller log`；詳細原因只在 controller 的 log。
- **不等待的 goal 這時已經回報成功**（`Switched`）。UI 想知道有沒有真的開始播，要用 `wait_until_finished`。

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
| `/switch_group_controller/play_choreo` | action `graphen_interfaces/action/PlayChoreo` | **播放編舞**：依名稱，或帶內容試播 |
| `/switch_group_controller/publish_choreo` | service `graphen_interfaces/srv/PublishChoreo` | 發佈到舞曲庫 |
| `/switch_group_controller/list_choreos` | service `graphen_interfaces/srv/ListChoreos` | 列出已發佈的名稱與內容的 sha256 |
| `/switch_group_controller/get_choreo` | service `graphen_interfaces/srv/GetChoreo` | 讀回已發佈的內容 |
| `/switch_group_controller/switch_mode` | action `graphen_interfaces/action/SwitchMode` | 切換系統模式；UI 的「停止」用它切到 `stop` |
| `/switch_group_controller/states` | topic `control_msgs/msg/DynamicJointState`（50 Hz） | 各關節的 state interface，用來顯示即時姿態 |
| `/joint_states` | topic `sensor_msgs/msg/JointState` | 同上，標準格式 |
| `/robot_description` | topic `std_msgs/msg/String` | URDF：關節限位 |

沒有設定 `choreo.plugin` 時，`play_choreo` 和三個 service 都不會出現。

### 4.1 PlayChoreo action

```
# Goal
string name                      # 舞曲庫中已發佈的名稱
string content                   # 或：腳本的 YAML 內容（試播，不存檔）。兩者擇一
PlaybackOptions options
  int8 on_same 0                 #   0 IGNORE / 1 RESTART / 2 RESTART_IF_FINISHED（§4.3）
  bool wait_until_finished false #   true：等播完才回傳
---
# Result
PlaybackResult result
  bool     success
  string   message
  string   mode                  #   回傳當下實際在播的類型：choreo / bvh / policy，或系統模式名稱（如 stop）
  string   item                  #   項目名稱：舞曲的名稱，試播為 "(preview)"；系統模式為空
  string[] interface_names       #   當下的 state 快照
  float64[] values
---
# Feedback（只在 wait_until_finished 時持續發布，頻率為 switch_mode_poll_rate，預設 20 Hz）
PlaybackFeedback feedback
  uint8   stage                  #   0 LOADING / 1 PLAYING（編舞不需要載入時間，只會看到 PLAYING）
  float64 elapsed                #   開始播放後經過的秒數
```

`PlaybackOptions` / `PlaybackResult` / `PlaybackFeedback` 是所有播放類 action（`SwitchMode`、`PlayChoreo`、`PlayBvh`、`RunPolicy`）共用的 msg，UI 處理結果的程式可以共用。

**goal 被拒絕（REJECTED，沒有 result）的情況：**

- controller 不在 active 狀態
- `name` 和 `content` 沒有擇一（都給或都沒給）
- `name` 不符合命名規則（§4.2）
- `on_same` 超出 0–2

**結果一覽：**

| 情況 | 狀態 | `message` |
|---|---|---|
| 切換成功，不等待 | SUCCEEDED | `Switched` |
| 已經在播同一段，沒有重播 | SUCCEEDED | `Already playing this, not restarted` |
| 等待後播完 | SUCCEEDED | `Finished` |
| 名稱沒有發佈 | ABORTED | `choreography '<name>' is not published` |
| 內容沒有通過檢查（§3.2） | ABORTED | 檢查的錯誤訊息 |
| 目前在播的不可中斷 | ABORTED | `[ X ] is not interruptible until it finishes; [ Y ] rejected.` |
| 等待中被其他播放取代 | ABORTED | `Preempted by [ Z ]` |
| 進入時的檢查失敗（§3.3） | ABORTED | `[ choreo:<name> ] failed, see the controller log` |
| 等待中 controller 被停用 | ABORTED | `Controller is not active` |
| 客戶端 cancel | CANCELED | `Stopped waiting; the motion keeps running` |

前三種失敗（沒有發佈、沒通過檢查、不可中斷）發生時**還沒有切換**，`result.mode` / `item` 仍是原本在播的。

> **cancel 只會停止等待，不會停止動作。** 要停止動作，請用 `switch_mode` 切換到 `stop`。

**進度條：** `elapsed / duration`。`duration` 由 UI 從腳本自行算出（所有軌道最後一個 `t` 的最大值），action 不會回傳 duration。

### 4.2 舞曲庫的 service

**命名規則**：名稱只允許 `[a-z0-9_-]`，1 ~ 64 個字元（會直接當成檔名）。

```
# PublishChoreo
string name
string content
---
bool success
string message        # 失敗時為原因（命名規則、§3.2 的檢查…）
string sha256         # 已發佈內容的 SHA-256（十六進位小寫）

# ListChoreos
---
string[] names        # 依名稱排序
string[] sha256       # 與 names 一一對應

# GetChoreo
string name
---
bool success
string message
string content
string sha256
```

- **同名發佈直接覆寫**，不保留歷史版本；沒有刪除介面（要移除只能到機器人上刪除 `<library_path>/<name>.choreo.yaml`）。
- 發佈只寫檔案，不會讓機器人動作。
- **重新發佈不影響正在播的那一次**；下一次播放才會使用新內容。
- **同步狀態**：`sha256` 是內容位元組的 SHA-256。UI 對本機的腳本算同樣的 hash，和 `list_choreos` 比對，就知道「已發佈、未變更」還是「有修改、尚未發佈」。controller 每次播放也會在 log 記下內容的 hash（前 12 碼），例如 `Change to [ choreo:show_1 ], published, sha256 5e32a6b4e584`。

### 4.3 `on_same`：已經在播同一段時的處理

這個設定**只影響這一次指令**，不會留給之後的指令。

| 值 | 名稱 | 播放中 | 已播完 |
|---|---|---|---|
| 0 | IGNORE（預設） | 不重播 | 不重播 |
| 1 | RESTART | 從頭重播（算是中斷，受不可中斷的限制） | 重播 |
| 2 | RESTART_IF_FINISHED | 不重播（吃掉重複送出的指令） | 重播 |

**「同一段」= 名稱相同而且內容相同**：

- 重新發佈之後，同名的舞曲被視為不同的內容，即使是 IGNORE 也會切換過去（受不可中斷的限制）。
- 試播以內容判斷：同樣的內容再送一次才算同一段。

UI 的建議：

- 「播放」按鈕用 **2**，按很多次也不會讓動作從頭開始抖動。
- 「重新播放」按鈕用 **1**。

`wait_until_finished: true` 搭配「不重播」時，等待的是目前正在播放的那一次。如果那一次已經播完，會立即回傳成功。

### 4.4 中斷規則

不是同一段時一律切換，不論對方是另一段編舞、系統模式、BVH 還是 policy。例外是「不可中斷」：

| 目前在播的 | 新的指令 | 結果 |
|---|---|---|
| 可中斷（預設） | 任何播放 | 切換 |
| 不可中斷，**播放中** | `preempt` 的系統模式（`stop` 等 hold） | 切換：停止永遠有效 |
| 不可中斷，**播放中** | 其他任何播放（包括自己帶 RESTART） | **拒絕**（ABORTED） |
| 不可中斷，已播完或已失敗 | 任何播放 | 切換 |

- **編舞的不可中斷寫在腳本裡**：頂層的 `interruptible: false`，由編輯的人決定哪一段要保護。
- 透過 `~/mode` topic 切換系統模式被拒絕時，只會在 controller log 印出 WARN，送出端收不到任何回應。所以 UI 應該用 action。
- 被其他播放取代時，原本在等待的 goal 會被 abort（`Preempted by [ Z ]`）。

### 4.5 BT 節點 `PlayChoreo`

| port | 預設 | 說明 |
|---|---|---|
| `choreo` | — | 舞曲庫中的名稱（BT 不能帶內容試播） |
| `action_name` | `/switch_group_controller/play_choreo` | |
| `timeout_ms` | 1000 | 等待 server 與接受 goal 的時間上限 |
| `wait_finished` | false | 播完才回傳 SUCCESS |
| `on_same` | `ignore` | `ignore` / `restart` / `restart_if_finished` |
| `finish_timeout_ms` | 0 | `wait_finished` 時等待播完的上限，0 表示不限。超過時回傳 FAILURE，但動作不會停 |

halt 時會 cancel goal，只是停止等待，動作不會停。範例見 `graphen_bt/trees/choreo_demo.xml`。停止用 `SwitchMode mode="stop"`。

---

## 5. 指令範例

```bash
source install/setup.bash

# 離線檢查：範圍、峰值速度、建議的 max_steps、限位、姿態中的 joint 名稱、是否可中斷。
# 回傳值：0 通過、1 有誤、2 用法錯誤
xacro $(ros2 pkg prefix ti5_170a_graphen)/share/ti5_170a_graphen/urdf/ti5_170a_graphen.urdf.xacro > /tmp/t170.urdf
ros2 run graphen_choreographer choreo_check src/graphen_bringup/motions/arm_demo.choreo.yaml \
  --urdf /tmp/t170.urdf --velocity-scale 0.25 --rate 100 --csv /tmp/arm_demo.csv
# CSV：time + 每個 joint 一欄。假設從第一個姿態、靜止開始，可以拿來預覽曲線

# 發佈、列出、讀回（需要 bringup 已啟動）
ros2 run graphen_bringup choreo_publish arm_demo src/graphen_bringup/motions/arm_demo.choreo.yaml
ros2 run graphen_bringup choreo_publish --list
ros2 run graphen_bringup choreo_publish --get arm_demo

# 依名稱播放（等待播完、顯示進度）
timeout 30 ros2 action send_goal --feedback /switch_group_controller/play_choreo \
  graphen_interfaces/action/PlayChoreo "{name: arm_demo, options: {on_same: 2, wait_until_finished: true}}"

# 從頭重播
timeout 10 ros2 action send_goal /switch_group_controller/play_choreo \
  graphen_interfaces/action/PlayChoreo "{name: arm_demo, options: {on_same: 1}}"

# 試播（直接帶內容，不存檔）：頸部在 1.5 s 內轉到 0.2 rad
timeout 10 ros2 action send_goal /switch_group_controller/play_choreo graphen_interfaces/action/PlayChoreo \
  "{content: 'version: 1
tracks:
  head:
    joints: [NECK_Y]
    keyframes:
      - {t: 1.5, positions: {NECK_Y: 0.2}}
', options: {wait_until_finished: true}}"

# 停止（任何時候都有效）
timeout 10 ros2 action send_goal /switch_group_controller/switch_mode \
  graphen_interfaces/action/SwitchMode "{mode: stop}"

# 讀取目前的姿態（例如 UI 的「擷取目前姿態成 pose」功能）
timeout 5 ros2 topic echo --once /joint_states

# 查看機器人端的上限
timeout 5 ros2 param get /switch_group_controller choreo.max_steps
timeout 5 ros2 param get /switch_group_controller choreo.velocity_limit_scale
```

用 Python（rclpy）試播、發佈、播放的範例：

```python
import hashlib
from rclpy.action import ActionClient
from graphen_interfaces.action import PlayChoreo
from graphen_interfaces.msg import PlaybackOptions
from graphen_interfaces.srv import ListChoreos, PublishChoreo

play = ActionClient(node, PlayChoreo, '/switch_group_controller/play_choreo')

# 試播：帶內容
goal = PlayChoreo.Goal(content=script_text)
goal.options.wait_until_finished = True
future = play.send_goal_async(goal, feedback_callback=lambda fb: print(fb.feedback.feedback.elapsed))
# goal_handle.accepted == False → name / content 沒有擇一、名稱不合規則，或 controller 未啟動
# result.result.success == False → 看 result.result.message（檢查不通過、不可中斷、被取代、失敗）

# 發佈
publish = node.create_client(PublishChoreo, '/switch_group_controller/publish_choreo')
response = publish.call_async(PublishChoreo.Request(name='show_1', content=script_text))
# response.success == False → response.message 是原因

# 同步狀態：本機內容的 hash 和機器人上已發佈的比對
local_sha = hashlib.sha256(script_text.encode('utf-8')).hexdigest()
listing = node.create_client(ListChoreos, '/switch_group_controller/list_choreos')

# 正式播放：依名稱
goal = PlayChoreo.Goal(name='show_1')
goal.options.on_same = PlaybackOptions.ON_SAME_RESTART_IF_FINISHED
```

完整的互動範例（送出、等待、取消、比對結果）可以參考 `graphen_bringup/scripts/playback_scenarios.py`。

---

## 6. 開發 UI 時的注意事項

### 6.1 目前的限制

1. **舞曲庫沒有刪除與歷史版本**：同名發佈直接覆寫。UI 自己要保存編輯歷史；要移除機器人上的舞曲只能手動刪檔案。
2. **機器人端的上限對所有編舞一致**（`choreo.*`）。某一段需要更快的動作時，要由控制工程師調整 `controllers.yaml` 並重啟，UI 不能放寬。
3. **表演進行中重新發佈**：正在播的那一次不受影響，但下一次播放就是新內容，沒有「凍結節目」的機制。表演期間避免發佈。
4. **進入時才發現的失敗只在 controller 的 log**（§3.3），result 只說明失敗。
5. **不能預先查詢 BVH 與 policy 的清單**：編舞有 `list_choreos`，另外兩種目前沒有對應的介面。

### 6.2 建議 UI 實作的檢查（與 controller 一致，可以在送出前就提示）

- 同一條軌道內 `t` 嚴格遞增，而且 ≥ 0
- 一個 joint 只能屬於一條軌道
- `pose` 必須包含軌道的全部 joint；`positions` 必須剛好是軌道的全部 joint
- `poses` 裡的每個 joint 名稱都是機器人的 joint
- 每個值都在 URDF 的位置限位內
- 每一段的峰值速度 `|Δ|/T × 係數（1 / 1.5 / 1.875）`：
  - ≤ URDF velocity × `choreo.velocity_limit_scale`
  - ≤ `choreo.max_steps × planner_rate`
- 最終以機器人端的檢查為準：發佈或試播失敗時，把 `message` 顯示給使用者

### 6.3 行為上容易誤會的地方

- 第一個 keyframe 寫在 `t: 0` 時，必須已經在那個姿態，否則不會動。接續上一個動作的腳本，要嘛先播一個「回到起始姿態」的腳本，要嘛把第一個 keyframe 的 `t` 設成大於 0。
- 發佈成功不代表從任何位置都能播：從目前位置到第一個姿態的那一段是開始播放時才檢查的（§3.3）。
- 不等待的 goal 回報 `Switched` 只代表已經切換；有沒有真的開始動、有沒有播完，要用 `wait_until_finished`。
- 不在任何軌道中的 joint 保持不動，**不會**被帶到姿態裡寫的值。
- `positions` 和 `pose` 的規則不同：前者必須剛好是軌道的 joint，後者可以多出其他 joint（但名稱必須存在）。
- action 的 cancel 只會停止等待，不會停止動作。UI 的「停止」按鈕要用 `switch_mode` 送出 `stop`。
- 試播的內容不會被保存；`result.item` 為 `(preview)`。
- 重新發佈之後，同名的舞曲是「不同的內容」：`on_same: 0` 也會切換過去。
- 速度平滑化只作用在第一段，而且只對 cubic / quintic 有效。如果想要從移動中平順地接過來，第一段請不要用 linear，也不要放在 `t: 0`。
- 「播完」的判斷依據是命令位置，不代表實機已經到位。

### 6.4 安全

- 新的腳本請依序在 mock → MuJoCo（`hardware:=mujoco_pd mujoco_fixed_base:=true`，`robot` 預設 ti5_170a）上驗證，最後才上實機。
- 試播會讓機器人立刻動作，而且任何能連到 ROS 網路的程式都可以送出；目前沒有「排練模式」的開關。對實機試播前確認周圍安全。
- 上實機前用 `choreo.velocity_limit_scale`（例如 0.25）壓低速度上限。
- 姿態的方向與碰撞未經實機確認的腳本，加上 `TODO(verify-on-hw)` 註解。
- 表演中不可以被打斷的段落在腳本寫 `interruptible: false`；`stop` 仍然可以隨時切入。
