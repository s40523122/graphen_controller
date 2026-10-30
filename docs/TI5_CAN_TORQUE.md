# Ti5 實機扭矩模式（`hardware:=real_pd`）

`Ti5Hardware` 的 effort 模式：主機端 `PdController` 以 200 Hz 計算力矩，`Ti5Hardware` 換成電流後以 CAN 電流模式送給馬達。
用途是讓實機的關節控制與 RL 訓練環境（IsaacLab `DelayedPDActuator`：顯式 PD、200 Hz、力矩零階保持）相同。
協議來源：鈦虎《C1 关节模组通讯使用说明》1.1.1 第 7 章。

> **尚未在實機驗證。** 標 `TODO(verify-on-hw)` 的項目都要先在吊掛、低增益下確認，未經同意不要以 `hardware:=real_pd` 啟動。

## 為什麼不用驅動器內的 PT（力位混合）模式

- 訓練的 PD 是 200 Hz 顯式計算，主機端 200 Hz PD 與它同構；驅動器內的 PT 是 kHz 級的連續 PD，反而與訓練不同。
- CAN 協議沒有公開 PT 8 位元組指令的打包格式，已知的 PT 指令（`0x5A`、7 位元組 `0x02`）無法設定 KP / KD / 前饋力矩。
- 主機端 PD 可以每週期改增益（模式切換），限幅也在軟體中。

## 使用的 CAN 指令

標準幀，ID = 馬達 CAN ID，多位元組數值皆為 little-endian。

| 指令 | 長度 | 用途 | 時機 |
|---|---|---|---|
| `41 40 00 00 00 00` | 6 | 讀減速比（int32） | configure（real / real_pd 都讀，速度換算需要） |
| `44 40 00 00 00 00` | 6 | 讀 KT（float，電機端 Nm/A） | configure（real_pd） |
| `08` | 1 | 讀位置（cnt） | activate |
| `1E` + int32 | 5 | 位置模式目標（cnt） | activate 時的位置保持、故障退回 |
| `41` | 1 | 回覆 8 位元組：電流 int16 mA、速度 int16 0.01 Hz（電機端）、位置 int32 cnt | 每週期 read() |
| `0A` | 1 | 錯誤碼（每週期每條通道輪詢一顆） | 每週期 read() |
| `1C` + int32 | 5 | 目標電流（mA），切換到電流模式 | 每週期 write() |

不寫入任何馬達參數（不送 `0x20`/`0x21` 電流上限、`0x59` 掉線保護、`0x0E` 存 FLASH）。

## 單位換算

- 位置：`q = cnt / cpr × 2π`（雙編外圈 cpr = 262144；單編 / 內圈 = 65536 × 減速比）
- 速度：`q̇ = v × 0.01 × 2π / ratio`（協議的速度一律是電機端 0.01 Hz）
- 力矩 → 電流：`I [mA] = τ / (KT × ratio) × 1000`（協議 7.5.2 的公式）
- effort state：`τ ≈ I × KT × ratio`（理想換算，未計減速器摩擦與效率）

## 每週期流程（200 Hz）

1. `read()`：所有通道先丟棄殘留 frame，送出全部馬達的 `0x41`（加上一顆的 `0x0A`），再以共用的 deadline（`hw_read_timeout_ms`，real_pd 預設 2 ms）收齊回覆。各通道的傳輸互相重疊；每條通道 3~4 顆馬達，約 1 ms。
2. `PdController::update()`：`τ = kp(q* − q) + kd(q̇* − q̇)`，限制在 `effort_limits`。
3. `write()`：以 URDF `<limit effort>` 再限制一次，換算電流後送出 `0x1C`。

PD 用的狀態只比 write 早約 1 ms（訓練的觀測沒有延遲）。

## 啟動與故障處理

- **activate**：讀取位置，以 `0x1E` 保持在目前位置。effort command 初始為 NaN；**所有 joint 都收到有限力矩**後才切換到電流模式（與 MujocoSystem 的「effort 收齊前暫停物理」相同的概念）。
- **退回位置保持**（`0x1E` 保持在最後量測位置，不去使能，避免直接軟倒），直到 lifecycle reset：
  - 啟動後收到 NaN / inf 力矩
  - 回饋連續缺漏超過 `max_missed_cycles`（預設 2）
  - deactivate
- 回饋缺漏未超過上限時沿用上一筆狀態。

## 參數

| 參數 | 來源 | 預設 | 說明 |
|---|---|---|---|
| `hw_read_timeout_ms` | launch / xacro | real 20、real_pd 2 | 收回覆的 deadline；real_pd 要遠小於 5 ms |
| `max_missed_cycles` | launch / xacro | 2 | 只用於 real_pd |
| `default_kp` / `default_kd` / `effort_limits` | `hardware/real_pd.yaml` | 訓練值 | 與 `mujoco_pd_t170a.yaml` 相同 |

## 待驗證（`TODO(verify-on-hw)`）

1. KT 的單位是電機端 Nm/A，`τ_out = I × KT × ratio` 成立；KT 的 float 為 IEEE 754 little-endian。
2. 電流正負號與 URDF 關節轉向一致。
3. 速度換算（手動轉動，比較 velocity 與位置微分）。
4. `0x41` 回覆在 1 Mbps、每通道 4 顆下能在 2 ms 內收齊。
5. 電流模式不受速度 / 加速度限制（協議 FAQ 14），而且通訊中斷時馬達維持最後電流：`0x59` 掉線保護的位元組格式與單位不明，尚未使用，需向原廠確認後加入。
6. 減速器摩擦造成的力矩落差是否需要補償。

建議的首次測試：吊掛、只開一個關節、降低增益，依序確認 1~3，再做小階躍，最後才開整條腿與全身。
