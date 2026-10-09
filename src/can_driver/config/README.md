# CAN 馬達設定說明

## 全域預設值（Encoder）

當使用 `encoder_type` + `reduction_ratio` 推算時，會用到以下預設值（可改）：

- **dual_outer**：雙精度編碼器輸出軸一圈計數，預設 `262144`
- **single_inner**：單精度編碼器馬達端一圈計數，單精度時輸出軸一圈 = `single_inner × reduction_ratio`，預設 `65536`

## 每顆馬達（motors[]）

每個 channel 的 `motors[]` 可依馬達類型任選一種寫法：

- **雙精度**（例：DualOuter）：多為 `262144`
- **單精度**（已知換算結果）：可填 `65536 × 減速比`，例如減速比 51 則填 `3342336`
- 其他馬達：依規格書填「輸出軸轉一圈對應的 encoder 計數」即可

## 編碼器類型 + 減速比（由程式換算）

**雙精度（DualOuter）**：輸出軸一圈 = 262144 cnt，不需減速比

```json
{ "name": "R_HIP_P", "encoder_type": "DualOuter" }
```

**單精度（SingleInner）**：輸出軸一圈 = 65536 × 減速比，需給減速比

```json
{ "name": "L_HIP_R", "encoder_type": "SingleInner", "reduction_ratio": 51 }
```

- `encoder_type` 可填 `"DualOuter"`、`"single_inner"` 或 `"SingleInner"`
- 未填 `counts_per_revolution` 時才會用 `encoder_type` + `reduction_ratio` 推算；有填則以 `counts_per_revolution` 為準
