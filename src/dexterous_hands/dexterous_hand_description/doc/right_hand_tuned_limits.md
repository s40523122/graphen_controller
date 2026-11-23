# 右手關節限制：ti5 調校版紀錄

本套件的 `urdf/right_hand.xacro` 採用**原版**限制，這也與 `dexterous_hand_hardware` 的轉換表 `joint_conversions_` 一致。

舊工作區的 `ti5_170a_graphen/urdf/hands/right_hand.urdf.xacro` 曾是 MoveIt 實際使用的右手模型。它在原版基礎上修改了以下內容，數值看起來是在實機上逐一量測並收緊的上限。左手沒有修改。若之後要回到 ti5 版，依下表修改 `urdf/right_hand.xacro` 對應關節的 `<axis>` 和 `<limit>` 即可。

- 來源：`~/NY_graphen/src/ti5_170a_graphen/urdf/hands/right_hand.urdf.xacro`
- 原版備份（與本套件相同）：`~/NY_graphen/src/ti5_170a_graphen/urdf/hands/right_hand_origin.urdf.xacro`

| 關節 | 原版 axis | 原版 limit [rad] | ti5 版 axis | ti5 版 limit [rad] |
|---|---|---|---|---|
| `right_thumb_mcp_joint` | `0 0 1` | [0, 0.6108652] (0° ~ 35°) | **`0 0 -1`** | [0, 0.4537856] (0° ~ 26°) |
| `right_thumb_pip_joint`（mimic thumb_mcp ×1.0853） | `0 0 1` | [0, 1.041888] (0° ~ 59.7°) | `0 0 1` | [0, 0.9424778] (0° ~ 54°) |
| `right_index_yaw_joint` | `0 0 1` | [-0.26118, 0.26118] (±15°) | `0 0 1` | [-0.2617994, 0.2617994] (±15°) |
| `right_index_mcp_joint` | `0 0 -1` | [0, 1.3089969] (0° ~ 75°) | `0 0 -1` | [0, 1.1693706] (0° ~ 67°) |
| `right_middle_yaw_joint` | `0 0 1` | [-0.26118, 0.26118] (±15°) | `0 0 1` | [-0.2617994, 0.2617994] (±15°) |
| `right_middle_mcp_joint` | `0 0 -1` | [0, 1.3089969] (0° ~ 75°) | `0 0 -1` | [0, 1.2217305] (0° ~ 70°) |
| `right_middle_pip_joint` | `0 0 -1` | [0, 1.4835299] (0° ~ 85°) | `0 0 -1` | [0, 1.3089969] (0° ~ 75°) |
| `right_ring_mcp_joint` | `0 0 -1` | [0, 1.3089969] (0° ~ 75°) | `0 0 -1` | [0, 1.1868239] (0° ~ 68°) |
| `right_ring_pip_joint` | `0 0 -1` | [0, 1.4835299] (0° ~ 85°) | `0 0 -1` | [0, 1.3089969] (0° ~ 75°) |
| `right_pinky_mcp_joint` | `0 0 -1` | [0, 1.3089969] (0° ~ 75°) | `0 0 -1` | [0, 1.2042772] (0° ~ 69°) |
| `right_pinky_pip_joint` | `0 0 -1` | [0, 1.4835299] (0° ~ 85°) | `0 0 -1` | [0, 1.2566371] (0° ~ 72°) |

注意事項：

- yaw 關節的差異只是精度不同（0.26118 與 0.2617994 都約等於 15°），不是實際調整。
- 不使用 MoveIt 時，JointTrajectoryController 不會依 URDF 限制截斷指令，所以這些限制主要影響 RViz 顯示和 joint_state_publisher_gui 的滑桿範圍。
- 硬體端 rad 與原始值的換算範圍寫死在 `dexterous_hand_hardware/src/dexterous_hand_hardware.cpp` 的 `joint_conversions_`（原版範圍）。若要真的限制實機行程，需要在那裡處理。
