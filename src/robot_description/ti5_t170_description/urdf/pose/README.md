# Ti5 arm zero pose (T / Q)

T 與 Q 的差別 = commit `10666da`（肩 `*_SHOULDER_R`、肘 `*_ELBOW_Y` 的 origin rpy + limit）。

| 檔案 | 意義 |
|------|------|
| `t_pose.xacro` | 機械零點（commit 前），macro `ti5_load_t_pose` |
| `q_pose.xacro` | Unitree retarget 零點（commit `10666da`），macro `ti5_load_q_pose` |

展開 / 轉 USD 時加 `-m pose:=t` 或 `-m pose:=q`。

```bash
cd /root/work/src/conver2usd/scripts

# 本體 — Q-pose
./convert_xacro_to_usd.sh \
  /root/work/src/robot_description/ti5_t170_description/urdf/T170-V2.1-A0.urdf.xacro \
  -o /root/work/src/robot_description/ti5_t170_description/usd/T170-V2.1-A0_q_pose.usd \
  -m use_ros2_control:=false \
  -m use_gazebo:=false \
  -m pose:=q \
  --floating-base \
  --keep-urdf

# 本體 + 靈巧手 — Q-pose
./convert_xacro_to_usd.sh \
  /root/work/src/robot_description/ti5_t170_description/urdf/ti5_with_dexterous_hand_real.urdf.xacro \
  -o /root/work/src/robot_description/ti5_t170_description/usd/T170-V2.1-A0_dexterous_hand_q_pose.usd \
  -m use_dexterous_hand:=true \
  -m use_ros2_control:=false \
  -m use_gazebo:=false \
  -m pose:=q \
  --floating-base \
  --keep-mimic \
  --keep-urdf
```
