#!/usr/bin/env bash
# 在主機上顯示機器人的姿態（MuJoCo 模型；MuJoCo 硬體的 udp 顯示不需要 ROS）
#   bash mujoco_viewer.sh                                    # MuJoCo 硬體：模型向 MujocoSystem 查詢、port 9870
#   bash mujoco_viewer.sh --model ../../robot_description/fb70/mjcf/t170_fixed.xml   # 固定基座（mujoco_fixed_base:=true）
#   bash mujoco_viewer.sh --source ros --model <MJCF>        # mock / real：顯示 /joint_states
# 視窗內按 End 重置模擬；Pause 切換手腕座標軸（x 紅、y 綠、z 藍）。controller 切到 teach 模式時自動進入
# 手臂示教，姿態發布在 /mujoco_viewer_teach/pose；--ghost-topic 疊加影子試播的軌跡（見 mujoco_viewer.py 開頭的說明）。
# 示教、--source ros 與幽靈需要 ROS 環境：先 source install/setup.bash，並以 PY 指定
# 能 import rclpy 的 python 3.10（見 README），例如 PY=~/teach_env/bin/python bash mujoco_viewer.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-$HOME/learnWalk_env/bin/python}"   # 已含 mujoco 與 glfw

# 從 SSH 執行時，預設把視窗送到本機 HDMI 螢幕（Xorg :1）
if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:1
  export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
fi

exec "$PY" -u "$HERE/mujoco_viewer.py" "$@"
