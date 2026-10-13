#!/usr/bin/env bash
# 在主機上顯示 graphen_mujoco 的模擬畫面（不需要網路，不需要 ROS）
#   bash mujoco_viewer.sh                                    # 浮動基座模型、port 9870
#   bash mujoco_viewer.sh --model ../models/t170_fixed.xml   # 固定基座（mujoco_fixed_base:=true）
# 視窗內按 End 重置模擬
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-$HOME/learnWalk_env/bin/python}"   # 已含 mujoco 與 glfw

# 從 SSH 執行時，預設把視窗送到本機 HDMI 螢幕（Xorg :1）
if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:1
  export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
fi

exec "$PY" -u "$HERE/mujoco_viewer.py" "$@"
