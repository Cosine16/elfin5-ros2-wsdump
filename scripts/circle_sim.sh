#!/usr/bin/env bash
# =====================================================================
# circle_sim.sh —— 一条命令跑圆周运动仿真（elfin5 + cos_shape）
#
# 流程：
#   1. 检测 elfin5 仿真是否已启动，没启动则调用 start_sim.sh 启动 moveit2
#      （含 Gazebo + MoveIt2 + RViz）
#   2. 等待 move_group 就绪
#   3. 在当前终端前台启动 circle_motion 节点（Ctrl+C 退出）
#
# 用法:
#   ./circle_sim.sh
#
# 启动后节点处于等待状态，需要 enable 才会动：
#   - 图形面板: ros2 run cos_shape circle_panel.py
#   - 或命令行: ros2 topic pub --once /circle_motion/enable std_msgs/msg/Bool "{data: true}"
# 注意机械臂需先摆到"法兰大致朝下"的姿态（见 cos_shape/README.md），
# 否则从零位直接 enable 会因目标姿态不可达而失败。
#
# 环境变量:
#   SIM_WS=<path>   覆盖工作空间路径（默认 <repo>/.build/apps，overlay 已串接 .build/vendor）
# =====================================================================
set -euo pipefail

export ROS_LOCALHOST_ONLY=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SIM_WS:-${APP_BUILD_DIR:-$REPO_ROOT/.build/apps}}"
SETUP="$WS/install/setup.bash"

if [ ! -f "$SETUP" ]; then
  echo "错误: 找不到 $SETUP" >&2
  echo "请先构建工作空间: bash $REPO_ROOT/scripts/build_apps.sh" >&2
  exit 1
fi
if [ ! -d "$WS/install/cos_shape" ]; then
  echo "错误: cos_shape 未构建，请先: bash $REPO_ROOT/scripts/build_apps.sh" >&2
  exit 1
fi

# 1) 仿真
if pgrep -f "ros2 launch elfin5_ros2_moveit2" >/dev/null 2>&1; then
  echo "[circle_sim] 检测到 elfin5 仿真已在运行，复用。"
else
  echo "[circle_sim] 启动 elfin5 仿真 (moveit2: Gazebo + MoveIt2 + RViz) ..."
  "$SCRIPT_DIR/start_sim.sh" moveit2
fi

# 2) 等 move_group 就绪
# colcon 生成的 setup.bash 引用了未定义的 COLCON_TRACE，source 时需临时关闭 nounset
set +u
source "$SETUP"
set -u
bash "$SCRIPT_DIR/wait_for_ros.sh" move_group node move_group 180

# 3) 圆周运动节点（前台运行，日志直接打在本终端）
echo "[circle_sim] 启动 circle_motion 节点（等待 enable，Ctrl+C 退出）..."
exec ros2 launch cos_shape circle_motion.launch.py
