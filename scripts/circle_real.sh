#!/usr/bin/env bash
# =====================================================================
# circle_real.sh —— 一条命令跑真机圆周运动（elfin5 + cos_shape）
#
# 流程：
#   1. 调用 start_real.py 打开实机 4 个终端（硬件驱动 → MoveIt+RViz →
#      basic_api → Elfin Control Panel，自动做环境检查并按依赖顺序等待）
#   2. 等待 move_group 就绪
#   3. 在当前终端前台启动 circle_motion 节点（Ctrl+C 退出）
#
# 用法:
#   ./circle_real.sh                # 默认 elfin5，硬件驱动用 sudo（终端里输密码）
#   ./circle_real.sh --no-sudo      # 实机终端不用 sudo
#   ./circle_real.sh --check        # 只做实机环境检查，不启动任何东西
#
# 启动后还需要（不会自动动，安全默认）：
#   1. 在 Elfin Control Panel: Clear Fault → Servo On 使能
#   2. 确认末端姿态"法兰大致朝下"（圆周运动锁定 enable 时刻的姿态）
#   3. 再 enable 圆周节点：
#      - 图形面板: ros2 run cos_shape circle_panel.py
#      - 或命令行: ros2 topic pub --once /circle_motion/enable std_msgs/msg/Bool "{data: true}"
#
# 关闭机械臂电源前，请先在 Control Panel 按 "Servo Off" 去使能。
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

# 1) 实机终端（--check 模式只检查环境，不继续往下走）
if [ "${1:-}" = "--check" ]; then
  exec python3 "$SCRIPT_DIR/start_real.py" --check
fi
echo "[circle_real] 启动实机 4 个终端 (驱动 → MoveIt+RViz → basic_api → 面板) ..."
python3 "$SCRIPT_DIR/start_real.py" "$@"

# 2) 等 move_group 就绪（实机含姿态识别，给足超时）
# colcon 生成的 setup.bash 引用了未定义的 COLCON_TRACE，source 时需临时关闭 nounset
set +u
source "$SETUP"
set -u
bash "$SCRIPT_DIR/wait_for_ros.sh" move_group node move_group 300

# 3) 圆周运动节点（前台运行，日志直接打在本终端）
echo "[circle_real] 启动 circle_motion 节点（默认不自动运动，Ctrl+C 退出）..."
echo "[circle_real] 请先在 Control Panel 使能（Clear Fault → Servo On），再 enable 圆周节点。"
exec ros2 launch cos_shape circle_motion.launch.py
