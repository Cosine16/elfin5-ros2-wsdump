#!/usr/bin/env bash
# =====================================================================
# 启动 elfin5 仿真（MoveIt2 / Gazebo）
#
# 两种运行方式（自动选择）：
#   1. 有图形终端模拟器（gnome-terminal 等）：分别打开终端窗口
#   2. 无终端模拟器（如 Docker 容器）：以后台进程方式启动，
#      日志写入 $WS/log/sim/<名称>.log
#
# 用法:
#   ./start_sim.sh             同时启动 moveit2、basic_api 与 gazebo
#   ./start_sim.sh moveit2     只启动 moveit2
#   ./start_sim.sh basic_api   只启动 basic_api 后台程序
#   ./start_sim.sh gazebo      只启动 gazebo (Elfin Control Panel GUI)
#   ./start_sim.sh stop        停止后台方式启动的所有仿真进程
#
# 环境变量:
#   SIM_SUDO=1      以 `sudo -E` 执行仿真命令（默认不用 sudo；已是 root 自动忽略）
#   SIM_WS=<path>   覆盖工作空间路径（默认 <repo>/.build/apps，overlay 已串接 .build/vendor）
# =====================================================================
set -euo pipefail

# FastRTPS 只走回环网卡：本机 enp2s0 常处于断网/EtherCAT 状态，
# 默认 DDS 组播发现在该网卡上超时，会导致节点互相看不见、话题无数据。
# gnome-terminal 以 bash -c 非交互方式运行（不读 ~/.bashrc），必须在这里导出。
export ROS_LOCALHOST_ONLY=1

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SIM_WS:-${APP_BUILD_DIR:-$REPO_ROOT/.build/apps}}"
SETUP="$WS/install/setup.bash"
LOG_DIR="$WS/log/sim"

# sudo 处理：默认不使用 sudo（GUI 程序用 root 跑容易有显示问题）；
# 设置 SIM_SUDO=1 时才用 `sudo -E`；已是 root 时忽略。
SUDO=""
[ "${SIM_SUDO:-0}" = "1" ] && [ "$(id -u)" != "0" ] && SUDO="sudo -E"

need_setup() {
  if [ ! -f "$SETUP" ]; then
    echo "错误: 找不到 $SETUP" >&2
    echo "请先构建工作空间: bash $REPO_ROOT/scripts/build_apps.sh" >&2
    exit 1
  fi
}

# 选择一个可用的系统终端模拟器；没有则返回非 0（调用方走后台模式）
detect_terminal() {
  for t in gnome-terminal x-terminal-emulator xfce4-terminal konsole; do
    command -v "$t" >/dev/null 2>&1 && { echo "$t"; return 0; }
  done
  return 1
}

# 在新终端窗口运行一个 ros2 launch（失败时窗口停留 5 秒便于查看错误）
#   open_term <窗口标题> <包名> <launch文件>
open_term() {
  local name="$1" pkg="$2" launch="$3"
  local inner
  if [ -n "$SUDO" ]; then
    # sudo 会重置 PATH（secure_path），必须在 sudo 内部重新 source 才能找到 ros2
    inner="sudo -E bash -c 'source $SETUP && ros2 launch $pkg $launch' || { echo; echo '>>> 启动失败，5 秒后关闭'; sleep 5; }"
  else
    inner="source '$SETUP' && ros2 launch $pkg $launch || { echo; echo '>>> 启动失败，5 秒后关闭'; sleep 5; }"
  fi
  case "$TERM_CMD" in
    gnome-terminal)
      gnome-terminal --title="$name" -- bash -c "$inner" &
      ;;
    x-terminal-emulator)
      x-terminal-emulator --title="$name" -- bash -c "$inner" &
      ;;
    xfce4-terminal)
      xfce4-terminal --title="$name" -x bash -c "$inner" &
      ;;
    konsole)
      konsole --title "$name" -e bash -c "$inner" &
      ;;
  esac
  echo "已打开终端窗口: $name"
}

# 后台方式运行一个 ros2 launch（容器/无终端模拟器时使用）
#   open_bg <名称> <包名> <launch文件>
open_bg() {
  local name="$1" pkg="$2" launch="$3"
  mkdir -p "$LOG_DIR"
  local log="$LOG_DIR/$name.log"
  nohup bash -c "source '$SETUP' && exec ros2 launch $pkg $launch" >"$log" 2>&1 &
  echo "已后台启动: $name (pid=$!) 日志: $log"
}

# 启动一个仿真组件：有终端开窗口，没有则后台运行
launch_one() {
  if [ -n "$TERM_CMD" ]; then
    open_term "$@"
  else
    open_bg "$@"
  fi
}

stop_all() {
  echo "停止后台仿真进程..."
  pkill -f "ros2 launch elfin5_ros2_moveit2" 2>/dev/null || true
  pkill -f "ros2 launch elfin_basic_api" 2>/dev/null || true
  echo "完成（GUI 窗口方式启动的请到对应窗口 Ctrl+C）。"
}

TERM_CMD="$(detect_terminal)" || TERM_CMD=""
if [ -z "$TERM_CMD" ]; then
  echo "提示: 未找到图形终端模拟器，使用后台进程模式（日志在 $LOG_DIR/）"
fi

case "${1:-}" in
  moveit2)
    need_setup
    launch_one "moveit2" elfin5_ros2_moveit2 elfin5.launch.py
    ;;
  gazebo)
    need_setup
    launch_one "gazebo" elfin_basic_api fake_elfin_gui.launch.py
    ;;
  basic_api)
    need_setup
    launch_one "basic_api" elfin5_ros2_moveit2 elfin5_basic_api.launch.py
    ;;
  ""|all)
    need_setup
    launch_one "moveit2" elfin5_ros2_moveit2 elfin5.launch.py
    launch_one "basic_api" elfin5_ros2_moveit2 elfin5_basic_api.launch.py
    launch_one "gazebo" elfin_basic_api fake_elfin_gui.launch.py
    ;;
  stop)
    stop_all
    ;;
  *)
    cat <<'HELP'
用法:
  ./start_sim.sh             同时启动 moveit2、basic_api 与 gazebo
  ./start_sim.sh moveit2     只启动 moveit2
  ./start_sim.sh basic_api   只启动 basic_api 后台程序
  ./start_sim.sh gazebo      只启动 gazebo (Elfin Control Panel GUI)
  ./start_sim.sh stop        停止后台方式启动的仿真进程

有图形终端模拟器（gnome-terminal 等）时自动打开终端窗口；
否则（如 Docker 容器内）以后台进程运行，日志在 $SIM_WS/log/sim/。

环境变量:
  SIM_SUDO=1      以 sudo -E 执行仿真命令（默认不用）
  SIM_WS=<path>   覆盖工作空间路径（默认 <repo>/.build/apps，overlay 已串接 .build/vendor）
HELP
    ;;
esac
