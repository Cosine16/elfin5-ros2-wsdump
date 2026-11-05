#!/usr/bin/env zsh
# elfin_env.zsh —— elfin_env.sh 的 zsh 版 (用 source 加载, 不要直接执行)
#
# 用法:
#   source ~/elfin5_ctl/cos_ws/scripts/elfin_env.zsh
#   hand_eye_calib          # 加载后可直接用手眼标定快捷命令
#
# 与 bash 版一致: 按 humble -> .build/vendor -> .build/apps 显式 source 三层 +
# ROS_LOCALHOST_ONLY=1 (与 start_real.py / start_sim.py 保持一致)

local _ENV_DIR="${0:A:h}"
local _WS="$_ENV_DIR/.."

for _layer in "${VENDOR_BUILD_DIR:-$_WS/.build/vendor}" "${APP_BUILD_DIR:-$_WS/.build/apps}"; do
    local _SETUP="$_layer/install/setup.zsh"
    if [ ! -f "$_SETUP" ]; then
        echo "[elfin_env] 错误: 未找到 $_SETUP" >&2
        echo "[elfin_env] 请先构建对应工作空间" >&2
        return 1
    fi
done

source "${ROS_ZSH_SETUP:-/opt/ros/humble/setup.zsh}"
source "${VENDOR_BUILD_DIR:-$_WS/.build/vendor}/install/setup.zsh"
source "${APP_BUILD_DIR:-$_WS/.build/apps}/install/setup.zsh"
export ROS_LOCALHOST_ONLY=1
export ELFIN5_OUTPUT_DIR="${ELFIN5_OUTPUT_DIR:-$_WS/outputs}"

# 手眼标定(眼在手上): 需先起 start_real.py + realsense2_camera, 棋盘格平放桌面
alias hand_eye_calib='ros2 run cos_realsense hand_eye_calib_node'
