#!/usr/bin/env bash
# elfin_env.sh —— 一次性配好 Elfin 工作空间的 ROS 环境 (用 source 加载, 不要直接执行)
#
# 用法:
#   source ~/elfin5_ctl/cos_ws/scripts/elfin_env.sh          # 普通用户终端
#   sudo bash -c 'source ~/elfin5_ctl/cos_ws/scripts/elfin_env.sh && exec bash'   # root shell
#
# 内容:
#   1. 按 humble -> .build/vendor -> .build/apps 显式 source 三层 (与 start_real.py 的
#      env_prelude 一致, 不依赖 .build/apps 构建时的串接快照)
#   2. export ROS_LOCALHOST_ONLY=1 (与 start_real.py / start_sim.py 保持一致;
#      不一致时 DDS 发现层隔离, rqt/CLI 完全看不到流水线节点 —— 2026-10-09 实测)

_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_WS="$_ENV_DIR/.."

for _layer in "${VENDOR_BUILD_DIR:-$_WS/.build/vendor}" "${APP_BUILD_DIR:-$_WS/.build/apps}"; do
    _SETUP="$_layer/install/setup.bash"
    if [ ! -f "$_SETUP" ]; then
        echo "[elfin_env] 错误: 未找到 $_SETUP" >&2
        echo "[elfin_env] 请先构建: scripts/build_vendor.sh -> scripts/build_apps.sh" >&2
        return 1 2>/dev/null || exit 1
    fi
done

source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${VENDOR_BUILD_DIR:-$_WS/.build/vendor}/install/setup.bash"
source "${APP_BUILD_DIR:-$_WS/.build/apps}/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ELFIN5_OUTPUT_DIR="${ELFIN5_OUTPUT_DIR:-$_WS/outputs}"

unset _ENV_DIR _WS _SETUP _layer
