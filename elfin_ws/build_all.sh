#!/usr/bin/env bash
# 独立构建脚本：脱离终端会话运行，避免 SIGINT 中断
set -e

# 清空从父终端继承的 ROS 环境变量，避免旧工作空间(catkin_ws)污染新工作空间的 setup 链
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH \
      ROS_LOCALHOST_ONLY ROS_DOMAIN_ID RMW_IMPLEMENTATION PYTHONPATH LD_LIBRARY_PATH \
      PKG_CONFIG_PATH COLCON_DEFAULTS_FILE 2>/dev/null || true

export PATH=/usr/bin:/bin:/usr/local/bin

# 工作空间根目录 = 本脚本所在目录
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理之前的失败产物
rm -rf "$WS/build" "$WS/install" "$WS/log"

source /opt/ros/humble/setup.bash
cd "$WS"

echo "=== 开始构建: $(date) ==="
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "=== 构建结束: $(date) exit=$? ==="

# 合并各包的 compile_commands.json 供 clangd 使用(见仓库根目录 .clangd)
python3 -c "import json,glob; m=[]; [m.extend(json.load(open(p))) for p in glob.glob('$WS/build/*/compile_commands.json')]; json.dump(m, open('$WS/build/compile_commands.json','w'), indent=1)"
echo "=== 已合并 compile_commands.json ==="
echo "=== 构建结束: $(date) exit=$? ==="
