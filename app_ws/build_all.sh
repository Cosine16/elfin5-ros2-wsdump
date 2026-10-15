#!/usr/bin/env bash
# 独立构建脚本：脱离终端会话运行，避免 SIGINT 中断
#
# 本工作空间(app_ws)是自研应用层 overlay，依赖 elfin_ws(厂家驱动层, underlay)。
# 必须先构建过 elfin_ws，本脚本会在构建前 source 它的 install/setup.bash，
# 生成的 app_ws/install/setup.bash 会自动串接 elfin_ws，使用时只需 source 本层。
set -e

# 清空从父终端继承的 ROS 环境变量，避免旧工作空间污染新工作空间的 setup 链
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH \
      ROS_LOCALHOST_ONLY ROS_DOMAIN_ID RMW_IMPLEMENTATION PYTHONPATH LD_LIBRARY_PATH \
      PKG_CONFIG_PATH COLCON_DEFAULTS_FILE 2>/dev/null || true

export PATH=/usr/bin:/bin:/usr/local/bin

# 工作空间根目录 = 本脚本所在目录
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNDERLAY="$WS/../elfin_ws/install/setup.bash"

if [ ! -f "$UNDERLAY" ]; then
    echo "错误: 未找到 $UNDERLAY" >&2
    echo "请先构建厂家驱动层: ~/cos_ws/elfin_ws/build_all.sh" >&2
    exit 1
fi

# 清理之前的失败产物
rm -rf "$WS/build" "$WS/install" "$WS/log"

source /opt/ros/humble/setup.bash
source "$UNDERLAY"
cd "$WS"

echo "=== 开始构建 app_ws (underlay: elfin_ws): $(date) ==="
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "=== 构建结束: $(date) exit=$? ==="

# 合并各包的 compile_commands.json 供 clangd 使用(见 app_ws/.clangd)
python3 -c "import json,glob; m=[]; [m.extend(json.load(open(p))) for p in glob.glob('$WS/build/*/compile_commands.json')]; json.dump(m, open('$WS/build/compile_commands.json','w'), indent=1)"
echo "=== 已合并 compile_commands.json ==="
echo "=== 构建结束: $(date) exit=$? ==="
