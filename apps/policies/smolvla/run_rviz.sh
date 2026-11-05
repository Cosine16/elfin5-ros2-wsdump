#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../.." && pwd)"
set +u
source "$repo_root/scripts/handeye_env.sh"
set -u
handeye_share="$(ros2 pkg prefix --share elfin5_handeye)"
moveit_share="$(ros2 pkg prefix --share elfin5_ros2_moveit2)"
rviz_dir="${HANDEYE_OUTPUT_DIR}/rviz"
mkdir -p "$rviz_dir"
xacro "$handeye_share/urdf/elfin5_d435i.urdf.xacro" > "$rviz_dir/robot.urdf"
"$script_dir/.venv/bin/python" - "$rviz_dir" "$moveit_share" <<'PY'
from pathlib import Path
import sys
import yaml

output = Path(sys.argv[1])
params = {
    'use_sim_time': True,
    'robot_description': (output / 'robot.urdf').read_text(),
    'robot_description_semantic': (Path(sys.argv[2]) / 'config/elfin5.srdf').read_text(),
}
(output / 'params.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
PY
exec ros2 run rviz2 rviz2 -d "$moveit_share/launch/elfin5_moveit2.rviz" \
  --ros-args --params-file "$rviz_dir/params.yaml"
