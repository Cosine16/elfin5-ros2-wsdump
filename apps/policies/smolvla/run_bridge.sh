#!/usr/bin/env bash
# Default to the same ROS domain as run_gz_sim.sh; override ROS_DOMAIN_ID for Classic.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../.." && pwd)"
if [[ ! -x "$script_dir/.venv/bin/python" ]]; then
  echo "Run python3 $script_dir/setup_smolvla.py first." >&2
  exit 1
fi
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-85}"
export ROS_LOCALHOST_ONLY=1
exec "$script_dir/.venv/bin/python" "$script_dir/smolvla_bridge.py" \
  --ros-args -p use_sim_time:=true \
  -p fallback_image:="${HANDEYE_OUTPUT_DIR:-$repo_root/outputs/handeye}/latest_camera.png" "$@"
