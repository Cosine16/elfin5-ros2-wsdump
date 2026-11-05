#!/usr/bin/env bash
# stdio MCP server: diagnostic output must go to stderr.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" || ! -x "$script_dir/.venv/bin/python" ]]; then
  echo "ROS 2 Humble and the MCP venv are required; see $script_dir/README.md." >&2
  exit 1
fi
set +u
source "$ros_setup" >&2
vendor_setup="${VENDOR_BUILD_DIR:-$repo_root/.build/vendor}/install/setup.bash"
if [[ -f "$vendor_setup" ]]; then
  source "$vendor_setup" >&2
fi
set -u
export ROS_LOCALHOST_ONLY=1
exec "$script_dir/.venv/bin/python" "$script_dir/elfin5_mcp_server.py" "$@"
