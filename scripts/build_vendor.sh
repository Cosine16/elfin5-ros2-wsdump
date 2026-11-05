#!/usr/bin/env bash
# Build the pinned vendor fork into an independent underlay.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${VENDOR_BUILD_DIR:-$repo_root/.build/vendor}"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Humble underlay missing: $ros_setup" >&2
  exit 1
fi
bash "$repo_root/scripts/prepare_vendor.sh"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH COLCON_DEFAULTS_FILE
set +u
source "$ros_setup"
set -u
mkdir -p "$build_dir"
cd "$repo_root"
colcon --log-base "$build_dir/log" build \
  --base-paths "$repo_root/third_party/elfin_robot_ros2" \
  --build-base "$build_dir/build" --install-base "$build_dir/install" \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
