#!/usr/bin/env bash
# Independent Humble/Harmonic profile; do not source the Classic overlay.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${HANDEYE_WS:-$repo_root/.build/handeye}"
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
for package in ros_gz_sim ros_gz_bridge gz_ros2_control moveit_ros_move_group xacro cv_bridge; do
  ros2 pkg prefix "$package" >/dev/null
done
mkdir -p "$build_dir"
cd "$repo_root"
colcon --log-base "$build_dir/log" build \
  --base-paths "$repo_root/third_party/elfin_robot_ros2" "$repo_root/apps/workflows/handeye/elfin5_handeye" \
  --packages-select elfin_description elfin5_gz_sim elfin5_ros2_moveit2 elfin5_handeye \
  --build-base "$build_dir/build" --install-base "$build_dir/install" \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release "$@"
