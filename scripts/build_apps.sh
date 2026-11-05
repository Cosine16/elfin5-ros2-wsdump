#!/usr/bin/env bash
# Classic/real-hardware overlay. Harmonic uses build_handeye.sh independently.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${APP_BUILD_DIR:-$repo_root/.build/apps}"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
underlay="${VENDOR_BUILD_DIR:-$repo_root/.build/vendor}/install/setup.bash"
for setup in "$ros_setup" "$underlay"; do
  if [[ ! -f "$setup" ]]; then
    echo "Missing underlay: $setup. Run bash scripts/build_vendor.sh first." >&2
    exit 1
  fi
done
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH COLCON_DEFAULTS_FILE
set +u
source "$ros_setup"
source "$underlay"
set -u
mkdir -p "$build_dir"
cd "$repo_root"
colcon --log-base "$build_dir/log" build \
  --base-paths "$repo_root/apps" --packages-skip elfin5_handeye \
  --build-base "$build_dir/build" --install-base "$build_dir/install" \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
