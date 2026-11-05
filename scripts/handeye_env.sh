#!/usr/bin/env bash
# Source from Bash to load the isolated Harmonic profile for calibration or VLA.
_handeye_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_handeye_setup="${HANDEYE_WS:-$_handeye_repo/.build/handeye}/install/setup.bash"
_handeye_ros="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
for _handeye_file in "$_handeye_ros" "$_handeye_setup"; do
  if [[ ! -f "$_handeye_file" ]]; then
    echo "Missing $_handeye_file; run bash $_handeye_repo/scripts/build_handeye.sh first." >&2
    return 1
  fi
done
source "$_handeye_ros"
source "$_handeye_setup"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${HANDEYE_ROS_DOMAIN_ID:-85}}"
export ROS_LOCALHOST_ONLY=1
export GZ_PARTITION="elfin5_handeye_${ROS_DOMAIN_ID}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export HANDEYE_OUTPUT_DIR="${HANDEYE_OUTPUT_DIR:-$_handeye_repo/outputs/handeye}"
mkdir -p "$HANDEYE_OUTPUT_DIR/ros_log"
export ROS_LOG_DIR="$HANDEYE_OUTPUT_DIR/ros_log"
unset _handeye_repo _handeye_setup _handeye_ros _handeye_file
