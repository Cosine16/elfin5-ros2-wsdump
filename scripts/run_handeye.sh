#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-launch}" in
  launch|calibrate|check|urdf) command="${1:-launch}" ;;
  *) echo 'Usage: bash scripts/run_handeye.sh {launch|calibrate|check|urdf} [arguments]' >&2; exit 2 ;;
esac
if (( $# )); then shift; fi
set +u
source "$repo_root/scripts/handeye_env.sh"
set -u
case "$command" in
  launch) exec ros2 launch elfin5_handeye handeye.launch.py "$@" ;;
  calibrate) exec ros2 run elfin5_handeye calibrate --ros-args \
    -p use_sim_time:=true -p output_dir:="$HANDEYE_OUTPUT_DIR" "$@" ;;
  check) exec ros2 "$@" ;;
  urdf)
    share="$(ros2 pkg prefix --share elfin5_handeye)"
    xacro "$share/urdf/elfin5_d435i.urdf.xacro" \
      | sed -E 's@file://[^" ]*/share/elfin_description/meshes/@package://elfin_description/meshes/@g' \
      > "$HANDEYE_OUTPUT_DIR/elfin5_d435i.urdf"
    check_urdf "$HANDEYE_OUTPUT_DIR/elfin5_d435i.urdf" ;;
esac
