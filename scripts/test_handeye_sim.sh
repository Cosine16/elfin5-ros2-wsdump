#!/usr/bin/env bash
# Run the Humble/Harmonic simulation and validate its public ROS interfaces.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export HANDEYE_OUTPUT_DIR="${HANDEYE_OUTPUT_DIR:-$repo_root/outputs/handeye}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-85}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
smoke_dir="$HANDEYE_OUTPUT_DIR/sim-smoke"
mkdir -p "$smoke_dir"
launch_log="$smoke_dir/launch.log"

set +u
source "$repo_root/scripts/handeye_env.sh"
set -u

setsid bash "$repo_root/scripts/run_handeye.sh" launch headless:=true rviz:=false   >"$launch_log" 2>&1 &
sim_pid=$!

cleanup() {
  kill -INT -- "-$sim_pid" 2>/dev/null || true
  for _ in {1..15}; do
    kill -0 "$sim_pid" 2>/dev/null || break
    sleep 1
  done
  kill -TERM -- "-$sim_pid" 2>/dev/null || true
  wait "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

startup_timeout="${SIM_STARTUP_TIMEOUT_SEC:-180}"
deadline=$((SECONDS + startup_timeout))
while true; do
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    echo "Simulation exited before becoming ready; last log lines:" >&2
    tail -n 100 "$launch_log" >&2
    exit 1
  fi
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -Fxq /joint_states <<<"$topics"     && grep -Fxq /camera/color/image_raw <<<"$topics"     && grep -Fxq /elfin_arm_controller/joint_trajectory <<<"$topics"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "Simulation topics did not become ready within ${startup_timeout}s" >&2
    tail -n 100 "$launch_log" >&2
    exit 1
  fi
  sleep 2
done

python3 "$repo_root/scripts/handeye_sim_smoke.py"   --timeout "${SIM_MESSAGE_TIMEOUT_SEC:-60}" --output "$smoke_dir/result.json"
ros2 action list -t >"$smoke_dir/actions.txt"
if ! grep -Fq '/elfin_arm_controller/follow_joint_trajectory' "$smoke_dir/actions.txt"; then
  echo "Elfin5 FollowJointTrajectory action is unavailable" >&2
  exit 1
fi

echo "PASS: Humble/Harmonic simulation, joint states, camera, and controller are ready."
