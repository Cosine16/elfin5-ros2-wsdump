#!/usr/bin/env bash
# Compatibility entry point for the shared simulation launch.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
exec bash "$repo_root/scripts/run_handeye.sh" launch headless:=true rviz:=false "$@"
