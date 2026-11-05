#!/usr/bin/env bash
# Reproduce the reviewed integration patches without changing the fork's commit.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vendor="$repo_root/third_party/elfin_robot_ros2"
mode="${1:-apply}"
if [[ "$mode" != apply && "$mode" != --check ]]; then
  echo 'Usage: bash scripts/prepare_vendor.sh [--check]' >&2
  exit 2
fi
if [[ ! -e "$vendor/.git" ]]; then
  echo 'Initialize vendor source first: git submodule update --init --recursive' >&2
  exit 1
fi
expected="$(cat "$repo_root/third_party/elfin_robot_ros2.revision")"
actual="$(git -C "$vendor" rev-parse HEAD)"
if [[ "$actual" != "$expected" ]]; then
  echo "Vendor revision mismatch: expected $expected, found $actual. Review the revision and patches together." >&2
  exit 1
fi
for patch in "$repo_root"/third_party/patches/elfin_robot_ros2/*.patch; do
  if git -C "$vendor" apply --reverse --check "$patch" 2>/dev/null; then
    echo "Already applied: $(basename "$patch")" >&2
  elif git -C "$vendor" apply --check "$patch"; then
    if [[ "$mode" == --check ]]; then
      echo "Applicable: $(basename "$patch")" >&2
    else
      git -C "$vendor" apply "$patch"
      echo "Applied: $(basename "$patch")" >&2
    fi
  else
    echo "Patch conflicts with vendor changes: $patch. Resolve explicitly; no reset was performed." >&2
    exit 1
  fi
done
