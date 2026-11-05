# Repository layout and development rules

- Put repository-maintained source and integrations under `apps/`, grouped by responsibility:
  `sensors/`, `robot/`, `bridges/`, `policies/`, `workflows/`, and `bringup/`.
- MCP adapters belong in `apps/bridges/mcp/`; model integration belongs in
  `apps/policies/`; tasks involving multiple capabilities belong in `apps/workflows/`.
- This is the Elfin5 workstation system-integration and experiment repository.
  Keep vendor ROS sources in the `third_party/elfin_robot_ros2/` submodule and
  other external dependencies in `third_party/`. Do not duplicate their source in `apps/`.
- Preserve the pinned vendor revision in `third_party/elfin_robot_ros2.revision`.
  Record integration build fixes in `third_party/patches/elfin_robot_ros2/` and
  apply/check them with `bash scripts/prepare_vendor.sh [--check]`. Do not silently
  switch vendor branches or use `git submodule update --remote` during layout changes.
- A capability directory is not automatically a colcon workspace. Keep existing ROS
  package names and package-local resource paths stable when reorganizing source.
- Build the vendor underlay with `bash scripts/build_vendor.sh`, then the
  Classic/real-hardware overlay with `bash scripts/build_apps.sh`. Build Harmonic
  calibration independently with `bash scripts/build_handeye.sh`. These profiles
  write to `.build/vendor/`, `.build/apps/`, and `.build/handeye/` respectively.
- Put generated data, calibration results and screenshots under `outputs/`.
  Never treat `.build/legacy/` as an active underlay or copy installed package trees
  to simulate a build. Rebuild from source after moving the checkout.
- Keep Python virtual environments local to the owning application and Git-ignored.
  `apps/bridges/` and `apps/policies/` are excluded from colcon discovery.
- Put repository build/diagnostic scripts in `scripts/`. Keep application-specific
  helpers beside their application; extract shared libraries only for actual reuse.
- Keep MCP, policy inference and robot execution boundaries explicit. A layout
  change must not silently switch execution backends or introduce simultaneous
  motion publishers. Shared bringup extraction is a separate, tested change.
- When changing paths, update launch/build scripts, documentation and editor paths.
  Validate shell/Python syntax, `colcon list --base-paths apps`, and relevant existing
  tests. State explicitly if ROS or hardware validation cannot run in the environment.
