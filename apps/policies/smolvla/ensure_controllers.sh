#!/usr/bin/env bash
# The vendor gazebo launch (elfin5_gazebo.launch.py) loads controllers via
# `ros2 control`, but ros2controlcli is not installed on this machine.
# This script loads/activates them through the spawner instead.
set -euo pipefail
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
for controller in joint_state_broadcaster elfin_arm_controller; do
  until timeout 30 ros2 run controller_manager spawner "$controller" 2>&1 | grep -q "activated"; do
    echo "waiting for controller_manager ..."
    sleep 3
  done
done
echo "controllers active"
