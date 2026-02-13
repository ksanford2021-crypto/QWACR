#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source your workspace first." >&2
  exit 1
fi

echo "Publishing a short forward command..."
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' >/dev/null
sleep 0.5

echo "One-shot joint state read:"
ros2 topic echo /joint_states --once
