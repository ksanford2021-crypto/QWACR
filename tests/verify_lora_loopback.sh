#!/usr/bin/env bash
set -euo pipefail

ROLE=${1:-base}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source your workspace first." >&2
  exit 1
fi

echo "Loopback test (no serial)."
echo "Start lora_bridge with serial disabled, then publish a cmd_vel."

ros2 run qwacr_comms lora_bridge --ros-args -p role:=$ROLE -p enable_serial:=false &
PID=$!

sleep 1
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}' >/dev/null
sleep 1

kill $PID
wait $PID 2>/dev/null || true

echo "Loopback test complete."
