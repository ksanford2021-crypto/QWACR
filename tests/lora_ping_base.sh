#!/usr/bin/env bash
set -euo pipefail

SERIAL_PORT=${1:-/dev/ttyUSB0}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source your workspace first." >&2
  exit 1
fi

if [[ ! -e "$SERIAL_PORT" ]]; then
  echo "Serial port $SERIAL_PORT not found." >&2
  exit 1
fi

echo "Starting base LoRa bridge..."
ros2 run qwacr_comms lora_bridge --ros-args -p role:=base -p serial_port:="$SERIAL_PORT" &
BRIDGE_PID=$!

sleep 2

echo "Publishing test cmd_vel (0.1 m/s)..."
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}' >/dev/null

sleep 1

echo "Waiting for telemetry on /lora/telemetry_in (Ctrl+C to stop early)..."
if timeout 10 ros2 topic echo /lora/telemetry_in --once; then
  echo "PASS: telemetry received"
else
  echo "WARN: no telemetry received (check robot side)" >&2
fi

kill "$BRIDGE_PID" >/dev/null 2>&1 || true
wait "$BRIDGE_PID" 2>/dev/null || true

echo "Base ping complete."
