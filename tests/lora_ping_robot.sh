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

echo "Starting robot LoRa bridge..."
ros2 run qwacr_comms lora_bridge --ros-args -p role:=robot -p serial_port:="$SERIAL_PORT" &
BRIDGE_PID=$!

sleep 2

echo "Waiting for cmd_vel on /lora/cmd_vel_in (Ctrl+C to stop early)..."
if timeout 5 ros2 topic echo /lora/cmd_vel_in --once; then
  echo "PASS: teleop received"
else
  echo "WARN: no teleop received (check base side)" >&2
fi

echo "Publishing dummy telemetry (no sensor data)"
ros2 topic pub -1 /lora/telemetry_out std_msgs/String '{data: "{\"gps_lat\":0.0,\"gps_lon\":0.0,\"gps_fix\":0,\"battery_v\":12.0,\"battery_a\":0.0,\"system_status\":0,\"odom_distance\":0.0,\"rssi\":0,\"snr\":0}" }' >/dev/null

echo "Waiting for telemetry to transmit..."
sleep 5

kill "$BRIDGE_PID" >/dev/null 2>&1 || true
wait "$BRIDGE_PID" 2>/dev/null || true

echo "Robot ping complete."
