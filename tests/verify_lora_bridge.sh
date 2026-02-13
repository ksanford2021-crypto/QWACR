#!/usr/bin/env bash
set -euo pipefail

PORT=${1:-/dev/ttyUSB0}
ROLE=${2:-base}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source your workspace first." >&2
  exit 1
fi

if [[ ! -e "$PORT" ]]; then
  echo "Serial port $PORT not found." >&2
  exit 1
fi

echo "Starting LoRa bridge (role=$ROLE, port=$PORT). Ctrl+C to stop."
ros2 run qwacr_comms lora_bridge --ros-args -p role:=$ROLE -p serial_port:=$PORT
