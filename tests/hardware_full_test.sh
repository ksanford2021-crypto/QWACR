#!/usr/bin/env bash
set -euo pipefail

SERIAL_PORT=${1:-/dev/ttyACM0}
BAUD_RATE=${2:-115200}
NO_LAUNCH=${NO_LAUNCH:-0}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source your workspace first." >&2
  exit 1
fi

LAUNCH_PID=""
cleanup() {
  if [[ -n "$LAUNCH_PID" ]]; then
    kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "$NO_LAUNCH" == "0" ]]; then
  echo "Launching motors_only with serial_port=$SERIAL_PORT baud_rate=$BAUD_RATE"
  ros2 launch mega_diff_drive_control motors_only.launch.py \
    serial_port:="$SERIAL_PORT" baud_rate:="$BAUD_RATE" &
  LAUNCH_PID=$!
  sleep 4
fi

echo "Checking /joint_states..."
if timeout 10 ros2 topic echo /joint_states --once >/dev/null; then
  echo "PASS: /joint_states is publishing"
else
  echo "FAIL: /joint_states not publishing" >&2
  exit 1
fi

echo "Checking /diff_cont/odom..."
if timeout 10 ros2 topic echo /diff_cont/odom --once >/dev/null; then
  echo "PASS: /diff_cont/odom is publishing"
else
  echo "FAIL: /diff_cont/odom not publishing" >&2
  exit 1
fi

echo "Sending a short forward command..."
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' >/dev/null
sleep 0.5

echo "Joint states after command:"
ros2 topic echo /joint_states --once

echo "Checking joint position signs (expect all positive)..."
positions=$(ros2 topic echo /joint_states --once --field position)
python3 - <<'PY'
import ast
import sys

raw = sys.argv[1]
try:
  values = ast.literal_eval(raw)
except Exception:
  print(f"FAIL: unable to parse positions: {raw}", file=sys.stderr)
  sys.exit(1)

if not isinstance(values, (list, tuple)) or len(values) < 4:
  print(f"FAIL: unexpected position array: {values}", file=sys.stderr)
  sys.exit(1)

negatives = [v for v in values if v < 0.0]
if negatives:
  print(f"FAIL: negative wheel positions found: {negatives}", file=sys.stderr)
  sys.exit(1)

print("PASS: all wheel positions positive")
PY
"$positions"

# Encoder counts are logged by the hardware interface every ~1s.
# We scan /rosout for the encoder log line.
echo "Looking for encoder count log in /rosout..."
if timeout 8 ros2 topic echo /rosout | grep -m 1 "Encoders:"; then
  echo "PASS: Encoder counts logged"
else
  echo "WARN: No encoder log seen (check log level or hardware feedback)" >&2
fi

echo "Hardware test complete."
