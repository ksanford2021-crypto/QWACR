#!/bin/bash
# Robust launch wrapper for QWACR
# Automatically prepares serial port and launches with retry logic

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
USE_RVIZ="${USE_RVIZ:-false}"

echo "========================================" echo "QWACR Robust Launch Script"
echo "========================================"
echo "Serial Port: $SERIAL_PORT"
echo "RViz: $USE_RVIZ"
echo ""

# Step 1: Prepare serial port
echo "[1/3] Preparing serial port..."
if [ -x "$SCRIPT_DIR/prepare_serial.sh" ]; then
    "$SCRIPT_DIR/prepare_serial.sh" "$SERIAL_PORT"
else
    echo "⚠ Warning: prepare_serial.sh not found, skipping preparation"
fi

# Step 2: Kill any existing ROS nodes
echo ""
echo "[2/3] Cleaning up any existing ROS nodes..."
killall -9 ros2 ros2_control_node rviz2 2>/dev/null || true
sleep 1

# Step 3: Launch ROS2
echo ""
echo "[3/3] Launching ROS2 system..."
cd ~/qwacr_ws
source install/setup.bash

# Launch with robust parameters
ros2 launch qwacr_build display.launch.py \
    serial_port:="$SERIAL_PORT" \
    use_rviz:="$USE_RVIZ" \
    2>&1 | tee /tmp/qwacr_launch.log

# If launch fails, show helpful error message
if [ $? -ne 0 ]; then
    echo ""
    echo "✗ Launch failed. Check /tmp/qwacr_launch.log for details"
    echo ""
    echo "Common fixes:"
    echo "  1. Check Arduino is connected: ls -l $SERIAL_PORT"
    echo "  2. Verify Arduino firmware is running"
    echo "  3. Try power-cycling the Arduino"
    echo "  4. Run manually: python3 ~/tests/test_all_motors.py"
    exit 1
fi
