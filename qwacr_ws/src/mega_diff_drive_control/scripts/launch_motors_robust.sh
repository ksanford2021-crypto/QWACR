#!/bin/bash
# Robust motor control launch script
# Performs serial warmup before launching ros2_control
# Use this script for reliable motor controller startup in Nav2 testing

set -e

SERIAL_PORT="${1:-/dev/ttyACM1}"
BAUD_RATE="${2:-115200}"

echo "========================================"
echo "QWACR Motor Control - Robust Launch"
echo "========================================"
echo "Serial Port: $SERIAL_PORT"
echo "Baud Rate: $BAUD_RATE"
echo ""

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "✗ Error: Serial port $SERIAL_PORT does not exist"
    echo ""
    echo "Available serial ports:"
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  No serial devices found"
    echo ""
    echo "Usage: $0 [serial_port] [baud_rate]"
    echo "Example: $0 /dev/ttyACM1 115200"
    exit 1
fi

# Step 1: Kill any processes using the serial port
echo "[1/3] Checking for processes using $SERIAL_PORT..."
PIDS=$(lsof -t "$SERIAL_PORT" 2>/dev/null || true)
if [ -n "$PIDS" ]; then
    echo "  Killing processes: $PIDS"
    kill -9 $PIDS 2>/dev/null || true
    sleep 0.5
else
    echo "  ✓ No processes using port"
fi

# Step 2: Perform serial warmup (critical for reliable ros2_control connection)
echo ""
echo "[2/3] Performing serial warmup..."
echo "  Opening serial connection for 2 seconds..."

# Use Python pyserial to properly open port and trigger Arduino reset
python3 -c "import serial, time; s=serial.Serial('${SERIAL_PORT}', ${BAUD_RATE}, timeout=2); time.sleep(2); s.close()"

if [ $? -eq 0 ]; then
    echo "  ✓ Serial warmup complete"
else
    echo "  ✗ Serial warmup failed"
    exit 1
fi

# Verify port is still accessible
if [ ! -e "$SERIAL_PORT" ]; then
    echo "✗ Serial port disappeared after warmup"
    exit 1
fi

# Step 3: Launch ros2_control
echo ""
echo "[3/3] Launching motor control..."
cd ~/qwacr_ws
source install/setup.bash

echo ""
echo "Starting motor control (ros2_control + controllers)..."
echo "Press Ctrl+C to stop"
echo ""

ros2 launch mega_diff_drive_control motors_only.launch.py \
    serial_port:="$SERIAL_PORT" \
    baud_rate:="$BAUD_RATE"

# If we reach here, launch exited
echo ""
echo "Motor control stopped"
