#!/bin/bash
# Serial port preparation script for QWACR
# Performs serial warmup to initialize Arduino communication
# This ensures ros2_control can connect reliably

SERIAL_PORT="${1:-/dev/ttyACM0}"
BAUD_RATE="${2:-115200}"

echo "Preparing serial port: $SERIAL_PORT @ $BAUD_RATE baud"

# Kill any processes using the serial port
if [ -e "$SERIAL_PORT" ]; then
    echo "Checking for processes using $SERIAL_PORT..."
    PIDS=$(lsof -t "$SERIAL_PORT" 2>/dev/null)
    if [ -n "$PIDS" ]; then
        echo "Killing processes: $PIDS"
        kill -9 $PIDS 2>/dev/null
        sleep 0.5
    fi
fi

# Wait for port to become available
echo "Waiting for $SERIAL_PORT to be available..."
for i in {1..30}; do
    if [ -e "$SERIAL_PORT" ]; then
        echo "✓ Serial port available"
        break
    fi
    sleep 0.5
    
    if [ $i -eq 30 ]; then
        echo "✗ Serial port not available after 15 seconds"
        exit 1
    fi
done

# Perform serial warmup (critical for reliable ros2_control connection)
echo "Performing serial warmup..."
echo "  Opening serial connection for 2 seconds..."

# Use Python pyserial to properly open port and trigger Arduino reset
python3 -c "import serial, time; s=serial.Serial('${SERIAL_PORT}', ${BAUD_RATE}, timeout=2); time.sleep(2); s.close()"

if [ $? -eq 0 ]; then
    echo "✓ Serial warmup complete"
else
    echo "✗ Serial warmup failed"
    exit 1
fi

# Verify port is still accessible
if [ ! -e "$SERIAL_PORT" ]; then
    echo "✗ Serial port disappeared after warmup"
    exit 1
fi

echo "✓ Serial port ready for ros2_control"
exit 0
