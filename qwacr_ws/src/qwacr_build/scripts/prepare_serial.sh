#!/bin/bash
# Serial port preparation script for QWACR
# Clears any stale serial connections and ensures clean state

SERIAL_PORT="${1:-/dev/ttyACM0}"

echo "Preparing serial port: $SERIAL_PORT"

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
        
        # Port is available, that's enough for now
        # (The hardware interface will handle actual communication)
        echo "✓ Serial port ready"
        exit 0
    fi
    sleep 0.5
done

echo "✗ Serial port not available after 15 seconds"
exit 1
