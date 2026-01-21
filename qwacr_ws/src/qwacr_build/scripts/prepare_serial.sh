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
        
        # Quick test: send request and check for response
        echo "Testing serial communication..."
        timeout 2 python3 -c "
import serial
import time
try:
    ser = serial.Serial('$SERIAL_PORT', 115200, timeout=1)
    time.sleep(0.5)
    ser.write(b'\x02\x00')  # Request feedback
    ser.flush()
    time.sleep(0.2)
    response = ser.read(100)
    ser.close()
    if len(response) > 0:
        print('✓ Serial communication OK')
        exit(0)
    else:
        print('⚠ No response from Arduino')
        exit(1)
except Exception as e:
    print(f'⚠ Serial test failed: {e}')
    exit(1)
" 2>/dev/null
        
        if [ $? -eq 0 ]; then
            echo "✓ Serial port ready"
            exit 0
        else
            echo "⚠ Serial test failed, retrying..."
        fi
    fi
    sleep 0.5
done

echo "✗ Serial port not available after 15 seconds"
exit 1
