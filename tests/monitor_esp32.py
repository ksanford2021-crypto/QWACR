#!/usr/bin/env python3
"""Simple serial monitor to view ESP32 debug output."""

import serial
import sys

if len(sys.argv) < 2:
    print("Usage: python3 monitor_esp32.py /dev/ttyUSB0")
    sys.exit(1)

port = sys.argv[1]
baud = 115200

print(f"Monitoring {port} at {baud} baud...")
print("Press Ctrl+C to exit\n")

try:
    ser = serial.Serial(port, baud, timeout=1)
    while True:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='replace')
                print(line, end='')
            except Exception as e:
                # Print raw bytes if decode fails
                data = ser.read(ser.in_waiting)
                print(f"[RAW] {data.hex()}")
except KeyboardInterrupt:
    print("\nExiting...")
except Exception as e:
    print(f"Error: {e}")
