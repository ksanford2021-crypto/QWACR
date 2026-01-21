#!/usr/bin/env python3
"""
Send raw bytes WITHOUT COBS encoding to test basic serial.
"""
import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
    time.sleep(2)
    
    # Send simple raw command: [0x01][left_float][right_float]
    # No COBS, just raw bytes
    command = bytearray([0x01])
    command.extend(struct.pack('<f', 2.0))   # Left
    command.extend(struct.pack('<f', 0.0))   # Right
    
    print(f"Sending RAW (no COBS): {' '.join(f'{b:02X}' for b in command)}")
    
    for i in range(20):
        ser.write(command)
        time.sleep(0.2)
    
    ser.close()
    print("Done")
    
except Exception as e:
    print(f"Error: {e}")
