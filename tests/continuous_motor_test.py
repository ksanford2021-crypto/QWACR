#!/usr/bin/env python3
"""
Runs motors continuously for voltage/signal measurement
"""
import serial
import struct
import time

def encode_cobs(data):
    encoded = bytearray()
    block = bytearray()
    for byte in data:
        if byte == 0x00:
            encoded.append(len(block) + 1)
            encoded.extend(block)
            block = bytearray()
        else:
            block.append(byte)
    encoded.append(len(block) + 1)
    encoded.extend(block)
    encoded.append(0x00)
    return bytes(encoded)

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
time.sleep(1)

print("=== CONTINUOUS MOTOR TEST ===")
print("Motors will run for 30 seconds at 3.0 rad/s")
print("Use multimeter to measure:")
print("  1. Encoder Vcc (blue wire) - should be ~5V")
print("  2. Signal A (yellow wire) - should toggle or read ~5V")
print("  3. Signal B (white wire) - should toggle or read ~5V")
print("\nStarting in 3 seconds...\n")
time.sleep(3)

# Run all motors forward
cmd = struct.pack('<Bff', 0x01, 3.0, 3.0)
ser.write(encode_cobs(cmd))
ser.flush()

print("MOTORS RUNNING - Measure voltages now!")
print("Press Ctrl+C to stop early")

# Keep sending commands every second to prevent timeout
try:
    for i in range(30):
        cmd = struct.pack('<Bff', 0x01, 3.0, 3.0)
        ser.write(encode_cobs(cmd))
        ser.flush()
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopped by user")

# Stop motors
cmd = struct.pack('<Bff', 0x01, 0.0, 0.0)
ser.write(encode_cobs(cmd))
ser.flush()

ser.close()
print("\nMotors stopped")
