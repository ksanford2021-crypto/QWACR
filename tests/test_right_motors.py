#!/usr/bin/env python3
"""
Test RIGHT motors only (FR + BR on Driver 2).
"""
import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

def encode_cobs(data):
    encoded = bytearray()
    code_idx = len(encoded)
    encoded.append(0)
    code = 1
    
    for byte in data:
        if byte == 0:
            encoded[code_idx] = code
            code_idx = len(encoded)
            encoded.append(0)
            code = 1
        else:
            encoded.append(byte)
            code += 1
    
    encoded[code_idx] = code
    encoded.append(0)
    return bytes(encoded)

def send_velocity(ser, left_vel, right_vel):
    command = bytearray([0x01])
    command.extend(struct.pack('<f', left_vel))
    command.extend(struct.pack('<f', right_vel))
    encoded = encode_cobs(bytes(command))
    ser.write(encoded)
    ser.flush()

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(2)
    
    print("\n=== RIGHT Motor Test (FR + BR) ===")
    
    print("\n1. Forward 3 rad/s (3 sec)")
    for i in range(30):
        send_velocity(ser, 0.0, 3.0)
        time.sleep(0.1)
    
    print("\n2. Stop (1 sec)")
    for i in range(10):
        send_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)
    
    print("\n3. Reverse -3 rad/s (3 sec)")
    for i in range(30):
        send_velocity(ser, 0.0, -3.0)
        time.sleep(0.1)
    
    print("\n4. Stop")
    for i in range(10):
        send_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)
    
    ser.close()
    print("\nTest complete - did both right motors spin?")
    
except Exception as e:
    print(f"Error: {e}")
