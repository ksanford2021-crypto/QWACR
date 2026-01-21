#!/usr/bin/env python3
"""
Combined PID test - forward then reverse with varying speeds.
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
    
    print("\n=== Combined Forward/Reverse PID Test ===")
    
    # FORWARD TESTS
    print("\n1. Ramp FORWARD: 0 -> 6 rad/s (3 sec)")
    for i in range(30):
        vel = i * 0.2
        send_velocity(ser, vel, 0.0)
        if i % 5 == 0:
            print(f"  {vel:.1f} rad/s", end='\r')
        time.sleep(0.1)
    
    print("\n2. HOLD: 6 rad/s (2 sec)")
    for i in range(20):
        send_velocity(ser, 6.0, 0.0)
        time.sleep(0.1)
    
    print("\n3. Ramp DOWN: 6 -> 0 rad/s (3 sec)")
    for i in range(30):
        vel = 6.0 - (i * 0.2)
        send_velocity(ser, vel, 0.0)
        if i % 5 == 0:
            print(f"  {vel:.1f} rad/s", end='\r')
        time.sleep(0.1)
    
    print("\n4. STOP (1 sec)")
    for i in range(10):
        send_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)
    
    # REVERSE TESTS
    print("\n5. Ramp REVERSE: 0 -> -6 rad/s (3 sec)")
    for i in range(30):
        vel = -(i * 0.2)
        send_velocity(ser, vel, 0.0)
        if i % 5 == 0:
            print(f"  {vel:.1f} rad/s", end='\r')
        time.sleep(0.1)
    
    print("\n6. HOLD: -6 rad/s (2 sec)")
    for i in range(20):
        send_velocity(ser, -6.0, 0.0)
        time.sleep(0.1)
    
    print("\n7. Ramp UP: -6 -> 0 rad/s (3 sec)")
    for i in range(30):
        vel = -6.0 + (i * 0.2)
        send_velocity(ser, vel, 0.0)
        if i % 5 == 0:
            print(f"  {vel:.1f} rad/s", end='\r')
        time.sleep(0.1)
    
    print("\n8. STOP")
    for i in range(10):
        send_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)
    
    ser.close()
    print("\n=== Test Complete ===")
    
except Exception as e:
    print(f"Error: {e}")
