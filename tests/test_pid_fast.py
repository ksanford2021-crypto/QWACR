#!/usr/bin/env python3
"""
Test PID with higher speeds - ramp to 6 rad/s.
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
    
    print("\n=== PID High Speed Test ===")
    
    # Phase 1: Ramp up from 0 to 6 rad/s over 3 seconds
    print("\n1. Ramp UP: 0 -> 6 rad/s (3 sec)")
    for i in range(30):
        vel = i * 0.2  # 0, 0.2, 0.4, ... 5.8 rad/s
        send_velocity(ser, vel, 0.0)
        if i % 5 == 0:
            print(f"  {vel:.1f} rad/s", end='\r')
        time.sleep(0.1)
    
    # Phase 2: Hold at 6 rad/s for 3 seconds
    print("\n2. HOLD: 6 rad/s (3 sec)")
    for i in range(30):
        send_velocity(ser, 6.0, 0.0)
        time.sleep(0.1)
    
    # Phase 3: Drop to 3 rad/s and hold
    print("\n3. Step DOWN: 6 -> 3 rad/s, hold (2 sec)")
    for i in range(20):
        send_velocity(ser, 3.0, 0.0)
        time.sleep(0.1)
    
    # Phase 4: Ramp down to 0
    print("\n4. Ramp to STOP: 3 -> 0 rad/s (2 sec)")
    for i in range(20):
        vel = 3.0 - (i * 0.15)
        send_velocity(ser, vel, 0.0)
        if i % 5 == 0:
            print(f"  {vel:.1f} rad/s", end='\r')
        time.sleep(0.1)
    
    # Phase 5: Stop for 1 second
    print("\n5. STOP (1 sec)")
    for i in range(10):
        send_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)
    
    # Phase 6: Reverse at -4 rad/s for 2 seconds
    print("\n6. REVERSE: -4 rad/s (2 sec)")
    for i in range(20):
        send_velocity(ser, -4.0, 0.0)
        time.sleep(0.1)
    
    # Phase 7: Stop
    print("\n7. STOP")
    for i in range(10):
        send_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)
    
    ser.close()
    print("\n=== Test Complete ===")
    
except Exception as e:
    print(f"Error: {e}")
