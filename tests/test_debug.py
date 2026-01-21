#!/usr/bin/env python3
"""
Simple debug test - just send commands and watch LED blink
"""
import serial
import struct
import time

def encode_cobs(data):
    """Encode data with COBS"""
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

# Connect
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
time.sleep(1)

# Send velocity command
print("Sending velocity command (should make LED blink)...")
cmd_vel = struct.pack('<Bff', 0x01, 5.0, 5.0)
print(f"Raw velocity command: {cmd_vel.hex()}")
encoded_vel = encode_cobs(cmd_vel)
print(f"Encoded: {encoded_vel.hex()}")
ser.write(encoded_vel)
ser.flush()
time.sleep(0.5)

# Check for any data
if ser.in_waiting > 0:
    data = ser.read(ser.in_waiting)
    print(f"Received {len(data)} bytes: {data.hex()}")
else:
    print("No data received")

# Try request
print("\nSending feedback request...")
cmd_req = struct.pack('<B', 0x02)
print(f"Raw request command: {cmd_req.hex()}")
encoded_req = encode_cobs(cmd_req)
print(f"Encoded: {encoded_req.hex()}")
ser.write(encoded_req)
ser.flush()
time.sleep(0.5)

# Check for response
if ser.in_waiting > 0:
    data = ser.read(ser.in_waiting)
    print(f"Received {len(data)} bytes: {data.hex()}")
else:
    print("No data received")

ser.close()
