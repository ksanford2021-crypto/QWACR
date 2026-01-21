#!/usr/bin/env python3
"""
Minimal test: send one command every 500ms for 10 seconds.
Watch RX LED—should blink repeatedly.
"""
import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

def encode_cobs(data):
    """Encode data using COBS"""
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

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
    time.sleep(2)
    
    for i in range(40):  # 40 × 0.5s = 20 seconds
        # Send: left motor forward at 100 rad/s, right 0
        command = bytearray([0x01])
        command.extend(struct.pack('<f', 100.0))
        command.extend(struct.pack('<f', 0.0))
        
        encoded = encode_cobs(bytes(command))
        print(f"Send {i+1}: {' '.join(f'{b:02X}' for b in encoded)}")
        ser.write(encoded)
        
        time.sleep(0.5)
    
    ser.close()
    print("Done")
except Exception as e:
    print(f"Error: {e}")
