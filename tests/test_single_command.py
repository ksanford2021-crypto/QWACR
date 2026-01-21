#!/usr/bin/env python3
"""
Send ONE velocity command and hold it for 5 seconds.
Left motors forward, right stopped.
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

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(2)
    
    # Build command: left=2.0 rad/s, right=0.0
    command = bytearray([0x01])
    command.extend(struct.pack('<f', 2.0))   # Left
    command.extend(struct.pack('<f', 0.0))   # Right
    
    encoded = encode_cobs(bytes(command))
    print(f"Sending: {' '.join(f'{b:02X}' for b in encoded)}")
    print("Left=2.0 rad/s, Right=0.0")
    print("Sending continuously for 10 seconds...")
    
    # Send every 100ms for 10 seconds to keep command alive
    for i in range(100):
        try:
            bytes_written = ser.write(encoded)
            if i % 10 == 0:  # Print every 1 second
                print(f"  {i/10:.1f}s - wrote {bytes_written} bytes")
            ser.flush()  # Force write
            time.sleep(0.1)
        except Exception as e:
            print(f"Write error at {i/10:.1f}s: {e}")
            break
    
    # Stop
    command_stop = bytearray([0x01])
    command_stop.extend(struct.pack('<f', 0.0))
    command_stop.extend(struct.pack('<f', 0.0))
    encoded_stop = encode_cobs(bytes(command_stop))
    ser.write(encoded_stop)
    print("STOP sent")
    
    ser.close()
    print("Done")
    
except Exception as e:
    print(f"Error: {e}")
