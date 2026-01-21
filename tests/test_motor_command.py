#!/usr/bin/env python3
"""
Simple script to send velocity commands to the Arduino motor controller.
Uses COBS encoding for serial communication.
"""
import serial
import struct
import time

# Configuration
PORT = "/dev/ttyACM0"  # WSL serial port
BAUD = 115200

def encode_cobs(data):
    """Encode data using COBS (Consistent Overhead Byte Stuffing)"""
    encoded = bytearray()
    code_idx = len(encoded)
    encoded.append(0)  # Placeholder for code byte
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
    encoded.append(0)  # Delimiter
    return bytes(encoded)

def send_velocity_command(ser, left_vel, right_vel):
    """Send a velocity command to the Arduino with COBS encoding."""
    # Command format: [0x01][left_vel_float][right_vel_float]
    command = bytearray()
    command.append(0x01)  # Start byte
    command.extend(struct.pack('<f', left_vel))   # Left velocity (4 bytes, little-endian float)
    command.extend(struct.pack('<f', right_vel))  # Right velocity (4 bytes, little-endian float)
    
    encoded = encode_cobs(bytes(command))
    print(f"Raw command: {' '.join(f'{b:02X}' for b in command)}")
    print(f"COBS encoded: {' '.join(f'{b:02X}' for b in encoded)}")
    print(f"Command: Left={left_vel} ticks/sec, Right={right_vel} ticks/sec")
    
    ser.write(encoded)
    time.sleep(0.1)

def main():
    try:
        # Open serial port
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to {PORT} at {BAUD} baud")
        time.sleep(2)  # Wait for Arduino to be ready
        
        # Clear any buffered data
        ser.reset_input_buffer()
        
        # Left-only checks to watch the driver fault LED
        print("\n=== Test 1: Left motor FORWARD (120 ticks/sec) ===")
        send_velocity_command(ser, 120.0, 0.0)
        time.sleep(2)

        print("\n=== Test 2: Left motor REVERSE (-120 ticks/sec) ===")
        send_velocity_command(ser, -120.0, 0.0)
        time.sleep(2)

        print("\n=== Test 3: Left motor small pulses (60 -> -60) ===")
        for cycle in range(3):
            print(f"  Pulse {cycle+1}: Forward 0.6s")
            send_velocity_command(ser, 60.0, 0.0)
            time.sleep(0.6)
            print(f"  Pulse {cycle+1}: Reverse 0.6s")
            send_velocity_command(ser, -60.0, 0.0)
            time.sleep(0.6)

        print("\n=== Test 4: STOP ===")
        send_velocity_command(ser, 0.0, 0.0)
        time.sleep(1)
        
        ser.close()
        print("\nTest complete!")
        
    except FileNotFoundError:
        print(f"Error: Serial port {PORT} not found. Check the port and try again.")
        print("Make sure pyserial is installed: pip install pyserial")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
