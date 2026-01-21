#!/usr/bin/env python3
"""
Test request/response odometry protocol on Arduino

Protocol:
- Send velocity command: [0x01][left_vel_float][right_vel_float] (COBS encoded)
- Request encoder data: [0x02] (COBS encoded)
- Receive feedback: [0x10][enc_fl][enc_bl][enc_fr][enc_br][vel_fl][vel_bl][vel_fr][vel_br]
"""

import serial
import struct
import time
import sys

BAUD = 115200
PORT = "/dev/ttyACM0"

def encode_cobs(data):
    """Encode data with COBS (Consistent Overhead Byte Stuffing)"""
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
    encoded.append(0x00)  # Delimiter
    
    return bytes(encoded)

def decode_cobs(data):
    """Decode COBS-encoded data"""
    decoded = bytearray()
    i = 0
    
    while i < len(data):
        if data[i] == 0x00:
            break
        
        block_size = data[i] - 1
        i += 1
        
        if block_size > 0:
            if i + block_size > len(data):
                break
            
            decoded.extend(data[i:i + block_size])
            
            # Add zero byte if this wasn't the last block
            if i + block_size < len(data) and data[i + block_size] != 0x00:
                decoded.append(0x00)
            
            i += block_size
        else:
            # block_size == 0 means this was a stuffed zero
            decoded.append(0x00)
    
    return bytes(decoded)

def send_velocity_command(ser, vel_left, vel_right):
    """Send velocity command to Arduino"""
    # Pack as floats: [0x01][left_float][right_float]
    cmd = struct.pack('<Bff', 0x01, vel_left, vel_right)
    encoded = encode_cobs(cmd)
    
    print(f"Sending velocity command: left={vel_left:.2f}, right={vel_right:.2f}")
    print(f"  Raw: {cmd.hex()}")
    print(f"  COBS: {encoded.hex()}")
    
    ser.write(encoded)
    ser.flush()

def request_feedback(ser):
    """Request encoder feedback from Arduino"""
    # Just send [0x02]
    cmd = struct.pack('<B', 0x02)
    encoded = encode_cobs(cmd)

    # Clear any stale bytes before requesting new feedback
    ser.reset_input_buffer()
    
    print(f"Requesting feedback...")
    print(f"  Raw: {cmd.hex()}")
    print(f"  COBS: {encoded.hex()}")
    
    ser.write(encoded)
    ser.flush()

def read_feedback(ser, timeout=2.0):
    """Read feedback packet (robust single-shot, delimiter-aware)."""
    start = time.time()
    raw = bytearray()

    # Poll for incoming bytes until delimiter seen or timeout
    while time.time() - start < timeout:
        if ser.in_waiting:
            raw.extend(ser.read(ser.in_waiting))
            if 0x00 in raw:
                break
        else:
            time.sleep(0.02)

    if not raw:
        print("ERROR: No response received!")
        return None

    # Trim after first delimiter, keep prior bytes
    if 0x00 in raw:
        raw = raw[: raw.index(0x00) + 1]

    print(f"Raw packet received ({len(raw)} bytes): {raw.hex()}")

    try:
        # Manual COBS decode (stuffed zeros handled)
        decoded = bytearray()
        i = 0
        while i < len(raw):
            if raw[i] == 0x00:
                break
            overhead = raw[i]
            block = overhead - 1
            i += 1
            if block > 0:
                decoded.extend(raw[i:i + block])
                i += block
                if i < len(raw) and raw[i] != 0x00:
                    decoded.append(0x00)
            else:
                decoded.append(0x00)

        print(f"Decoded packet ({len(decoded)} bytes): {decoded.hex()}")

        if len(decoded) < 33:
            print(f"ERROR: Packet too short! Expected 33 bytes, got {len(decoded)}")
            return None
        if decoded[0] != 0x10:
            print(f"ERROR: Wrong packet type! Expected 0x10, got 0x{decoded[0]:02x}")
            return None

        enc_fl, enc_bl, enc_fr, enc_br = struct.unpack('<iiii', decoded[1:17])
        vel_fl, vel_bl, vel_fr, vel_br = struct.unpack('<ffff', decoded[17:33])

        print(f"Encoders: FL={enc_fl}, BL={enc_bl}, FR={enc_fr}, BR={enc_br}")
        print(f"Velocities: FL={vel_fl:.3f}, BL={vel_bl:.3f}, FR={vel_fr:.3f}, BR={vel_br:.3f} rad/s")

        return {
            'encoders': (enc_fl, enc_bl, enc_fr, enc_br),
            'velocities': (vel_fl, vel_bl, vel_fr, vel_br)
        }
    except Exception as e:
        print(f"ERROR parsing feedback: {e}")
        return None

def test_request_response():
    """Test request/response protocol"""
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
        print(f"Connected to {PORT} at {BAUD} baud\n")
        # Give the port a moment to settle (matches debug behavior)
        time.sleep(1.0)
        ser.reset_input_buffer()
        
        # Test 1: Send velocity command
        print("=== Test 1: Send velocity command ===")
        send_velocity_command(ser, 5.0, 5.0)
        time.sleep(0.5)
        
        # Test 2: Request feedback (should show motor moving)
        print("\n=== Test 2: Request feedback after setting velocity ===")
        request_feedback(ser)
        fb1 = read_feedback(ser)

        # Test 3: Send reverse command
        print("\n=== Test 3: Send reverse velocity command ===")
        send_velocity_command(ser, -3.0, -3.0)
        time.sleep(0.8)
        
        print("\n=== Test 4: Request feedback after reverse command ===")
        request_feedback(ser)
        fb2 = read_feedback(ser)
        
        # Test 5: Stop motors
        print("\n=== Test 5: Stop motors ===")
        send_velocity_command(ser, 0.0, 0.0)
        time.sleep(0.5)
        
        print("\n=== Test 6: Request final feedback ===")
        request_feedback(ser)
        fb3 = read_feedback(ser)
        
        if not (fb1 or fb2 or fb3):
            print("FAILED: No feedback received in any request")
            return False
        
        print("\n✓ Feedback received; test completed")
        ser.close()
        return True
    
    except Exception as e:
        print(f"ERROR: {e}")
        return False

if __name__ == '__main__':
    success = test_request_response()
    sys.exit(0 if success else 1)
