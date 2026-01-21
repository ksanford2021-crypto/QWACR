#!/usr/bin/env python3
"""
Test request/response odometry protocol on Arduino (FIXED DECODER)

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

def send_velocity_command(ser, vel_left, vel_right):
    """Send velocity command to Arduino"""
    # Pack as floats: [0x01][left_float][right_float]
    cmd = struct.pack('<Bff', 0x01, vel_left, vel_right)
    encoded = encode_cobs(cmd)
    
    print(f"Sending velocity command: left={vel_left:.2f}, right={vel_right:.2f}")
    ser.write(encoded)
    ser.flush()

def request_feedback(ser):
    """Request encoder feedback from Arduino"""
    # Just send [0x02]
    cmd = struct.pack('<B', 0x02)
    encoded = encode_cobs(cmd)
    
    print(f"Requesting feedback...")
    ser.write(encoded)
    ser.flush()

def read_feedback(ser, timeout=2.0):
    """Read feedback packet from Arduino with proper COBS decoding"""
    start_time = time.time()
    raw_packet = bytearray()
    
    # Read until we get the delimiter (0x00) or timeout
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            raw_packet.extend(byte)
            
            # Check for end-of-packet marker
            if byte == b'\x00':
                break
        else:
            time.sleep(0.01)
    
    if len(raw_packet) == 0:
        print("ERROR: No response received!")
        return None
    
    print(f"Raw packet received ({len(raw_packet)} bytes): {raw_packet.hex()}")
    
    try:
        # Manual COBS decode
        decoded = bytearray()
        i = 0
        
        while i < len(raw_packet):
            if raw_packet[i] == 0x00:
                break
            
            overhead = raw_packet[i]
            block_size = overhead - 1
            i += 1
            
            if block_size > 0:
                if i + block_size > len(raw_packet):
                    break
                
                decoded.extend(raw_packet[i:i + block_size])
                i += block_size
                
                # Insert 0x00 if next byte exists and isn't delimiter
                if i < len(raw_packet) and raw_packet[i] != 0x00:
                    decoded.append(0x00)
            else:
                # Stuffed zero
                decoded.append(0x00)
        
        print(f"Decoded packet ({len(decoded)} bytes): {decoded.hex()}")
        
        if len(decoded) < 33:
            print(f"ERROR: Packet too short! Expected 33 bytes, got {len(decoded)}")
            return None
        
        # Parse: [0x10][enc_fl][enc_bl][enc_fr][enc_br][vel_fl][vel_bl][vel_fr][vel_br]
        if decoded[0] != 0x10:
            print(f"ERROR: Wrong packet type! Expected 0x10, got 0x{decoded[0]:02x}")
            return None
        
        enc_fl, enc_bl, enc_fr, enc_br = struct.unpack('<iiii', decoded[1:17])
        vel_fl, vel_bl, vel_fr, vel_br = struct.unpack('<ffff', decoded[17:33])
        
        print(f"✓ Encoders: FL={enc_fl}, BL={enc_bl}, FR={enc_fr}, BR={enc_br}")
        print(f"✓ Velocities: FL={vel_fl:.3f}, BL={vel_bl:.3f}, FR={vel_fr:.3f}, BR={vel_br:.3f} rad/s")
        
        return {
            'encoders': (enc_fl, enc_bl, enc_fr, enc_br),
            'velocities': (vel_fl, vel_bl, vel_fr, vel_br)
        }
    
    except Exception as e:
        print(f"ERROR parsing feedback: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_request_response():
    """Test request/response protocol"""
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f"Connected to {PORT} at {BAUD} baud\n")
        
        # Clear buffer
        time.sleep(0.5)
        ser.reset_input_buffer()
        
        # Test 1: Send velocity command
        print("=== Test 1: Send velocity command (5.0 rad/s both) ===")
        send_velocity_command(ser, 5.0, 5.0)
        time.sleep(1.0)
        
        # Test 2: Request feedback
        print("\n=== Test 2: Request feedback ===")
        request_feedback(ser)
        feedback = read_feedback(ser, timeout=2.0)
        
        if feedback is None:
            print("FAILED: No feedback received")
            return False
        
        # Test 3: Different velocity
        print("\n=== Test 3: Send reverse command (-3.0 rad/s both) ===")
        send_velocity_command(ser, -3.0, -3.0)
        time.sleep(1.0)
        
        print("\n=== Test 4: Request feedback ===")
        request_feedback(ser)
        feedback = read_feedback(ser, timeout=2.0)
        
        if feedback is None:
            print("FAILED: No feedback received")
            return False
        
        # Test 5: Stop
        print("\n=== Test 5: Stop motors ===")
        send_velocity_command(ser, 0.0, 0.0)
        time.sleep(0.5)
        
        print("\n=== Test 6: Final feedback request ===")
        request_feedback(ser)
        feedback = read_feedback(ser, timeout=2.0)
        
        if feedback is None:
            print("FAILED: No feedback received")
            return False
        
        print("\n✅ All tests passed!")
        ser.close()
        return True
    
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    success = test_request_response()
    sys.exit(0 if success else 1)
