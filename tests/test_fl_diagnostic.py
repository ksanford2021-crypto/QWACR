#!/usr/bin/env python3
"""
Diagnostic: Test Front Left motor individually
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

def decode_cobs(raw):
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
    return bytes(decoded)

def send_command(ser, left_vel, right_vel):
    cmd = struct.pack('<Bff', 0x01, left_vel, right_vel)
    ser.write(encode_cobs(cmd))
    ser.flush()
    time.sleep(0.1)

def request_feedback(ser):
    cmd = struct.pack('<B', 0x02)
    ser.reset_input_buffer()
    ser.write(encode_cobs(cmd))
    ser.flush()
    
    start = time.time()
    while time.time() - start < 1.0:
        if ser.in_waiting:
            raw = ser.read(ser.in_waiting)
            if 0x00 in raw:
                raw = raw[:raw.index(0x00) + 1]
                decoded = decode_cobs(raw)
                if len(decoded) >= 33:
                    enc_fl, enc_bl, enc_fr, enc_br = struct.unpack('<iiii', decoded[1:17])
                    vel_fl, vel_bl, vel_fr, vel_br = struct.unpack('<ffff', decoded[17:33])
                    return enc_fl, enc_bl, enc_fr, enc_br, vel_fl, vel_bl, vel_fr, vel_br
        time.sleep(0.02)
    return None

# Connect
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
time.sleep(1)
ser.reset_input_buffer()

print("=== FRONT LEFT MOTOR DIAGNOSTIC ===\n")

print("[Test 1] FL Only - Forward 4.0 rad/s, others 0")
send_command(ser, 4.0, 0.0)  # FL=4, others=0
time.sleep(1)
fb = request_feedback(ser)
if fb:
    enc_fl, enc_bl, enc_fr, enc_br, vel_fl, vel_bl, vel_fr, vel_br = fb
    print(f"  FL encoder: {enc_fl}, velocity: {vel_fl:.3f} rad/s")
    print(f"  BL encoder: {enc_bl}, velocity: {vel_bl:.3f} rad/s")
    print(f"  FR encoder: {enc_fr}, velocity: {vel_fr:.3f} rad/s")
    print(f"  BR encoder: {enc_br}, velocity: {vel_br:.3f} rad/s")
    if vel_fl > 0.5:
        print("  ✓ FL responding")
    else:
        print("  ✗ FL NOT responding")
else:
    print("  ✗ No feedback received")

print("\n[Test 2] FL + BL (Left pair) - Forward 4.0 rad/s")
send_command(ser, 4.0, 0.0)
time.sleep(1)
fb = request_feedback(ser)
if fb:
    enc_fl, enc_bl, enc_fr, enc_br, vel_fl, vel_bl, vel_fr, vel_br = fb
    print(f"  FL encoder: {enc_fl}, velocity: {vel_fl:.3f} rad/s")
    print(f"  BL encoder: {enc_bl}, velocity: {vel_bl:.3f} rad/s")
    if vel_fl > 0.5 and vel_bl > 0.5:
        print("  ✓ Both FL and BL responding")
    elif vel_fl > 0.5:
        print("  ⚠ Only FL responding, BL stuck")
    elif vel_bl > 0.5:
        print("  ⚠ Only BL responding, FL stuck")
    else:
        print("  ✗ Neither FL nor BL responding")
else:
    print("  ✗ No feedback received")

print("\n[Test 3] Stop - Check if FL stops")
send_command(ser, 0.0, 0.0)
time.sleep(0.5)
fb = request_feedback(ser)
if fb:
    enc_fl, enc_bl, enc_fr, enc_br, vel_fl, vel_bl, vel_fr, vel_br = fb
    print(f"  FL velocity: {vel_fl:.3f} rad/s")
    print(f"  BL velocity: {vel_bl:.3f} rad/s")
    if abs(vel_fl) < 0.1 and abs(vel_bl) < 0.1:
        print("  ✓ Both motors stopped")
    else:
        print("  ✗ Motors still moving")
else:
    print("  ✗ No feedback received")

print("\n[Test 4] Check all motors still responding")
send_command(ser, 2.0, 2.0)
time.sleep(1)
fb = request_feedback(ser)
if fb:
    enc_fl, enc_bl, enc_fr, enc_br, vel_fl, vel_bl, vel_fr, vel_br = fb
    print(f"  FL: {vel_fl:.3f} rad/s")
    print(f"  BL: {vel_bl:.3f} rad/s")
    print(f"  FR: {vel_fr:.3f} rad/s")
    print(f"  BR: {vel_br:.3f} rad/s")
    active = sum([1 for v in [vel_fl, vel_bl, vel_fr, vel_br] if abs(v) > 0.5])
    print(f"  {active}/4 motors responding")
else:
    print("  ✗ No feedback received")

send_command(ser, 0.0, 0.0)
ser.close()
print("\n=== DIAGNOSTIC COMPLETE ===")
