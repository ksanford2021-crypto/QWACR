# QWACR Test Scripts

This directory contains various test scripts for validating the QWACR motor control system.

## Current/Active Tests

### test_all_motors.py
**Purpose:** Comprehensive test of all 4 motors across multiple scenarios  
**Usage:** `python3 test_all_motors.py`  
**Tests:** Forward, reverse, left turn, right turn  
**Status:** ✅ Primary integration test - use this to verify overall system health

### test_request_response_fixed.py
**Purpose:** Validate request/response protocol for odometry feedback  
**Usage:** `python3 test_request_response_fixed.py`  
**Tests:** Send velocity commands, request encoder/velocity feedback, verify data integrity  
**Status:** ✅ Validates non-blocking feedback protocol (0x02 request → 0x10 response)

### test_fl_diagnostic.py
**Purpose:** Isolate and diagnose Front-Left motor issues  
**Usage:** `python3 test_fl_diagnostic.py`  
**Tests:** FL-only commands, FL+BL pair, all motors  
**Status:** ✅ Diagnostic tool - use when FL motor not responding

### test_debug.py
**Purpose:** Minimal working example of serial communication  
**Usage:** `python3 test_debug.py`  
**Tests:** Send velocity command, request feedback, read response  
**Status:** ✅ Reference implementation for ROS 2 node serial pattern

### test_fl_hardware.ino
**Purpose:** Direct hardware test bypassing main firmware  
**Usage:** Upload via Arduino IDE or arduino-cli  
**Tests:** Direct PWM/DIR/SLP pin control for FL motor  
**Status:** ✅ Hardware validation tool - use to verify driver/motor/wiring

## Legacy Tests (Historical)

### test_motor_command.py
Early motor control test (basic velocity commands)

### test_pid_*.py
Various PID tuning tests:
- `test_pid_combined.py` - Combined forward/reverse PID test
- `test_pid_fast.py` - Fast velocity changes
- `test_pid_reverse_fast.py` - Reverse direction testing
- `test_pid_varying.py` - Variable velocity testing

### test_reverse.py
Simple reverse direction test

### test_right_motors.py
Right motor pair isolation test

### test_raw_bytes.py
Low-level serial byte inspection

### test_simple_repeat.py
Basic repeated command test

### test_single_command.py
Single velocity command test

---

## Quick Start

**Full system validation:**
```bash
cd /home/kyle/tests
python3 test_all_motors.py
```

**Odometry protocol validation:**
```bash
python3 test_request_response_fixed.py
```

**Troubleshooting specific motor:**
```bash
python3 test_fl_diagnostic.py  # Replace FL with target motor if needed
```
