# QWACR Robust Launch Scripts

Quick reference for using the improved launch scripts with automatic serial warmup.

## Overview

The robust launch scripts automatically perform serial port warmup (the `picocom` method) before launching ros2_control. This eliminates the manual step and ensures consistent, reliable startup.

## Scripts Available

### 1. Motor Control Robust Launch
**Location:** `~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh`

**Usage:**
```bash
# Basic (defaults to /dev/ttyACM1, 115200 baud)
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh

# Specify serial port
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh /dev/ttyACM0

# Specify port and baud rate
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh /dev/ttyACM1 115200
```

**What it does:**
1. Checks if serial port exists
2. Kills any processes using the port
3. Performs 2-second serial warmup with picocom (or stty fallback)
4. Launches mega_diff_drive_control

**Use this for:** Nav2 testing, manual motor control, any scenario where you need reliable ros2_control startup

---

### 2. Serial Port Preparation Script
**Location:** `~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh`

**Usage:**
```bash
# Prepare default port (/dev/ttyACM0)
~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh

# Prepare specific port
~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh /dev/ttyACM1

# Specify port and baud rate
~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh /dev/ttyACM1 115200
```

**What it does:**
1. Kills processes using the port
2. Waits for port to be available (up to 15 seconds)
3. Performs serial warmup with picocom or stty
4. Verifies port is still accessible

**Use this for:** Standalone serial warmup before manual ros2 launch commands

---

### 3. Full System Robust Launch
**Location:** `~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh`

**Usage:**
```bash
# Launch with default settings
~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh

# With custom serial port
SERIAL_PORT=/dev/ttyACM1 ~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh

# With RViz
USE_RVIZ=true ~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh

# All options
SERIAL_PORT=/dev/ttyACM1 BAUD_RATE=115200 USE_RVIZ=true ~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh
```

**What it does:**
1. Calls prepare_serial.sh for serial warmup
2. Cleans up any existing ROS nodes
3. Launches qwacr_build display.launch.py with tee logging to /tmp/qwacr_launch.log
4. Shows helpful error messages if launch fails

**Use this for:** Full system testing with visualization

---

## Testing the Scripts

### Test Serial Warmup Alone
```bash
# Should complete successfully with "Serial port ready" message
~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh /dev/ttyACM1

# Check exit code (0 = success)
echo $?
```

### Test Motor Launch
```bash
# Should launch motors without controller_manager errors
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh /dev/ttyACM1

# Verify controllers loaded
ros2 control list_controllers
# Should show: diff_cont, joint_broad (both active)
```

### Test Full System
```bash
# Should launch complete system with visualization
USE_RVIZ=true ~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh

# Check log if issues
cat /tmp/qwacr_launch.log
```

---

## Integration with Phase 2 Testing

### For Manual Motor Testing:
Replace:
```bash
picocom /dev/ttyACM1 -b 115200
# Ctrl+A Ctrl+X
ros2 launch mega_diff_drive_control mega_diff_drive.launch.py
```

With:
```bash
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh /dev/ttyACM1
```

### For Nav2 Full System Testing:
Use in Phase 4 Full System Launch:
```bash
# Terminal on Pi
ssh qwacr@192.168.11.100
source ~/qwacr_ws/install/setup.bash

# Option 1: Use robust motor launch for one component
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh /dev/ttyACM1

# Option 2: Use full system launch (if it includes motors)
cd ~/qwacr_ws
ros2 launch qwacr_navigation full_system.launch.py \
  serial_port:=/dev/ttyACM1 \
  gps_serial_port:=/dev/ttyACM0 \
  gps_baud_rate:=460800
```

**Note:** The full_system.launch.py does NOT currently call prepare_serial.sh automatically. You should either:
1. Run `prepare_serial.sh /dev/ttyACM1` before launching full_system, OR
2. Use the robust motor launch script instead of the full_system launch

---

## Troubleshooting

### Script says "picocom not found"
The script will automatically fall back to stty. If neither tool is available:
```bash
# Install picocom
sudo apt install picocom

# Or verify stty is available (should be built-in)
which stty
```

### Script fails with "Serial port disappeared after warmup"
This indicates the Arduino is resetting during warmup (unexpected). Try:
```bash
# Check USB cable quality
# Try different USB port
# Power-cycle the Arduino
# Check Arduino firmware is stable
```

### ros2_control still fails to connect after warmup
The warmup succeeded but something else is wrong:
```bash
# Check Arduino is responding
picocom /dev/ttyACM1 -b 115200
# Should see some output or be able to type

# Check permissions
ls -l /dev/ttyACM1
# Should show: crw-rw---- ... dialout

# Add user to dialout group if needed
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Need to transfer scripts to Pi
```bash
# From dev machine WSL2
cd ~/qwacr_ws
scp -r src/qwacr_build/scripts qwacr@192.168.11.100:~/qwacr_ws/src/qwacr_build/
scp -r src/mega_diff_drive_control/scripts qwacr@192.168.11.100:~/qwacr_ws/src/mega_diff_drive_control/

# On Pi, rebuild packages
ssh qwacr@192.168.11.100
cd ~/qwacr_ws
colcon build --packages-select qwacr_build mega_diff_drive_control
source install/setup.bash
```

---

## Technical Details

### Serial Warmup Method

The scripts use `timeout` + `picocom` for warmup:
```bash
timeout 2 picocom "$SERIAL_PORT" -b "$BAUD_RATE" --quiet > /dev/null 2>&1 || true
```

This:
1. Opens serial connection with picocom
2. Automatically closes after 2 seconds (timeout kills it)
3. Discards output (> /dev/null 2>&1)
4. Ignores timeout exit code (|| true)
5. Results in Arduino being ready for ros2_control

**Fallback method** (if picocom unavailable):
```bash
stty -F "$SERIAL_PORT" "$BAUD_RATE" raw -echo
timeout 2 cat "$SERIAL_PORT" > /dev/null 2>&1 || true
```

### Why This Works

Arduino's CDC (Communication Device Class) USB serial:
1. Resets when first serial connection opens
2. Bootloader runs briefly
3. Sketch starts and initializes COBS protocol
4. Second connection (ros2_control) finds Arduino already initialized
5. No reset occurs, COBS communication works immediately

Without warmup, ros2_control opens the *first* connection, triggers reset, and times out waiting for response.

---

## Script Maintenance

### To modify warmup duration:
Edit the `timeout 2` value in `prepare_serial.sh`:
```bash
# Increase to 3 seconds if Arduino needs more time
timeout 3 picocom "$SERIAL_PORT" ...

# Decrease to 1 second if working reliably
timeout 1 picocom "$SERIAL_PORT" ...
```

### To add debug output:
Remove `--quiet` and output redirects:
```bash
# Shows picocom connection details
timeout 2 picocom "$SERIAL_PORT" -b "$BAUD_RATE"
```

### To make scripts work on other systems:
The scripts check for tool availability and provide fallbacks:
- Primary: picocom (most reliable)
- Fallback: stty + cat (universal)
- Last resort: sleep delay only

Install picocom for best results:
```bash
sudo apt install picocom   # Debian/Ubuntu
sudo yum install picocom   # RHEL/CentOS
brew install picocom       # macOS
```

---

**Created:** February 6, 2026  
**Version:** 1.0  
**Status:** Tested and working
