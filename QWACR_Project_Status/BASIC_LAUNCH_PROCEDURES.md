# Basic Launch Procedures for QWACR Robot

**Date:** February 4, 2026  
**Status:** Verified Working on Dev Machine with Arduino Mega 2560

## Overview
This document provides step-by-step procedures for launching the ros2_control stack with teleop keyboard control.

## Prerequisites
- Arduino Mega 2560 connected to USB port (or Raspberry Pi with Arduino)
- ROS2 Jazzy workspace sourced
- Arduino firmware uploaded (COBS binary protocol at 115200 baud)
- ROS daemon running (`ros2 daemon start` if needed)

---

## Launch Procedure

### Step 1: Identify Serial Port
```bash
ls /dev/ttyACM*
# or
ls /dev/ttyUSB*
```
**Expected:** `/dev/ttyACM0` (Arduino Mega) or similar

---

### Step 2: Launch ros2_control Stack
Open a new terminal and run:
```bash
cd ~/qwacr_ws
~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh /dev/ttyACM0
```

**What this does:**
- Runs `test_all_motors.py` for 3 seconds (workaround for serial initialization race condition)
- Launches hardware interface and controller_manager
- Spawns `diff_cont` (DiffDriveController) and `joint_broad` (JointStateBroadcaster)

**Expected output:**
- "Feedback parsed successfully" messages at ~30Hz
- COBS packet communication logs
- "Successfully activated controller diff_cont"
- "Successfully activated controller joint_broad"

**Wait until:** Controllers show as ACTIVE (check logs or run `ros2 control list_controllers` in another terminal)

---

### Step 3: Verify Controllers Active (Optional)
In a new terminal:
```bash
source ~/qwacr_ws/install/setup.bash
ros2 control list_controllers
```

**Expected output:**
```
diff_cont         diff_drive_controller/DiffDriveController    active
joint_broad       joint_state_broadcaster/JointStateBroadcaster active
```

---

### Step 4: Launch Twist-to-Stamped Bridge
Open a new terminal and run:
```bash
source ~/qwacr_ws/install/setup.bash
ros2 run qwacr_build twist_to_stamped.py \
  --ros-args \
  -r cmd_vel:=/cmd_vel \
  -r cmd_vel_stamped:=/diff_cont/cmd_vel
```

**IMPORTANT:** 
- The output topic MUST be `/diff_cont/cmd_vel` (NOT `/diff_cont/cmd_vel_stamped`)
- The controller subscribes to `/diff_cont/cmd_vel` and expects `TwistStamped` messages

**What this does:**
- Subscribes to `/cmd_vel` (Twist messages from teleop)
- Converts to TwistStamped with proper frame_id
- Publishes to `/diff_cont/cmd_vel` (controller input)

---

### Step 5: Launch Teleop Keyboard
Open a new terminal and run:
```bash
source ~/qwacr_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q/z` - Increase/decrease max speeds
- `w/x` - Increase/decrease only linear speed
- `e/c` - Increase/decrease only angular speed
- `Ctrl+C` - Exit

**Expected behavior:**
- Pressing keys should immediately move the motors
- Release key to stop (with cmd_vel_timeout)

---

## Verification Steps

### Check Topic Publishing
```bash
# Check teleop output
ros2 topic echo /cmd_vel

# Check bridge output
ros2 topic echo /diff_cont/cmd_vel

# Check topic connections
ros2 topic info /diff_cont/cmd_vel
```

**Expected for `/diff_cont/cmd_vel`:**
- Type: `geometry_msgs/msg/TwistStamped`
- Publisher count: 1 (twist_to_stamped.py)
- Subscription count: 1 (controller_manager)

---

## Troubleshooting

### Motors Not Responding
1. **Check controller subscription:**
   ```bash
   ros2 topic info /diff_cont/cmd_vel
   ```
   - Must show 1 subscriber (the controller)
   - Must show 1 publisher (the bridge)

2. **Check bridge remapping:**
   - Verify bridge is publishing to `/diff_cont/cmd_vel` (NOT `cmd_vel_stamped`)
   - Kill bridge and restart with correct remapping

3. **Check hardware interface:**
   - Look for "Feedback parsed successfully" in launch_robust.sh terminal
   - Should see continuous COBS packet communication

4. **Check ROS daemon:**
   ```bash
   ros2 daemon status
   ```
   - If daemon is off, CLI tools won't see active nodes
   - Start with: `ros2 daemon start`

### Controller Spawn Timeout
- **Solution:** Use `launch_robust.sh` - it includes the `test_all_motors.py` workaround
- **Root cause:** Serial port initialization race condition
- **Manual workaround:** Run `~/tests/test_all_motors.py` for 3 seconds before launching

### Serial Port Permission Denied
```bash
sudo chmod 666 /dev/ttyACM0
# or add user to dialout group:
sudo usermod -a -G dialout $USER
# then logout/login
```

---

## Shutdown Procedure

1. Stop teleop keyboard: `Ctrl+C` in teleop terminal
2. Stop bridge: `Ctrl+C` in bridge terminal (or `pkill -f twist_to_stamped.py`)
3. Stop ros2_control: `Ctrl+C` in launch_robust.sh terminal

**Note:** Controllers will cleanly deactivate and hardware interface will close serial port.

---

## Known Working Configuration

- **Dev Machine:** Ubuntu 24.04 (WSL2), ROS2 Jazzy
- **Hardware:** Arduino Mega 2560 at /dev/ttyACM0
- **controller_manager:** 4.42.2-1noble.20260126.195504
- **diff_drive_controller:** 4.36.0-1noble.20260126.195627
- **Serial Protocol:** COBS binary at 115200 baud
- **Workaround:** test_all_motors.py (3 sec) before launch

---

## Raspberry Pi Specific Notes

### Bridge Script Installation Issue
If `ros2 run qwacr_build twist_to_stamped.py` fails with "No executable found", run directly with python3:
```bash
python3 ~/qwacr_ws/src/qwacr_build/scripts/twist_to_stamped.py \
  --ros-args \
  -r cmd_vel:=/cmd_vel \
  -r cmd_vel_stamped:=/diff_cont/cmd_vel
```

### SSH from Windows
If using USB ethernet adapter:
- SSH from PowerShell works even when adapter attached to WSL2
- WSL2 can't SSH when USB device is attached to it
- From PowerShell: `ssh qwacr@192.168.11.100`

---

## Next Steps / TODO

- [x] Test procedure on Raspberry Pi 5 with same Arduino - ✅ Working perfectly!
- [x] Test procedure on Raspberry Pi 5 over SSH (from dev machine) - ✅ PowerShell SSH works
- [ ] Document autonomous navigation launch procedures
- [ ] Add RViz visualization procedures
- [ ] Create combined launch file to simplify startup
- [ ] Integrate LiDAR and GPS with ros2_control stack
