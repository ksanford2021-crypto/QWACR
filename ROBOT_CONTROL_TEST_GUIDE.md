# QWACR Pi SSH Access & Robot Control Testing Guide

**Date:** February 3, 2026  
**Hardware:** Raspberry Pi 5, Arduino Mega, 4-motor differential drive  
**Location:** Campus (via ethernet)

---

## Part 1: SSH Access via Ethernet on Campus WiFi

### Network Configuration Options

**Option A: Direct Ethernet Connection (Recommended for initial testing)**
- Your laptop ethernet → Pi ethernet
- Pi will use **campus WiFi** for internet
- Access Pi at: `192.168.11.100` (or use mDNS: `QWACRpi.local`)

**Option B: Pi as WiFi Access Point**
- Pi broadcasts WiFi network: `QWACR-Pi` (password: `qwacr2026`)
- Connect your laptop to Pi's WiFi
- Access Pi at: `192.168.4.1`

---

### Method 1: Direct Ethernet Connection (EASIEST)

#### On Your Pi (if not already configured):
```bash
# Configure static IP on ethernet for direct connection
sudo tee -a /etc/dhcpcd.conf << EOF

# Static IP for direct ethernet connection
interface eth0
    static ip_address=192.168.11.100/24
    static routers=192.168.11.1
EOF

sudo systemctl restart dhcpcd
```

#### On Your Laptop:
```bash
# 1. Connect ethernet cable between laptop and Pi

# 2. Configure your laptop's ethernet interface
# Linux/Mac:
sudo ip addr add 192.168.11.50/24 dev eth0  # Replace eth0 with your interface name
sudo ip link set eth0 up

# Or use NetworkManager GUI:
# - IPv4 Method: Manual
# - Address: 192.168.11.50
# - Netmask: 255.255.255.0
# - Gateway: (leave empty)

# 3. Test connectivity
ping 192.168.11.100

# 4. SSH to Pi
ssh qwacr@192.168.11.100
# Password: (your Pi password)
```

#### Alternative: Use mDNS (avahi)
```bash
# If mDNS is working on your network:
ping QWACRpi.local
ssh qwacr@QWACRpi.local
```

---

### Method 2: WiFi Access Point Mode

If direct ethernet doesn't work, switch Pi to AP mode:

#### On Your Pi (via keyboard/monitor or existing SSH):
```bash
# Switch to Access Point mode
/home/qwacr/switch_to_ap.sh
# Pi will reboot
```

#### After Pi Reboots:
```bash
# 1. On your laptop, connect to WiFi:
#    SSID: QWACR-Pi
#    Password: qwacr2026

# 2. SSH to Pi
ssh qwacr@192.168.4.1
```

---

## Part 2: Initial Pi Setup Check

Once SSH'd into the Pi, verify the setup:

```bash
# 1. Check ROS2 installation
ros2 --version
# Expected: ros2 doctor version 0.11.0

# 2. Check workspace
source ~/qwacr_ws/install/setup.bash
ros2 pkg list | grep qwacr
# Expected: qwacr_build, qwacr_gps, qwacr_navigation

# 3. Check serial devices
ls -l /dev/ttyACM*
# Expected: /dev/ttyACM0 (or similar) for Arduino

# 4. Check Arduino connection
dmesg | tail -20
# Look for: "USB ACM device" or "Arduino Mega" connection messages
```

---

## Part 3: Arduino-Pi Connection Test

### Step 1: Identify Arduino Serial Port

```bash
# List all USB serial devices
ls -l /dev/ttyACM* /dev/ttyUSB*

# Most likely:
# /dev/ttyACM0 - GPS module
# /dev/ttyACM1 - Arduino Mega (if GPS is on ACM0)

# Check which device is which:
dmesg | grep -i "tty"

# Or use udevadm:
udevadm info -a -n /dev/ttyACM1 | grep -i "product\|manufacturer"
```

### Step 2: Test Serial Communication

```bash
# Source workspace
source ~/qwacr_ws/install/setup.bash

# Test serial port availability
bash ~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh /dev/ttyACM1
# Expected: ✓ Serial port ready

# Check if Arduino is responding (optional - low level test)
# Install screen if not already:
sudo apt install screen -y

# Open serial monitor (115200 baud):
screen /dev/ttyACM1 115200
# You should see nothing initially - Arduino is waiting for commands
# Press Ctrl+A then K to exit screen
```

### Step 3: Launch ROS2 Hardware Interface

```bash
# Terminal 1: Launch robot state publisher and controllers
source ~/qwacr_ws/install/setup.bash

# Use the display launch file (includes controllers)
ros2 launch qwacr_build display.launch.py \
    use_hardware:=true \
    serial_port:=/dev/ttyACM1 \
    baud_rate:=115200

# Expected output:
# - robot_state_publisher started
# - controller_manager started
# - joint_state_broadcaster spawned
# - diff_drive_controller spawned
# - No errors about serial port
```

**Common Issues:**
- **Error: "Failed to open serial port"**
  - Check port name: `ls -l /dev/ttyACM*`
  - Check permissions: `sudo usermod -a -G dialout $USER` then logout/login
  - Check if another process is using it: `lsof /dev/ttyACM1`

- **Error: "Controller failed to spawn"**
  - Arduino not responding - check USB cable
  - Wrong baud rate - should be 115200
  - Arduino firmware not uploaded

---

## Part 4: Robot Control Tests

### Test 1: Check Published Topics

```bash
# In a new terminal (Terminal 2):
source ~/qwacr_ws/install/setup.bash

# List active topics
ros2 topic list
# Expected topics:
# /cmd_vel (input for robot)
# /diff_cont/cmd_vel_unstamped
# /diff_cont/odom (wheel odometry output)
# /joint_states (motor joint states)
# /tf (transforms)

# Check odometry output
ros2 topic echo /diff_cont/odom --once
# Should show position, orientation, velocities
```

### Test 2: Teleop Keyboard Control

```bash
# Terminal 2: Launch keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

# Controls will appear:
# i = forward
# , = backward
# j = rotate left
# l = rotate right
# k = stop
# q/z = increase/decrease max speeds
```

**Safety:** Start with LOW speeds! Press 'z' several times before moving.

### Test 3: Manual Velocity Commands

```bash
# Terminal 2: Send test commands

# Drive forward slowly (0.2 m/s)
ros2 topic pub -1 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Stop
ros2 topic pub -1 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place (0.3 rad/s)
ros2 topic pub -1 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Stop
ros2 topic pub -1 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Test 4: Monitor Encoder Feedback

```bash
# Terminal 3: Watch encoder data
source ~/qwacr_ws/install/setup.bash

ros2 topic echo /diff_cont/odom

# You should see:
# - position.x, position.y increasing as robot moves
# - orientation.z changing as robot rotates
# - twist.linear.x showing forward velocity
# - twist.angular.z showing rotation velocity
```

### Test 5: Check Joint States (Low-Level Feedback)

```bash
# Watch raw joint states (motor angles)
ros2 topic echo /joint_states

# You should see 4 joints:
# - wheel_front_left_joint
# - wheel_back_left_joint
# - wheel_front_right_joint
# - wheel_back_right_joint
# Each with position (radians) and velocity (rad/s)
```

---

## Part 5: Troubleshooting

### Arduino Not Responding

```bash
# 1. Check Arduino is detected
lsof /dev/ttyACM1
dmesg | tail -20

# 2. Check permissions
ls -l /dev/ttyACM1
# Should show: crw-rw---- 1 root dialout ...

# Add user to dialout group if needed:
sudo usermod -a -G dialout qwacr
# Then logout and login

# 3. Test with screen
screen /dev/ttyACM1 115200
# Should connect without errors
# Ctrl+A then K to exit

# 4. Check if Arduino firmware is uploaded
# Re-upload qwacr_main.ino using Arduino IDE if needed
```

### Motors Not Moving

```bash
# Check if commands are being sent:
ros2 topic echo /diff_cont/cmd_vel_unstamped

# Check motor driver power:
# - 24V power supply ON
# - Motor drivers have power LED lit
# - Sleep pins controlled by Arduino (should be HIGH when active)

# Check Arduino firmware:
# - Sleep pin management correct
# - Motor directions correct
# - PID controller running
```

### High CPU Usage / Lag

```bash
# Check CPU usage
htop

# If high CPU from ROS2:
# - Reduce topic rates
# - Check for topic storms (echo topics to see rate)

# Optimize:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Add to ~/.bashrc if not already there
```

### Network Issues

```bash
# If SSH disconnects frequently:
# 1. Use ethernet instead of WiFi (more stable)
# 2. Disable power management on WiFi:
sudo iw dev wlan0 set power_save off

# 3. Keep SSH alive:
# Add to ~/.ssh/config on your laptop:
Host QWACRpi
    HostName 192.168.11.100
    User qwacr
    ServerAliveInterval 60
    ServerAliveCountMax 3
```

---

## Part 6: Complete Test Checklist

### Pre-Flight Checklist
- [ ] Motors connected to 24V power supply
- [ ] Arduino Mega connected to Pi via USB
- [ ] Pi powered on and connected to network
- [ ] SSH access working
- [ ] All emergency stops accessible
- [ ] Robot on blocks or in open space

### Hardware Interface Test
- [ ] `/dev/ttyACM1` (or correct port) accessible
- [ ] `display.launch.py` starts without errors
- [ ] `/diff_cont/odom` publishing
- [ ] `/joint_states` publishing

### Motion Control Test
- [ ] Teleop keyboard responds to commands
- [ ] Forward motion works (i key)
- [ ] Backward motion works (, key)
- [ ] Left rotation works (j key)
- [ ] Right rotation works (l key)
- [ ] Stop command works (k key)
- [ ] All 4 motors respond

### Encoder Feedback Test
- [ ] Odometry position updates when driving
- [ ] Odometry orientation updates when rotating
- [ ] Joint states show correct velocities
- [ ] No encoder dropout or errors

### Safety Test
- [ ] Emergency stop accessible
- [ ] Motors stop immediately when commanded
- [ ] No unexpected motion on startup
- [ ] Sleep pins controlling motor drivers correctly

---

## Quick Command Reference

```bash
# SSH to Pi (ethernet)
ssh qwacr@192.168.11.100

# SSH to Pi (WiFi AP mode)
ssh qwacr@192.168.4.1

# Source workspace
source ~/qwacr_ws/install/setup.bash

# Launch robot interface
ros2 launch qwacr_build display.launch.py use_hardware:=true serial_port:=/dev/ttyACM1

# Launch teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

# Check topics
ros2 topic list
ros2 topic echo /diff_cont/odom
ros2 topic echo /joint_states

# Stop all ROS2 nodes
killall -9 python3 ros2

# Check serial devices
ls -l /dev/ttyACM*
dmesg | grep tty
```

---

## Success Criteria

✅ **SSH Connection Working:** Can connect reliably via ethernet or WiFi  
✅ **Arduino Detected:** Shows up as `/dev/ttyACM1` with correct permissions  
✅ **Hardware Interface Running:** Controllers spawned without errors  
✅ **Topics Publishing:** Odometry and joint states active  
✅ **Motion Control:** Robot responds to keyboard/manual commands  
✅ **Encoder Feedback:** Position and velocity data updating correctly  
✅ **Safe Operation:** Can stop robot immediately, no unexpected motion

---

## Next Steps After Basic Control Works

1. **Sensor Integration:** Add Aurora SLAM and GPS
2. **EKF Fusion:** Test localization.launch.py
3. **Navigation Stack:** Test full_system.launch.py
4. **Field Testing:** Outdoor navigation with GPS waypoints

**Document any issues encountered for debugging!**
