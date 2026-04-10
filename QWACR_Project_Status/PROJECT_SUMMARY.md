# QWACR Project Summary

**Quad-Wheel Autonomous Clamping Robot**  
**Status**: Phase 4 Complete - Nav2 navigation stack configured and verified, ready for hardware integration  
**Date**: January 28, 2026

## System Overview

QWACR is a 4-wheel differential drive autonomous robot with:
- Arduino Mega 2560 motor controller with Hall encoder feedback
- ROS2 Jazzy hardware interface for real-time control
- Aurora SLAM sensor (LiDAR + visual + inertial SLAM with onboard processing)
- RTK GPS with dual-antenna heading (cm-level positioning, sub-degree heading)
- robot_localization dual EKF for local (odom) and global (map) sensor fusion
- Nav2 autonomous navigation stack with GPS waypoint following
- Battery-powered operation (24V 256Wh LiFePO4)
- Velocity tracking: 97% @ 3 rad/s, 83% @ 6 rad/s

## Architecture

### Hardware Stack
- **Motors**: 4× Pololu G2 24V 19:1 motors (100:1 effective with gearbox)
- **Encoders**: Hall effect encoders, 3200 counts/rev after quadrature
- **Controller**: Arduino Mega 2560 @ 115200 baud serial
- **Aurora SLAM**: Network-connected LiDAR/visual/inertial SLAM sensor (192.168.11.1)
- **GPS**: RTK GPS with dual-antenna heading (cm-level, sub-degree accuracy)
- **Power**: 24V 256Wh LiFePO4 battery with 15A fuse

### Software Stack
```
GPS Waypoint → gps_waypoint_follower → Nav2 (NavigateThroughPoses action)
                                              ↓
                                    BT Navigator + Global Planner
                                              ↓
                                    Local Planner (DWB Controller)
                                              ↓
                               /cmd_vel → TwistStamped Bridge → /diff_cont/cmd_vel_stamped
                                                                       ↓
                                                             DiffDriveController
                                                                       ↓
                                                        MegaDiffDriveHardware
                                                                       ↓
                                                       Arduino (PID + Encoders)

Sensors → robot_localization (Dual EKF) → /odometry/local, /odometry/global → Nav2
  ├─ Aurora SLAM: /odom, /scan, /imu
  ├─ GPS: /gps/enu_odom (position, velocity, heading)
  └─ Wheel Encoders: /joint_states
```

## Key Components

### 1. Arduino Firmware (`qwacr_arduino/qwacr_main/`)
- **Control**: Feedforward + PID (Kff=32, Kp=25, Ki=0.3, Kd=0.2)
- **Protocol**: COBS (Consistent Overhead Byte Stuffing) serial encoding
- **Loop Rate**: 30Hz control loop
- **Features**:
  - Velocity command processing (left/right pairs)
  - Encoder position tracking with direction detection (XOR logic)
  - Velocity calculation with exponential moving average
  - Binary feedback packets (encoder counts + velocities)

### 2. ROS2 Hardware Interface (`qwacr_ws/src/mega_diff_drive_control/`)
- **Class**: `MegaDiffDriveHardware` (SystemInterface plugin)
- **Communication**: Non-blocking POSIX serial I/O
- **Lifecycle**: Managed hardware interface with on_activate/on_deactivate
- **Interfaces**:
  - Command: velocity (4 wheels)
  - State: position (rad), velocity (rad/s)
- **Packet Format**:
  - Request: `0x02` (feedback request)
  - Command: `0x01 + 8 bytes (2× float velocities)`
  - Feedback: `0x10 + 32 bytes (4× int32 encoders + 4× float velocities)`

### 3. Controller Configuration (`qwacr_ws/src/qwacr_build/config/`)
- **DiffDriveController**: Differential drive kinematics with odometry
- **Parameters**:
  - Wheel separation: 0.295m
  - Wheel radius: 0.055m
  - Max linear velocity: 0.5 m/s
  - Max angular velocity: 1.0 rad/s
  - Odometry publishing: 30Hz on `/diff_cont/odom`
  - TF broadcasting: `odom → base_link`

### 4. Robot Description (`qwacr_ws/src/qwacr_build/urdf/`)
- **URDF**: Complete robot model with 4 wheels, base, and caster
- **Joints**: 4× continuous joints for wheel rotation
- **Inertials**: Realistic mass distribution (total ~10kg)
- **Visuals**: Cylinder wheels (0.11m diameter) + rectangular base

## Performance Metrics

### Velocity Tracking (Battery Power, 24V)
| Target (rad/s) | Achieved | Error | Tracking % |
|----------------|----------|-------|------------|
| 3.0            | 2.91     | -3%   | 97%        |
| 6.0            | 5.0      | -17%  | 83%        |

### Control Loop Timing
- Arduino loop: 30Hz (33ms period)
- ROS2 controller: 30Hz synchronized
- Serial latency: ~10-20ms roundtrip

## Known Issues & Workarounds

### 1. Controller Spawn Timeout (Intermittent)
**Symptom**: Controllers timeout waiting for hardware interface on some launches  
**Workaround**: Run `python3 tests/test_all_motors.py` for 3 seconds before launch  
**Root Cause**: Serial port initialization race condition  
**Status**: Documented, workaround reliable

### 2. Motor FL Inconsistency
**Symptom**: Front-left motor occasionally unresponsive or erratic  
**Investigation**: Hardware connection issue (loose encoder/power)  
**Status**: Monitoring, does not affect testing with 3 working motors

## File Structure

```
/home/kyle/
├── qwacr_ws/                          # ROS2 workspace
│   ├── src/
│   │   ├── mega_diff_drive_control/   # Hardware interface plugin
│   │   │   ├── src/mega_diff_drive_hardware.cpp
│   │   │   └── include/.../mega_diff_drive_hardware.hpp
│   │   ├── qwacr_build/               # Robot description & launch files
│   │   │   ├── urdf/qwacr.urdf.xacro
│   │   │   ├── config/diff_drive_controller.yaml
│   │   │   ├── launch/display.launch.py
│   │   │   └── scripts/twist_to_stamped.py
│   │   └── ros_controls.jazzy.repos    # ros2_control dependencies
│   ├── install/                        # Built packages
│   └── build/                          # Build artifacts
├── qwacr_arduino/                     # Arduino firmware
│   └── qwacr_main/
│       ├── qwacr_main.ino             # Main control loop
│       ├── motor_control.h            # PWM + PID implementation
│       ├── quadrature_decoder.h       # Encoder ISR handlers
│       ├── serial_protocol.h          # COBS encoding/decoding
│       └── test_request_response.py   # Serial communication test
└── tests/                             # Validation scripts
    ├── test_all_motors.py             # 4-motor sequence test
    ├── test_pid_combined.py           # PID tuning validation
    └── README.md                      # Test documentation
```

## Launch Instructions

### 1. Standard Launch (with workaround)
```bash
# Clear serial blockage
cd ~/tests && timeout 3 python3 test_all_motors.py

# Launch hardware interface + controllers
cd ~/qwacr_ws
source install/setup.bash
ros2 launch qwacr_build display.launch.py serial_port:=/dev/ttyACM0

# In new terminal: Start TwistStamped bridge
ros2 run qwacr_build twist_to_stamped.py --ros-args -r cmd_vel:=/cmd_vel -r cmd_vel_stamped:=/diff_cont/cmd_vel

# In new terminal: Start teleop keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### 2. Verify Odometry
```bash
# Check odometry topic
ros2 topic echo /diff_cont/odom

# Check TF tree
ros2 run tf2_tools view_frames

# Verify base_link → odom transform
ros2 run tf2_ros tf2_echo odom base_link
```

## Development Timeline

1. **Phase 1**: Arduino firmware development with PID tuning
2. **Phase 2**: ROS2 hardware interface with serial communication
3. **Phase 3**: Controller integration and teleop testing
4. **Phase 4**: Encoder feedback and velocity tracking (CURRENT)
5. **Phase 5** (Next): Increase robustness, wireless control, autonomous navigation

## Technical Achievements

✅ **Feedforward + PID control** with 97% velocity tracking  
✅ **COBS serial protocol** with CRC validation  
✅ **Non-blocking serial I/O** in ROS2 hardware interface  
✅ **Encoder feedback** with XOR direction detection  
✅ **Odometry publishing** via DiffDriveController  
✅ **TF broadcasting** for odom → base_link transform  
✅ **Keyboard teleop** with responsive motor control  
✅ **Battery operation** with stable 24V power delivery  

## Next Steps (Phase 5+)

1. **Robustness Improvements**:
   - Fix serial initialization race condition
   - Add automatic recovery from controller timeouts
   - Investigate FL motor hardware issue

2. **Wireless Control**:
   - Add ESP32/WiFi module for remote operation
   - Implement safety watchdog (auto-stop on connection loss)

3. **Autonomous Navigation**:
   - Integrate sensors (LIDAR, IMU, cameras)
   - Implement SLAM with Nav2
   - Add obstacle avoidance

4. **Clamping Mechanism**:
   - Design and integrate gripper hardware
   - Add gripper control to ROS2 interface

## Repository

**GitHub**: https://github.com/ksanford2021-crypto/QWACR  
**Branch**: master  
**Latest Commit**: Complete teleop system with encoder feedback and odometry

## Contact & Notes

- Controller spawn timeout workaround is 100% reliable
- All 4 motors functional with FL occasionally requiring reset
- System tested with battery power for extended operation (>1 hour)
- Ready for Phase 5 development (robustness & wireless)
