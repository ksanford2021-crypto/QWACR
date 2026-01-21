# QWACR Launch Scripts

This directory contains robust launch utilities for the QWACR robot.

## Quick Start

### Option 1: Robust Launch (Recommended)
```bash
cd ~/qwacr_ws/src/qwacr_build/scripts
./launch_robust.sh
```

This script automatically:
- Cleans up any stale serial connections
- Kills existing ROS nodes
- Prepares the serial port
- Launches the complete system

### Option 2: Manual Launch
```bash
# 1. Prepare serial port (recommended)
./prepare_serial.sh /dev/ttyACM0

# 2. Launch ROS2
cd ~/qwacr_ws
source install/setup.bash
ros2 launch qwacr_build display.launch.py serial_port:=/dev/ttyACM0 use_rviz:=false

# 3. In new terminal: Start TwistStamped bridge
cd ~/qwacr_ws && source install/setup.bash
ros2 run qwacr_build twist_to_stamped.py --ros-args -r cmd_vel:=/cmd_vel -r cmd_vel_stamped:=/diff_cont/cmd_vel

# 4. In new terminal: Start teleop
cd ~/qwacr_ws && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

## Scripts

### launch_robust.sh
**Purpose**: All-in-one robust launch with automatic serial preparation  
**Usage**: `./launch_robust.sh`  
**Environment Variables**:
- `SERIAL_PORT` - Serial port (default: `/dev/ttyACM0`)
- `USE_RVIZ` - Launch RViz (default: `false`)

**Example**:
```bash
SERIAL_PORT=/dev/ttyUSB0 USE_RVIZ=true ./launch_robust.sh
```

### prepare_serial.sh
**Purpose**: Prepare serial port for reliable communication  
**Usage**: `./prepare_serial.sh [serial_port]`  
**Actions**:
1. Kills any processes using the serial port
2. Waits for port availability
3. Tests serial communication with Arduino
4. Reports status

**Example**:
```bash
./prepare_serial.sh /dev/ttyACM0
```

### twist_to_stamped.py
**Purpose**: Bridge Twist messages to TwistStamped for DiffDriveController  
**Usage**: See teleop_twist_keyboard integration above

**Why needed**: `teleop_twist_keyboard` publishes `Twist`, but `DiffDriveController` expects `TwistStamped`.

### arduino_sim.py / serial_sim.py
**Purpose**: Arduino simulator for Gazebo testing  
**Usage**: Automatically launched when `use_gazebo:=true`

## Troubleshooting

### Controllers timeout on spawn
**Symptom**: `spawner_diff_cont: waiting for service /controller_manager/list_controllers to become available...`

**Solutions**:
1. Use `launch_robust.sh` (handles this automatically)
2. Run `python3 ~/tests/test_all_motors.py` for 3 seconds, then launch
3. Check Arduino connection: `ls -l /dev/ttyACM0`
4. Verify Arduino firmware is running (LED should blink)

### Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Motor FL not responding
- Check hardware connections (encoder, power, signal)
- Try power-cycling the Arduino
- Run diagnostic: `python3 ~/tests/test_fl_diagnostic.py`

### "Cannot find package qwacr_build"
```bash
cd ~/qwacr_ws
colcon build --packages-select qwacr_build mega_diff_drive_control
source install/setup.bash
```

## Launch File Arguments

The main launch file (`display.launch.py`) supports:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gazebo` | `false` | Launch Gazebo simulation |
| `use_rviz` | `true` | Launch RViz visualization |
| `use_sim_time` | auto | Use simulated time (auto-set based on Gazebo) |
| `serial_port` | `/tmp/ttyV0` | Serial port for Arduino |
| `baud_rate` | `115200` | Serial baud rate |
| `world` | `` | Gazebo world file path |
| `gui` | `true` | Show Gazebo GUI |

**Examples**:
```bash
# Hardware mode without RViz
ros2 launch qwacr_build display.launch.py serial_port:=/dev/ttyACM0 use_rviz:=false

# Gazebo simulation
ros2 launch qwacr_build display.launch.py use_gazebo:=true use_rviz:=true

# Custom world
ros2 launch qwacr_build display.launch.py use_gazebo:=true world:=/path/to/world.sdf
```

## System Architecture

```
launch_robust.sh
    ↓
prepare_serial.sh (cleanup + test)
    ↓
display.launch.py
    ├─→ robot_state_publisher (URDF → /robot_description)
    ├─→ ros2_control_node (hardware interface)
    ├─→ controller_manager
    ├─→ spawner joint_broad (JointStateBroadcaster)
    ├─→ spawner diff_cont (DiffDriveController)
    └─→ (optional) rviz2

In separate terminals:
twist_to_stamped.py (Twist → TwistStamped bridge)
teleop_twist_keyboard (keyboard → /cmd_vel)
```

## Development Tips

### View Controller Status
```bash
ros2 control list_controllers
```

### Check Hardware Interface
```bash
ros2 control list_hardware_interfaces
```

### Monitor Topics
```bash
# Joint states (encoder feedback)
ros2 topic echo /joint_states

# Odometry
ros2 topic echo /diff_cont/odom

# Command velocity
ros2 topic echo /diff_cont/cmd_vel
```

### Debug Serial Communication
```bash
# Raw serial monitor
screen /dev/ttyACM0 115200

# Test motor response
python3 ~/tests/test_all_motors.py
```

## Known Issues

1. **Intermittent controller timeout**: Use `launch_robust.sh` or run `test_all_motors.py` before manual launch
2. **Motor FL inconsistency**: Hardware issue being investigated, doesn't affect 3-motor operation
3. **Serial permission denied**: Add user to `dialout` group (see troubleshooting above)

## Support

For issues, check:
- Launch log: `/tmp/qwacr_launch.log`
- ROS log: `~/.ros/log/`
- GitHub: https://github.com/ksanford2021-crypto/QWACR
