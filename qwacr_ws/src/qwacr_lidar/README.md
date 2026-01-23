# QWACR LiDAR Package

ROS2 launch configuration for the SLAMTEC Aurora LiDAR on the QWACR robot.

## Hardware: SLAMTEC Aurora

- **Range**: 20 meters
- **Angular Resolution**: 0.5°
- **Scan Rate**: 10-20 Hz (configurable)
- **Interface**: USB (typically `/dev/ttyUSB0` or `/dev/ttyUSB1`)
- **Baud Rate**: 256000

## Features

- Pre-configured Aurora-specific parameters (range, scan rate)
- Static TF transform: `base_link` → `laser_frame`
- RViz visualization config
- Launch arguments for easy customization

## Topics

### Published
- `/scan` (sensor_msgs/LaserScan): 2D laser scan data from Aurora

## Transforms (TF)

- `base_link` → `laser_frame`
  - Translation: `[0.15, 0.0, 0.12]` meters (front-center-up)
  - Rotation: `[0, 0, 0]` radians (facing forward)

**Adjust in [aurora.launch.py](launch/aurora.launch.py) if your mounting position differs.**

## Installation

The Aurora uses the `rplidar_ros` driver (compatible with Aurora):

```bash
# On Raspberry Pi (included in pi_setup.sh)
sudo apt install ros-jazzy-rplidar-ros

# Build this package
cd ~/qwacr_ws
colcon build --packages-select qwacr_lidar
source install/setup.bash
```

## Usage

### Launch Aurora LiDAR

```bash
# Basic launch (no visualization)
ros2 launch qwacr_lidar aurora.launch.py

# With RViz visualization
ros2 launch qwacr_lidar aurora.launch.py use_rviz:=true

# Custom serial port
ros2 launch qwacr_lidar aurora.launch.py serial_port:=/dev/ttyUSB0

# Higher scan rate (up to 20 Hz)
ros2 launch qwacr_lidar aurora.launch.py scan_frequency:=15.0
```

### View scan data

```bash
# Echo laser scan messages
ros2 topic echo /scan

# Check scan rate
ros2 topic hz /scan

# View TF tree
ros2 run tf2_tools view_frames
```

### Monitor performance

```bash
# Scan frequency and data rate
ros2 topic hz /scan
ros2 topic bw /scan

# Node info
ros2 node info /rplidar_node
```

## Hardware Setup

### 1. Connect Aurora to Raspberry Pi

- Plug Aurora USB cable into Pi USB port
- Verify device enumeration:
  ```bash
  ls -l /dev/ttyUSB*
  ```
- Typical device: `/dev/ttyUSB1` (GPS often takes `/dev/ttyUSB0`)

### 2. Serial Port Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, or reboot

# Verify permissions
groups $USER  # Should include 'dialout'
```

### 3. Test Aurora Connection

```bash
# Check if Aurora responds
sudo chmod 666 /dev/ttyUSB1  # Temporary permission if needed
ros2 launch qwacr_lidar aurora.launch.py
```

Expected output:
```
[rplidar_node]: RPLIDAR running on ROS2 package rplidar_ros
[rplidar_node]: Firmware Ver: X.XX
[rplidar_node]: Hardware Rev: X
[rplidar_node]: RPLidar health status: OK
```

## Configuration Files

### [config/aurora_params.yaml](config/aurora_params.yaml)

Aurora-specific parameters:
- `serial_port`: Device path (default: `/dev/ttyUSB1`)
- `serial_baudrate`: 256000 (Aurora default)
- `frame_id`: `laser_frame`
- `scan_frequency`: 10.0 Hz (adjust 10-20 Hz)
- `range_min`: 0.15 m
- `range_max`: 20.0 m
- `scan_mode`: Standard (or Express/Boost if supported)

### [launch/aurora.launch.py](launch/aurora.launch.py)

Launch file arguments:
- `serial_port`: Override device path
- `scan_frequency`: Set scan rate (10-20 Hz)
- `use_rviz`: Enable/disable visualization

## Mounting Position

Current TF assumes Aurora is mounted:
- **X**: 0.15 m forward from `base_link`
- **Y**: 0.0 m (centered)
- **Z**: 0.12 m up from `base_link`
- **Orientation**: Facing forward (0° yaw)

**To adjust:** Edit the `static_transform_publisher` arguments in [aurora.launch.py](launch/aurora.launch.py):

```python
arguments=[
    '0.15', '0.0', '0.12',  # x, y, z (meters)
    '0', '0', '0',           # roll, pitch, yaw (radians)
    'base_link',
    'laser_frame'
],
```

## Troubleshooting

**No `/scan` topic:**
- Check device connection: `ls -l /dev/ttyUSB*`
- Verify permissions: `groups $USER` includes `dialout`
- Check serial port parameter matches actual device
- Try different USB port or cable

**Scan rate lower than expected:**
- Aurora supports 10-20 Hz; default is 10 Hz
- Increase `scan_frequency` parameter: `scan_frequency:=15.0`
- Check USB bandwidth (USB 2.0 may limit performance)

**TF errors or missing transforms:**
- Verify `static_transform_publisher` is running: `ros2 node list`
- Check TF tree: `ros2 run tf2_tools view_frames`
- Ensure `base_link` exists (published by robot_state_publisher or URDF)

**Aurora not responding or health errors:**
- Power cycle the Aurora (unplug USB, wait 5 sec, replug)
- Check Aurora firmware version (may need update)
- Verify baud rate: 256000 for Aurora (vs 115200 for A1M8)

**Wrong baud rate:**
- A1M8 uses 115200, Aurora uses 256000
- If you see "cannot bind to the specified serial port" or timeout errors, check baud in `aurora_params.yaml`

## Integration with Navigation

For Nav2 integration, the `/scan` topic is used directly by:
- **Costmap 2D**: Obstacle detection layer
- **AMCL**: Localization (if using pre-built map)
- **SLAM Toolbox**: Simultaneous localization and mapping

Example costmap configuration (add to your Nav2 params):

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    data_type: "LaserScan"
```

## Performance Notes

- **Standard mode**: 10 Hz, good for most navigation tasks
- **Express mode**: Up to 15 Hz, better for fast-moving robots
- **Boost mode**: Up to 20 Hz (check Aurora firmware support)

Higher scan rates improve obstacle detection but increase CPU/USB bandwidth usage. Test on Pi 5 to find optimal balance.

## Next Steps

1. **Test Aurora hardware**: Connect and verify `/scan` publishes
2. **Calibrate mounting position**: Adjust TF if needed
3. **Record rosbags**: Capture scan data for offline analysis
4. **Integrate with Nav2**: Add to costmap configuration
5. **SLAM mapping**: Use SLAM Toolbox to build environment map

## Example: Record Scan Data

```bash
# Record 30 seconds of scan data
ros2 bag record -o aurora_test /scan /tf /tf_static -d 30

# Playback and visualize
ros2 bag play aurora_test
ros2 launch qwacr_lidar aurora.launch.py use_rviz:=true
```

## License

MIT License
