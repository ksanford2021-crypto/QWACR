# QWACR Navigation Package

ROS2 navigation stack configuration for the QWACR autonomous robot.

## Features

- **Nav2 Integration**: Full navigation stack with costmaps, planners, and controllers
- **GPS Waypoint Following**: Convert GPS coordinates to map frame goals
- **Multi-sensor Fusion**: Aurora SLAM scan, wheel odometry, RTK GPS
- **Outdoor Optimized**: Configured for large outdoor environments (half-mile range)

## Package Structure

```
qwacr_navigation/
├── config/
│   ├── nav2_params.yaml           # Main Nav2 configuration
│   └── example_waypoints.json     # Example GPS waypoints
├── launch/
│   └── navigation.launch.py       # Launch Nav2 stack
├── qwacr_navigation/
│   └── gps_waypoint_follower.py   # GPS waypoint follower node
└── maps/                           # Static maps (if needed)
```

## Configuration

### Local Costmap
- **Size**: 10m × 10m rolling window
- **Resolution**: 5cm
- **Sensors**: Aurora /scan (20m range)
- **Update Rate**: 5Hz

### Global Costmap
- **Frame**: map (GPS-referenced)
- **Resolution**: 10cm
- **Sensors**: Aurora /scan for obstacles
- **Update Rate**: 1Hz

### Controller
- **Type**: DWB (Dynamic Window Approach)
- **Max velocity**: 0.5 m/s linear, 1.0 rad/s angular
- **Goal tolerance**: 0.25m position, 0.25 rad orientation

## Usage

### 1. Launch Navigation Stack

```bash
ros2 launch qwacr_navigation navigation.launch.py
```

### 2. GPS Waypoint Following

**Load waypoints from file:**
```bash
ros2 run qwacr_navigation gps_waypoint_follower \
  --ros-args \
  -p waypoint_file:=/path/to/waypoints.json \
  -p origin_lat:=26.3747 \
  -p origin_lon:=-80.1009 \
  -p loop_waypoints:=false
```

**Waypoint JSON format:**
```json
{
  "waypoints": [
    {"name": "Start", "lat": 26.374700, "lon": -80.100900, "yaw": 0.0},
    {"name": "Goal", "lat": 26.374800, "lon": -80.101000, "yaw": 1.57}
  ]
}
```

### 3. Manual Goal Sending

Using RViz2:
1. Open RViz2 with Nav2 panel
2. Click "2D Goal Pose" tool
3. Click and drag on map to set goal

Using command line:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0}}}}"
```

## Integration with QWACR Systems

### Required Running Nodes

Before launching navigation:

1. **Localization (EKF filters)**:
   ```bash
   ros2 launch robot_localization ekf_local_global.launch.py
   ```

2. **Aurora SLAM**:
   ```bash
   ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.11.1
   ```

3. **GPS Driver**:
   ```bash
   ros2 run qwacr_gps gps_driver_node
   ```

4. **Motor Control**:
   ```bash
   ros2 launch qwacr_control diff_drive_controller.launch.py
   ```

### Topic Connections

**Inputs to Nav2:**
- `/scan` - Aurora laser scan
- `/odom` - From EKF local filter
- `/map` → `/odom` → `/base_link` transforms from EKF

**Outputs from Nav2:**
- `/cmd_vel` - Velocity commands to motor controller

## Customization

### Adjusting Robot Speed

Edit `config/nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.5        # Maximum forward speed (m/s)
      max_vel_theta: 1.0    # Maximum rotation speed (rad/s)
```

### Adjusting Obstacle Avoidance

Edit inflation radius in `config/nav2_params.yaml`:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.8  # Larger = more cautious
```

### Changing Goal Tolerance

```yaml
controller_server:
  ros__parameters:
    general_goal_checker:
      xy_goal_tolerance: 0.25   # Position tolerance (m)
      yaw_goal_tolerance: 0.25  # Orientation tolerance (rad)
```

## Troubleshooting

**Navigation not starting:**
- Check that all transforms are publishing: `ros2 run tf2_ros tf2_echo map base_link`
- Verify scan data: `ros2 topic echo /scan`
- Check Nav2 status: `ros2 topic echo /diagnostics`

**Robot not moving:**
- Verify `/cmd_vel` is publishing: `ros2 topic echo /cmd_vel`
- Check motor controller is receiving commands
- Ensure costmaps are clear: `ros2 topic echo /local_costmap/costmap`

**GPS waypoints not working:**
- Confirm GPS origin matches between GPS driver and waypoint follower
- Check GPS fix quality: `ros2 topic echo /fix`
- Verify map frame is being published by global EKF

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
