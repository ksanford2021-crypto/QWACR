# Hardware Testing Guide

## Quick Reference

### Run Complete Hardware Test
```bash
source /home/kyle/qwacr_ws/install/setup.bash
ros2 run qwacr_navigation hardware_test
```

### Run Individual Tests
```bash
# Test Aurora SLAM sensor only
ros2 run qwacr_navigation hardware_test --test aurora

# Test GPS only
ros2 run qwacr_navigation hardware_test --test gps

# Test EKF localization only
ros2 run qwacr_navigation hardware_test --test ekf

# Test motor control only
ros2 run qwacr_navigation hardware_test --test motors
```

## What Each Test Checks

### Aurora SLAM Test
- `/odom` - 6DOF SLAM odometry from Aurora sensor
- `/scan` - LaserScan data for obstacle detection
- `/imu` - IMU data (orientation, angular velocity, acceleration)
- Measures: Data rates (Hz) for each topic

### GPS Test
- `/gps/enu_odom` - GPS position, velocity, and heading in ENU frame
- `/fix` - Raw GPS NavSatFix data
- `/gps/heading` - Dual-antenna heading
- Measures: Update rates, RTK fix quality

### EKF Test
- `/odometry/local` - Local odometry (odom frame) from first EKF
- `/odometry/global` - Global odometry (map frame) from second EKF
- Measures: Fusion output rates, transform availability

### Motor Test
- `/diff_cont/odom` - Wheel odometry from motor encoders
- Measures: Controller update rate

### TF Tree Test
- Reminds you to check transform tree structure
- Expected: map → odom → base_link
- Run: `ros2 run tf2_tools view_frames`

## Expected Results (Pass Criteria)

### Aurora SLAM
- ✓ All topics publishing
- ✓ Odometry rate: 10-30 Hz
- ✓ Scan rate: 5-10 Hz
- ✓ IMU rate: 20-100 Hz

### GPS
- ✓ All topics publishing
- ✓ Update rate: 1-10 Hz
- ✓ RTK fix available (status in /fix message)
- ✓ Heading from dual antenna

### EKF
- ✓ Both local and global odometry publishing
- ✓ Update rate: 20-30 Hz
- ✓ No warnings about missing transforms

### Motors
- ✓ Wheel odometry publishing
- ✓ Update rate: 20-30 Hz

## Troubleshooting

### Aurora SLAM Not Publishing
1. Check network connection: `ping 192.168.11.1`
2. Verify Aurora is in AP mode
3. Launch Aurora SDK: `ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.11.1`

### GPS Not Publishing
1. Check serial connection: `ls /dev/ttyUSB*` or `/dev/ttyACM*`
2. Verify baud rate in GPS driver config
3. Launch GPS driver: `ros2 run qwacr_gps gps_driver_node`
4. Check for RTK corrections

### EKF Not Publishing
1. Verify sensor data is available (run sensor tests first)
2. Launch EKF: `ros2 launch robot_localization ekf_local_global.launch.py`
3. Check for transform errors: `ros2 run tf2_ros tf2_echo map odom`

### Motors Not Responding
1. Check serial connection: `ls /dev/ttyACM*`
2. Verify Arduino is powered and programmed
3. Launch hardware interface: `ros2 launch qwacr_build robot_bringup.launch.py`

## Integration Test Sequence

### 1. Hardware Validation (No Movement)
```bash
# Terminal 1: Launch Aurora SLAM
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.11.1

# Terminal 2: Launch GPS
ros2 run qwacr_gps gps_driver_node

# Terminal 3: Launch EKF
ros2 launch robot_localization ekf_local_global.launch.py

# Terminal 4: Launch motors (optional, for encoder data)
ros2 launch qwacr_build robot_bringup.launch.py

# Terminal 5: Run hardware test
ros2 run qwacr_navigation hardware_test
```

### 2. Full System Test (With Nav2)
```bash
# Once all hardware tests pass:
ros2 launch qwacr_navigation navigation_with_robot.launch.py use_hardware:=true

# In another terminal, run test again to verify all systems operational
ros2 run qwacr_navigation hardware_test
```

### 3. Visualization Check
```bash
# View in RViz
ros2 launch qwacr_navigation rviz.launch.py

# Check TF tree
ros2 run tf2_tools view_frames
# This creates frames.pdf - check that map->odom->base_link chain exists
```

## Success Criteria for Phase 5

Before proceeding to autonomous navigation:
- [ ] All hardware tests pass
- [ ] TF tree has complete chain: map → odom → base_link
- [ ] Costmaps populate with real scan data in RViz
- [ ] Can manually set navigation goals in RViz (even if they fail due to planning)
- [ ] No continuous errors in any node logs
- [ ] GPS shows RTK fixed status
- [ ] Aurora SLAM provides stable odometry

## Next Steps After Tests Pass

1. **Short-Range Navigation Test**
   - Set 2D Nav Goal 5-10m away in RViz
   - Verify robot plans path and attempts to navigate
   - Tune controller parameters if needed

2. **GPS Waypoint Test**
   - Record current GPS position
   - Create waypoint 10-20m away
   - Run: `ros2 run qwacr_navigation gps_waypoint_follower --ros-args -p waypoint_file:=test_waypoints.json`

3. **Long-Range Test**
   - Gradually increase waypoint distances
   - Test up to full 800m range
   - Monitor GPS quality throughout

4. **Behavior Tree Customization**
   - Create custom BT with GPS quality checks
   - Add battery monitoring
   - Implement outdoor recovery behaviors
