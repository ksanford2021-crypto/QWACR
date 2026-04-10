# Phase 2: Nav2 & Sensor Fusion Testing Plan
**Date:** February 6, 2026  
**Objective:** Validate robot_localization dual EKF sensor fusion and Nav2 autonomous navigation

---

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                            │
├─────────────────────────────────────────────────────────────┤
│ Motors (Arduino)  │ GPS (LG580P)  │ Aurora LiDAR         │
│ /dev/ttyACM1      │ /dev/ttyACM0  │ 192.168.11.1         │
│ 115200 baud       │ 460800 baud   │ Ethernet             │
└─────────────────────────────────────────────────────────────┘
            │                │               │
            ▼                ▼               ▼
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR TOPICS                             │
├─────────────────────────────────────────────────────────────┤
│ /diff_cont/odom                  - Wheel encoders           │
│ /slamware_ros_sdk_server_node/odom - Aurora SLAM odom       │
│ /slamware_ros_sdk_server_node/imu_raw_data - Aurora IMU     │
│ /slamware_ros_sdk_server_node/scan - LiDAR scan             │
│ /gps/fix                         - GPS position             │
│ /gps/enu_odom                    - GPS in ENU frame         │
│ /gps/heading                     - Dual-antenna heading     │
└─────────────────────────────────────────────────────────────┘
            │                │               │
            ▼                ▼               ▼
┌─────────────────────────────────────────────────────────────┐
│            ROBOT_LOCALIZATION (Dual EKF)                     │
├─────────────────────────────────────────────────────────────┤
│ ekf_local_odom:  wheel + Aurora odom + Aurora IMU           │
│   → /odometry/local (smooth, no GPS jumps)                  │
│                                                              │
│ ekf_global_map:  local sensors + GPS                        │
│   → /odometry/global (GPS-corrected, map frame)             │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    NAV2 STACK                                │
├─────────────────────────────────────────────────────────────┤
│ • Local Costmap:  10m×10m, /odometry/local, odom frame      │
│ • Global Costmap: Large area, map frame                     │
│ • Planner:        NavFn (Dijkstra)                          │
│ • Controller:     DWB (Dynamic Window)                      │
│ • Smoother:       Path smoothing                            │
│ • Behaviors:      Spin, backup, wait                        │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                    /cmd_vel → Motors
```

---

## Pre-Flight Checklist

### Hardware Verification
- [ ] Battery charged (>24V measured)
- [ ] 30A fuse installed and secure
- [ ] Aurora 12V power connected (red=+12V, black=GND)
- [ ] GPS antenna mounted with clear sky view
- [ ] GPS USB connected to Pi (/dev/ttyACM0)
- [ ] Arduino USB connected to Pi (/dev/ttyACM1)
- [ ] Aurora ethernet cable to Pi (192.168.11.1)
- [ ] Pi powered on, SSH accessible at 192.168.11.100 (ethernet) or 192.168.4.1 (WiFi AP)

### Software Build Status
- [ ] qwacr_navigation rebuilt with topic remappings: `cd ~/qwacr_ws && colcon build --packages-select qwacr_navigation`
- [ ] Workspace sourced: `source ~/qwacr_ws/install/setup.bash`
- [ ] localization.launch.py has Aurora topic remappings
- [ ] nav2_params.yaml updated with `/slamware_ros_sdk_server_node/scan` topic

---

## Test Sequence

## Phase 1: Sensor Stack Validation (15 minutes)

### 1.1 Launch Individual Sensors

**Terminal 1: Aurora LiDAR**
```bash
ssh qwacr@192.168.11.100
source ~/qwacr_ws/install/setup.bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.launch.py
```

**Wait 30 seconds for Aurora to initialize**

**Verify Aurora:**
```bash
# In new terminal
ros2 topic hz /slamware_ros_sdk_server_node/scan  # Should be ~5-10 Hz
ros2 topic hz /slamware_ros_sdk_server_node/odom  # Should be ~10-30 Hz
ros2 topic hz /slamware_ros_sdk_server_node/imu_raw_data  # Should be ~30-100 Hz
ros2 topic list | grep slamware  # Should show 20+ topics
```

**Terminal 2: GPS Driver**
```bash
ssh qwacr@192.168.11.100
source ~/qwacr_ws/install/setup.bash
ros2 launch qwacr_gps gps.launch.py serial_port:=/dev/ttyACM0 baud_rate:=460800
```

**Verify GPS (outdoors only):**
```bash
ros2 topic hz /gps/fix  # Should be ~1-10 Hz
ros2 topic echo /gps/fix --once  # Check status=STATUS_FIX, lat/lon reasonable
ros2 topic hz /gps/enu_odom  # Should be publishing
```

**Terminal 3: Motor Control (with serial warmup)**
```bash
ssh qwacr@192.168.11.100
source ~/qwacr_ws/install/setup.bash

# Use robust launch script (handles serial warmup automatically)
~/qwacr_ws/src/mega_diff_drive_control/scripts/launch_motors_robust.sh /dev/ttyACM1

# Alternative: Manual method if script not available
# picocom /dev/ttyACM1 -b 115200
# Wait 2 seconds, then Ctrl+A Ctrl+X to exit
# ros2 launch mega_diff_drive_control mega_diff_drive.launch.py
```

**Verify Motors:**
```bash
ros2 topic hz /diff_cont/odom  # Should be ~30 Hz
ros2 topic hz /joint_states  # Should be ~30 Hz
ros2 control list_controllers  # All should be "active"
```

### 1.2 Sensor Stack Health Check Script

**Run comprehensive check:**
```bash
#!/bin/bash
echo "=== QWACR Sensor Stack Health Check ==="
echo ""
echo "1. GPS Fix Status:"
timeout 2 ros2 topic hz /gps/fix 2>&1 | head -1

echo ""
echo "2. Wheel Encoders:"
timeout 2 ros2 topic hz /joint_states 2>&1 | head -1

echo ""
echo "3. Aurora LiDAR Scan:"
timeout 2 ros2 topic hz /slamware_ros_sdk_server_node/scan 2>&1 | head -1

echo ""
echo "4. Aurora IMU:"
timeout 2 ros2 topic hz /slamware_ros_sdk_server_node/imu_raw_data 2>&1 | head -1

echo ""
echo "5. Aurora SLAM Odometry:"
timeout 2 ros2 topic hz /slamware_ros_sdk_server_node/odom 2>&1 | head -1

echo ""
echo "6. Motor Odometry:"
timeout 2 ros2 topic hz /diff_cont/odom 2>&1 | head -1

echo ""
echo "7. Controller Status:"
ros2 control list_controllers

echo ""
echo "=== Health Check Complete ==="
```

**Expected Results:**
- GPS: 1-10 Hz (outdoors only)
- Wheel encoders: 30 Hz
- Aurora scan: 5-10 Hz
- Aurora IMU: 30-100 Hz
- Aurora odom: 10-30 Hz
- Motor odom: 30 Hz
- All controllers: active

---

## Phase 2: EKF Sensor Fusion Testing (20 minutes)

### 2.1 Launch Dual EKF

**Terminal 4: Localization (EKF nodes)**
```bash
ssh qwacr@192.168.11.100
source ~/qwacr_ws/install/setup.bash
ros2 launch qwacr_navigation localization.launch.py
```

**Watch for startup messages:**
- Should see "ekf_local_odom" and "ekf_global_map" nodes starting
- Check for any topic remapping warnings
- Look for "Listening for IMU data on topic imu" (should be remapped correctly)

### 2.2 Verify EKF Outputs

**Check EKF publishing:**
```bash
ros2 topic hz /odometry/local   # Should be ~30 Hz
ros2 topic hz /odometry/global  # Should be ~30 Hz

# View sample output
ros2 topic echo /odometry/local --once
ros2 topic echo /odometry/global --once
```

**Check TF tree:**
```bash
ros2 run tf2_ros tf2_echo map base_link
# Should show transform without errors

# View full TF tree
ros2 run tf2_tools view_frames
# Generates frames.pdf showing: map → odom → base_link → sensor frames
```

**Verify each EKF is receiving data:**
```bash
# Check diagnostics
ros2 topic echo /diagnostics | grep ekf

# Local EKF should fuse:
# - odometry/wheel (from /diff_cont/odom remapped)
# - odom (from /slamware_ros_sdk_server_node/odom remapped)
# - imu (from /slamware_ros_sdk_server_node/imu_raw_data remapped)

# Global EKF adds:
# - /gps/enu_odom (no remapping, already correct)
```

### 2.3 EKF Fusion Quality Test

**Static test (robot stationary):**
```bash
# Record 30 seconds of data while robot sits still
ros2 bag record -o static_test \
  /diff_cont/odom \
  /slamware_ros_sdk_server_node/odom \
  /odometry/local \
  /odometry/global \
  /gps/enu_odom \
  /tf /tf_static
```

**Expected:** All odometry topics should show minimal drift (<5cm over 30s)

**Movement test (manual drive):**
```bash
# Terminal 5: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel

# Drive in a square pattern: forward 2m → turn 90° → repeat 4x
# Record data during movement
ros2 bag record -o square_test_$(date +%Y%m%d_%H%M%S) \
  /diff_cont/odom \
  /slamware_ros_sdk_server_node/odom \
  /odometry/local \
  /odometry/global \
  /gps/fix \
  /gps/enu_odom \
  /slamware_ros_sdk_server_node/scan \
  /tf /tf_static
```

**Expected Results:**
- `/diff_cont/odom` alone: moderate drift, may not return to start
- `/odometry/local` (with Aurora): minimal drift, better square closure
- `/odometry/global` (with GPS): GPS-corrected, best square closure (outdoors)

### 2.4 EKF Covariance Analysis

**Check covariance growth during movement:**
```bash
ros2 topic echo /odometry/local --field pose.covariance
# Should be low initially, grow slightly during movement

ros2 topic echo /odometry/global --field pose.covariance
# Should reset/decrease when GPS fixes arrive
```

---

## Phase 3: Nav2 Stack Testing (30 minutes)

### 3.1 Launch Nav2 (Without Hardware First)

**Test Nav2 with mock robot (no motors moving):**
```bash
# On dev machine (for safety, easier debugging)
cd ~/qwacr_ws
source install/setup.bash

# Launch Nav2 test mode
ros2 launch qwacr_navigation nav2_test.launch.py use_rviz:=true
```

**This launches:**
- Nav2 stack (planner, controller, costmaps, behavior server)
- RViz2 with Nav2 panel
- No robot hardware (cmd_vel published but nothing moves)

### 3.2 RViz Configuration

**In RViz2, add displays:**
1. **Fixed Frame:** `map`
2. **Add → TF** - Show coordinate frames
3. **Add → LaserScan** - Topic: `/slamware_ros_sdk_server_node/scan`
4. **Add → Map** - Topic: `/global_costmap/costmap`
5. **Add → Map** - Topic: `/local_costmap/costmap`
6. **Add → Odometry** - Topic: `/diff_cont/odom` (red - wheel only)
7. **Add → Odometry** - Topic: `/odometry/local` (green - fused local)
8. **Add → Odometry** - Topic: `/odometry/global` (blue - GPS corrected)
9. **Add → Path** - Topic: `/plan` (global plan)
10. **Add → Path** - Topic: `/local_plan` (local plan)
11. **Add → Marker** - Topic: `/waypoints` (if using waypoint follower)

**Save config:** File → Save Config As → `~/qwacr_ws/src/qwacr_navigation/config/nav2_fusion_test.rviz`

### 3.3 Costmap Verification

**Check costmap topics:**
```bash
ros2 topic list | grep costmap
# Should show:
# /global_costmap/costmap
# /global_costmap/costmap_updates
# /local_costmap/costmap
# /local_costmap/costmap_updates
```

**Verify obstacle detection:**
```bash
# Place obstacle in front of Aurora LiDAR (box, chair, etc.)
ros2 topic echo /local_costmap/costmap --once
# Costmap should show obstacle as high-cost cells

# View in RViz - local costmap should show obstacle in red
```

**Check costmap parameters:**
```bash
ros2 param get /local_costmap/local_costmap update_frequency  # Should be 5.0 Hz
ros2 param get /global_costmap/global_costmap update_frequency  # Should be 1.0 Hz
ros2 param get /local_costmap/local_costmap resolution  # Should be 0.05 (5cm)
```

### 3.4 Manual Goal Testing (Simulated)

**Send 2D Nav Goal in RViz:**
1. Click "2D Goal Pose" button (top toolbar)
2. Click on map 5-10m away from robot
3. Drag to set orientation

**Observe:**
- Global plan appears (green line from robot to goal)
- Local plan appears (cyan line following global plan)
- `/cmd_vel` is published (check: `ros2 topic echo /cmd_vel`)
- Robot does NOT move (no hardware yet, just planning verification)

**Expected behavior:**
- Planner generates path avoiding obstacles
- Controller generates velocity commands
- Local costmap updates as robot "virtually" moves
- Goal reached: Nav2 status shows "Succeeded"

**Test multiple goals:**
- Straight line goal (simple)
- Goal behind obstacle (requires path around)
- Goal requiring turn in place
- Goal very close to robot (<1m)

### 3.5 Nav2 Status Monitoring

**Check Nav2 lifecycle states:**
```bash
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /behavior_server
# All should be "active [3]"
```

**Monitor navigation action:**
```bash
ros2 action list
# Should show /navigate_to_pose

ros2 action info /navigate_to_pose
# Shows clients and servers connected
```

---

## Phase 4: Full System Integration (Hardware Nav2) (30 minutes)

### 4.1 Full System Launch on Pi

**⚠️ SAFETY: Robot will move autonomously! Clear 5m radius around robot**

**Terminal 1: Full system launch**
```bash
ssh qwacr@192.168.11.100
source ~/qwacr_ws/install/setup.bash

ros2 launch qwacr_navigation full_system.launch.py \
  serial_port:=/dev/ttyACM1 \
  gps_serial_port:=/dev/ttyACM0 \
  gps_baud_rate:=460800 \
  use_rviz:=false
```

**This launches:**
- Robot state publisher (URDF/TF)
- Motor controllers (with serial warmup)
- Aurora SLAM (reminder to launch separately)
- GPS driver
- Dual EKF (local + global)
- Nav2 stack
- No RViz (use remote RViz from dev machine)

**Terminal 2: Launch Aurora separately**
```bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.launch.py
```

**Terminal 3: Remote RViz (on dev machine)**
```bash
# Set ROS_DOMAIN_ID to match Pi
export ROS_DOMAIN_ID=0

# Launch RViz
rviz2 -d ~/qwacr_ws/src/qwacr_navigation/config/nav2_fusion_test.rviz
```

### 4.2 System Health Check

**Run full stack verification:**
```bash
# Check all nodes active
ros2 node list | wc -l  # Should be 15+ nodes

# Check critical topics
ros2 topic hz /odometry/local  # EKF local
ros2 topic hz /odometry/global  # EKF global with GPS
ros2 topic hz /slamware_ros_sdk_server_node/scan  # Aurora scan for Nav2
ros2 topic hz /cmd_vel  # Nav2 controller output
ros2 topic hz /diff_cont/cmd_vel  # Motor input

# Check TF chain
ros2 run tf2_ros tf2_echo map base_link
```

### 4.3 Controlled Movement Test

**Start with small goals:**

**Goal 1: 1m forward**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Observe:**
- Robot accelerates smoothly
- Stops at goal (~25cm tolerance)
- Costmaps update as robot moves
- No collisions with obstacles

**Goal 2: 2m lateral**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Goal 3: Return to start (using map frame)**
```bash
# Note starting position from RViz or /odometry/global
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 4.4 Obstacle Avoidance Test

**Setup:**
- Place obstacle 2m in front of robot
- Send goal behind obstacle

**Expected behavior:**
- Planner generates path around obstacle
- Robot follows curved path
- Local costmap shows obstacle
- Robot maintains safe distance (>0.8m inflation radius)

**Dynamic obstacle test:**
- Send goal 5m ahead
- While robot moving, place obstacle in path
- Robot should: stop, replan, navigate around, continue to goal

### 4.5 GPS Waypoint Following Test

**Create test waypoints (outdoors required):**

**Get current GPS position:**
```bash
ros2 topic echo /gps/fix --once
# Note lat/lon
```

**Create waypoint file:** `~/test_waypoints.json`
```json
{
  "waypoints": [
    {"name": "Start", "lat": <CURRENT_LAT>, "lon": <CURRENT_LON>, "yaw": 0.0},
    {"name": "North 10m", "lat": <LAT+0.00009>, "lon": <CURRENT_LON>, "yaw": 0.0},
    {"name": "East 10m", "lat": <LAT+0.00009>, "lon": <LON+0.00012>, "yaw": 1.57},
    {"name": "Return", "lat": <CURRENT_LAT>, "lon": <CURRENT_LON>, "yaw": 3.14}
  ]
}
```

**Note:** 0.00009° lat ≈ 10m, 0.00012° lon ≈ 10m (at Florida latitude)

**Launch waypoint follower:**
```bash
ros2 run qwacr_navigation gps_waypoint_follower \
  --ros-args \
  -p waypoint_file:=~/test_waypoints.json \
  -p origin_lat:=<START_LAT> \
  -p origin_lon:=<START_LON> \
  -p loop_waypoints:=false
```

**Expected:**
- Robot navigates to each GPS waypoint sequentially
- Uses /odometry/global for GPS-corrected positioning
- Reaches each waypoint within 0.5m tolerance
- Completes full square path

---

## Phase 5: Data Collection & Analysis (20 minutes)

### 5.1 Record Full Navigation Run

**Start recording before navigation:**
```bash
ros2 bag record -o nav2_full_test_$(date +%Y%m%d_%H%M%S) \
  /diff_cont/odom \
  /slamware_ros_sdk_server_node/odom \
  /slamware_ros_sdk_server_node/scan \
  /slamware_ros_sdk_server_node/imu_raw_data \
  /gps/fix \
  /gps/enu_odom \
  /odometry/local \
  /odometry/global \
  /plan \
  /local_plan \
  /cmd_vel \
  /diff_cont/cmd_vel \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /tf \
  /tf_static &

RECORD_PID=$!
```

**Execute test navigation sequence:**
```bash
# Square pattern: 5m × 5m
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}}"

sleep 10

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 0.707, z: 0.707}}}}"

sleep 10

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 5.0}, orientation: {w: 0.0, z: 1.0}}}}"

sleep 10

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: -0.707, z: 0.707}}}}"
```

**Stop recording:**
```bash
kill $RECORD_PID
```

### 5.2 Playback and Analysis

**Playback bag file (on dev machine):**
```bash
ros2 bag play nav2_full_test_<timestamp>
```

**In separate terminals, analyze:**

**Odometry comparison:**
```bash
# Compare wheel odom vs fused odom
ros2 topic echo /diff_cont/odom --field pose.pose.position
ros2 topic echo /odometry/local --field pose.pose.position
```

**Path accuracy:**
- Load bag in RViz2
- Plot all odometry paths
- Measure final position error (should be <0.5m for 5m×5m square)

**Velocity profiles:**
```bash
ros2 topic echo /cmd_vel --field linear.x
# Check for smooth acceleration/deceleration
```

### 5.3 Performance Metrics

**Extract metrics from bag:**
```bash
ros2 bag info nav2_full_test_<timestamp>

# Topic message counts
# Duration
# Average rates
```

**Calculate:**
- **Path accuracy:** Final position error vs start position
- **Goal tolerance:** Distance from goal when Nav2 reports success
- **Replanning events:** Count of new global plans generated
- **Obstacle avoidance:** Minimum distance to obstacles maintained
- **Velocity adherence:** Max velocity achieved vs configured max (0.5 m/s)

---

## Success Criteria

### Phase 1: Sensor Stack ✅
- [ ] GPS publishing at 1-10 Hz outdoors with STATUS_FIX
- [ ] Aurora scan at 5-10 Hz, 360° coverage
- [ ] Aurora odom at 10-30 Hz
- [ ] Aurora IMU at 30-100 Hz
- [ ] Wheel odom at 30 Hz
- [ ] All 4 wheel encoders reporting positions

### Phase 2: EKF Fusion ✅
- [ ] `/odometry/local` publishing at 30 Hz
- [ ] `/odometry/global` publishing at 30 Hz
- [ ] TF tree complete: `map → odom → base_link → sensors`
- [ ] Static test: <5cm drift over 30 seconds
- [ ] Movement test: Square closure error <0.5m with Aurora, <0.25m with GPS

### Phase 3: Nav2 Simulation ✅
- [ ] Nav2 nodes all active
- [ ] Costmaps updating with scan data
- [ ] Obstacles visible in local costmap
- [ ] Global planner generates valid paths
- [ ] Controller outputs cmd_vel
- [ ] Goals reachable without errors

### Phase 4: Hardware Nav2 ✅
- [ ] Robot navigates to 1m forward goal
- [ ] Robot navigates to lateral goal (requires path planning)
- [ ] Obstacle avoidance working (robot stops/replans)
- [ ] GPS waypoint navigation completes square pattern
- [ ] No collisions during any test
- [ ] Robot returns to start position within 0.5m

### Phase 5: Data Validation ✅
- [ ] Bag files recorded for all tests
- [ ] Path accuracy <0.5m error for 5m×5m square
- [ ] Velocity limits respected (<0.5 m/s)
- [ ] Costmaps properly inflated around obstacles
- [ ] No TF transform errors during navigation

---

## Troubleshooting Guide

### Issue: EKF not publishing /odometry/local

**Check:**
```bash
ros2 node info /ekf_local_odom
# Look at "Subscriptions" - should list odometry/wheel, odom, imu
```

**Fix:**
- Verify topic remapping in localization.launch.py
- Check that source topics are publishing: `/diff_cont/odom`, `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/imu_raw_data`

### Issue: Nav2 not planning paths

**Check:**
```bash
ros2 lifecycle get /planner_server  # Should be "active [3]"
ros2 param get /global_costmap/global_costmap update_frequency
ros2 topic hz /slamware_ros_sdk_server_node/scan  # Must be publishing for costmap
```

**Fix:**
- Ensure scan topic configured in nav2_params.yaml: `/slamware_ros_sdk_server_node/scan`
- Check global costmap has valid transform: `ros2 run tf2_ros tf2_echo map base_link`

### Issue: Robot not moving on cmd_vel

**Check:**
```bash
ros2 topic echo /cmd_vel  # Should show velocities when goal active
ros2 topic echo /diff_cont/cmd_vel  # Should match /cmd_vel if remapped correctly
```

**Fix:**
- Verify topic remapping in controller launch
- Check motor controller active: `ros2 control list_controllers`

### Issue: GPS waypoints not working

**Check:**
```bash
ros2 topic echo /gps/fix --field status  # Must be status=2 (STATUS_FIX)
ros2 topic echo /odometry/global --field pose.pose.position  # Should be in meters (ENU)
```

**Fix:**
- GPS needs clear sky view (outdoors)
- Verify GPS origin coordinates match between driver and waypoint follower
- Check map frame is being published by ekf_global_map

### Issue: Costmaps not updating

**Check:**
```bash
ros2 param get /local_costmap/local_costmap plugins  # Should list obstacle_layer
ros2 topic echo /local_costmap/costmap --once  # Should have non-zero data
```

**Fix:**
- Verify scan topic in nav2_params.yaml
- Check observation_sources configured correctly
- Ensure Aurora scan is publishing

---

## Configuration Files Summary

| File | Purpose | Key Settings |
|------|---------|--------------|
| `ekf_local_global.yaml` | EKF sensor fusion | Wheel, Aurora, GPS topic configs |
| `localization.launch.py` | EKF node launch | Topic remappings for Aurora |
| `nav2_params.yaml` | Nav2 stack config | Costmaps, planner, controller params |
| `full_system.launch.py` | Complete system | All sensors + EKF + Nav2 |
| `example_waypoints.json` | GPS waypoints | Lat/lon/yaw for testing |

---

## Next Steps After Validation

1. **Tune Nav2 parameters:**
   - Increase max velocity if stable: `max_vel_x: 1.0`
   - Adjust goal tolerance if needed: `xy_goal_tolerance: 0.15`
   - Tune costmap inflation for tighter passages

2. **Create mission waypoint files:**
   - Survey actual field with GPS
   - Create waypoint JSON for mowing patterns
   - Test long-distance waypoint navigation (100m+)

3. **Implement fail-safes:**
   - Battery monitoring node
   - GPS fix loss detection
   - Motor controller watchdog
   - Emergency stop service

4. **Phase 3: LoRa/HaLow integration**
   - Remote telemetry
   - Video streaming
   - Emergency commands

---

**Document Version:** 1.0  
**Last Updated:** February 6, 2026  
**Status:** Ready for testing
