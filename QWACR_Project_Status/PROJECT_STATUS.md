# QWACR Project Status - March 12 & 18–19, 2026

## 🎉 MILESTONE: Phase 4 Complete - Nav2 Navigation Stack Configured and Verified

### Summary
Complete autonomous navigation system configured with Nav2, robot_localization EKF sensor fusion, GPS waypoint following, and RViz visualization. All software components tested and ready for hardware integration. BT navigator, docking server, and all Nav2 nodes launching successfully.

---

## 📅 Status Update – March 12, 2026

Low-level drive hardware, firmware, and ROS 2 integration have now been exercised on the real robot. Motor directions, encoder feedback, and per-wheel current limiting are all behaving as expected, and `/joint_states` provides consistent wheel positions and velocities.

Additionally, the RPLIDAR S2 has been successfully integrated on the Raspberry Pi 5 using `/dev/ttyAMA0` mapped to GPIO14/15. **Those RX/TX pins are now reserved for LiDAR** and should not be reused for other UART peripherals.

**Highlights since January 28:**
- Resolved left/right motor sign conventions entirely in the ROS 2 hardware plugin; teleop "forward" now corresponds to physically forward motion.
- Fixed wheel order mapping between Arduino feedback `[FL, BL, FR, BR]` and URDF joint names so all four wheel joints report correct positions/velocities.
- Extended the binary feedback protocol to include per-wheel motor currents and added soft current limiting on the Arduino using PWM scaling (target ≈2.6 A per channel).
- Moved velocity estimation to ROS by differentiating encoder positions, yielding smoother and more stable `/joint_states.velocity` data.
- Verified robust launch procedure on the Pi using `launch_robust.sh` and the Twist→TwistStamped bridge, with controllers reliably activating.

**New Planning Work (Fire Sensor Payload):**
- Created `QWACR_Project_Status/Fire_Sensor_Integration.md` documenting a planned multi-sensor fire-detection payload (SEN54 particulate/VOC/humidity/temperature, MLX90640 thermal IR array, SPEC Sensors DGS2 H₂ and CO gas modules, and a TCA9548A I²C multiplexer).
- Defined bus topology and Pi 5 pin strategy:
  - I²C1 on GPIO2/3 feeding a TCA9548A hub with separate channels for the SEN54 and MLX90640.
  - Additional UARTs to be enabled via overlays on **unused** GPIOs for the DGS2 modules, explicitly avoiding the LiDAR UART pins on GPIO14/15.

---

## 📅 Status Update – March 18–19, 2026

Fire-sensor payload, IMU, and HaLow video streaming have all been
integrated on the robot Pi 5. A single robust video stream can now be
switched between the front and left cameras, and all fire‑sensor data
are available as ROS 2 topics for eventual LoRa telemetry.

**New highlights since March 12:**

- **DGS2 gas sensors online:**
  - `/dev/ttyAMA2` → DGS2 CO, `/dev/ttyAMA4` → DGS2 H₂.
  - Two ROS 2 nodes (`dgs2_co_node`, `dgs2_h2_node`) decode the DGS2
    serial protocol and publish calibrated concentrations and
    temperatures.

- **I²C mux + fire sensors working:**
  - TCA9548A confirmed at 0x70 on I²C bus 1.
  - Port mapping (current wiring):
    - Port `0x01` → SEN54 at 0x69 (environmental node running).
    - Port `0x02` → front MLX90640 at 0x33 (thermal node running).
    - Port `0x10` → reserved for left MLX90640 (hardware partially
      wired, still to be finalized).

- **SparkFun IMU integrated:**
  - Qwiic ISM330DHCX on mux port `0x80` with dedicated ROS 2 node
    `sparkfun_imu_node.py` in `qwacr_imu`.
  - Node publishes `/imu/data` in SI units (m/s², rad/s).
  - `robot_localization` configs (`ekf_local_global.yaml` and the dual
    EKF example) updated so both local and global EKFs fuse `/imu/data`
    with wheel odom and GPS, with gravity removal handled in the EKF.

- **HaLow video streaming validated:**
  - Manual `rpicam-vid` → GStreamer pipeline over HaLow reproduced
    clean, low‑latency video from the front camera to the base station.
  - `qwacr_comms/video_publisher` now spawns the same pipeline
    programmatically using parameters from `halow_config.yaml`.
  - Stable single stream at 640×480, 15 fps, ≈1.2 Mbps.

- **Dual‑camera experiments and final design:**
  - Two simultaneous H.264 streams over HaLow were tested and found to
    cause heavy packet loss and corruption.
  - Final design is **single stream, switchable camera**:
    - `halow_video.launch.py` → front camera (index 0) to port 5000.
    - `halow_left_video.launch.py` → left camera (index 1) to port 5000.
  - Base station always runs the same GStreamer receiver on port 5000
    and does not need to know which camera is active.

- **Documentation updates:**
  - `HALOW_VIDEO_STREAMING_GUIDE.md` now documents the validated
    pipeline, HaLow IP layout, and single‑stream/front‑vs‑left camera
    switching.
  - `IMU_INTEGRATION_GUIDE.md` and `Fire_Sensor_Integration.md` will be
    kept in sync with the current wiring and topic names.

**Next integration focus (late March–April):**

- Adapt `qwacr_comms` LoRa bridge to carry:
  - Teleop commands from base station → robot.
  - Fire‑sensor and status telemetry from robot → base station.
  - Camera‑select commands (`front` vs `left`) for the HaLow stream.
- Stand up a **base‑station Pi 5** with both HaLow and LoRa radios
  running an operator console: one GStreamer window + a lightweight
  ROS‑based UI for teleop and fire‑sensor readouts.


---

## ✅ Completed Work

### Hardware Integration
- **Motor Drivers:** 2× Pololu Dual G2 High-Power Motor Drivers wired and operational
- **Motors:** 4× DC motors with 1600 PPR encoders using 2× edge counting (3200 CPR effective)
- **Power Distribution:** 24V supply powering both drivers in parallel
- **Signal Wiring:** All PWM, DIR, and sleep pins connected to Arduino Mega
- **Encoder Wiring:** All 4 encoders on interrupt pins (2, 18, 20, 3) with direction sensing

### Firmware Development
- **PID Control:** Velocity control implemented with Arduino PID library
  - Current tuning: Kp=18.0, Ki=0.2, Kd=0.05, Kff≈32.0
  - Velocity safety limits: ±9.42 rad/s (90 RPM)
  - Emphasis on smooth response with enough torque for in-place turns
- **Serial Protocol:** COBS encoding for reliable packet framing
  - Command format: [0x01][left_vel_float][right_vel_float]
  - Request format: [0x02] - queries encoder data
  - Response format: [0x10][enc_fl:i32][enc_bl:i32][enc_fr:i32][enc_br:i32][vel_fl:f32][vel_bl:f32][vel_fr:f32][vel_br:f32][cur_fl:f32][cur_bl:f32][cur_fr:f32][cur_br:f32]
  - 115200 baud, USB connection
  - ROS side now computes joint velocities from encoder positions for stability
- **Sleep Pin Management:** Active HIGH control prevents startup motion
  - Drivers sleep by default, wake on first command
  - Auto-sleep after 2s timeout
  - Verified on hardware: no unintended motion at boot or after timeouts
- **Encoder Feedback:** 2× edge counting on all 4 motors (channel A edges + channel B direction)
  - 3200 counts per revolution effective (gear ratio included)
  - Interrupt-driven for accuracy
  - **NEW: Request/Response Pattern** - ROS2 pulls data on-demand at configurable rate (10-50Hz typical)

### Testing & Validation
- **Individual Motor Tests:** Each motor verified operational
- **Left/Right Side Tests:** Both driver boards confirmed working
- **Combined Tests:** All 4 motors responding simultaneously
- **Direction Tests:** Forward, reverse, and turning all functional
- **PID Tuning:** Multiple iterations to balance responsiveness and smoothness while preserving torque
- **Request/Response Protocol:** Tested and verified - encoder and current queries work reliably
- **Current Limiting:** Verified that per-wheel PWM scaling prevents sustained currents above ≈2.6 A without hard shutdown

---

## 📊 Current System Status

### Working Features
✅ 4-motor differential drive control  
✅ PID velocity tracking (smooth, responsive)  
✅ Encoder feedback integration (2× edge counting)  
✅ COBS serial protocol (reliable packet framing)  
✅ Sleep pin control (safe startup)  
✅ Forward/reverse/turning operations  
✅ Safety velocity limits (prevent runaway)  
✅ **ROS2 hardware interface + DiffDriveController** publishing /diff_cont/odom and TF  
✅ **Twist→TwistStamped bridge** (executable, remappable)  
✅ **Robust launch scripts** (serial prep, controller spawn)  
✅ Teleop keyboard driving motors successfully

### Design Decisions
- **Request/Response Over Streaming:** Allows ROS2 to pull odometry at any rate without blocking command RX
- **Conservative PID Gains:** Prioritizes smooth motion over aggressive tracking
- **2× Edge Counting:** Channel A edges with channel B direction for encoder feedback
- **COBS Framing:** Handles serial noise gracefully

### Performance Metrics
- **Command Response:** ~100ms to reach setpoint
- **Velocity Tracking:** Smooth acceleration/deceleration
- **Jitter Level:** Minimal with current PID gains
- **Motor Synchronization:** Left/right pairs track well
- **Feedback Latency:** <5ms when requested (no buffer blocking)

---


## ✅ Completed Work (Phase 1-4)

### Phase 1: Hardware Foundation ✅
- 4-motor differential drive with PID control
- COBS serial protocol (115200 baud)
- Encoder feedback (2× edge counting, 3200 CPR)
- Sleep pin management

### Phase 2: ROS2 Hardware Interface ✅
- mega_diff_drive_control with DiffDriveController
- Request/response encoder feedback
- TF publishing (odom→base_link)
- Teleop control operational

### Phase 3: Sensor Integration & Localization ✅
- Aurora SLAM sensor integrated (/odom, /scan, /imu)
- RTK GPS with dual-antenna heading
- GPS driver enhanced with velocity parsing
- Dual EKF configuration (local + global frames)
- robot_localization fully configured

### Phase 4: Navigation Stack ✅
- Complete qwacr_navigation package with Nav2
- Local/global costmaps configured
- DWB controller + NavFn planner
- GPS waypoint follower system
- Docking server for charging
- RViz verification complete

---

## 🎯 Current Phase: Hardware Integration Testing (Phase 5)

### Phase 5: System Integration Plan

**Objective:** Integrate all individually tested systems through EKF sensor fusion and Nav2 navigation

**Status:** Launch files created, ready for sequential testing

#### Integration Architecture
```
Layer 1: Robot State Publisher (URDF/TF tree)
Layer 2: Hardware Interface (motor control + wheel odometry)
Layer 3: Sensors (Aurora SLAM, GPS with dual-antenna heading)
Layer 4: Localization (Dual EKF: local + global fusion)
Layer 5: Navigation (Nav2 stack with GPS waypoint following)
Layer 6: Visualization (RViz with all sensor overlays)
```

### Created Launch Files ✅

1. **`sensors.launch.py`** - Sensors only (no motors, no Nav2)
   - Aurora SLAM connection (manual launch required)
   - GPS driver with ENU conversion
   - Static TF publishers for sensor frames
   - Use for: Testing sensors at home without motors

2. **`localization.launch.py`** - Sensors + Dual EKF
   - Includes sensors.launch.py
   - Local EKF (odom frame): Aurora + IMU + wheel odom (if available)
   - Global EKF (map frame): Local sensors + GPS
   - Use for: Validating sensor fusion before Nav2

3. **`full_system.launch.py`** - Complete autonomous system
   - All 6 layers integrated
   - Hardware interface with motor control
   - Dual EKF sensor fusion (4 sources: wheel, Aurora, IMU, GPS)
   - Nav2 navigation stack
   - RViz visualization
   - Use for: Full autonomous navigation testing

### Sequential Testing Plan

#### Week 1: Sensors Only (At Home, No Motors)
```bash
# Test 1: GPS data quality
ros2 launch qwacr_navigation sensors.launch.py gps_serial_port:=/dev/ttyACM0

# Verify:
[ ] /fix publishing (NavSatFix messages)
[ ] /gps/enu_odom publishing (position + velocity + heading)
[ ] /gps/heading publishing (dual-antenna heading)
[ ] GPS RTK fix quality (status 5 = best)
[ ] Position stability (log 100 samples, check variance)

# Test 2: Aurora SLAM (launch separately first)
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1

# Then launch sensors
ros2 launch qwacr_navigation sensors.launch.py

# Verify:
[ ] /odom publishing (Aurora SLAM odometry)
[ ] /scan publishing (LiDAR scan data)
[ ] /imu publishing (Aurora IMU data)
[ ] TF: odom→base_link from Aurora

# Test 3: EKF Fusion (Aurora + GPS only)
ros2 launch qwacr_navigation localization.launch.py

# Verify:
[ ] /odometry/local publishing (local EKF output)
[ ] /odometry/global publishing (global EKF with GPS)
[ ] TF tree: map→odom→base_link complete
[ ] Compare raw sensors vs fused output in RViz
[ ] Check covariance convergence
```

#### Week 2: Full System (Bring Motors from Campus)
```bash
# Test 4: Wheel odometry integration
# Connect Arduino Mega with motors
ros2 launch qwacr_navigation full_system.launch.py \
    serial_port:=/dev/ttyACM1 \
    gps_serial_port:=/dev/ttyACM0

# Verify:
[ ] /diff_cont/odom publishing (wheel encoder odometry)
[ ] All 4 sensor sources active (wheel, Aurora, IMU, GPS)
[ ] EKF fusing all inputs
[ ] Nav2 stack receiving fused /odometry/global
[ ] Costmaps populating from /scan
[ ] Can send 2D Nav Goals in RViz

# Test 5: Short-range navigation
# In RViz, send 2D Nav Goal 5-10m away

# Verify:
[ ] Path planned successfully
[ ] Robot follows path smoothly
[ ] Obstacle avoidance from LiDAR
[ ] EKF position estimate stable
[ ] Reaches goal within tolerance

# Test 6: GPS waypoint navigation
ros2 run qwacr_navigation gps_waypoint_follower \
    --waypoints ~/qwacr_ws/src/qwacr_navigation/config/example_waypoints.json

# Verify:
[ ] Waypoints converted to map frame correctly
[ ] Long-range navigation (50-100m)
[ ] GPS corrections integrated smoothly
[ ] Returns to start position accurately
```

#### Week 3: Field Testing
```bash
# Test 7: Outdoor validation (small area 50m × 50m)
[ ] GPS accuracy in open sky
[ ] Aurora SLAM performance outdoors
[ ] EKF handling GPS jumps smoothly
[ ] Costmap updates from real obstacles
[ ] Battery runtime estimation

# Test 8: Extended range (up to 500m)
[ ] Multi-waypoint mission
[ ] Data logging for analysis
[ ] Performance tuning based on results
```

### Testing Tools Available

**Existing:**
- ✅ `hardware_test.py` - Sensor health monitoring
  - Checks Aurora topics
  - Checks GPS fix and heading
  - Checks EKF outputs
  - Data rate verification

**To Create:**
- [ ] `test_ekf_fusion.py` - Compare raw vs fused sensor data
- [ ] `test_transforms.py` - Validate complete TF tree
- [ ] `test_gps_accuracy.py` - Stationary GPS accuracy logging

### Known Configuration

**EKF Sensor Mapping:**
- Local EKF (odom frame):
  - odom0: `odometry/wheel` → remapped to `diff_cont/odom`
  - odom1: `/odom` (Aurora SLAM)
  - imu0: `/imu` (Aurora IMU)

- Global EKF (map frame):
  - odom0: `odometry/wheel` → remapped to `diff_cont/odom`
  - odom1: `/odom` (Aurora SLAM)
  - odom2: `/gps/enu_odom` (GPS with position + velocity + heading)
  - imu0: `/imu` (Aurora IMU)

**Current Blockers:** Full Nav2-on-hardware field testing still pending
**Next Immediate Action:** Continue on-robot Phase 5 tests to validate wheel odometry, current limiting, and turning torque, then integrate sensors + EKF + Nav2 on the physical platform

---

## 📁 Project Structure


### Arduino Code
- `/home/kyle/qwacr_arduino/qwacr_main/`
  - `qwacr_main.ino` - Main control loop
  - `motor_control.h` - PID motor controller
  - `serial_protocol.h` - COBS packet handling
  - `quadrature_decoder.h` - Encoder decoding

### Test Scripts
- `/home/kyle/tests/` (to be created)
  - Motor test scripts (Python)
  - PID tuning utilities
  - Serial debugging tools

### ROS 2 Workspace
- `/home/kyle/qwacr_ws/` (future)
  - Hardware interface package
  - Custom messages/services
  - Navigation stack configuration

---

## 🔧 Key Technical Decisions

### Motor Control Architecture
- **Choice:** PID velocity control vs. position control
- **Rationale:** Velocity control better for continuous navigation
- **Result:** Smooth differential drive operation achieved

### Serial Protocol
- **Choice:** COBS encoding with PacketSerial
- **Rationale:** Reliable framing, handles byte stuffing automatically
- **Result:** Zero packet corruption, reliable communication

### Sleep Pin Management
- **Choice:** Active HIGH with auto-sleep timeout
- **Rationale:** Prevents startup motion, saves power when idle
- **Result:** Safe operation, no unexpected motor activation

### PID Tuning Strategy
- **Choice:** Conservative gains (Kp=7, Ki=0.6, Kd=0.15)
- **Rationale:** Prioritize smoothness over aggressive tracking
- **Result:** Acceptable response with minimal oscillation

---

## 📈 Lessons Learned

### What Worked Well
1. **Incremental Testing:** Testing each motor individually before combined operation
2. **Debug Instrumentation:** LED indicators and serial debug messages
3. **Safety First:** Sleep pins and velocity limits prevented runaway conditions
4. **Parallel Wiring:** Both drivers sharing power supply simplified layout

### Challenges Overcome
1. **Serial Buffer Blocking:** Continuous feedback flooded RX, solved by disabling streaming
2. **Startup Motion:** Floating sleep pins caused unwanted activation, fixed with explicit LOW
3. **Solder Joint Issues:** M1 not responding initially, fixed with reflow
4. **PID Oscillation:** Initial gains too aggressive, tuned down iteratively

### Future Improvements
1. Add encoder calibration routine for absolute distance accuracy
2. Consider adaptive PID or gain scheduling based on load/velocity
3. Expose per-wheel currents on a dedicated ROS topic (and/or JointState effort) for plotting and logging
4. Add higher-level health monitoring for overcurrent, encoder faults, and watchdog timeouts

---

## 🚀 Timeline

- **Jan 15, 2026:** Project kickoff, hardware ordered
- **Jan 16, 2026:** Initial firmware development, wiring began
- **Jan 17, 2026:** ✅ **Phase 1 Complete** - All motors operational
- **Jan 18-20, 2026:** Sensor installation and ROS 2 setup (planned)
- **Jan 21-25, 2026:** Navigation stack integration (planned)

---

## 👥 Team & Resources

**Developer:** Kyle  
**Hardware:** Arduino Mega 2560, Pololu Dual G2 drivers, custom motors  
**Software Stack:** Arduino, ROS 2 Jazzy, Python, C++  
**Development Environment:** WSL Ubuntu 24.04, VS Code

---

*This milestone represents successful completion of the foundational motor control system. The robot is now ready for sensor integration and autonomous navigation development.*
