# QWACR Project Status - January 22, 2026

## 🎉 MILESTONE: Phase 2 Complete - ROS2 Hardware Interface, Odometry, Teleop

### Summary
ROS2 hardware interface is operational with encoder feedback and odometry publishing. Teleop controls all motors reliably; robust launch scripts handle serial prep and controller startup. DiffDriveController publishes /diff_cont/odom and TF (odom→base_link), with Twist→TwistStamped bridge fixed and executable.

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
  - Tuning: Kp=7.0, Ki=0.6, Kd=0.15
  - Velocity safety limits: ±15 rad/s
  - Smooth response with minimal jitter
- **Serial Protocol:** COBS encoding for reliable packet framing
  - Command format: [0x01][left_vel_float][right_vel_float]
  - Request format: [0x02] - queries encoder data
  - Response format: [0x10][enc_fl:i32][enc_bl:i32][enc_fr:i32][enc_br:i32][vel_fl:f32][vel_bl:f32][vel_fr:f32][vel_br:f32]
  - 115200 baud, USB connection
- **Sleep Pin Management:** Active HIGH control prevents startup motion
  - Drivers sleep by default, wake on first command
  - Auto-sleep after 2s timeout
- **Encoder Feedback:** 2× edge counting on all 4 motors (channel A edges + channel B direction)
  - 3200 counts per revolution effective (gear ratio included)
  - Interrupt-driven for accuracy
  - **NEW: Request/Response Pattern** - ROS2 pulls data on-demand at configurable rate (10-50Hz typical)

### Testing & Validation
- **Individual Motor Tests:** Each motor verified operational
- **Left/Right Side Tests:** Both driver boards confirmed working
- **Combined Tests:** All 4 motors responding simultaneously
- **Direction Tests:** Forward, reverse, and turning all functional
- **PID Tuning:** Multiple iterations to balance responsiveness and smoothness
- **Request/Response Protocol:** Tested and verified - encoder queries work reliably

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

## 🎯 Next Phase: ROS 2 Integration (Phase 2)

### Next Week: GPS + LiDAR Integration (Weeks 3-4)
**Priority:** Bring up GPS and LiDAR sensors, fuse with wheel odometry, prepare for Nav2

**Week 3 (Immediate):**
1. **Hardware Setup**
   - [ ] Connect GPS UART to Raspberry Pi (9600 baud)
   - [ ] Connect SLAMTEC Aurora LiDAR USB
   - [ ] Verify both devices appear in `/dev/` and ROS topic latency

2. **GPS Integration**
   - [ ] Install/verify `gps_common` ROS 2 driver
   - [ ] Publish NavSatFix on `/fix` topic
   - [ ] Record sample GPS data; verify fix quality and accuracy
   - [ ] Build GPS→ENU converter (lat/lon → meters in local frame)

3. **LiDAR Integration (Aurora)**
   - [ ] Install/verify `rplidar_ros` driver (or SLAMTEC SDK)
   - [ ] Publish LaserScan on `/scan` topic @ 10-20 Hz
   - [ ] Add TF broadcaster for `/base_link` → `/laser_frame`
   - [ ] Record 10+ scans; visualize in RViz

4. **Sensor Fusion**
   - [ ] Create fusion node: GPS + wheel odom → global pose (/odom in GPS frame)
   - [ ] Publish TF: `/map` ← GPS origin, `/odom` ← wheel frame
   - [ ] Test in RViz: visualize GPS points + laser + odometry drift

**Week 4 (Integration & Tuning):**
1. **Local Costmap**
   - [ ] Build local costmap from LiDAR scans using `costmap_2d`
   - [ ] Test obstacle detection; record performance

2. **Testing & Validation**
   - [ ] Record rosbags for 5+ minutes of driving
   - [ ] Analyze GPS fix rate, LiDAR range accuracy, fusion stability
   - [ ] Identify drift between wheel odom and GPS

3. **Nav2 Prep**
   - [ ] Create nav2 config with GPS goal (lat/lon → ENU goal)
   - [ ] Test global planner on recorded map
   - [ ] Dry-run controller in real world (manual GPS waypoint)

**Blockers & Decisions:**
- **GPS Accuracy:** ±1–5 m acceptable for Phase 3. If worse, may need RTK base station (defer).
- **LiDAR Range:** Verify 12 m+ range indoors; record bag for troubleshooting if weak.
- **Odometry Drift:** Monitor GPS vs. wheel odom; if >1 m/min, investigate encoder calibration.

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
1. Implement request/response feedback instead of streaming
2. Add encoder calibration routine for accurate odometry
3. Consider adaptive PID based on load/velocity
4. Add motor current monitoring for fault detection

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
