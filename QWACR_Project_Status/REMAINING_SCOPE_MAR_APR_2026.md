# QWACR – Remaining Scope (Mid-March to End of April 2026)

Project window remaining: ~1.5 months  
Context (as of 2026-03-19): Core drive, localization, Nav2, fire sensor
payload, IMU, and single-stream HaLow video are integrated on the robot
Pi. Remaining work is mainly LoRa/base-station integration, operator
tooling, and full-system validation with a dedicated base-station Pi.

---

## 1. Fire / Environmental Sensor Payload

### 1.1 Current status (March 2026)

**What’s already working:**
- DGS2 H₂ and CO sensors are wired to dedicated UARTs and running as
  ROS 2 nodes publishing gas concentrations and auxiliary data.
- TCA9548A mux and SEN54 + front MLX90640 are online and stable on the
  I²C bus; the second (left) MLX90640 is partially wired but not yet
  finalized.
- SparkFun IMU is integrated on the mux and fused via
  `robot_localization`.

The remaining scope for the fire payload is therefore about **refining
ROS topics, summaries, and LoRa telemetry**, not basic bring-up.

### 1.2 Remaining tasks (fire payload)

**Technical work:**
- Define a compact fire status message for LoRa/base-station use (for
  example, max thermal reading, gas levels, alarm flags) derived from
  the existing detailed topics.
- Finish wiring and validation of the left MLX90640 and bring it up as
  a second thermal node (mirroring the front implementation).
- Add a small health/diagnostics node that:
  - Monitors all fire-sensor topics for stale data or obvious faults
    (for example, I²C errors).
  - Publishes a simple boolean plus reason string ("fire_payload_ok",
    "sensor_missing", and similar).

**Documentation:**
- Update `Fire_Sensor_Integration.md` to reflect the actual port
  mapping, addresses, and ROS topic names.

**Deliverables:**
- fire_status ROS message and publishing node.
- Left MLX90640 node and confirmed topics for both thermal views.
- Diagnostics topic for fire payload health.
- Updated fire-sensor integration guide.

---

## 2. HaLow Video – Single Stream with Switchable Cameras

### 2.1 Current status

- HaLow link is stable for a **single** H.264 stream at
  640×480/15 fps/≈1.2 Mbps.
- Two cameras are physically connected and working under libcamera.
- `qwacr_comms/video_publisher` implements the validated
  `rpicam-vid` → GStreamer pipeline.
- Launches:
  - `halow_video.launch.py` → front camera (index 0) to port 5000.
  - `halow_left_video.launch.py` → left camera (index 1) to port 5000.
- Base station uses a single GStreamer pipeline on port 5000 regardless
  of which camera is active.

Experiments showed that two simultaneous video streams saturate HaLow
and produce heavy corruption, so the design choice is:

> One good stream at a time, with operator-selectable front vs left.

### 2.2 Remaining tasks

**Robot side:**
- Add a simple camera-select control path (likely over LoRa) so the
  operator can request `front` vs `left` from the base-station Pi
  without SSH access.
- Optionally add a lightweight camera switcher node on the robot that
  exposes a ROS service or topic (for example `/set_camera`) and
  internally starts or stops the appropriate `video_publisher`
  instance.

**Base-station side and docs:**
- Extend HALOW_VIDEO_STREAMING_GUIDE.md with an operator-oriented
  section describing:
  - Base-station Pi workflow (GStreamer on Linux).
  - How camera selection is performed via control commands.

**Deliverables:**
- Robot-side camera-select handler (node or service) and a
  corresponding base-station control hook.
- Updated documentation describing the single-stream, switchable-camera
  architecture.

---

## 3. Vehicle-Side System Integration & Testing

### 3.1 Sensor fusion and Nav2 validation

**Remaining tasks:**
- Validate that the integrated IMU and fire-sensor topics are correctly
  fused into the existing EKF and Nav2 stack, and tune parameters as
  needed without breaking current localization.
- Execute a staged testing plan (see NAVIGATION_AUTONOMY_TESTING_GUIDE.md and SENSOR_FUSION_EKF_TESTING_GUIDE.md):
  - Re-validate TF tree (map → odom → base_link) with new sensors.
  - Confirm EKF stability with real IMU + GPS + LiDAR + wheel odometry.
  - Ensure costmaps correctly incorporate LiDAR and any additional obstacle signals.
  - Run waypoint-following missions with obstacle avoidance enabled, logging performance.
- Document tuned parameters and any changes to EKF/ Nav2 configs.

**Deliverables:**
- Updated EKF parameter files and Nav2 configs (if needed).
- Field test notes demonstrating:
  - Reliable waypoint following.
  - Obstacle avoidance behavior.
  - Sufficient localization accuracy for mission requirements.

---

## 4. Base Station Pi 5 (HaLow + LoRa)

### 4.1 Base station hardware & OS bring-up

**Remaining tasks:**
- Prepare a separate Raspberry Pi 5 as the base-station Pi with:
  - HaLow module for high-bandwidth video from the robot.
  - LoRa module for low-bandwidth, long-range command and fire/health
    telemetry.
- Install Ubuntu + ROS 2 Jazzy on the base-station Pi.
- Configure HaLow and LoRa interfaces according to
  LORA_HALOW_INTEGRATION_PLAN.md.

**Deliverables:**
- Documented base station image/setup steps.
- Verified network connectivity: base station ↔ robot over HaLow and LoRa.

---

### 4.2 Telemetry and command streaming

**Remaining tasks:**
- Extend `qwacr_comms` on the robot to:
  - Subscribe to key telemetry topics:
    - Fire sensor payload summaries (fire_status and related
      environment or gas metrics).
    - Core robot state (pose or odometry summary, velocity, battery
      status, and a simple Nav2 mission state).
  - Serialize and send compact telemetry over LoRa to the base-station
    Pi at a low but steady rate.
- Implement base-station counterpart nodes to:
  - Receive and decode LoRa messages.
  - Republish telemetry as ROS topics on the base-station Pi.
  - Provide simple logging and a minimal operator UI (for example rqt
    plus a small custom panel).

**Deliverables:**
- End-to-end data path: sensor topics on robot → LoRa/HaLow → ROS topics on base station Pi.
- Minimal dashboard or RViz config on base station to monitor fire sensors and robot state.

---

### 4.3 Teleoperation and camera/mission commands from base station

**Remaining tasks:**
- Implement teleop command forwarding:
  - Base-station Pi runs a teleop node (keyboard or joystick) that
    publishes compact drive commands.
  - Base-station `qwacr_comms` packages those commands into LoRa
    packets and sends them to the robot.
  - Robot-side `qwacr_comms` unpacks and feeds them into the existing
    control stack (for example `/cmd_vel` or a command-mux input).
- Implement camera-select commands:
  - Base-station UI exposes a simple Front or Left toggle.
  - This sends a small LoRa command that the robot’s camera-select
    handler uses to start the appropriate HaLow video pipeline.
- (Stretch) Support waypoint uploads from base station:
  - Base-station script or UI sends a list of GPS or map-frame
    waypoints over LoRa or HaLow.
  - Robot receives, validates, and loads them into the waypoint
    follower node.

**Deliverables:**
- Verified teleop loop: operator on base station can safely drive robot via network.
- Verified remote waypoint missions initiated from base station and executed on robot.

---

## 5. Integration, Testing, and Documentation

### 5.1 Test campaigns

**Planned efforts:**
- Indoor bench tests:
  - Validate all sensors and comms paths without motion.
  - Confirm base-station visualization of fire-sensor data.
- Outdoor controlled tests:
  - Short-range teleop and waypoint missions.
  - Obstacle avoidance demos with live monitoring.
  - Fire-sensor behavior under controlled stimuli (e.g., heat sources, limited smoke tests per safety guidelines).

**Deliverables:**
- Short test procedures for:
  - Fire sensor payload.
  - Nav2 + localization.
  - Base station comms and teleop.
- Collected logs/bags for final analysis.

### 5.2 Final wrap-up

**Remaining tasks:**
- Update all relevant guides:
  - Fire_Sensor_Integration.md
  - LORA_HALOW_INTEGRATION_PLAN.md
  - NAVIGATION_AUTONOMY_TESTING_GUIDE.md
  - PROJECT_STATUS.md and PROJECT_SUMMARY.md
- Capture known limitations, future work, and maintenance notes.

**Final deliverables:**
- Fully integrated robot + base station system capable of:
  - Autonomous waypoint navigation with obstacle avoidance.
  - Continuous telemetry and fire-sensor data streaming to the base station via HaLow/LoRa.
  - Remote teleoperation and mission control from the base station Pi 5.
