# Final Testing and Integration

Date: 2026-03-27  
Robot: QWACR Autonomous Ground Robot

This document captures the high-level steps for final subsystem testing and integration on the robot, with a focus on the IMU configuration (through the TCA9548A I²C mux), lidar over UART, GPS, and the current status of the motor encoders / Arduino interface.

---

## 1. IMU Bring-Up and Verification

### 1.1 Hardware context

- IMU: SparkFun 9DoF IMU Breakout – ISM330DHCX (Qwiic).
- I²C topology:
  - Raspberry Pi 5 I²C1 (SDA1/SCL1) → TCA9548A I²C mux at address 0x70.
  - IMU is connected to **mux port 7** (SDA7/SCL7) on the TCA9548A.
- Effective IMU device address (once mux port 7 is enabled): **0x6B** on bus 1.

### 1.2 One-time verification of wiring (I²C scan)

On the robot Pi (logged in as `qwacr`):

```bash
sudo i2cdetect -y 1          # should show 70 (mux) only
sudo i2cset -y 1 0x70 0x80   # enable mux port 7 (bit 7 → 0x80)
sudo i2cdetect -y 1          # should now show both 70 and 6b
```

If `0x6b` appears after enabling port 7, the IMU wiring and mux configuration are correct.

### 1.3 Launching the IMU node (ROS 2)

Workspace: `~/qwacr_ws`  
Package: `qwacr_imu`  
Node script: `qwacr_imu/sparkfun_imu_node.py`

Because of how the Python console script is installed, the IMU node is launched via `python3 -m` instead of `ros2 run`:

1. In **Terminal 1** on the robot Pi:

  ```bash
  cd ~/qwacr_ws
  source install/setup.bash

  # Optional: reselect mux port 7 before launching (defensive)
  sudo i2cset -y 1 0x70 0x80

  # Start the IMU node, explicitly setting mux_port to 128 (0x80)
  python3 -m qwacr_imu.sparkfun_imu_node --ros-args -p mux_port:=128
  ```

  - `mux_port` is specified in **decimal**; 128 corresponds to `0x80`, which is mux **port 7**.
  - The node uses the SparkFun driver to configure the ISM330DHCX and publishes `sensor_msgs/Imu` on `/imu/data`.

2. In **Terminal 2** (separate SSH session):

  ```bash
  cd ~/qwacr_ws
  source install/setup.bash

  ros2 topic list | grep imu
  ros2 topic echo /imu/data
  ros2 topic hz /imu/data
  ```

  - With the robot stationary, expect:
    - `angular_velocity` values near zero (small noise around each axis).
    - `linear_acceleration` showing gravity distributed across axes depending on the IMU orientation (roughly 9.8 m/s^2 in total magnitude).
  - When the robot is moved/tilted, both angular velocity and linear acceleration should change accordingly.

### 1.4 Notes on interpreting IMU output and TF (updated April 7, 2026)

- The `orientation` field is intentionally disabled (`orientation_covariance[0] = -1.0`); pose fusion is handled by `robot_localization`.
- The `linear_acceleration` field initially included gravity with the IMU mounted such that +Y was forward and **-Z was down**. This meant:
  - At rest, `linear_acceleration.z ≈ -9.7 m/s^2`.
  - Clockwise yaw rotation produced a **positive** `angular_velocity.z`.
- On 2026-04-07, the static transform from `base_link` to `imu_link` in `qwacr_navigation/launch/sensors.launch.py` was updated to:
  - First rotate +90° about Z (yaw = +π/2) so IMU +Y aligns with base +X.
  - Then rotate 180° about X (roll = +π) so IMU -Z becomes base +Z.
  - Combined RPY ≈ (roll = 3.1416, pitch = 0.0, yaw = 1.5708).
- After this TF change and relaunching `sensors.launch.py`:
  - At rest, the EKF should see gravity as **+9.8 m/s² on Z** in the robot frame.
  - Counter-clockwise (left) yaw should produce **positive** `angular_velocity.z`; clockwise should be negative, matching REP-103.
  - These conventions are required for `robot_localization` to correctly fuse IMU + wheel odometry + GPS.

---

### 1.5 Local EKF (wheel + IMU) outdoor test (April 9, 2026)

Goal: validate that the **local EKF** (`/odometry/local`) correctly fuses wheel odometry translation with IMU yaw outside, using the current configuration where wheel yaw is de-weighted and IMU dominates heading.

#### 1.5.1 Bring-up sequence on the robot Pi

All commands below run on the **robot Pi** (`qwacr@192.168.11.100`) in separate terminals.

1. **Terminal 1 – IMU via mux port 7**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash

   # Ensure mux port 7 (0x80) is selected
   sudo i2cset -y 1 0x70 0x80

   # Start IMU node with mux_port=128 (0x80)
   python3 -m qwacr_imu.sparkfun_imu_node --ros-args -p mux_port:=128
   ```

   - Verify `/imu/data` is publishing:

     ```bash
     ros2 topic echo /imu/data
     ros2 topic hz /imu/data
     ```

2. **Terminal 2 – ros2_control hardware + diff_drive_controller**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh /dev/ttyACM0
   ```

   - Wait for `diff_cont` and `joint_broad` to report **active**.
   - Check that `/odometry/wheel` and `/joint_states` are publishing.

3. **Terminal 3 – Local + global EKF (robot_localization)**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 launch qwacr_navigation localization.launch.py
   ```

   - This uses `params/ekf_local_global.yaml`, where:
     - Wheel odom contributes **x, y, vx, vy** only.
     - IMU provides rotational components (roll/pitch/yaw rates, accelerations).
   - Confirm topics:

     ```bash
     ros2 topic list | egrep "odometry/local|odometry/wheel|imu/data"
     ```

#### 1.5.2 Outdoor test maneuvers

With all three terminals running and the robot on level ground (motors off or in safe state; you can push the robot by hand):

1. **Straight-line push (5–10 m)**
   - Push the robot straight forward; avoid intentional steering.
   - In another terminal, record or monitor:

     ```bash
     cd ~/qwacr_ws
     source install/setup.bash
     ros2 bag record /odometry/local /odometry/wheel /imu/data -o ekf_local_outdoor_$(date +%Y%m%d_%H%M)
     ```

   - Expected behavior:
     - `/odometry/wheel` and `/odometry/local` both show roughly straight motion in +X.
     - Y drift in `/odometry/local` should be **smaller** than raw wheel-only y drift, because IMU yaw constrains heading.

2. **Return along the same path**
   - Pull/push the robot back approximately along the same line.
   - The `/odometry/local` endpoint should be close to the start (small residual position error is OK).

3. **In-place yaw rotations**
   - If/when it is safe to rotate in place (motors enabled or by carefully pivoting the robot):
     - Perform ~90° left and right turns.
     - Monitor `/odometry/local` and `/imu/data` yaw (e.g., with `odom_yaw_printer.py` or RViz).
   - Expected behavior:
     - Yaw increments of ~±1.57 rad for 90° turns.
     - IMU yaw and EKF yaw closely track each other (small lag/noise OK).

4. **Small loop (e.g., 2 m square)**
   - Push the robot in a simple square or rectangle.
   - `/odometry/local` should form a roughly closed loop; final pose near the start.

#### 1.5.3 Post-test analysis

Back indoors on the dev machine or base-station Pi:

- Inspect the recorded bag (`ekf_local_outdoor_*`) in RViz:
  - Plot `/odometry/local` and `/odometry/wheel` paths together.
  - Check that the EKF path is smoother and exhibits less sideways drift than the wheel-only path.
- If EKF yaw still drifts noticeably despite the IMU, revisit:
  - `/imu/data` sign conventions and covariance settings.
  - The static transform from `base_link` to `imu_link` in `sensors.launch.py`.

---

## 2. Encoders and Arduino Motor Control – Current Status

### 2.1 Intended architecture

- Microcontroller: Arduino Mega (or equivalent) connected to the Raspberry Pi over USB (e.g., `/dev/ttyACM*`).
- Responsibilities on the Arduino side:
  - Read quadrature encoders on each drive motor.
  - Drive motor controllers (PWM + direction or similar interface).
  - Report encoder counts / velocities back to the Pi.
- On the Pi, a ROS 2 hardware interface / `ros2_control` controller consumes encoder data and publishes wheel odometry for the EKF.

### 2.2 Observed problem (March 2026)

- During full robot assembly, the electrical connections from the drive motors and encoders to the Arduino became **mechanically very stiff / constrained**.
- Symptoms during bench testing:
  - Unreliable or missing encoder feedback.
  - Motor control commands not behaving as expected (inconsistent movement / no movement).
- Likely contributing factors:
  - Strain on encoder and motor signal wires due to tight routing and robust bundling.
  - Possible intermittent connections at the Arduino screw terminals / headers.

### 2.3 Current mitigation and open tasks

- For initial indoor testing, **motor motion is deferred**; focus is on validating all passive sensors and comms:
  - IMU (`qwacr_imu` on mux port 7).
  - GPS (`qwacr_gps`).
  - Fire sensing stack via the TCA9548A.
  - HaLow video and LoRa telemetry.
- Motor/encoder TODOs before full motion testing:
  - [ ] Loosen or reroute the harness from motors/encoders to the Arduino to remove mechanical strain.
  - [ ] Individually continuity‑check encoder channel wiring from sensor to Arduino pins.
  - [ ] Re‑verify that encoder inputs, PWM outputs, and direction pins match the documented pinout in the Arduino firmware and PIN_ASSIGNMENT notes.
  - [ ] Re‑run standalone Arduino motor/encoder test sketches (no ROS) to confirm clean encoder counts in both directions.
  - [ ] Once stable, re‑enable the full ROS 2 `ros2_control` stack and validate wheel odometry topics while the robot is on blocks.

---

## 3. Lidar (RPLIDAR S3) over UART0

### 3.1 Hardware / UART context

- Sensor: RPLIDAR S3.
- Wiring (Pi 5 40‑pin header):
  - Lidar TX (green) → Pi RX0 (GPIO15, physical pin 10).
  - Lidar RX (yellow) → Pi TX0 (GPIO14, physical pin 8).
  - Lidar 5 V and GND to Pi 5 V / GND.
- Pi boot config (`/boot/firmware/config.txt`) includes:

  ```text
  dtoverlay=uart0,txd0_pin=14,rxd0_pin=15
  ```

  which exposes the lidar UART as `/dev/ttyAMA0`.

### 3.2 Lidar launch command

On the robot Pi:

```bash
cd ~/qwacr_ws
source install/setup.bash

ros2 launch rplidar_ros rplidar_s3_launch.py \
  serial_port:=/dev/ttyAMA0 \
  serial_baudrate:=1000000 \
  frame_id:=laser
```

This uses the `DenseBoost` scan mode by default and publishes `sensor_msgs/LaserScan` on `/scan` with frame `laser`.

---

### 3.3 Visualizing lidar data in RViz via XLaunch

On the Windows laptop:

1. Start **XLaunch / VcXsrv** with:
  - Any window mode (e.g., *One large window*).
  - *Start no client*.
  - Enable **Disable access control**.
  - Optional extra parameters: `-ac -listen tcp`.

2. From a terminal on the laptop, SSH to the Pi (no X forwarding needed):

  ```bash
  ssh qwacr@192.168.11.100
  ```

On the robot Pi SSH session:

3. Point `DISPLAY` at the laptop's IP (example uses `192.168.11.50` from previous sessions):

  ```bash
  export DISPLAY=192.168.11.50:0.0
  cd ~/qwacr_ws
  source install/setup.bash
  rviz2
  ```

4. In RViz:
  - Set **Fixed Frame** (Global Options) to `laser` (type it manually if it is not in the dropdown).
  - Add a **LaserScan** display and set **Topic** to `/scan`.
  - The live environment should appear as a 2D ring of points around the lidar.

---

## 4. GPS (LG580P) over /dev/ttyACM1

### 4.1 Hardware / USB context

- GPS: LG580P receiver (USB‑C serial).
- Connection: LG580P USB‑C → Pi USB‑A; appears as one of `/dev/ttyACM*` devices.
- On this robot, the Arduino occupies `/dev/ttyACM0`; the GPS is **`/dev/ttyACM1`**.
- Configured baud rate: **460800**.

### 4.2 GPS launch command

On the robot Pi:

```bash
cd ~/qwacr_ws
source install/setup.bash

ros2 launch qwacr_gps gps.launch.py \
  serial_port:=/dev/ttyACM1 \
  baud_rate:=460800
```

This starts `gps_driver_node`, which:

- Reads NMEA from `/dev/ttyACM1` at 460800 baud.
- Sets the ENU origin on first valid fix.
- Publishes:
  - `/gps/fix` (`sensor_msgs/NavSatFix`).
  - `/gps/enu_pose` and `/gps/enu_odom` (if ENU enabled).
  - `/gps/heading` when heading messages are available.

---

## 5. Suggested Final Integration Sequence (High Level)

1. **Sensors only, robot stationary**
   - Bring up IMU, GPS, fire sensors, and verify all key topics are publishing.
2. **Network and comms**
  - Verify HaLow video stream and LoRa telemetry in both directions.
3. **Encoders + motors on blocks**
   - Fix Arduino/encoder wiring.
   - Run low‑speed motor tests with wheels off the ground; verify odometry.
4. **Full motion tests**
   - Run EKF with IMU + wheel odometry (and GPS outdoors).
   - Validate autonomous navigation behaviors once base mobility is trusted.

---

## 6. Base-Station Pi and Operator GUI (March 29, 2026)

### 6.1 Direct Ethernet link to base-station Pi

- **Base-station Pi hostname/user:** `qwacrpi`.
- **Robot networking (Aurora/robot Pi):**
  - Aurora sensor IP: `192.168.11.1`.
  - Robot Pi eth0 IP: `192.168.11.100` (as documented in PI_ACCESS_POINT_SETUP).
- **Base-station networking (this Pi):**
  - eth0 is used for a direct cable to the laptop.
  - Static IPv4 address on base-station Pi eth0: `192.168.11.101/24`.
  - Laptop wired interface (eth1 on dev machine) is `192.168.11.50/24`.

To establish the wired link (one-time commands, already validated):

On the **base-station Pi** console/SSH:

```bash
sudo ip addr add 192.168.11.101/24 dev eth0
```

On the **laptop** (Ubuntu dev machine):

```bash
sudo ip addr add 192.168.11.50/24 dev eth1
ping -c 3 192.168.11.101
ssh qwacrpi@192.168.11.101
```

> Note: For initial access (before IPv4 was set), IPv6 link-local + `ssh -6 qwacrpi@fe80::...%eth1` was used, but once the static IPv4 addresses above are configured, normal IPv4 SSH is preferred.

### 6.2 Running the operator GUI on the base-station Pi

Workspace: `~/qwacr_ws` on the **base-station Pi**.  
Package: `qwacr_operator_gui` (Qt + rclpy).

On the laptop (Windows) with VcXsrv / XLaunch running:

1. Start **VcXsrv/XLaunch** as in section 3.3 ("Start no client", **Disable access control**).
2. From an Ubuntu terminal on the laptop, SSH to the base-station Pi with X forwarding:

   ```bash
   ssh -X qwacrpi@192.168.11.101
   ```

3. In the SSH session on the base-station Pi, point `DISPLAY` at the laptop (example IP `192.168.11.50`):

   ```bash
   export DISPLAY=192.168.11.50:0.0
   source /opt/ros/jazzy/setup.bash
   source ~/qwacr_ws/install/setup.bash
   ros2 run qwacr_operator_gui operator_gui
   ```

**Result:** the QWACR Operator GUI window appears on the laptop, but is running on the base-station Pi. The left pane shows "Waiting for video on UDP port 5000..." until the HaLow video stream is active.

### 6.3 Embedded video feed (HaLow RTP/H.264)

The GUI now embeds the GStreamer receive pipeline directly into the left **Video** panel using `ximagesink` with the widget's XID.

- Embedded pipeline (conceptually equivalent to):

  ```bash
  gst-launch-1.0 \
    udpsrc port=5000 \
      caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
    rtph264depay ! avdec_h264 ! videoconvert ! \
    ximagesink sync=false xid=<video-widget-xid>
  ```

- This is the same receive pipeline described in HALOW_VIDEO_STREAMING_GUIDE, but now driven automatically by the GUI when it starts.
- When HaLow video is not running, the GUI shows the message **"Waiting for video on UDP port 5000..."** in the video panel.
- When the robot-side HaLow encoder is started (e.g. `ros2 launch qwacr_comms halow_video.launch.py` with `udpsink host=192.168.11.101 port=5000`), the stream should appear directly inside the GUI.

### 6.4 Keyboard teleop and camera shortcuts (operator GUI)

The operator GUI publishes control commands over ROS 2:

- `/cmd_vel_teleop` (`geometry_msgs/Twist`) → consumed by `qwacr_comms/command_mux` → `/cmd_vel`.
- `/camera_select` (`std_msgs/String`, values `"front"` or `"left"`).
- `/mission_command` (`std_msgs/String`, values `"manual"`, `"rtb"`, `"waypoint:default"`, `"loiter"`, `"estop"`).

Mouse buttons and keyboard shortcuts are both supported when the GUI window is focused:

- **Teleop buttons (with keys in labels):**
  - Forward (W)
  - Back (X)
  - Left (A)
  - Right (D)
  - Stop (S)

- **Keyboard mappings (no mouse needed):**
  - Forward: `W`.
  - Backward: `X`.
  - Turn left: `A`.
  - Turn right: `D`.
  - Stop: `S`.

- **Camera selection:**
  - Buttons labeled **Front (1)** and **Left (2)**.
  - Keyboard shortcuts: `1` selects **front** camera, `2` selects **left** camera (publishes `/camera_select`).

> Reminder (from BASIC_LAUNCH_PROCEDURES): the GUI only drives `/cmd_vel_teleop` → `/cmd_vel`. To move the real robot, the full stack must also include `qwacr_comms/command_mux` and the Twist-to-TwistStamped bridge so `/cmd_vel` is bridged to `/diff_cont/cmd_vel` for the diff_drive controller.

### 6.5 LoRa + operator GUI end-to-end test (March 31, 2026)

Goal: confirm that teleop commands from the **base-station operator GUI** travel
over **LoRa** and arrive on the robot as `/lora/cmd_vel_in`, without actually
driving the motors.

#### 6.5.1 Robot Pi (LoRa receiver + echo only)

- Workspace: `~/qwacr_ws` on the **robot Pi** (`qwacr@192.168.11.100`).
- LoRa radio is on `/dev/ttyUSB0`.

In **Terminal 1** on the robot Pi:

```bash
cd ~/qwacr_ws
source install/setup.bash
ros2 run qwacr_comms lora_bridge --ros-args -p role:=robot -p serial_port:=/dev/ttyUSB0
```

In **Terminal 2** on the robot Pi (verification only, no motors):

```bash
cd ~/qwacr_ws
source install/setup.bash
ros2 topic echo /lora/cmd_vel_in
```

Expected: when we later press teleop controls in the GUI, this echo should show
`geometry_msgs/Twist` messages with `linear.x`/`angular.z` matching the
requested motion.

#### 6.5.2 Base-station Pi (LoRa base role + command_mux + GUI)

- Workspace: `~/qwacr_ws` on the **base-station Pi** (`qwacrpi@192.168.11.101`).
- LoRa radio is also on `/dev/ttyUSB0`.

In **Terminal 1** on the base-station Pi (LoRa bridge, base role):

```bash
cd ~/qwacr_ws
source install/setup.bash
ros2 run qwacr_comms lora_bridge --ros-args -p role:=base -p serial_port:=/dev/ttyUSB0
```

In **Terminal 2** on the base-station Pi (command multiplexer):

```bash
cd ~/qwacr_ws
source install/setup.bash
ros2 run qwacr_comms command_mux
```

In **Terminal 3** on the base-station Pi (GUI, displayed via XLaunch on the
Windows laptop):

```bash
export DISPLAY=192.168.11.50:0.0   # laptop wired IP
cd ~/qwacr_ws
source install/setup.bash
ros2 run qwacr_operator_gui operator_gui
```

With the GUI focused on the laptop, press and hold **W** (or click
**Forward (W)**) for ~1 second, then release. On the robot Pi's
`/lora/cmd_vel_in` echo you should see:

- While holding **W**: `linear.x ≈ +0.4`, `angular.z ≈ 0.0`.
- After release / pressing Space or **Stop**: `linear.x ≈ 0.0`, `angular.z ≈ 0.0`.

This confirms the path:

`GUI (/cmd_vel_teleop)` → `command_mux (/cmd_vel)` → `lora_bridge (base)` →
LoRa radios → `lora_bridge (robot)` → `/lora/cmd_vel_in`.

> Note: the updated GUI uses **WASD only** for teleop (`W`/`S`/`A`/`D` +
> Space). Older IJKL and arrow-key bindings were intentionally removed to
> simplify training.

#### 6.5.3 Future TODO – update robot Pi GUI

The updated `qwacr_operator_gui` (WASD-only teleop and embedded GStreamer
receiver) is currently synchronized and built on the **base-station Pi**. The
robot Pi still has an older copy of the package and should be updated before
running the GUI there directly.

Planned steps (not yet executed):

- Copy the updated package from the dev machine to the robot Pi:

  ```bash
  scp -r ~/qwacr_ws/src/qwacr_operator_gui qwacr@192.168.11.100:~/qwacr_ws/src/
  ```

- Rebuild on the robot Pi:

  ```bash
  cd ~/qwacr_ws
  colcon build --packages-select qwacr_operator_gui
  source install/setup.bash
  ```

Once this is done, the robot Pi will match the base-station Pi for GUI
behavior and can also host the operator GUI if desired.

### 6.6 LoRa + ros2_control full-motion stack (software path only)

On 2026-03-31 we also exercised the **full teleop stack** from the
base-station GUI, through LoRa, all the way to `/diff_cont/cmd_vel` on the
robot. The ROS side behaved as expected; lack of wheel motion is now believed
to be a **hardware / Arduino / motor-driver issue**, not a software one.

#### 6.6.1 Robot Pi launch sequence (with motors enabled)

Terminals on the **robot Pi** (`qwacr@192.168.11.100`):

1. **ros2_control stack + controllers**

   ```bash
   cd ~/qwacr_ws
   ~/qwacr_ws/src/qwacr_build/scripts/launch_robust.sh /dev/ttyACM0
   ```

   - Starts the hardware interface + `controller_manager`.
   - Spawns `diff_cont` (diff-drive) and `joint_broad` (joint_state_broadcaster).
   - Wait until both controllers report `active`.

2. **LoRa bridge (robot role)**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 run qwacr_comms lora_bridge \
     --ros-args -p role:=robot -p serial_port:=/dev/ttyUSB0
   ```

3. **Command multiplexer (prioritizes LoRa/teleop/auto)**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 run qwacr_comms command_mux
   ```

4. **Twist→TwistStamped bridge into the controller**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 run qwacr_build twist_to_stamped.py \
     --ros-args \
     -r cmd_vel:=/cmd_vel \
     -r cmd_vel_stamped:=/diff_cont/cmd_vel
   ```

   - Subscribes to `/cmd_vel` (from `command_mux`).
   - Publishes `geometry_msgs/TwistStamped` on `/diff_cont/cmd_vel`, which the
     `diff_drive_controller` consumes.

**Verification on the robot:**

- `ros2 topic echo /lora/cmd_vel_in` shows teleop commands arriving over LoRa.
- `ros2 topic echo /cmd_vel` matches those commands after muxing.
- `ros2 topic echo /diff_cont/cmd_vel` shows stamped commands feeding the
  controller.
- `ros2 topic info /diff_cont/cmd_vel` shows 1 publisher (bridge) and at least
  1 subscriber (controller).

#### 6.6.2 Base-station Pi launch sequence (LoRa + GUI)

Terminals on the **base-station Pi** (`qwacrpi@192.168.11.101`):

1. **LoRa bridge (base role)**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 run qwacr_comms lora_bridge \
     --ros-args -p role:=base -p serial_port:=/dev/ttyUSB0
   ```

2. **Command multiplexer (base side)**

   ```bash
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 run qwacr_comms command_mux
   ```

   - Consumes `/cmd_vel_teleop` from the GUI and publishes `/cmd_vel` for
     `lora_bridge` to send.

3. **Operator GUI via XLaunch on the laptop**

   ```bash
   export DISPLAY=192.168.11.50:0.0   # laptop wired IP
   cd ~/qwacr_ws
   source install/setup.bash
   ros2 run qwacr_operator_gui operator_gui
   ```

With this setup, pressing **W/A/S/D/X** in the GUI results in:

- `/cmd_vel_teleop` changing on the base-station Pi.
- `/cmd_vel` changing on both base-station and robot Pis (after mux).
- `/diff_cont/cmd_vel` changing on the robot Pi.

#### 6.6.3 Current status and recommended next hardware tests

During March 31, 2026 testing, all software checks above passed, but the wheels
did **not** rotate. This strongly suggests a fault below ROS:

- Arduino firmware not driving the motor outputs as expected.
- Motor power / main breaker / e-stop chain not latched.
- Loose or strained motor/encoder wiring between Arduino, drivers, and motors.

For hardware debugging, it is recommended to temporarily bypass LoRa and use
the simpler, known-good SSH teleop path from BASIC_LAUNCH_PROCEDURES:

- Run `launch_robust.sh` and `twist_to_stamped.py` on the robot Pi.
- From an SSH session or local console, run:

  ```bash
  source ~/qwacr_ws/install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

If the wheels still do not move under direct `teleop_twist_keyboard`, the
issue is confirmed to be purely hardware/firmware, not HaLow/LoRa/GUI.

---

## 7. April 9, 2026 Status Update (HaLow Video + EKF)

### 7.1 HaLow video and dual-camera switching

Status: **PASS (manual pipeline over HaLow)**

- HaLow link verified between robot Pi and operator machine (`10.192.5.x`).
- Front and left Pi cameras were both streamed successfully by switching
  `rpicam-vid --camera 0` and `--camera 1` while keeping the same RTP/H.264
  receive pipeline on UDP port `5000`.
- Observed blur on one camera view was traced to the physical housing/plastic
  window in front of the lens (optical issue), not transport or codec issues.
- Base-station Pi GStreamer currently lacks `avdec_h264` plugin; full
  embedded-video-in-GUI validation on base-station Pi is deferred until
  `gstreamer1.0-libav` is installed.

### 7.2 EKF bring-up fixes made today

Status: **Partially resolved; stack now using correct topics/config**

- GPS/Arduino serial conflict resolved operationally:
  - Arduino motor interface remains on `/dev/ttyACM0`.
  - GPS is launched on `/dev/ttyACM1`.
- Stale Aurora/Slamware topic assumptions were removed from the localization
  path; EKF now uses active robot topics:
  - Wheel odom: `/diff_cont/odom`
  - IMU: `/imu/data`
  - GNSS odom: `/gps/enu_odom`
- Important deployment lesson confirmed:
  - After copying edited launch/config files to robot Pi, `colcon build` was
    required so the installed launch/config artifacts were refreshed.
  - After build + relaunch, behavior changed as expected.

### 7.3 Remaining EKF issue observed

Status: **Open issue**

- During stationary tests, `/odometry/local` linear twist previously drifted
  away from zero even when `/diff_cont/odom` was near zero.
- A `/set_pose` reset successfully zeroed the state momentarily, then drift
  resumed in some runs.
- Stationary rosbag was captured for follow-up analysis:
  - `~/qwacr_ws/ekf_stationary_20260318_1357_0.mcap`

### 7.4 Plan for April 10 (re-enable IMU linear acceleration carefully)

Goal: reintroduce IMU linear acceleration only after confirming stable baseline.

1. **Baseline run (accel disabled in EKF IMU config)**
   - Confirm stationary `/odometry/local` twist remains near zero.
   - Record short stationary bag for baseline.

2. **Controlled accel re-enable test**
   - Re-enable `ax, ay, az` fusion in EKF config.
   - Keep `imu0_remove_gravitational_acceleration: true`.
   - Relaunch and test stationary drift for 30-60 s.

3. **Acceptance criteria**
   - Stationary: `|vx|, |vy|` remain close to 0 (no monotonic runaway).
   - Slow push test: local odom responds to real motion and settles at rest.

4. **If drift returns immediately**
   - Disable accel fusion again and proceed with gyro + wheel odom for motion
     testing so autonomy integration is not blocked.

