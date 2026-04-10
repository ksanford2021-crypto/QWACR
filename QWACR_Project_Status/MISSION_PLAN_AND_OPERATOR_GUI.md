# QWACR Mission Plan & Operator GUI

Date: 2026-03-20

## 1. High-Level Mission Concept

Operating environment: battery container yard + base-station shelter.

- Robot starts at the base station ("home" pose) in an **idle** state.
- Each battery container has a known waypoint in the Nav2 `map` frame (e.g., `A5`, `B7`, ...).
- When a fire alarm is triggered for container `A5`, the operator:
  - Confirms alarm/location.
  - Uses the operator GUI to send a **waypoint mission** for `A5` via LoRa.
- Robot transitions from **idle** to **autonomous waypoint** mode, drives to `A5` using Nav2.
- At any time, the operator can:
  - Take over in **manual teleop** mode.
  - Command **Return to Base**.
  - Command **Loiter** (hold position or perform a small local pattern).

Core principles:
- Robot electronics and ROS stack are powered and listening, but
  **motors are disabled** until a mission is active.
- LoRa is the authoritative link for mission selection and teleop.
- HaLow is dedicated to video streaming and higher-rate telemetry.

---

## 2. Mission Modes & Topic Interfaces

Mission modes are tracked by a small ROS node on the robot:

- Node: `mission_manager` (package `qwacr_comms`).
- Input:
  - `/mission_command` (`std_msgs/String`)
    - Example payloads: `"idle"`, `"manual"`, `"rtb"`, `"loiter"`,
      `"waypoint:A5"`, `"estop"`.
- Outputs:
  - `/mission_state` (`std_msgs/String`)
    - Current mode: `"idle"`, `"manual"`, `"rtb"`, `"loiter"`,
      `"waypoint:default"`, `"estop"` (ID-specific logic can be added).
  - `/motors_enabled` (`std_msgs/Bool`)
    - `True` in `manual`, `rtb`, `loiter`, `waypoint:*`.
    - `False` in `idle` and `estop`.

Intended consumers:
- Diff-drive controller / Arduino bridge:
  - Subscribes to `/motors_enabled` and disables driver enable lines or
    ignores `cmd_vel` when false.
- Nav2 / mission executor node:
  - Observes `/mission_state` and `/mission_command`.
  - Starts/cancels Nav2 goals for `waypoint:*` and `rtb`.

---

## 3. Base-Station Operator GUI (Qt)

Package: `qwacr_operator_gui`  
Node: `operator_gui` (Python, PyQt5)

### 3.1 Layout

Single window with two main regions:

- **Left: Video panel**
  - Currently a placeholder label that explains the HaLow RTP video
    window should be run separately (GStreamer client) and placed beside
    the GUI on the base-station display.
  - Future work: embed a video widget by integrating with GStreamer or
    an image-transport subscriber.

- **Right: Control & telemetry panel**
  - Telemetry (from `/lora/telemetry_in`):
    - GPS: lat, lon, fix state.
    - Battery: voltage, current.
    - Odom: distance traveled.
    - Fire: compact `fire_status` summary once added to LoRa telemetry.
  - Teleop:
    - Buttons for Forward/Back/Left/Right/Stop.
    - Keyboard teleop when window focused:
      - Forward: `W`
      - Backward: `X`
      - Left turn: `A`
      - Right turn: `D`
      - Stop: `S`
    - Publishes `/cmd_vel_teleop` (`geometry_msgs/Twist`).
  - Camera select:
    - Buttons: **Front (1)**, **Left (2)**.
    - Keyboard shortcuts when window focused:
      - `1` → select **Front** camera.
      - `2` → select **Left** camera.
    - Publishes `/camera_select` (`std_msgs/String` with `"front"` or
      `"left"`), to be bridged over LoRa and/or used directly on the
      robot.
  - Mission mode:
    - Buttons: **Manual Teleop**, **Waypoint (existing)**, **Return to
      Base**, **Loiter**, **E-STOP**.
    - Publishes `/mission_command` (`std_msgs/String`) with simple
      string protocol: `"manual"`, `"waypoint:default"` (or later
      `"waypoint:A5"`), `"rtb"`, `"loiter"`, `"estop"`.

### 3.2 ROS Interfaces (Base-Station GUI)

- Subscriptions:
  - `/lora/telemetry_in` (`std_msgs/String` JSON from `lora_bridge`)
    - Parsed into a local `TelemetrySnapshot` for display.
- Publications:
  - `/cmd_vel_teleop` (`geometry_msgs/Twist`)
  - `/camera_select` (`std_msgs/String`)
  - `/mission_command` (`std_msgs/String`)

The GUI integrates ROS spinning into the Qt event loop using a timer
that periodically calls `rclpy.spin_once`.

---

## 4. LoRa / Mission Command Flow (Concept)

Current LoRa bridge (`qwacr_comms/lora_bridge.py`) transports:
- Telemetry JSON (`/lora/telemetry_out` → UART → radio), and
- Teleop commands (UART → `/lora/cmd_vel_in`).

Planned extension for mission control:
- Add a small **mission command** channel over LoRa:
  - Base-station side:
    - Subscribe to `/mission_command` (from GUI).
    - Encode a compact mission packet (e.g., a few bytes with a command
      ID and an optional waypoint ID like `A5`).
    - Send over UART → LoRa to robot.
  - Robot side:
    - LoRa bridge decodes mission packets.
    - Publishes corresponding `/mission_command` String locally.

This ensures mission selection and waypoint IDs travel over LoRa while
keeping packet sizes small.

---

## 5. GPS / Map Waypoints for Containers and Base

Goal: for each battery container and the base station, define a stable
pose in the Nav2 `map` frame, addressable by a short ID (e.g., `A5`).

### 5.1 Practical workflow for initial Nav2 testing

1. **Bring up localization and mapping**
   - Run Nav2 with your existing map or perform an initial SLAM pass to
     generate a 2D occupancy map of the yard.
   - Ensure the robot has a stable `map → odom → base_link` TF chain.

2. **Establish a map reference for the base station**
   - Drive the robot to its intended "home" position (dock/base
     station) and align it to a sensible heading.
   - Record the robot pose in the `map` frame using, for example:
     - `ros2 topic echo /amcl_pose` or
     - `ros2 run tf2_ros tf2_echo map base_link`.
   - Save this as `BASE` pose in a small YAML or JSON file in the
     robot’s config package (e.g., `waypoints.yaml`).

3. **Mark each container location**
   - For each container (e.g., `A5`):
     - Drive the robot to the desired inspection/observation pose near
       that container.
     - Again, record the `map`-frame pose (x, y, yaw) when stopped.
     - Store it in the same `waypoints.yaml` under the key `A5`.
   - Optionally, also record the corresponding GPS coordinates for each
     container and the base if you plan to use GPS for coarse alignment
     or cross-checking, but Nav2 will operate in the `map` frame.

4. **Waypoint file structure (example)**

   ```yaml
   waypoints:
     BASE:
       x: 0.0
       y: 0.0
       yaw: 0.0
     A5:
       x: 12.3
       y: -4.7
       yaw: 1.57
     B7:
       x: 25.8
       y: 3.2
       yaw: 0.0
   ```

   A simple mission executor node can load this file at startup and
   resolve `waypoint:A5` to a Nav2 goal pose.

5. **Use during Nav2 tests (next week)**
   - Start with:
     - Nav2 running with the yard map and localization.
     - `mission_manager` and future `mission_executor` nodes.
   - Manually trigger waypoint missions from the terminal first (e.g.,
     `ros2 topic pub /mission_command std_msgs/String "data: 'waypoint:A5'"`).
   - Confirm the robot:
     - Leaves `idle`,
     - Drives to the saved `A5` pose, and
     - Can be interrupted by a `manual` command (teleop override) or
       `rtb`.

### 5.2 Relationship between GPS and map

- For Nav2, the authoritative frame is `map`.
- GPS can be used for:
  - Building an approximate transform between a global frame (e.g., UTM)
    and `map` (via `navsat_transform_node` or a custom alignment), and
  - Cross-checking that containers and base station are where you expect
    them globally.
- For early testing, you can treat GPS as auxiliary and focus on
  **`map`-frame waypoints** collected via localization, then layer GPS
  into the system later if needed.

---

## 6. Next Steps

- Implement a `mission_executor` node on the robot that:
  - Loads `waypoints.yaml`.
  - Listens to `/mission_state` and `/mission_command`.
  - Calls Nav2 `NavigateToPose` for `waypoint:*` and `rtb`.
  - Cancels goals when switching to `manual` or `estop`.
- Extend LoRa protocol to carry compact mission commands (IDs like
  `BASE`, `A5`) and map them back to `/mission_command` on the robot.
- Add a simple waypoint selector UI to the operator GUI so the pilot can
  choose a specific container ID instead of a generic
  `"waypoint:default"`.
