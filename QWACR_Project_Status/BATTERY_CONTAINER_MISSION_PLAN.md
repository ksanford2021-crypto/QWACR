# QWACR Battery Container Response Mission Plan

**Project:** QWACR Autonomous Ground Robot  
**Document:** Battery Container Response Mission Plan  
**Date Created:** February 24, 2026  
**Status:** Draft – For BT & Mission Node Design

---

## 🎯 Mission Overview

The QWACR robot provides **autonomous response** to potential fire events in a yard of battery containers. Each container has a **pre-mapped GPS position** on a **static global map**. When a specific container raises a possible fire alarm, the robot must:

1. **Sit idly in low-power mode** near a base station until a mission request arrives over **LoRa**.  
2. **Wake up the full autonomy stack** (sensors, localization, Nav2, mission node, video) when a mission request is received.  
3. **Navigate autonomously** from base to the targeted container’s location using:
   - Pre-built static map (map server)
   - RTK GPS, Aurora SLAM, IMU, wheel odometry (dual EKF)
   - Nav2 navigation stack with custom BT (qwacr_outdoor_nav.xml)
4. **Provide continuous telemetry** during the mission:
   - Fire/thermal sensor data → uplink over LoRa  
   - Live video stream → uplink over HaLow
5. **On arrival at the target container**, perform one of:
   - **Preferred (v2):** Patrol/loiter in a small pattern around the container.  
   - **Baseline (v1):** Loiter in a hold position in front of the container.
6. Allow a **remote operator** to:
   - Take over **teleop control** at any time via LoRa (override autonomy).  
   - **Return control to autonomy** at any time.  
   - Command the robot to **return to base**.
7. **Navigate back to base** autonomously when commanded, then return to low-power idle.

---

## 🗺️ Map & Battery Container Database

### Static Map Assumptions

- The operating area uses a **pre-built, static 2D Nav2 map** (`map` frame) that does **not** change during normal operations.  
- Dynamic obstacles (vehicles, people, movable equipment) are handled **purely via costmaps** (obstacle layers) built from LiDAR, with no persistent map changes.  
- Localization uses:
  - Aurora SLAM odometry + IMU  
  - RTK GPS (ENU odometry with position, velocity, heading)  
  - Wheel odometry  
  - Dual EKF (local `odom`, global `map`) as defined in existing localization configs.

### Battery Container Position Database

- Each battery container is assigned a **unique container ID** (e.g. `BC_01`, `BC_02`, …).  
- For each container, we store:
  - `container_id`  
  - GPS coordinates: `lat`, `lon`, (optional `alt`)  
  - Optional **map-frame pose** if precomputed: `x`, `y`, `yaw` in `map` frame  
  - Optional **loiter/hold-point pose** in front of the container (`x_loiter`, `y_loiter`, `yaw_loiter`)

**Intended representation (config file, not yet implemented):**

- A YAML or JSON file in the navigation package, e.g.  
  - `qwacr_navigation/config/battery_containers.yaml`  
  or  
  - `qwacr_navigation/config/battery_containers.json`
- The **mission node** reads this file at startup and provides a lookup:  
  `container_id → (map pose and/or GPS pose)`.

**Map-frame vs GPS representation:**

- **Preferred at runtime:** Use **map-frame poses** (`geometry_msgs/PoseStamped` in `map`) when available for fastest, deterministic goal placement.  
- When only GPS is available:
  - The existing GPS→ENU node converts GPS to local ENU (`gps/enu_odom`).  
  - A simple converter node (or mission node utility) can map ENU coordinates into `map` frame using the current transform tree.

### No-Go Zones (Geofencing)

Certain regions of the yard are **explicitly off-limits** for the robot (safety, regulatory, or operational constraints). These are represented as **no-go zones** on the same global map used for containers.

**Representation:**

- Each no-go zone is a **2D polygon** in `map` frame:
  - `zone_id`
  - `type` (e.g. `hard_keepout`, `soft_prefer_avoid`)
  - `vertices`: ordered list of `{x, y}` points in `map` frame defining the polygon.
- These can be stored alongside containers in the same config file or in a companion file, e.g.:
  - `qwacr_navigation/config/battery_containers_and_zones.yaml`  
  - or `qwacr_navigation/config/no_go_zones.yaml`.

**Enforcement:**

- **Global/Local Planning (Nav2):**
  - No-go zones are projected into the **global and local costmaps** as **lethal obstacles** using either:
    - A dedicated keepout layer plugin (e.g. a polygon-based `keepout_layer`), or
    - A preprocessed static map where no-go regions are encoded as **occupied** cells.
  - This ensures that **global planners** (NavFn or future planners) and **local controllers** (DWB) will **not plan paths through these regions**.

- **Mission Manager:**
  - Validates that:
    - Selected **container loiter points** are not inside hard no-go polygons.
    - **Base station** pose is not in a no-go zone.
    - Any operator-provided GPS targets received over LoRa are checked against no-go polygons before being accepted.
  - If a requested goal is inside a hard no-go zone, the mission manager:
    - Rejects the goal.
    - Reports an error/status over LoRa.

**Behavior Under Teleop:**

- Teleop can physically command the robot toward a no-go zone. Policy options:
  - **Strict:** controllers enforce keepout even under teleop by leaving costmaps active, preventing commands that would drive into lethal cells.
  - **Relaxed (not recommended for safety-critical areas):** mission manager may allow teleop to override costmaps within limits.
- Final policy choice will be recorded here once decided; default assumption for now is **strict geofencing even under teleop** for hard keepout zones.

---

## 🔋 Power & Activation Modes

The robot has distinct **power / system modes** to minimize wear and power draw:

1. **Power-Idle / LoRa-Only Mode (Default Rest State)**
   - Location: Near base station (dock or parking zone).  
   - Running:
     - Core OS + essential services.  
     - LoRa radio + `lora_bridge` node (or equivalent) for receiving:
       - Mission requests (container ID / GPS).  
       - Teleop commands.  
       - Return-to-base commands.  
   - Not running:
     - Aurora SLAM SDK, Nav2 stack, EKF nodes.  
     - LiDAR, camera drivers, heavy CPU nodes.  
   - Goal: Very low CPU + sensor power draw, but always listening on LoRa.

2. **Mission-Active Mode (Autonomous or Teleop)**
   - Triggered by **LoRa mission request** (battery container alarm) or explicit wake command.  
   - System startup sequence (conceptual):
     1. Bring up sensors (Aurora, LiDAR, GPS, IMU).  
     2. Start localization (dual EKF).  
     3. Start Nav2 stack + BT navigator (using `qwacr_outdoor_nav.xml`).  
     4. Start mission manager node.  
     5. Start video streaming pipeline.  
   - Robot transitions from **IDLE** → **AUTO_EN_ROUTE** or **TELEOP** depending on operator commands.

3. **Post-Mission / Return-to-Idle Mode**
   - After returning to base and being commanded to stand down:  
     - Mission manager commands Nav2 to stop.  
     - High-power nodes (Nav2, Aurora, LiDAR, camera, EKF) are shut down.  
     - System returns to **LoRa-only idle**.

**Implementation Note:**
- Power-mode orchestration is expected to be done via **systemd services + launch files** on the Raspberry Pi (not handled inside the Nav2 BT).  
- This document defines when transitions should occur; the exact service/launch orchestration is a separate implementation detail.

---

## 🚦 Operational Modes & Mission Timeline

From the perspective of the **mission planning node** and BT logic, the robot moves through the following **high-level modes**:

1. `IDLE` (LoRa-only)
   - Base station position known (map-frame pose or GPS).  
   - Waiting for **mission request**.

2. `AUTO_EN_ROUTE` (Autonomous to Container)
   - Received `container_id` (and/or GPS target) from LoRa.  
   - Mission node resolves `container_id` to a pose in `map` frame.  
   - Uses Nav2 `NavigateToPose` action (with `qwacr_outdoor_nav.xml`) to drive from base to container.

3. `AUTO_ON_SITE` (Loiter / Patrol Near Container)
   - Robot has reached the target loiter pose (or the container pose itself).  
   - Behaviors:
     - v1: hold a fixed pose with Nav2 active (goal tolerance + costmaps maintain position).  
     - v2: follow a small local pattern of waypoints around the container.
   - Continues to stream sensor data and video.

4. `TELEOP` (Operator Override)
   - At any time, LoRa teleop commands can switch robot to **teleop override**.  
   - Autonomy pauses (no active NavigateToPose goals).  
   - Operator drives manually while telemetry and video continue.  
   - When teleop stops, operator can:  
     - Command **resume autonomous mission**, or  
     - Command **return to base**.

5. `AUTO_RETURN` (Autonomous Back to Base)
   - On `return_to_base` command from operator:  
   - Mission node issues `NavigateToPose` to a base-station pose in `map`.  
   - After arrival, mission node transitions to **Post-Mission / Return-to-Idle**.

6. `FAULT / ABORT` (Safety State)
   - If critical errors occur (e.g., localization failure, repeated Nav2 failures), mission node can:
     - Stop the robot.  
     - Signal error over LoRa.  
     - Optionally await operator instructions.

---

## 🧠 Mission Planning Node (Concept)

A dedicated **mission manager node** in the qwacr_navigation package will own mission logic and interact with Nav2 and comms:

### Responsibilities

- Maintain mission **state machine / BT blackboard** (mode: IDLE, AUTO_EN_ROUTE, AUTO_ON_SITE, TELEOP, AUTO_RETURN, FAULT).  
- Subscribe to **LoRa command topics** via `lora_bridge`:
  - Mission request: `container_id` (and/or GPS) + optional priority.  
  - Teleop velocity commands / teleop enable flag.  
  - `return_to_base` command.  
  - Optional `cancel_mission`.
- Look up **container positions** in the battery container database.  
- Start and monitor **Nav2 NavigateToPose** actions for:
  - Container target.  
  - Optional loiter/hold point.  
  - Base station.  
- Decide when to **enter/exit TELEOP override** and when to hand control back to autonomy.  
- Publish **mode/status** topic for monitoring and logging.

### Interactions

- **With Nav2 / BT:**
  - Uses `nav2_msgs/action/NavigateToPose` calls to BT Navigator.  
  - BT (qwacr_outdoor_nav.xml) handles path planning, following, and local recoveries for each individual goal.

- **With GPS / Mapping:**
  - If container/base poses are stored in GPS only, the mission node (or a helper) converts GPS to map frame via:
    - `gps/enu_odom`  
    - TF: `map → odometry/global → base_link`

- **With LoRa:**
  - `lora_bridge` converts radio packets to ROS2 messages and vice versa.  
  - Mission node exposes simple, robust messages designed for low bandwidth and high latency.

---

## 🧭 IMU Integration (SparkFun 9DoF ISM330DHCX + MMC5983MA)

The Aurora sensor previously provided both LiDAR and IMU data. With the move to a weatherproof **SLAMTEC S2** LiDAR (2D only), inertial data will come from a dedicated **SparkFun 9DoF IMU Breakout – ISM330DHCX, MMC5983MA (Qwiic)** mounted on the robot and connected to the Raspberry Pi.

### Hardware & OS Integration

- Interface: **I2C via Qwiic** directly to the Raspberry Pi.  
- Steps (conceptual):
  - Enable I2C on the Pi (raspi-config or equivalent).  
  - Verify device presence with `i2cdetect` (confirm ISM330DHCX / MMC5983MA addresses).  
  - Mount the IMU as close as practical to the robot’s **center of rotation**; its frame will be `imu_link` in URDF.

### ROS 2 Driver Strategy

- Preferred approach is to use (or adapt) an **existing C/C++ driver library** for ISM330DHCX + MMC5983MA and wrap it in a small ROS 2 node running on the Pi.  
- The node will publish at least:
  - `sensor_msgs/Imu` on a topic such as `/imu/data` (orientation, angular_velocity, linear_acceleration).  
  - Optionally `sensor_msgs/MagneticField` on `/imu/mag` (for heading sanity checks or future use).
- The IMU node will also publish a static or parameterized **frame_id** (e.g. `imu_link`). A static transform `base_link → imu_link` will be defined in URDF or via a static_transform_publisher.

### Localization (robot_localization EKF) Update

Both the **local** and **global** EKF instances will fuse this IMU:

- **Inputs to EKF (post-Aurora):**
  - Wheel odometry (from `diff_cont/odom`).  
  - RTK GPS ENU odometry (position + velocity + heading).  
  - IMU (SparkFun 9DoF) on `/imu/data`.  
  - S2 LiDAR is used **only for obstacle costmaps**, not as a direct odometry source.

- EKF configuration changes (high level):
  - Add `imu0` in EKF params with:
    - `imu0: /imu/data`  
    - `imu0_config`: enable angular velocity and linear acceleration; optionally orientation if magnetometer/initial alignment is trusted.  
    - `imu0_remove_gravitational_acceleration: true` (if driver publishes raw accel including gravity).  
    - Reasonable IMU covariance values (tuned during testing).
  - Apply the same IMU configuration to both **local** and **global** EKF nodes.

This restores a robust **GPS + wheel odom + IMU** fusion similar to the previous Aurora-based setup, with the S2 LiDAR focused on high-quality obstacle detection for Nav2 costmaps.

---

## 🌲 Behavior Tree Design (High-Level)

There will effectively be **two layers of behavior logic**:

1. **Navigation BT (Low-Level, Already Implemented)**
   - File: `qwacr_ws/src/qwacr_navigation/config/qwacr_outdoor_nav.xml`  
   - Used by Nav2 `bt_navigator` for each `NavigateToPose` goal.  
   - Responsibilities:
     - Compute and follow path to goal.  
     - Run limited recoveries (clear costmaps, spin, wait) on failure.  
     - Decide success/failure for a single navigation task.

2. **Mission-Level Behavior (High-Level, To Be Implemented)**

   This can be implemented either as:
   - A **state machine in Python** (simpler initially), or  
   - A custom **mission BT** (with BehaviorTree.CPP or py_trees) coordinated by the mission node.

   Conceptual BT outline for mission-level behavior:

   - Root: `ReactiveFallback`
     - Branch 1 (higher priority): `IsTeleopActive?` → `RunTeleopMode`
     - Branch 2: `MissionSequence`
       - `WaitForMissionRequest` (blocking condition over LoRa)  
       - `WakeSystemIfNeeded` (ensure sensors/Nav2/EKF are running)  
       - `SetGoalFromContainerID` (writes `{goal}` on blackboard)  
       - `NavigateToPose(goal)` (Nav2 action client call)  
       - `OnSiteBehavior`:
         - v1: `LoiterAtFrontOfContainer`  
         - v2: `PatrolAroundContainer`  
       - `WaitForReturnToBaseCommand`  
       - `SetGoalToBase` (writes `{goal}`)  
       - `NavigateToPose(goal)` back to base

   Teleop override always preempts lower-priority autonomous branches.

---

## 📡 Telemetry & Streaming

### LoRa Telemetry

While the mission is active (AUTO_EN_ROUTE, AUTO_ON_SITE, TELEOP, AUTO_RETURN), the robot streams:

- Fire / thermal sensor measurements.  
- Basic health and status:
  - Current mode.  
  - GPS fix quality / Aurora health summary.  
  - Battery state of charge.  
  - High-level Nav2 status (en route, on-site, returning, fault).

Telemetry rates must be conservative (LoRa bandwidth-limited).  Aggregation or down-sampling may be needed.

### HaLow Video

- Video stream (H.264 over GStreamer) is active during **mission-active** modes.  
- May be throttled or disabled in IDLE / LoRa-only mode to save power.

---

## ✅ Scope for Next Iterations

This document will serve as the **reference** for designing and iterating on:

1. The **mission manager node** in qwacr_navigation (topics, actions, state machine).  
2. The **battery container database format** and lookup logic.  
3. The **mission-level behavior structure** (state machine vs BT) and its interaction with Nav2 BT.  
4. The **power mode orchestration** (how and when to launch/stop subsystems).

Planned next concrete steps (implementation-side):

- Define a `battery_containers.yaml` format and add a first version under `qwacr_navigation/config/`.  
- Scaffold a `mission_manager.py` node in `qwacr_navigation/qwacr_navigation/` with:
  - Mode state machine.  
  - Minimal LoRa command interfaces (stub topics).  
  - Calls into `NavigateToPose` for container and base goals.
- Refine this document as those interfaces and behaviors become concrete.
