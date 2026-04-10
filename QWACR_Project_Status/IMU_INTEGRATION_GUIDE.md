# QWACR IMU Integration Guide – SparkFun 9DoF (ISM330DHCX)

**Robot:** QWACR Autonomous Ground Robot  
**IMU:** SparkFun 9DoF IMU Breakout – ISM330DHCX, MMC5983MA (Qwiic)  
**Scope:** Wiring, ROS 2 node, and EKF integration on Raspberry Pi

---

## 1. Hardware Wiring (Raspberry Pi)

- Connect the IMU to the Raspberry Pi **I²C bus** (3.3V logic only):
  - VCC → Pi 3.3V (physical pin 1 or 17)
  - GND → Pi GND
  - SDA → Pi SDA1 (physical pin 3)
  - SCL → Pi SCL1 (physical pin 5)
- Enable I²C on the Pi (raspi-config or equivalent) and verify with `i2cdetect -y 1`.
- Note: **Multiple I²C devices** (e.g., IMU + fire sensors) can share SDA/SCL on the same bus as long as each has a **unique address** and is 3.3V‑safe. No extra I²C pins are required; add a second bus or an I²C mux only if you run into address conflicts or bus loading issues.

---

## 2. ROS 2 IMU Node (qwacr_imu)

Package location: `qwacr_ws/src/qwacr_imu`

- Node script: `qwacr_imu/sparkfun_imu_node.py`
- Dependencies:
  - ROS 2: `rclpy`, `sensor_msgs`  
  - Python: `sparkfun-qwiic-ism330dhcx` (installed via pip on the Pi)

**Node behavior:**

- Creates an I²C connection using the SparkFun Qwiic Python driver.  
- Reads accelerometer + gyroscope data at a configurable rate.  
- Publishes `sensor_msgs/Imu` on topic `imu/data`:
  - `orientation` left unset (`orientation_covariance[0] = -1.0`).  
  - `angular_velocity` from gyro (deg/s → rad/s).  
  - `linear_acceleration` from accel (g → m/s²).  
- Parameters:
  - `frame_id` (default: `imu_link`).  
  - `publish_rate_hz` (default: `100.0`).

**Running the node (after building):**

```bash
cd ~/qwacr_ws
colcon build --packages-select qwacr_imu
source install/setup.bash
ros2 run qwacr_imu sparkfun_imu_node
```

---

## 3. EKF Integration (robot_localization)

EKF config file: `qwacr_ws/src/robot_localization/params/ekf_local_global.yaml`

Both EKF instances now fuse the new IMU topic `/imu/data`:

- **Local EKF (ekf_local_odom):**
  - Inputs: `odometry/wheel` (remapped to `diff_cont/odom` in full_system), optional `odom` (Aurora, if present), and `/imu/data`.
  - IMU settings (high level):
    - `imu0: /imu/data`  
    - `imu0_config`: angular velocities (`vroll`, `vpitch`, `vyaw`) and linear accelerations (`ax`, `ay`, `az`) enabled; orientation disabled.  
    - `imu0_remove_gravitational_acceleration: true` (driver publishes raw accel including gravity).

- **Global EKF (ekf_global_map):**
  - Inputs: `odometry/wheel`, optional `odom`, `/gps/enu_odom` (RTK GPS ENU odometry), and `/imu/data`.
  - IMU settings mirror the local EKF.

This yields a fused pose and velocity that combine **wheel odometry + IMU** for short‑term motion with **RTK GPS** for absolute positioning.

---

## 4. I²C Bus Sharing with Fire Sensors

- The Raspberry Pi’s primary I²C bus (SDA1/SCL1) can host **multiple I²C devices** in parallel (IMU, fire sensors, etc.).
- Requirements:
  - Each device must have a **unique 7‑bit I²C address**.  
  - All devices must be compatible with **3.3V** logic (use level shifters or dedicated regulators if not).  
  - Total bus capacitance and cable length must remain within I²C spec; keep wiring short and tidy.
- If you later encounter address conflicts or need to isolate noisy devices, you can:
  - Add a dedicated I²C multiplexer (e.g., TCA9548A / Qwiic Mux).  
  - Or enable/use a second I²C bus on the Pi and move some sensors there.

For now, plan on using **one I²C bus** for the SparkFun IMU and fire sensors, provided their addresses don’t clash and they’re all 3.3V‑compatible.
