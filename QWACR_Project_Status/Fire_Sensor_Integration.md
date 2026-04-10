# Fire Sensor Integration

Date: 2026-03-12 (updated 2026-03-19)
Target platform: Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy)

## 0. Current status snapshot (March 2026)

- TCA9548A mux on I²C1 is healthy at address 0x70; SEN54 and the
  **front** MLX90640 are online on their respective ports and publishing
  ROS topics.
- DGS2 H₂ and CO modules are wired to dedicated UARTs and are running as
  ROS nodes, publishing calibrated gas concentrations.
- The SparkFun IMU on the same I²C stack is integrated via
  `qwacr_imu` and fused in `robot_localization`.

The rest of this document describes the wiring and configuration that
led to this working state and can be treated as the reference for any
repairs or future payload revisions.

## 1. Purpose

This document describes how to integrate a multi-sensor "fire sensing" payload with a Raspberry Pi 5. The payload combines:

- Particulate/VOC/humidity/temperature: Sensirion SEN54 (SparkFun breakout)
- Thermal IR array: MLX90640 32×24 (SparkFun IR Array Sensor, 110° FOV)
- Combustion gases: SPEC Sensors DGS2 digital gas modules (H₂ and CO variants)
- I²C expansion: TCA9548A 1‑to‑8 I²C multiplexer (Adafruit #2717)

The goal is a single Pi 5 that can:

- Detect smoke/particulates, VOCs, relative humidity, and temperature
- Detect hot spots / flames via thermal imaging
- Detect combustion gases (H₂ and CO)
- Expose these data streams to higher-level software (e.g., ROS 2 nodes)

---

## 2. Bill of Materials (electrical interfaces only)

**Environmental + particulate:**
- Sensirion SEN54 module on SparkFun breakout
  - Power: 4.5–5.5 V (5 V nominal)
  - Interface: I²C at 3.3 V logic
  - Typical current: up to ~110 mA

**Thermal IR:**
- MLX90640 32×24 thermal IR array (SparkFun, 110°×75° FOV)
  - Power: 3.3 V
  - Interface: I²C (3.3 V)
  - Current: < 25 mA

**Gas sensors:**
- SPEC Sensors DGS2 digital gas modules (H₂ and CO variants)
  - Power: per datasheet (typically low-current 3.3–5 V range; confirm before wiring)
  - Interface: digital UART (logic-level serial) with calibrated, temperature-compensated output
  - Each module appears as an independent UART device to the Pi

**I²C hub / multiplexer:**
- TCA9548A 1‑to‑8 I²C multiplexer (Adafruit #2717)
  - Power: 3.3 or 5 V; logic is I²C 3.3/5 V tolerant
  - Interface: I²C, base address 0x70 (configurable 0x70–0x77)

**Raspberry Pi 5:**
- Provides:
  - Primary I²C bus on GPIO 2 (SDA1) / GPIO 3 (SCL1)
  - Multiple UART-capable GPIOs via device-tree overlays (PL011 / mini UARTs)
  - 3.3 V and 5 V rails on 40‑pin header

---

## 3. High-level bus topology

### 3.1 I²C topology

- Pi 5 I²C1 (SDA1/SCL1) → TCA9548A multiplexer (address 0x70)
- From TCA9548A ports (as currently wired on the robot):
  - Port 0 → SEN54 (5 V power, 3.3 V I²C)
  - Port 1 → MLX90640 #1 (front IR array, 3.3 V power, I²C)
  - Port 4 → MLX90640 #2 (right-side IR array, 3.3 V power, I²C; code may still refer to this as "left" but the physical mounting is on the right)
- Remaining TCA9548A ports are free for future fire-related sensors.

This keeps the MLX90640 and SEN54 logically separated (helps if address collisions occur or if we want to isolate bus timing) while still using a single Pi I²C bus.

### 3.2 UART topology (gas sensors)

- DGS2‑H₂ module → dedicated Pi UART (e.g., `/dev/ttyAMA2`)
- DGS2‑CO module → dedicated Pi UART (e.g., `/dev/ttyAMA3`)

Each DGS2 module gets its own TX/RX pair configured via Pi 5 device-tree overlays, similar to how `/dev/ttyAMA0` was enabled for the RPLIDAR.

---

## 4. Raspberry Pi 5 pin assignments (proposed)

### 4.1 Power rails

- 5 V rail:
  - SEN54 VCC → Pi 5 V pin (e.g., physical pin 2 or 4)
  - DGS2 modules VCC: use 3.3 V or 5 V based on datasheet (plan to use 3.3 V if supported for UART logic safety; otherwise 5 V plus a level shifter to 3.3 V for Pi RX)
- 3.3 V rail:
  - TCA9548A VCC → Pi 3.3 V pin (e.g., physical pin 1)
  - MLX90640 VDD → Pi 3.3 V
  - If SEN54 I²C lines are 3.3 V tolerant (per SparkFun note, they are): connect directly to TCA9548A channel

All grounds for sensors and the Pi must be common.

### 4.2 I²C pins

- Pi I²C1:
  - SDA1 → GPIO 2 (physical pin 3)
  - SCL1 → GPIO 3 (physical pin 5)

Connect these to the **TCA9548A**:

- TCA9548A SDA → Pi SDA1 (GPIO 2)
- TCA9548A SCL → Pi SCL1 (GPIO 3)

On the TCA9548A side ports:

- Port 0 (for SEN54):
  - SDA0 → SEN54 SDA
  - SCL0 → SEN54 SCL
- Port 1 (for MLX90640):
  - SDA1 → MLX90640 SDA
  - SCL1 → MLX90640 SCL

Pull-ups:
- The Pi and breakouts typically include I²C pull-ups. Ensure **only one set** of SDA/SCL pull-ups is present per bus segment to avoid overly strong pull currents.

### 4.3 UART pins (additional RX/TX via overlays)

We assume `/dev/ttyAMA0` is already in use for the RPLIDAR as in earlier work, mapped to GPIO14/15. For the DGS2 modules, we will configure additional UARTs on spare GPIO pins.

Concrete pin plan:

- DGS2‑H₂ UART (UART2):
  - Pi TX2 (GPIO4, physical pin 7) → DGS2‑H₂ RX
  - Pi RX2 (GPIO5, physical pin 29) → DGS2‑H₂ TX
- DGS2‑CO UART (UART3):
  - Pi TX3 (GPIO12, physical pin 32) → DGS2‑CO RX
  - Pi RX3 (GPIO13, physical pin 33) → DGS2‑CO TX

Ensure that the DGS2 TX logic level is **3.3 V**. If the module uses 5 V UART levels, introduce a level shifter for the Pi RX line.

---

## 5. Pi 5 device-tree configuration (I²C + extra UARTs)

### 5.1 I²C1 (default)

On most Pi 5 images, I²C1 on GPIO 2/3 is enabled via `raspi-config` or by ensuring:

```text
i2c_arm=on
```

(or equivalent) is set in `/boot/firmware/config.txt`.

Verify from Linux:

```bash
sudo apt-get install -y i2c-tools
sudo i2cdetect -y 1
```

You should see the TCA9548A address (default 0x70) appear on bus 1.

### 5.2 UART overlays (concrete config)

In `/boot/firmware/config.txt`, in addition to the existing `uart0` overlay used for the lidar, add overlays for extra UARTs using the pin plan from §4.3.

Lines to add (keep the lidar line if it already exists):

```text
# Lidar on GPIO14/15 (already configured)
dtoverlay=uart0,txd0_pin=14,rxd0_pin=15

# DGS2-H2 on UART2 (GPIO4/5)
dtoverlay=uart2,txd2_pin=4,rxd2_pin=5

# DGS2-CO on UART3 (GPIO12/13)
dtoverlay=uart3,txd3_pin=12,rxd3_pin=13
```

After editing, reboot:

```bash
sudo reboot
```

Then confirm devices exist:

```bash
ls -l /dev/ttyAMA*
```

We expect to see additional `ttyAMA*` devices (e.g., `/dev/ttyAMA2`, `/dev/ttyAMA3`) to assign to the DGS2 sensors.

---

## 6. Sensor bring-up and test procedures

### 6.1 I²C multiplexer and sensors

1. **Scan base bus** (Pi ↔ TCA9548A):
   ```bash
   sudo i2cdetect -y 1
   ```
   Confirm 0x70 (TCA9548A) is present.

2. **Select port 0 and scan (SEN54):**
   - Write to TCA9548A to enable port 0 (register write 0x01):
     ```bash
     sudo i2cset -y 1 0x70 0x01
     sudo i2cdetect -y 1
     ```
   - Confirm the SEN54 address appears (see Sensirion datasheet for default).

3. **Select port 1 and scan (MLX90640):**
   - Enable port 1 (register write 0x02):
     ```bash
     sudo i2cset -y 1 0x70 0x02
     sudo i2cdetect -y 1
     ```
   - Confirm the MLX90640 address (commonly 0x33) appears.

4. **Software libraries:**
   - SEN54: Sensirion provides an I²C library (Arduino/C). On Pi/ROS we can either:
     - Use a Python port of the SEN5x driver, or
     - Implement the I²C commands directly (per datasheet) in a ROS node.
   - MLX90640: Use an existing MLX90640 driver (Python/C++) configured to talk through the TCA9548A (i.e., select channel then read frame).

### 6.2 DGS2 gas sensors (UART)

1. Confirm the relevant `/dev/ttyAMA*` devices exist after overlays.
2. For each DGS2 module:
   - Connect TX/RX as described in §4.3.
   - Consult the DGS2 datasheet for serial settings (baud rate, parity, framing). Configure a simple test using `minicom` or a small Python script.
   - Verify that periodic gas concentration data are received and parsable.

3. Once basic serial I/O is working, wrap each sensor in its own ROS 2 node that:
   - Opens the corresponding serial port.
   - Parses the DGS2 output frames.
   - Publishes gas concentration on dedicated topics (e.g., `/gas/h2`, `/gas/co`).

---

## 7. ROS 2 integration sketch (optional)

At a high level, the fire sensor stack can be represented as:

- Node `sen54_node`:
  - Talks over I²C via TCA9548A port 0
  - Publishes:
    - `/environment/pm` (PM1.0/2.5/4.0/10)
    - `/environment/voc_index`
    - `/environment/humidity`
    - `/environment/temperature`

- Node `mlx90640_node`:
  - Talks over I²C via TCA9548A port 1
  - Publishes thermal image data (e.g., `/thermal/image_raw` or `/thermal/heatmap`)

- Node `dgs2_h2_node`:
  - Uses UART `/dev/ttyAMA2`
  - Publishes `/gas/h2`

- Node `dgs2_co_node`:
  - Uses UART `/dev/ttyAMA3`
  - Publishes `/gas/co`

- Higher-level node(s):
  - Subscribe to all above topics to implement fire/smoke/overheat detection logic and to trigger alarms/logging as needed.

---

## 8. Safety and design considerations

- Ensure proper 5 V and 3.3 V power budgeting on the Pi 5: SEN54 can draw up to ~110 mA; combined with other peripherals, confirm the 5 V rail remains within spec.
- Maintain common ground across all sensors and the Pi.
- Verify all UART signal levels are 3.3 V-compatible before direct connection to the Pi; use level shifters if any module uses 5 V logic.
- Route the MLX90640 field of view clear of obstructions and reflective surfaces to avoid false positives.
- Locate gas sensors so that air exchange is representative of the protected volume (not trapped in stagnant pockets).

This document captures a first-pass integration plan for the fire sensing payload on the Raspberry Pi 5. Pin assignments and overlay names should be finalized against the current Raspberry Pi 5 hardware and firmware documentation before building wiring harnesses.

---

## 9. On-robot command checklist (run later on the Pi)

This section collects the exact commands to run on the Pi once network access is available. You can finish wiring now and execute these when you are back at the robot.

### 9.1 Configure UART overlays and reboot

1. Edit the Pi boot config:

  ```bash
  sudo nano /boot/firmware/config.txt
  ```

2. Ensure the following lines exist in the file (lidar + two DGS2 UARTs):

  ```text
  dtoverlay=uart0,txd0_pin=14,rxd0_pin=15
  dtoverlay=uart2,txd2_pin=4,rxd2_pin=5
  dtoverlay=uart3,txd3_pin=12,rxd3_pin=13
  ```

3. Save and exit, then reboot:

  ```bash
  sudo reboot
  ```

4. After reboot, confirm UART devices:

  ```bash
  ls -l /dev/ttyAMA*
  dmesg | grep -i ttyAMA
  ```

  You should see `/dev/ttyAMA0` (lidar) plus two new `ttyAMA*` devices for the DGS2 sensors.

### 9.2 Install I²C tools and verify the I²C hub

1. Install the I²C tools package (once the Pi has internet access):

  ```bash
  sudo apt-get update
  sudo apt-get install -y i2c-tools
  ```

2. Scan the base I²C bus to confirm the TCA9548A is visible (address 0x70 expected on bus 1):

  ```bash
  sudo i2cdetect -y 1
  ```

3. Select **port 0** (SEN54) and scan:

  ```bash
  sudo i2cset -y 1 0x70 0x01
  sudo i2cdetect -y 1
  ```

4. Select **port 1** (MLX90640 #1) and scan:

  ```bash
  sudo i2cset -y 1 0x70 0x02
  sudo i2cdetect -y 1
  ```

5. Select **port 2** (MLX90640 #2) and scan:

  ```bash
  sudo i2cset -y 1 0x70 0x04
  sudo i2cdetect -y 1
  ```

Each scan should show the corresponding sensor address on the bus (see the SEN54 and MLX90640 datasheets for default addresses).
