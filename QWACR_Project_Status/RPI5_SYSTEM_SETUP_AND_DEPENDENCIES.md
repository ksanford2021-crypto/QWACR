# Raspberry Pi 5 System Setup (Robot & Base Station)

Date: 2026-03-22

This guide standardizes how to bring up both Raspberry Pi 5 boards (8 GB & 16 GB)
so either can be used as the **robot Pi** or the **base-station Pi**.

We install **Ubuntu Server 24.04 64-bit**, **ROS 2 Jazzy from apt**, and all
common dependencies needed for:

- `qwacr_ws` (including `qwacr_comms`, `qwacr_operator_gui`, `qwacr_imu`,
  fire-sensor packages, etc.).
- `mega_diff_drive_control` and `qwacr_build` (ros2_control + controllers).
- HaLow/LoRa tooling and GStreamer video.

---

## 1. Flash OS Image

Do this for each Pi 5.

1. Open **Raspberry Pi Imager** on your PC.
2. **Choose OS**:
   - `Other general purpose OS` → `Ubuntu` →
     - **Ubuntu Server 24.04 LTS (64-bit) for Raspberry Pi 5**.
3. **Choose Storage**: select the SD card.
4. Click the gear icon (advanced options) and set:
   - Hostname: e.g. `qwacrpi-a` and `qwacrpi-b`.
   - Enable SSH: `Use password authentication`.
   - Username: e.g. `kyle`.
   - Password: strong password.
   - Wi‑Fi: configure only if needed; Ethernet is preferred for setup.
5. Write the image and safely eject the SD card.
6. Insert SD into Pi, connect Ethernet (to the router) and power.

---

## 2. First Login and Basic System Setup

On your dev machine (WSL or Linux), once the Pi has booted:

1. Find the Pi’s IP (one of):
   - Use `sudo nmap -sn 10.0.0.0/24` and look for a new host.
   - Or use the router’s device list.
2. SSH in (replace `<ip>` and `<user>`):

   ```bash
   ssh <user>@<ip>
   ```

3. Update and reboot:

   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo reboot
   ```

4. After reboot, SSH in again.

Optional but recommended:

- Set timezone:

  ```bash
  sudo timedatectl set-timezone America/New_York
  ```

---

## 3. Install Core Build Tools & Libraries

On **each** Pi:

```bash
sudo apt update
sudo apt install -y \
  build-essential git curl wget \
  python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  cmake ninja-build \
  libssl-dev libffi-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  v4l-utils
```

Initialize rosdep (once per Pi):

```bash
sudo rosdep init || true   # ignore error if already initialized
rosdep update
```

---

## 4. Install ROS 2 Jazzy via apt (not from source)

We use the official **apt** packages for Jazzy—no source build.

Follow the standard steps (summarized here).

1. Set locale (usually already UTF‑8 on Ubuntu 24.04, but safe to run):

   ```bash
   sudo apt update
   sudo apt install -y locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. Add ROS 2 apt repository:

   ```bash
   sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   sudo apt install -y curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
     sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Install ROS 2 Jazzy + common stacks:

   ```bash
   sudo apt update
   sudo apt install -y \
     ros-jazzy-ros-base \
     ros-jazzy-desktop \
     ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
     ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
     ros-jazzy-controller-manager \
     ros-jazzy-joint-state-broadcaster \
     ros-jazzy-diff-drive-controller
   ```

4. Add ROS to shell startup:

   ```bash
   echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
   source /opt/ros/jazzy/setup.bash
   ```

This gives both Pis the same ROS 2 Jazzy installation from apt.

---

## 5. Bring `qwacr_ws` Onto Each Pi

You can either clone from GitHub or copy from your dev machine.

### 5.1 Clone from GitHub (recommended long term)

On each Pi:

```bash
cd ~
mkdir -p qwacr_ws/src
cd qwacr_ws/src
# Example:
# git clone git@github.com:<YOUR_ORG>/qwacr_ws_sources.git
# or clone individual repos as you currently have them structured.
```

Alternatively, for now you can copy the existing workspace:

### 5.2 Copy from dev machine

From your dev machine:

```bash
scp -r ~/qwacr_ws <user>@<pi-ip>:/home/<user>/
```

---

## 6. Install ROS Dependencies for `qwacr_ws`

On each Pi:

```bash
cd ~/qwacr_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

This will pull in any remaining package dependencies declared in your
package.xml files (for example, sensor_msgs, nav_msgs, tf2, etc.).

---

## 7. Build Workspace

On each Pi:

```bash
cd ~/qwacr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Add workspace to shell startup
echo 'source ~/qwacr_ws/install/setup.bash' >> ~/.bashrc
source ~/qwacr_ws/install/setup.bash
```

---

## 8. Extra Dependencies for `mega_diff_drive_control` and `qwacr_build`

Most controller-related dependencies are already covered by the apt
install in Section 4, but ensure the following are present on **robot**
Pis (and optionally on the base-station Pi for debugging):

```bash
sudo apt install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-diff-drive-controller
```

After installing, rebuild if needed:

```bash
cd ~/qwacr_ws
colcon build --packages-select mega_diff_drive_control qwacr_build
```

(Adjust package names to match the exact ones in your `src/` tree.)

---

## 9. Role-Specific Launch Patterns

Because both Pis share the same OS, ROS 2, and `qwacr_ws`, the "role"
comes down to which launch files you run and how the hardware is wired.

### 9.1 Robot Pi

Typical commands (examples):

```bash
source /opt/ros/jazzy/setup.bash
source ~/qwacr_ws/install/setup.bash

# Communications (LoRa + mission manager + command mux)
ros2 launch qwacr_comms robot_comms.launch.py

# Sensor/nav bringup (your existing launch that starts lidar, IMU,
# fire sensors, EKF, Nav2, diff-drive controller, etc.)
ros2 launch <your_robot_package> robot_bringup.launch.py
```

Robot-specific configuration lives in:
- `/boot/firmware/config.txt` (UART overlays, camera, I²C mux).
- Your existing guides:
  - `Fire_Sensor_Integration.md`
  - `IMU_INTEGRATION_GUIDE.md`
  - `PI_ACCESS_POINT_SETUP.md` (for HaLow client on robot).

### 9.2 Base-Station Pi

```bash
source /opt/ros/jazzy/setup.bash
source ~/qwacr_ws/install/setup.bash

# LoRa bridge + operator console
ros2 launch qwacr_comms base_station.launch.py

# Qt operator GUI
ros2 run qwacr_operator_gui operator_gui

# GStreamer HaLow video receiver (separate terminal, per existing guide)
# e.g.:
# gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264 ! \
#   rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

Base-station-specific network configuration (HaLow AP/client, LoRa USB)
should follow:
- `HALOW_VIDEO_STREAMING_GUIDE.md`
- `LORA_HALOW_INTEGRATION_PLAN.md`

---

## 10. Summary

- **ROS 2 Jazzy is installed from apt** (official packages) on both Pis.
- Both boards share the same `~/qwacr_ws` layout and are built with
  `colcon`.
- Extra controller-related dependencies for `mega_diff_drive_control` and
  `qwacr_build` are pulled in via apt and rosdep.
- Choosing **robot vs base-station** becomes a matter of wiring and
  which launch files you run, not different OS images or ROS installs.
