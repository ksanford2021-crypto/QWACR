#!/bin/bash
set -e

echo "=== ROS 2 Jazzy Setup for Raspberry Pi 5 ==="

# Update system packages
echo "[1/5] Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  python3-pip \
  python3-dev \
  libssl-dev \
  libffi-dev \
  libncursesw5-dev \
  libsqlite3-dev \
  tk-dev \
  libgdbm-dev \
  libc6-dev \
  libbz2-dev

# Add user to dialout group for serial port access
echo "[2/5] Configuring serial port access..."
sudo usermod -a -G dialout $USER
echo "Note: Log out and back in for dialout group to take effect"

# Add ROS 2 Jazzy repository and install
echo "[3/5] Installing ROS 2 Jazzy..."
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-core \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-py \
  python3-colcon-common-extensions

# Install sensor-specific packages
echo "[4/5] Installing sensor packages..."
sudo apt-get install -y \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  python3-serial

# Create workspace directory
echo "[5/5] Setting up ROS 2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 and add to bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

echo ""
echo "=== Setup Complete ==="
echo "Next steps:"
echo "  1. Log out and back in to activate dialout group"
echo "  2. Clone your QWACR repo:"
echo "     cd ~/ros2_ws/src && git clone https://github.com/ksanford2021-crypto/QWACR.git"
echo "  3. Build packages:"
echo "     cd ~/ros2_ws && colcon build --packages-select qwacr_gps qwacr_lidar"
echo "  4. Source the workspace:"
echo "     source ~/ros2_ws/install/setup.bash"
echo "  5. Connect sensors and test:"
echo "     ros2 launch qwacr_gps gps.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200"
