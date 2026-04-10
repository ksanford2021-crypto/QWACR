# Raspberry Pi Access Point Configuration Guide

**Created:** January 29, 2026  
**Purpose:** Configure Raspberry Pi to work both as WiFi Access Point (on campus) and WiFi client (at home)

---

## Overview

This setup allows the Pi to operate in two modes:
1. **WiFi Client Mode (Home):** Normal WiFi connection to home network
2. **Access Point Mode (Campus):** Pi broadcasts its own WiFi network for reliable SSH access

---

## Access Point Details

**Network Name (SSID):** `QWACR-Pi`  
**Password:** `qwacr2026`  
**Pi IP Address:** `192.168.4.1`  
**DHCP Range:** `192.168.4.2` - `192.168.4.20`  
**SSH Command:** `ssh qwacr@192.168.4.1`

---

## Switch Scripts Location

Located in: `/home/qwacr/`

### Switch to Access Point Mode (for campus)
```bash
/home/qwacr/switch_to_ap.sh
```

### Switch to WiFi Client Mode (for home)
```bash
/home/qwacr/switch_to_wifi.sh
```

Both scripts automatically reboot the Pi after switching modes.

---

## Complete Setup Instructions (for new Pi)

### Prerequisites
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install hostapd dnsmasq -y
```

### Step 1: Stop Services
```bash
sudo systemctl stop hostapd
sudo systemctl stop dnsmasq
```

### Step 2: Configure Static IP for Access Point
```bash
# Backup existing config
sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.backup

# Add static IP configuration
sudo tee -a /etc/dhcpcd.conf << EOF

# Static IP for Access Point
interface wlan0
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant
EOF

# Save as AP config
sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.ap
```

### Step 3: Configure DHCP Server (dnsmasq)
```bash
# Backup original
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig

# Create new config
sudo tee /etc/dnsmasq.conf << EOF
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
domain=wlan
address=/gw.wlan/192.168.4.1
EOF
```

### Step 4: Configure hostapd (Access Point Software)
```bash
sudo tee /etc/hostapd/hostapd.conf << EOF
interface=wlan0
driver=nl80211
ssid=QWACR-Pi
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=qwacr2026
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF
```

### Step 5: Point System to hostapd Config
```bash
sudo tee /etc/default/hostapd << EOF
DAEMON_CONF="/etc/hostapd/hostapd.conf"
EOF
```

### Step 6: Enable IP Forwarding
```bash
sudo sed -i 's/#net.ipv4.ip_forward=1/net.ipv4.ip_forward=1/' /etc/sysctl.conf
sudo sysctl -p
```

### Step 7: Enable Services on Boot
```bash
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
```

### Step 8: Create Switch Scripts

**Create switch_to_ap.sh:**
```bash
sudo tee /home/qwacr/switch_to_ap.sh << 'EOF'
#!/bin/bash
echo "Switching to Access Point mode..."
sudo cp /etc/dhcpcd.conf.ap /etc/dhcpcd.conf
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
echo "AP mode configured. Rebooting in 3 seconds..."
sleep 3
sudo reboot
EOF

sudo chmod +x /home/qwacr/switch_to_ap.sh
```

**Create switch_to_wifi.sh:**
```bash
sudo tee /home/qwacr/switch_to_wifi.sh << 'EOF'
#!/bin/bash
echo "Switching to WiFi client mode..."
sudo cp /etc/dhcpcd.conf.backup /etc/dhcpcd.conf
sudo systemctl stop hostapd
sudo systemctl stop dnsmasq
sudo systemctl disable hostapd
sudo systemctl disable dnsmasq
echo "WiFi client mode configured. Rebooting in 3 seconds..."
sleep 3
sudo reboot
EOF

sudo chmod +x /home/qwacr/switch_to_wifi.sh
```

### Step 9: Initial Boot Mode
```bash
# If at home, switch to WiFi client mode:
/home/qwacr/switch_to_wifi.sh

# If on campus, switch to AP mode:
/home/qwacr/switch_to_ap.sh
```

---

## Usage Guide

### At Home
1. Ensure Pi is in WiFi client mode
2. Pi connects to home WiFi automatically
3. SSH using: `ssh qwacr@raspberrypi.local` or router-assigned IP

### On Campus
1. Before bringing Pi to campus, run: `/home/qwacr/switch_to_ap.sh`
2. Pi will reboot and broadcast "QWACR-Pi" network
3. On laptop:
   - Connect to WiFi network: `QWACR-Pi`
   - Enter password: `qwacr2026`
   - Wait for connection
4. SSH to Pi: `ssh qwacr@192.168.4.1`

### Switching Between Modes
- **To AP mode:** `/home/qwacr/switch_to_ap.sh`
- **To WiFi client mode:** `/home/qwacr/switch_to_wifi.sh`
- Both scripts reboot automatically

---

## Hardware Connections (On Campus)

```
Aurora LiDAR (Ethernet) → Pi Ethernet Port (eth0)
                          Aurora IP: 192.168.11.1
                          Pi eth0 IP: 192.168.11.x

Pi WiFi (wlan0) → Creates "QWACR-Pi" Access Point
                  Pi wlan0 IP: 192.168.4.1

Laptop WiFi → Connects to "QWACR-Pi"
              Laptop gets IP: 192.168.4.x

Laptop USB-C Ethernet → School network (for internet when needed)
```

---

## Internet Sharing (Optional)

### From Laptop to Pi (when needed for updates)

**On Windows:**
1. Settings → Network & Internet → Mobile hotspot or Internet sharing
2. Share from: Ethernet (USB-C adapter)
3. Share over: WiFi (already connected to QWACR-Pi)

**On Mac:**
1. System Preferences → Sharing → Internet Sharing
2. Share from: USB Ethernet
3. To computers using: WiFi (already connected to QWACR-Pi)

**On Linux:**
```bash
# Enable forwarding (already done on Pi)
# Use NetworkManager GUI or nmcli to share connection
```

---

## Troubleshooting

### Can't See QWACR-Pi Network
```bash
# SSH via Ethernet if possible, or connect monitor/keyboard
sudo systemctl status hostapd
sudo systemctl status dnsmasq

# Restart services
sudo systemctl restart hostapd
sudo systemctl restart dnsmasq
```

### Can't Connect to Network
- Check password: `qwacr2026`
- Check Pi's wlan0 status: `ip addr show wlan0`
- Should show: `192.168.4.1/24`

### SSH Connection Refused
```bash
# Check if Pi is in correct mode
cat /etc/dhcpcd.conf | grep "192.168.4.1"

# If not found, Pi is in WiFi client mode
# Run switch_to_ap.sh
```

### Need to Reset
```bash
# Restore original WiFi config
sudo cp /etc/dhcpcd.conf.backup /etc/dhcpcd.conf
sudo systemctl disable hostapd
sudo systemctl disable dnsmasq
sudo reboot
```

---

## Configuration Files Reference

- **Main config:** `/etc/dhcpcd.conf`
- **Backup (WiFi client):** `/etc/dhcpcd.conf.backup`
- **AP config:** `/etc/dhcpcd.conf.ap`
- **hostapd config:** `/etc/hostapd/hostapd.conf`
- **dnsmasq config:** `/etc/dnsmasq.conf`
- **Switch scripts:** `/home/qwacr/switch_to_*.sh`

---

## Second Pi Setup (Campus Pi)

To configure the second Pi on campus with identical settings:

1. Copy this guide to the second Pi
2. Follow all steps in "Complete Setup Instructions" section
3. Use same SSID and password for consistency, OR
4. Change SSID in Step 4 to differentiate (e.g., `QWACR-Pi-2`)

---

## ROS2 Network Configuration

### Cyclone DDS for Better Network Discovery

**Recommended for all systems, especially WSL2 users:**

The default FastDDS has multicast discovery issues on WSL2 and some networks. Use Cyclone DDS instead:

**On Raspberry Pi:**
```bash
# Install Cyclone DDS
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# Add to .bashrc for automatic use
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

**On Development Computer (including WSL2):**
```bash
# Install Cyclone DDS
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# Add to .bashrc for automatic use
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

### ROS_DOMAIN_ID Configuration

**Both Pi and laptop must use the same domain:**

```bash
# Set domain ID (0-101, default is 0)
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
source ~/.bashrc
```

**To verify topics are visible between machines:**
```bash
# On Pi
ros2 topic list

# On laptop (should see same list)
ros2 topic list
```

### WSL2 Specific Issues

If using Windows Subsystem for Linux (WSL2):
- Multicast is limited by default
- **Solution 1:** Use Cyclone DDS (see above) - **RECOMMENDED**
- **Solution 2:** Configure Windows Firewall to allow WSL network
- **Solution 3:** Use X11 forwarding to run RViz on Pi

**X11 Forwarding from WSL2:**
```bash
# Install X server on Windows (VcXsrv or Xming)
# Connect with X11 forwarding
ssh -X qwacr@192.168.4.1

# Run RViz on Pi, display on Windows
rviz2
```

---

## Aurora LiDAR Ethernet Configuration

The Aurora sensor requires a static IP configuration on the robot Pi's Ethernet interface.

**Aurora sensor default IP:** `192.168.11.1`  
**Robot Pi Ethernet IP (eth0):** `192.168.11.100`  
**Base-station Pi Ethernet IP (eth0, direct laptop link):** `192.168.11.101`

### Automatic Configuration (Ubuntu with NetworkManager)

**Create systemd service to configure eth0 on boot:**

```bash
# Create service file
sudo tee /etc/systemd/system/aurora-eth0.service << 'EOF'
[Unit]
Description=Configure eth0 for Aurora LiDAR
After=network.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'ip addr show eth0 | grep -q "192.168.11.100" || ip addr add 192.168.11.100/24 dev eth0'
ExecStart=/bin/ip link set eth0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable aurora-eth0.service
sudo systemctl start aurora-eth0.service

# Verify
ip addr show eth0 | grep "192.168.11.100"
```

**Manual configuration (if service isn't running):**
```bash
# Apply static IP manually
sudo ip addr add 192.168.11.100/24 dev eth0
sudo ip link set eth0 up

# Verify
ip addr show eth0
```

**Test connection (Aurora must be connected and powered):**
```bash
ping -c 3 192.168.11.1
```

**Launch Aurora SDK:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/qwacr/aurora_ws/install/setup.bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.11.1
```

**Key Aurora topics:**
- `/slamware_ros_sdk_server_node/scan` - LaserScan data (~10 Hz)
- `/slamware_ros_sdk_server_node/odom` - SLAM odometry
- `/slamware_ros_sdk_server_node/imu_raw_data` - IMU data
- `/slamware_ros_sdk_server_node/map` - SLAM map

---

## Notes

- Pi must be rebooted when switching modes
- AP mode disables normal WiFi client functionality
- Aurora LiDAR accessible at `192.168.11.1` when connected via Ethernet
- Scripts are safe to run multiple times
- Original WiFi config always backed up as `.backup`
- **Use Cyclone DDS for reliable ROS2 networking**
- **Set matching ROS_DOMAIN_ID on all devices**

---

## Quick Reference Card

**AP Network:** QWACR-Pi  
**AP Password:** qwacr2026  
**Pi AP IP:** 192.168.4.1  
**Aurora IP:** 192.168.11.1  
**Robot Pi eth0 IP:** 192.168.11.100  
**Base-station Pi eth0 IP:** 192.168.11.101  
**SSH Command (AP mode):** `ssh qwacr@192.168.4.1`  
**Switch to AP:** `/home/qwacr/switch_to_ap.sh`  
**Switch to WiFi:** `/home/qwacr/switch_to_wifi.sh`
