# LoRa & HaLow Communication Integration Plan

**Date:** February 4, 2026  
**Status:** Updated March 19, 2026 – LoRa bridge and basic telemetry are
implemented in `qwacr_comms`; HaLow video is running as a single
switchable camera stream. This document now serves as the reference plan
for extending telemetry to include the fire payload and for integrating
the base-station operator console.  
**Target:** <800m range with obstacle penetration

---

## Hardware Inventory

### LoRa Hardware: Heltec LoRa 32 (863-928 MHz)
**Specifications:**
- **MCU:** ESP32 (dual-core, WiFi, BLE)
- **LoRa:** SX1276/SX1278 chip
- **Frequency:** 863-928 MHz (configurable)
- **Display:** 0.96" 128x64 OLED
- **Battery:** Li-Po management onboard
- **Power:** USB-C charging + JST battery connector
- **Source:** www.heltec.cn

**Quantity Needed:**
- 2× units (1 on robot, 1 at base station)

### HaLow Hardware: HaLow-U USB Adapter (Robot Pi + Base-station Pi)
**Specifications:**
- **Standard:** 802.11ah (HaLow)
- **Interface:** USB
- **Modes:** AP & Client mode support
- **Use:** Video streaming + dense data

**Quantity Needed:**
- 2× units (1 on Pi, 1 on laptop)

---

## System Architecture

### Communication Layers (current high-level)

```
Robot Side (Raspberry Pi 5):
┌──────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 5                           │
│                                                              │
│  ┌─────────────────┐  ┌──────────────────┐  ┌───────────┐  │
│  │  Heltec LoRa32  │  │  HaLow-U Adapter │  │  Arduino  │  │
│  │  (USB/Serial)   │  │  (USB)           │  │  Mega     │  │
│  └────────┬────────┘  └────────┬─────────┘  └─────┬─────┘  │
│           │ /dev/ttyUSB0       │ wlan1            │ /dev/   │
│           │ (UART)             │ (IP network)     │ ttyACM0 │
│  ┌────────▼────────────────────▼──────────────────▼───────┐ │
│  │           ROS2 Communication Manager                   │ │
│  │  - lora_bridge node (telemetry + teleop)              │ │
│  │  - video_publisher node (single stream, front/left)   │ │
│  │  - command_mux node (failover logic)                  │ │
│  │  - ros2_control_node (Arduino motor control)          │ │
│  └──────────────────────┬─────────────────────────────────┘ │
│                         │                                    │
│                /cmd_vel → DiffDriveController               │
│                /lora/status → monitoring                    │
└──────────────────────────────────────────────────────────────┘

Base Station (Pi 5 or laptop):
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  ┌─────────────────┐         ┌──────────────────────┐  │
│  │  Heltec LoRa32  │         │   HaLow-U Adapter    │  │
│  │  (USB to laptop)│         │   (USB to laptop)    │  │
│  └────────┬────────┘         └──────────┬───────────┘  │
│           │ UART                        │ wlan         │
│  ┌────────▼──────────────────────────────▼───────────┐  │
│  │        ROS2 Base Station Manager                  │  │
│  │  - lora_bridge node (command TX + telemetry RX)  │  │
│  │  - operator_console (teleop + fire + camera ctl) │  │
│  └────────────────────┬───────────────────────────────┘  │
│                       │                                  │
│         teleop_keyboard → /cmd_vel → LoRa               │
│         status display ← /lora/status                   │
└─────────────────────────────────────────────────────────┘
```

---

## LoRa Communication Protocol

### Packet Structure (Binary, Optimized for Low Bandwidth)

#### Downlink: Robot → Base (Telemetry)
```python
TELEMETRY_PACKET = {
    'header': 0xA5,           # 1 byte - packet start marker
    'msg_id': 0x01,           # 1 byte - telemetry packet ID
    'timestamp': uint32,      # 4 bytes - milliseconds since boot
    'gps_lat': float,         # 4 bytes - latitude (degrees)
    'gps_lon': float,         # 4 bytes - longitude (degrees)
    'gps_fix': uint8,         # 1 byte - 0=no fix, 1=2D, 2=3D
    'battery_v': uint16,      # 2 bytes - voltage * 100 (e.g., 1234 = 12.34V)
    'battery_a': uint16,      # 2 bytes - current * 100 (e.g., 523 = 5.23A)
    'system_status': uint8,   # 1 byte - bitfield (see below)
    'odom_distance': float,   # 4 bytes - total distance traveled (m)
    'rssi': int8,             # 1 byte - LoRa RSSI
    'snr': int8,              # 1 byte - LoRa SNR
    'crc16': uint16           # 2 bytes - CRC for error detection
}  # Total: 28 bytes

# system_status bitfield:
# bit 0: E-stop active
# bit 1: Auto mode enabled
# bit 2: Battery low (<20%)
# bit 3: Battery critical (<10%)
# bit 4: GPS valid
# bit 5: LiDAR operational
# bit 6: Motors enabled
# bit 7: Error flag (check error topic)
```

**Transmission Rate:** 1 Hz (1 packet per second)

#### Uplink: Base → Robot (Teleop Control)
```python
TELEOP_PACKET = {
    'header': 0xA5,           # 1 byte - packet start marker
    'msg_id': 0x02,           # 1 byte - teleop packet ID
    'timestamp': uint32,      # 4 bytes - milliseconds since boot
    'linear_x': int16,        # 2 bytes - velocity * 1000 (-2.0 to 2.0 m/s)
    'angular_z': int16,       # 2 bytes - velocity * 1000 (-3.0 to 3.0 rad/s)
    'flags': uint8,           # 1 byte - control flags (see below)
    'crc16': uint16           # 2 bytes - CRC for error detection
}  # Total: 13 bytes

# flags bitfield:
# bit 0: E-stop command
# bit 1: Resume from E-stop
# bit 2: Enable auto mode
# bit 3: Disable auto mode
# bit 4: Return home command
# bit 5-7: Reserved
```

**Transmission Rate:** 10 Hz (when teleop active), 1 Hz (heartbeat when idle)

#### Emergency Stop Packet
```python
ESTOP_PACKET = {
    'header': 0xA5,           # 1 byte
    'msg_id': 0xFF,           # 1 byte - emergency stop ID
    'timestamp': uint32,      # 4 bytes
    'crc16': uint16           # 2 bytes
}  # Total: 8 bytes - minimal size for fastest transmission
```

**Transmission:** Sent 5 times immediately when E-stop pressed

---

## ESP32 Firmware for Heltec LoRa 32

### Architecture: Smart Bridge Mode
**Why:** ESP32 can handle protocol encoding/decoding, freeing up Pi CPU and reducing UART traffic

### Firmware Components

**Robot Side ESP32:**
```
┌─────────────────────────────────────────┐
│  Heltec ESP32 Firmware (Robot)         │
│                                         │
│  UART ←→ Protocol Parser ←→ LoRa Radio │
│           ↓                              │
│     OLED Display (status)               │
│     - Battery voltage                   │
│     - LoRa RSSI/SNR                     │
│     - GPS position                      │
│     - Connection status                 │
└─────────────────────────────────────────┘
```

**Base Station ESP32:**
```
┌─────────────────────────────────────────┐
│  Heltec ESP32 Firmware (Base)          │
│                                         │
│  UART ←→ Protocol Parser ←→ LoRa Radio │
│           ↓                              │
│     OLED Display (status)               │
│     - Teleop commands sent              │
│     - Telemetry received                │
│     - Link quality (RSSI/SNR)           │
│     - Robot distance estimate           │
└─────────────────────────────────────────┘
```

### Development Platform
- **Framework:** Arduino IDE or PlatformIO
- **Library:** RadioLib or LoRa.h (Heltec official)
- **OLED:** Heltec ESP32 Dev-Boards library
- **Protocol:** Custom binary with CRC16

---

## ROS2 Integration

### Package Structure
```
qwacr_ws/src/qwacr_comms/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── lora_config.yaml        # LoRa parameters
│   └── halow_config.yaml       # HaLow video settings
├── launch/
│   ├── lora_bridge.launch.py   # Launch LoRa node
│   ├── halow_video.launch.py   # Launch video streams
│   └── full_comms.launch.py    # Launch everything
├── scripts/
│   ├── lora_bridge.py          # Main LoRa bridge node
│   ├── lora_protocol.py        # Packet encode/decode
│   ├── command_mux.py          # Failover & priority logic
│   ├── telemetry_publisher.py  # Collect & publish telemetry
│   └── video_compressor.py     # Camera compression setup
└── esp32_firmware/
    ├── robot_side/
    │   └── robot_lora.ino
    └── base_station/
        └── base_lora.ino
```

### Key ROS2 Nodes

#### 1. lora_bridge (Robot Side)
```python
# Subscriptions:
/cmd_vel                    # geometry_msgs/Twist - from command_mux
/battery/state              # sensor_msgs/BatteryState
/gps/fix                    # sensor_msgs/NavSatFix
/diff_cont/odom             # nav_msgs/Odometry

# Publications:
/lora/cmd_vel_in            # geometry_msgs/Twist - received commands
/lora/telemetry_out         # qwacr_msgs/LoRaTelemetry - status sent
/lora/link_quality          # qwacr_msgs/LinkQuality - RSSI/SNR

# Services:
/lora/emergency_stop        # std_srvs/Trigger
```

#### 2. command_mux (Robot Side)
```python
# Subscriptions:
/cmd_vel_teleop             # From teleop_keyboard (via WiFi/HaLow)
/lora/cmd_vel_in            # From LoRa bridge
/cmd_vel_autonomous         # From Nav2/autonomy

# Publications:
/cmd_vel                    # To DiffDriveController (via twist_to_stamped)

# Logic:
# Priority: LoRa E-stop > LoRa teleop > WiFi/HaLow teleop > Autonomous
# Watchdog: Stop robot if no command for 2 seconds
```

#### 3. telemetry_publisher (Robot Side)
```python
# Subscriptions:
/battery/state
/gps/fix
/diff_cont/odom
/diagnostics

# Publications:
/lora/telemetry_out         # Aggregated telemetry @ 1 Hz

# Functionality:
# - Collects data from multiple sources
# - Formats into LoRa telemetry packet
# - Sends to lora_bridge for transmission
```

---

## HaLow Video Streaming

### Camera Setup
**Hardware:**
- Camera 1: Aurora LiDAR built-in camera (ethernet-based, already publishing ROS topics)
- Camera 2: USB webcam (front, rear, or side view for additional coverage)

### Video Pipeline (Robot Side)

**ASB Camera (Additional View):**
```bash
# Single USB camera for complementary view
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=20
**USB Camera (Additional View):**
```bash
# Camera 1: Front navigation camera (higher priority)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=20/1 ! \
  videoconvert ! \
  x264enc tune=zerolatency bitrate=3500 speed-preset=ultrafast ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=<base_halow_ip> port=5000

# Camera 2: Rear/side camera (lower priority)
gst-launch-1.0 v4l2src device=/dev/video1 ! \
  video/x-raw,width=1280,height=720,framerate=15/1 ! \
  videoconvert ! \
  x264enc tune=zerolatency bitrate=3000 speed-preset=ultrafast ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=<base_halow_ip> port=5001
```

**ROS2 Integration:**
```bash
# Alternative: Use ROS2 camera nodes
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[1280,720] \
  -p camera_frame_id:=camera_front_link \
  -r image_raw:=/camera_front/image_raw

# Curora camera - compress existing topic
ros2 run image_transport republish raw compressed \
  --ros-args \
  -p in:=/camera/image \
  -p out/compressed/jpeg_quality:=60

# USB camera - publish as ROS2 topic
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  View Aurora camera
rqt_image_view /camera_aurora/image_raw/compressed

# View USB camera  
rqt_image_view /camera_usb/image_raw/compressed

# Or use dual-pane viewer
ros2 run rqt_image_view rqt_image_view
```

**Bandwidth Allocation (Revised):**
- Aurora camera compressed: 3-4 Mbps @ 720p 20fps
- USB camera compressed: 2-3 Mbps @ 720p 15fps  
- LiDAR scan data: 1-2 Mbps
- Total: 6-9 Mbps (well within HaLow capability)vdec_h264 ! \
  videoconvert ! \
  autovideosink

# Or use ROS2 rqt_image_view
rqt_image_view /camera_front/image_raw/compressed
```

---

## HaLow Network Configuration

### Robot Side (Pi as HaLow AP)
```bash
# Install drivers (if needed - HaLow-U should be plug-and-play)
# Configure as Access Point

# /etc/network/interfaces.d/halow
auto wlan1
iface wlan1 inet static
    address 10.20.0.1
    netmask 255.255.255.0

# hostapd config for HaLow AP
# /etc/hostapd/hostapd_halow.conf
interface=wlan1
driver=nl80211
ssid=QWACR_HaLow
hw_mode=a
channel=36  # HaLow channel - verify with hardware docs
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=your_secure_password_here
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```

### Base Station (Laptop as HaLow Client)
```bash
# Connect to QWACR_HaLow network
# Static IP or DHCP (if dnsmasq configured on Pi)
# Should auto-connect with NetworkManager
```

**Network Layout:**
- HaLow subnet: 10.20.0.0/24
- Pi (robot): 10.20.0.1
- Laptop (base): 10.20.0.100 (DHCP or static)
- Use for: Video streams, optional ROS2 DDS

---

## Implementation Phases

### Phase 2A: LoRa Telemetry & Teleop (2-3 weeks)

#### Week 1: ESP32 Firmware Development
- [ ] Set up Heltec ESP32 development environment
- [ ] Implement LoRa protocol (packet encode/decode)
- [ ] Test LoRa range in static conditions
- [ ] Implement OLED status display
- [ ] Battery voltage monitoring display

#### Week 2: ROS2 Integration
- [ ] Create qwacr_comms package
- [ ] Implement lora_bridge node (Python)
- [ ] Implement telemetry_publisher node
- [ ] Test UART communication Pi ↔ ESP32
- [ ] Verify packet transmission/reception

#### Week 3: Teleop Control & Testing
- [ ] Implement command_mux node (failover logic)
- [ ] Integrate with twist_to_stamped bridge
- [ ] Test teleop control via LoRa
- [ ] Range testing outdoors (<800m)
- [ ] Obstacle penetration testing (buildings/trees)

### Phase 2B: HaLow Video Streaming (2 weeks)

#### Week 1: HaLow Network Setup
- [ ] Install HaLow-U drivers on Pi and laptop
- [ ] Configure Pi as HaLow AP
- [ ] Test network connectivity and bandwidth
- [ ] Measure latency and packet loss
- [ ] Range testing without video load

#### Week 2: Video Implementation
- [ ] Connect USB cameras to Pi
- [ ] Test GStreamer pipelines locally
- [ ] Configure dual video streams (front + rear)
- [ ] Implement video_compressor node (ROS2)
- [ ] Test video streaming over HaLow @ distance
- [ ] Tune compression for bandwidth/quality

### Phase 2C: Integrated Communication System (1 week)

#### Integration Testing
- [ ] Run LoRa teleop + HaLow video simultaneously
- [ ] Verify bandwidth allocation (3-4 Mbps per camera)
- [ ] Test failover (disable HaLow, verify LoRa control works)
- [ ] Emergency stop testing via LoRa
- [ ] Long-duration stability test (1+ hour operation)

#### Autonomous Integration
- [ ] Integrate with Nav2 autonomous navigation
- [ ] Test autonomous waypoint following with LoRa monitoring
- [ ] Test return-home on communication loss
- [ ] Final outdoor mission test (<800m range)

---

##USB Hub Configuration (Powered Hub Strongly Recommended):**
```
Raspberry Pi 5 USB Ports:
├── USB Port 1 → Arduino Mega (/dev/ttyACM0) - Motor control
├── USB Port 2 → GPS Module (/dev/ttyACM1) - Position data
├── USB Port 3 → Powered USB Hub (externally powered) ──┐
└── USB Port 4 → (available for expansion)              │
                                                         │
    Powered USB Hub (4-7 ports, 2A+ power supply): ─────┘
    ├── Port 1 → Heltec LoRa 32 (/dev/ttyUSB0) - Communication
    ├── Port 2 → HaLow-U Adapter (wlan1) - Video network
    ├── Porhub port
- Network device: `wlan1` (after driver install)
- Should be plug-and-play on recent Linux kernels

**USB Camera → Hub:**
- USB to hub port
- Device: `/dev/video0`
- High current device, benefits from powered hub

**Note:** Arduino and GPS on direct Pi ports for maximum reliability. Communication devices (LoRa, HaLow, Camera) on powered hub for current capacity.
- Keep on direct Pi port for reliability

**GPS Module → Pi USB (Direct):**
- USB to Pi (currently /dev/ttyACM1 or ttyUSB)
- Keep on direct Pi port for stable positioning data

**USB Camera → Hub
- HaLow + Camera can draw 800mA combined
- Prevents voltage drops and USB disconnects
- Provides stable 5V to all devices
- Pi's USB current limiter won't throttle devices

**Ethernet Devices:**
- Aurora LiDAR: Ethernet to Pi eth0 at 192.168.11.1 (LiDAR + built-in camera)

**Heltec LoRa 32 → Pi USB:**
- USB-C cable from Heltec to hub
- Serial device: `/dev/ttyUSB0`
- Power: 5V from USB hub (150mA typical)
- Li-Po battery optional for portable base station only
- USB-C cable from Heltec to Pi USB port
- Serial device: `/dev/ttyUSB0` (or `/dev/ttyUSB1` if Arduino is USB0)
- Power: 5V from USB (no separate battery needed while on robot)
- Can also power from Li-Po for portable base station

**HaLow-U Adapter → Pi USB:**
- USB-A to Pi USB3 port
- Network device: `wlan1` (after driver install)
- Should be plug-and-play on recent Linux kernels

**Arduino Mega → Pi USB:**
- USB-A to B cable
- Serial device: `/dev/ttyACM0` (existing motor control)
- Already configured and working

### Base Station Connections

**Heltec LoRa 32:**
- USB-C to laptop
- Serial device: `/dev/ttyUSB0` or `/dev/ttyACM0` (Linux)
- COM port (Windows)

**HaLow-U Adapter:**
- USB-A to laptop
- WiFi device managed by NetworkManager

---

## Message Definitions (Custom)

### qwacr_msgs/LoRaTelemetry.msg
```
# LoRa telemetry packet
std_msgs/Header header
float64 gps_latitude          # degrees
float64 gps_longitude         # degrees
uint8 gps_fix_type            # 0=no fix, 1=2D, 2=3D
float32 battery_voltage       # volts
float32 battery_current       # amps
bool emergency_stop_active
bool autonomous_mode_enabled
bool battery_low
bool battery_critical
bool gps_valid
bool lidar_operational
bool motors_enabled
bool error_flag
float32 odometry_distance     # meters traveled
int8 rssi                     # LoRa signal strength
int8 snr                      # LoRa signal-to-noise
```

### qwacr_msgs/LinkQuality.msg
```
# Communication link quality
std_msgs/Header header
string interface              # "lora", "halow", "wifi"
bool connected
int8 rssi                     # dBm
int8 snr                      # dB (LoRa only)
float32 packet_loss_rate      # 0.0 to 1.0
float32 latency_ms            # milliseconds
float32 bandwidth_mbps        # megabits per second
```

---

## Configuration Files

### lora_config.yaml
```yaml
lora_bridge:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    telemetry_rate: 1.0           # Hz
    teleop_rate: 10.0              # Hz when active
    teleop_timeout: 2.0            # seconds
    max_retries: 3
    
    # LoRa parameters (configured on ESP32, documented here)
    frequency: 915.0               # MHz (US) or 868.0 (EU)
    spreading_factor: 7            # 7-12 (higher = longer range, slower)
    bandwidth: 125.0               # kHz
    coding_rate: 5                 # 5-8
    tx_power: 20                   # dBm (max for Heltec)
```

### halow_config.yaml
```yaml
video_publisher:
  ros__parameters:
    camera1_device: "/dev/video0"
    camera1_name: "front"
    camera1_width: 1280
    camera1_height: 720
    camera1_fps: 20
    camera1_bitrate: 3500          # kbps
    
    camera2_device: "/dev/video1"
    camera2_name: "rear"
    camera2_width: 1280
    camera2_height: 720
    camera2_fps: 15
    camera2_bitrate: 3000          # kbps
    
    compression: "h264"
    quality: 60                    # 0-100
    
    halow_interface: "wlan1"
    base_station_ip: "10.20.0.100"
    stream1_port: 5000
    stream2_port: 5001
```

---

## Testing Procedures

### LoRa Range Test Protocol
1. **Setup:**
   - Robot stationary with LoRa active
   - Base station with laptop + Heltec
   - GPS logger on base station (phone)
   
2. **Test:**
   - Walk away from robot in straight line
   - Monitor RSSI/SNR on base station OLED
   - Record distance when signal degrades
   - Test teleop commands at various distances
   
3. **Obstacle Test:**
   - Walk behind buildings
   - Enter wooded areas
   - Monitor link quality and control responsiveness
   
4. **Success Criteria:**
   - Reliable teleop control at 400m+ open field
   - Control maintained through 1-2 obstacles (buildings/trees)
   - Latency <500ms for teleop commands

### HaLow Video Test Protocol
1. **Bandwidth Test:**
   - Stream dual cameras @ full settings
   - Monitor network throughput on Pi
   - Verify ~7-8 Mbps total (both cameras)
   
2. **Range Test:**
   - Drive robot away while monitoring video
   - Record distance when video degrades
   - Check for freezing vs complete loss
   
3. **Combined Test:**
   - LoRa teleop + HaLow video simultaneously
   - Verify no interference
   - Confirm LoRa takes priority if bandwidth limited

---

## Expected Performance

### USB & Network Device Management on Pi

**USB Devices (5 total - need powered USB hub):**
- `/dev/ttyACM0` - Arduino Mega (motor control) - ~100mA
- `/dev/ttyACM1` - GPS module - ~50mA  
- `/dev/ttyUSB0` - Heltec LoRa 32 (communication) - ~150mA (TX mode)
- `/dev/video0` - USB Camera (front/rear view) - ~500mA
- `wlan1` - HaLow-U adapter - ~300mA

**Ethernet Devices:**
- Aurora LiDAR at 192.168.11.1 (publishes /camera/image + /scan)

**Total USB Current:** ~1.1A (exceeds single USB port limit)

**USB Hub Required:** 
- **Powered USB hub** (externally powered) strongly recommended
- Pi 5 USB ports: 4× USB3.0, 600mA total budget per port
- Cameras + HaLow can draw significant current
- Powered hub prevents brownouts and USB device resets

**Video Sources:**
- Camera 1: Aurora built-in camera (via ethernet/ROS topic: `/camera/image`)
- Camera 2: USB webcam (front or rear, `/dev/video0`)

### LoRa Communication
- **Range (open field):** 500-800m
- **Range (light obstacles):** 300-500m
- **Range (heavy obstacles):** 200-400m
- **Latency:** 200-500ms typical
- **Throughput:** ~5 kbps effective (teleop + telemetry)

### HaLow Video
- **Range (open field):** 800-1000m
- **Range (obstacles):** 400-600m
- **Throughput:** 7-10 Mbps dual stream
- **Latency:** 300-800ms (video encoding + network)

### Critical Items

**USB Hub (Required):**
- [ ] Powered USB hub (4-7 ports, 2A+ power adapter)
- [ ] Recommendation: Anker or Sabrent powered hub
- [ ] Must have external power supply (not bus-powered)

### LoRa System
- [x] 2× Heltec LoRa 32 (863-928 MHz) - **OWNED**
- [ ] 2× Li-Po batteries (500-1000 mAh, JST connector) - **For base station only**
- [ ] 2× LoRa antennas (915 MHz, SMA connector) - may be included with Heltec
- [ ] USB-C cables for programming and power

### HaLow System
- [x] 2× HaLow-U USB adapters - **OWNED**
- [ ] USB extension cables (if needed for antenna placement)

### Cameras
- [x] Aurora LiDAR built-in camera - **OWNED** (already integrated)
- [ ] 1× USB webcam (720p+ capable) - **Only need ONE additional camera**

### Sensors
- [x] GPS module (USB) - **OWNED** (needs driver configuration)
- [ ] IMU module (future - I2C or USB
- [x] 2× Heltec LoRa 32 (863-928 MHz) - **OWNED**
- [ ] Powered USB hub mounting bracket
- [ ] Enclosure for Heltec on robot (weatherproof optional)
- [ ] USB camera mount (adjustable)
- [ ] Antenna mounts (elevated for better range)
- [ ] Cable management (velcro straps, zip ties

### HaLow System
- [x] 2× HaLow-U USB adapters - **OWNED**
- [ ] USB extension cables (if needed for placement)

### CamerasCritical Hardware:**
   - **Powered USB hub** (top priority - prevents USB issues)
   - Li-Po batteries for Heltec boards (base station)
   - LoRa antennas (if not included)
   - One USB camera (if not owned)

2. **GPS Driver Configuration (Existing Hardware):**
   - Debug GPS not publishing from Jan 30 session
   - Try baud rates: 9600, 38400, 115200
   - Verify NMEA output format
   - Test outdoors for GPS fix

3. **Development Environment Setup:**
   - Install Arduino IDE or PlatformIO
   - Install Heltec ESP32 board support
   - Install required libraries (RadioLib, Heltec OLED)

4. **Initial Testing:**
   - Test Heltec LoRa 32 basic LoRa communication
   - Measure RSSI between boards in static test
   - Display telemetry on OLED

5. **Aurora Camera Integration:**
   - Verify /camera/image topic from Aurora (already tested Jan 30)
   - Test image_transport compression
   - Prepare for HaLow streaming

6. **Create ROS2 Package:**
   - `ros2 pkg create qwacr_comms --build-type ament_python`
   - Set up basic node structure
   - Prepare for ESP32 firmware integration

7  - Install Heltec ESP32 board support
   - Install required libraries (RadioLib, Heltec OLED)

3. **Initial Testing:**
   - Test Heltec LoRa 32 basic LoRa communication
   - Measure RSSI between boards in static test
   - Display telemetry on OLED

4. **Create ROS2 Package:**
   - `ros2 pkg create qwacr_comms --build-type ament_python`
   - Set up basic node structure
   - Prepare for ESP32 firmware integration

5. **HaLow Driver Installation:**
   - Research HaLow-U Linux driver installation
   - Test plug-and-play USB detection
   - Configure as WiFi interface

Ready to start with ESP32 firmware development first, or want to verify HaLow hardware compatibility?
