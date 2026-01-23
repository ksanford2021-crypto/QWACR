# QWACR GPS Driver Package

ROS2 GPS driver package for the QWACR robot. Provides GPS NMEA sentence parsing, NavSatFix publishing, and GPS-to-ENU (East-North-Up) coordinate conversion.

## Features

- **NMEA Parsing**: Supports GPGGA and GPRMC sentence formats
- **NavSatFix Publishing**: Standard ROS2 GPS messages on `/gps/fix` topic
- **ENU Conversion**: Local tangent plane approximation for converting GPS coordinates to local East-North-Up meters
- **Auto-Origin**: Automatically sets ENU origin on first valid GPS fix
- **Configurable Parameters**: Serial port, baud rate, frame ID, origin coordinates

## Topics

### Published
- `/gps/fix` (sensor_msgs/NavSatFix): GPS position fix
- `/gps/enu_pose` (geometry_msgs/PoseStamped): Position in local ENU coordinates (if enabled)

## Parameters

- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port device for GPS
- `baud_rate` (int, default: 9600): Serial communication baud rate
- `frame_id` (string, default: "gps"): TF frame ID for GPS messages
- `publish_enu` (bool, default: true): Enable ENU pose publishing
- `origin_lat` (double, default: 0.0): ENU origin latitude (auto-set if 0.0)
- `origin_lon` (double, default: 0.0): ENU origin longitude (auto-set if 0.0)
- `publish_rate` (double, default: 10.0): Target publishing rate in Hz

## Usage

### Build the package
```bash
cd ~/qwacr_ws
colcon build --packages-select qwacr_gps
source install/setup.bash
```

### Run the GPS driver node
```bash
# Using launch file (recommended)
ros2 launch qwacr_gps gps.launch.py

# With custom serial port
ros2 launch qwacr_gps gps.launch.py serial_port:=/dev/ttyACM0

# Run node directly
ros2 run qwacr_gps gps_driver_node
```

### View GPS data
```bash
# Watch NavSatFix messages
ros2 topic echo /gps/fix

# Watch ENU pose
ros2 topic echo /gps/enu_pose

# Check node info
ros2 node info /gps_driver
```

## Hardware Setup

1. Connect GPS module via USB to Raspberry Pi
2. Verify device appears (usually `/dev/ttyUSB0` or `/dev/ttyACM0`):
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```
3. Ensure user has serial port permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in for changes to take effect
   ```

### USB-C Serial (CH342) — Simplest path

The LG580P breakout exposes a CH342 USB-serial converter on the USB-C port. Connecting the Pi’s USB-A to the LG580P USB-C provides both power and a serial device (e.g., `/dev/ttyUSB0`). Use this if you have a free USB port.

- Plug Pi USB-A ↔ LG580P USB-C
- Verify the device:
  ```bash
  ls -l /dev/ttyUSB*
  ```
- Launch with USB serial:
  ```bash
  ros2 launch qwacr_gps gps.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200
  ```
- Adjust `baud_rate` to match the LG580P’s UART setting (often 115200 or 38400).

### UART-GPIO Wiring (Pi 5) — When USB ports are tight

If you choose to use the Pi GPIO UART for data, wire the LG580P UART at 3.3V logic levels directly to the Raspberry Pi 5 header:

- LG580P `TXO` → Pi GPIO15 (`RXD`)
- LG580P `RXI` → Pi GPIO14 (`TXD`)
- LG580P `GND` → Pi any `GND`

Notes:
- If powering the LG580P via USB-C only, do NOT also feed 5V from the Pi. Share only GND.
- The UART is bidirectional; it will both receive NMEA and send RTCM corrections for RTK.

Enable the UART on Ubuntu 24.04 for Raspberry Pi:

1) Disable serial console (if present) by editing `/boot/firmware/cmdline.txt` and removing any `console=serial0,115200` or `console=ttyAMA0,115200` entry. Save and exit.

2) Enable the UART overlay by adding to `/boot/firmware/usercfg.txt` (or `/boot/firmware/config.txt` if `usercfg.txt` is not present):
```
enable_uart=1
# or explicitly
dtoverlay=uart0
```

3) Reboot, then verify the device symlink exists:
```bash
ls -l /dev/serial0
```

4) Launch the driver using the GPIO UART:
```bash
ros2 launch qwacr_gps gps.launch.py serial_port:=/dev/serial0 baud_rate:=115200
```

If the LG580P default baud differs (e.g., 38400), set `baud_rate` accordingly.

## Dependencies

- ROS2 Jazzy
- Python packages:
  - rclpy
  - sensor_msgs
  - geometry_msgs
  - pyserial (install: `pip install pyserial`)

## Coordinate Systems

### GPS Frame
- Origin: WGS84 geodetic coordinates (latitude, longitude, altitude)
- Units: Degrees and meters (altitude)

### ENU Frame (Local Tangent Plane)
- Origin: First valid GPS fix (or specified origin_lat/origin_lon)
- X-axis: East
- Y-axis: North
- Z-axis: Up
- Units: Meters

The ENU conversion uses a local tangent plane approximation suitable for navigation within ~100km of the origin.

## Troubleshooting

**No GPS data received:**
- Check serial port: `ls -l /dev/ttyUSB*`
- Verify baud rate matches GPS module (typically 9600 or 4800)
- Ensure GPS has clear view of sky for satellite lock
- Check user permissions: `groups $USER` should include `dialout`

**Using GPIO UART but still no data:**
- Confirm the serial console was removed from `/boot/firmware/cmdline.txt`
- Confirm UART enabled via `enable_uart=1` or `dtoverlay=uart0`
- Check wiring (TX↔RX crossed) and shared ground
- Try a lower baud like 38400 if unknown

**Poor GPS accuracy:**
- Wait for satellite lock (8+ satellites recommended)
- Ensure antenna has unobstructed sky view
- Check fix quality in NavSatFix message status

**Import error for serial:**
```bash
pip install pyserial
```

## RTK Corrections (NTRIP → UART)

For centimeter-level accuracy and dual-antenna heading, feed RTCM3 corrections to the LG580P. The Pi can run an NTRIP client and forward RTCM over the same GPIO UART used for NMEA.

Option A — RTKLIB `str2str` (recommended):
```bash
sudo apt update && sudo apt install -y rtklib
str2str -in ntrip://USER:PASS@CASTER:PORT/MOUNT -out serial://ttyAMA0:115200:8:n:1
# Replace ttyAMA0 with the canonical device backing /dev/serial0 (check with ls -l /dev/serial0)
```

Option B — Any NTRIP client → tty: Use your preferred client to output raw RTCM to `/dev/serial0` at the receiver’s configured baud.

Notes:
- Ensure the LG580P is configured to accept RTCM on UART.
- NMEA and RTCM share the same UART link bidirectionally.
- Verify RTK fix status in the GGA quality field or device-specific messages.

## Dual Antenna Heading

The LG580P computes heading from the baseline between the two GNSS antennas. Over UART, many receivers output `$GPHDT` (true heading) in addition to standard NMEA. If needed, we can extend `gps_driver_node` to parse `$GPHDT` and publish a heading/pose orientation message.

## License

MIT License - See LICENSE file for details
