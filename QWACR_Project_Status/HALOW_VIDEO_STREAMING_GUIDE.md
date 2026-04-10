# QWACR HaLow Video Streaming Guide

**Robot:** QWACR Autonomous Ground Robot  
**Subsystem:** HaLow Video Telemetry  
**Camera:** Raspberry Pi Camera Module 3 (v3), CSI interface  
**Transport:** H.264 over RTP/UDP via HaLow

---

## 1. Overview

Goal: stream low-latency video from the Pi-mounted Camera Module 3 to a base station over the HaLow WiFi link.

Architecture:

- **Producer (robot Pi):**
  - Linux V4L2 device for Pi Camera 3 (e.g., `/dev/video0`).  
  - GStreamer pipeline run by `qwacr_comms/video_publisher` ROS 2 node.  
  - Encodes to H.264 and sends RTP/UDP to the base station IP/port.

- **Consumer (base station):**
  - GStreamer pipeline receiving RTP/UDP and decoding H.264 for display.

All network traffic rides on the HaLow interface as long as routes to the base station IP point through that interface.

### 1.1 HaLow IP layout used on QWACR

For the current QWACR robot + dev setup we use:

- HaLow AP radio: `10.192.5.11`
- HaLow client radio (USB HaLow-U on robot Pi): `10.192.5.12`
- Robot Pi HaLow USB interface: `10.192.5.13`
- **Base‑station HaLow host:** `10.192.5.100` (via DHCP from the AP)
  - Historically this was the Windows dev PC.
  - In the 2026‑03‑29+ configuration, this address is instead used by the
    **base‑station Raspberry Pi** running the operator GUI and GStreamer
    receiver.

The important distinction is:

- `10.192.5.12` is the *radio* itself (USB HaLow-U). You use this address only for configuring the radio’s web UI.
- `10.192.5.13` is the *Pi’s network interface* behind that radio. All robot traffic (SSH, ROS, video RTP) originates from this IP, so the base station pings and streams to/from `10.192.5.13`, not `.12`.
- `10.192.5.100` is the **base‑station HaLow host** (either the Windows PC
  or, in the current setup, the base‑station Pi) which runs the GStreamer
  receiver and/or operator GUI.

In other words, once the radios are configured and linked, you mostly ignore `10.192.5.12` and instead treat `10.192.5.13` as the robot’s address on the HaLow network.

**Important:** all video traffic (the `host=` in `udpsink` and the
`base_station_ip` parameter) must target the **base‑station HaLow host
IP** (`10.192.5.100` in this layout), *not* the AP (`10.192.5.11`) or the
USB HaLow-U radio (`10.192.5.12`).

### 1.2 Robot Pi HaLow netplan configuration

On the robot Pi we make the HaLow USB interface static so it is stable across reboots. The interface name in our setup is `enx00c0cab4c0f0` (it is derived from the MAC and may differ on another Pi).

Create `/etc/netplan/50-halow.yaml` on the robot Pi with:

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enx00c0cab4c0f0:
      dhcp4: false
      addresses:
        - 10.192.5.13/16
```

Apply and verify:

```bash
sudo chmod 600 /etc/netplan/50-halow.yaml
sudo netplan apply
ip addr show enx00c0cab4c0f0   # should show 10.192.5.13/16
ping 10.192.5.11               # AP radio reachable
```

When you later move this HaLow client radio to a different Pi, repeat the same file creation on that Pi, adjusting only the interface name if it changes.

With this layout, the base station pings the robot at `10.192.5.13`, and the robot pings the AP at `10.192.5.11` to confirm the link.

---

## 2. Robot-Side Setup (Raspberry Pi)

### 2.0 Install libcamera / rpicam-apps from source (Ubuntu 24.04 on Pi 5)

On Ubuntu 24.04 on a Raspberry Pi 5, installing `rpicam-apps` from the package repo currently fails due to a libavcodec API mismatch. For Camera Module 3 support we therefore install `rpicam-apps` **from source** and disable the optional libav encoder.

Do this only on Pis that directly host one or more Pi cameras (the robot Pi with the two Camera Module 3 units, and any separate dev Pi you use for camera bring-up):

```bash
sudo apt update
sudo apt install -y git meson ninja-build pkg-config

cd ~
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps

# Clean any previous build directory if it exists
rm -rf build

# Configure and build with libav encoder disabled
meson setup build --buildtype=release -Denable_libav=disabled
ninja -C build
sudo ninja -C build install
sudo ldconfig
```

Quick sanity checks on the Pi after install:

```bash
rpicam-hello
rpicam-still -o test.jpg
```

If both commands work, the libcamera stack and Camera Module 3 are healthy. The HaLow video pipeline below then builds on top of this camera stack.

### 2.1 Verify Camera Module 3 is Detected

1. Physically connect the Camera Module 3 to the Pi CSI port.  
2. Ensure the Pi uses a kernel/firmware that supports Camera Module 3 and that the camera overlay is enabled (see main mission docs / Ubuntu-on-Pi docs).  
3. After reboot, check for V4L2 devices:

```bash
v4l2-ctl --list-devices
```

You should see an entry for the Pi camera sensor (e.g., `imx708`) with a device like `/dev/video0`.

### 2.2 Test Raw Video on the Pi

Before involving ROS 2 or HaLow, confirm you can see video locally:

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=20/1 ! \
  videoconvert ! autovideosink
```

If you see a live preview, the camera driver and V4L2 path are working.

### 2.3 Configure HaLow Video Parameters

Edit the HaLow video config:  
`qwacr_ws/src/qwacr_comms/config/halow_config.yaml`

Key parameters under `video_publisher.ros__parameters`:

- `camera1_device`: currently **unused** for the rpicam-vid pipeline
  (Pi Camera 3 is accessed via `rpicam-vid`, not `/dev/video*`), kept as
  a placeholder for potential future USB camera support.  
- `camera1_width`, `camera1_height`: e.g., `1280x720` or `1920x1080`.  
- `camera1_fps`: typical 15–30 FPS; lower FPS saves bandwidth.  
- `camera1_bitrate`: H.264 bitrate in kbps (e.g., `2500–4000` for 720p).  
- `halow_interface`: name of the robot Pi interface connected to the HaLow client radio (for our current setup: `enx00c0cab4c0f0`).  
- `base_station_ip`: IP of the **Windows/base‑station PC** on the HaLow
  network (for our setup: `10.192.5.100`). Do **not** set this to the
  AP (`10.192.5.11`) or the USB HaLow-U radio (`10.192.5.12`); those
  hosts do not run the GStreamer receiver.
- `stream1_port`: UDP port for this stream (e.g., `5000`).

Leave `camera2_*` blank or as-is; multi-camera streaming is not
currently supported with the rpicam-vid method.

### 2.4 Start the HaLow Video Node (current launch procedure)

On the **robot Pi** (reachable over HaLow, e.g. `ssh qwacr@10.192.5.13`):

```bash
cd ~/qwacr_ws

# One‑time (or after code changes): build qwacr_comms
colcon build --packages-select qwacr_comms

# Standard QWACR environment setup
source install/setup.bash

# Launch the HaLow video node in its own terminal
ros2 launch qwacr_comms halow_video.launch.py
```

This fits alongside the robust launch scripts described in
`ROBUST_LAUNCH_SCRIPTS_GUIDE.md`: bring up motors/full system using the
robust scripts, and run `halow_video.launch.py` in a separate SSH
terminal on the same Pi when you want video.

The `video_publisher` node now runs a shell pipeline equivalent to the
validated manual command in section **2.5**, with values substituted from
`halow_config.yaml`, for example:

```bash
rpicam-vid -t 0 -n --width 1280 --height 720 --framerate 20 \
  --codec yuv420 -o - | \
  gst-launch-1.0 -v fdsrc ! \
    videoparse width=1280 height=720 framerate=20/1 format=i420 ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=3500 speed-preset=ultrafast ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=<base_station_ip> port=5000 sync=false
```

### 2.5 Manual `rpicam-vid` → GStreamer test pipeline (validated)

For quick, non‑ROS bring‑up of Camera Module 3 on the Pi 5 over HaLow, the
following **single‑shot shell pipeline has been verified to work end‑to‑end**
from the robot Pi to the dev machine (as of 2026‑03‑04):

On the robot Pi (HaLow up, replacing `<base_station_ip>` as needed):

```bash
rpicam-vid -t 0 -n --width 640 --height 480 --framerate 15 \
  --codec yuv420 -o - | \
  gst-launch-1.0 -v fdsrc ! \
    videoparse width=640 height=480 framerate=15/1 format=i420 ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=1200 speed-preset=ultrafast ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=<base_station_ip> port=5000 sync=false
```

Key points:

- `rpicam-vid` uses `--codec yuv420` and writes raw YUV frames to stdout.
- A local GStreamer pipeline on the Pi reads those frames from `fdsrc`,
  converts them, and uses `x264enc` + `rtph264pay` to produce standard
  H.264/RTP/UDP—this avoids the broken `--codec h264` path in `rpicam-vid`
  on the current Ubuntu 24.04 Pi 5 image.
- The resulting RTP stream is received on the base station by the same
  H.264 GStreamer pipeline described in section **3** (with UDP port `5000`).

In all examples in this section, `<base_station_ip>` should be replaced
with the **Windows/base‑station PC IP** (`10.192.5.100` in the current
lab setup), not the AP or radio address.

This pipeline is a good "known‑good" reference when debugging HaLow video:
if it works but the ROS 2 `video_publisher` node does not, the problem is
likely in ROS/config (not the camera, encoder, or HaLow link).

### 2.6 Single-stream, switchable cameras (front vs left)

In practice, the HaLow link and Pi CPU budget support **one robust H.264
stream at a time** far better than two simultaneous streams. Rather than
running two degraded feeds concurrently, QWACR therefore uses a
**single-stream, switchable-camera** design:

- The robot Pi hosts two Pi Camera Module 3 units:
  - **Front camera**: libcamera index `0` (nearest the Ethernet jack).
  - **Left camera**: libcamera index `1` (furthest from the Ethernet jack).
- Only **one encoder/stream** runs at once, always to `base_station_ip`
  on port `5000`.
- The operator chooses which view to stream by launching one of two
  ROS 2 launch files on the **robot Pi**:

**Front camera (default):**

```bash
cd ~/qwacr_ws
source install/setup.bash
ros2 launch qwacr_comms halow_video.launch.py
```

This uses `config/halow_config.yaml` and streams from camera index `0`
(`front`) at 640x480/15 fps, bitrate ≈ 1200 kbps.

**Left camera:**

```bash
cd ~/qwacr_ws
source install/setup.bash
ros2 launch qwacr_comms halow_left_video.launch.py
```

This reuses `halow_config.yaml` but overrides `camera1_index` to `1` and
`camera1_name` to `left`, so the same encoding parameters and UDP port
are used, just from the other camera.

On the base station, the GStreamer **receive** pipeline stays exactly
the same (port `5000`); switching cameras only changes which encoder is
running on the robot.

---

## 3. Base Station Setup

The base station does **not** host any cameras; it only receives and
displays video over HaLow. It therefore does **not** need `rpicam-apps`
or libcamera installed, only GStreamer (and ROS 2 if you are monitoring
topics there).

### 3.1 Linux base station (including Raspberry Pi)

On a Linux base station (laptop or Raspberry Pi), install GStreamer if
needed and run a receive pipeline:

```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264" ! \
  rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

- `port` must match `stream1_port` in `halow_config.yaml` (typically
  `5000`).  
- `caps` tell GStreamer to treat incoming packets as H.264 RTP.  
- `sync=false` keeps latency low at the expense of strict AV sync.

This is the configuration used when the **base station itself is a
Raspberry Pi** with a display. The operator can SSH into that
base-station Pi from a laptop, start the receiver above, and then
start/stop the appropriate `halow_*.launch.py` on the robot Pi over the
HaLow link.

### 3.2 Windows base station (PowerShell)

For a Windows dev machine connected to HaLow, run the receiver **in
PowerShell or Windows Terminal (not inside WSL2)**. Assuming GStreamer
is installed under `C:\gstreamer\1.0\msvc_x86_64\bin`:

```powershell
$env:Path += ";C:\gstreamer\1.0\msvc_x86_64\bin"

gst-launch-1.0 -v `
  udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" `
    ! rtph264depay `
    ! avdec_h264 `
    ! videoconvert `
    ! autovideosink sync=false
```

Notes for reliability:

- Always run from a Windows shell so packets that arrive on the HaLow
  NIC are visible to GStreamer; WSL2 will not reliably see this UDP
  traffic.
- If `gst-launch-1.0` is not found, either adjust `$env:Path` as above
  or start from the GStreamer "Developer Command Prompt".
- If a previous pipeline crashed or was closed without stopping, use
  Task Manager or `taskkill /IM gst-launch-1.0.exe /F` to clear stale
  processes before re-running.
- Ensure the Windows firewall allows inbound UDP on the chosen port
  (e.g. `5000`).

If the HaLow network is correctly routed, you should see live video from the Pi camera.

For two-camera operation on the robot Pi, configure both `camera1_*` and `camera2_*` blocks in `halow_config.yaml` with distinct UDP ports (for example `5000` and `5002`) and run matching `udpsrc` pipelines or viewers on the base station.

---

## 4. Performance Tuning

- **Resolution/FPS:** Lower either to reduce bandwidth and CPU load.  
- **Bitrate:** Start around `2500–3500` kbps for 720p; adjust based on link quality.  
- **Latency:** `tune=zerolatency` and `speed-preset=ultrafast` on `x264enc` minimize latency at some cost to compression efficiency.

If the link is unstable:

- Reduce resolution and FPS.  
- Lower bitrate.  
- Verify HaLow signal strength and channel settings.

---

## 5. Troubleshooting Checklist

- **No `/dev/video0`:**
  - Check camera ribbon cable.  
  - Verify camera firmware/overlay is enabled for Camera Module 3.  
  - Confirm your Ubuntu-on-Pi image includes Pi camera support.

- **Local preview works, but no remote video:**
  - Confirm `base_station_ip` is reachable from the robot (`ping` over HaLow).  
  - Check that the base station firewall allows UDP on `stream1_port`.  
  - Use `tcpdump`/`wireshark` on the base station to see if RTP packets arrive.

- **Stuttering / dropped frames:**
  - Lower `camera1_fps` and `camera1_bitrate`.  
  - Avoid other heavy traffic on the HaLow link.

This guide pairs with the communications package in `qwacr_ws/src/qwacr_comms` and the HaLow configuration file `qwacr_ws/src/qwacr_comms/config/halow_config.yaml`.

---

## Appendix: Observed Errors & Workarounds

This appendix captures issues encountered during on-hardware bring‑up and the current mitigations. It is intentionally low‑level/diagnostic.

### A. MJPEG over HaLow (ad‑hoc tests)

**Symptom:** Base station GStreamer command

```bash
gst-launch-1.0 -v udpsrc port=5000 caps="image/jpeg,framerate=20/1" ! \
  jpegdec ! videoconvert ! autovideosink sync=false
```

shows a single static frame from the Pi, but does not update.

**Robot side command used:**

```bash
rpicam-vid -t 0 --width 1280 --height 720 --framerate 20 \
  --codec mjpeg -o udp://<base_station_ip>:5000
```

**Diagnosis:**

- `rpicam-vid` with `--codec mjpeg -o udp://...` does **not** send standard RTP/JPEG packets; it sends a custom MJPEG stream over UDP.
- Treating the stream as raw `image/jpeg` in GStreamer decodes an initial frame but then misinterprets subsequent packets, resulting in a frozen image.

**What does *not* work reliably:**

- Decoding as RTP/JPEG on the base station:

  ```bash
  gst-launch-1.0 udpsrc port=5000 \
    caps="application/x-rtp,media=video,encoding-name=JPEG,payload=26" ! \
    rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink sync=false
  ```

  This produces repeated warnings such as:

  - `Received invalid RTP payload, dropping`
  - `GstRtpJPEGDepay: Could not decode stream`

  confirming the stream is **not** valid RTP.

**Takeaway:** Use these MJPEG tests only as a quick link sanity‑check; the long‑term, supported path is H.264 over RTP via the `video_publisher` node.

### B. WSL2 vs. native Windows for video receive

**Symptom:** Running the base‑station GStreamer receiver inside WSL2 (Ubuntu) sees no video even though the Pi is sending to the correct HaLow IP.

**Root cause:**

- The Pi targets the Windows host HaLow address (e.g. `10.192.5.100`).
- WSL2 has its own virtual NIC and IP (typically `172.x.x.x`) and does **not** automatically receive arbitrary UDP traffic that arrives on the Windows HaLow interface.
- `netstat` inside WSL only shows local sockets; it does not prove packets are reaching WSL.

**Workaround:**

- Run the GStreamer receiver **on Windows**, not inside WSL, e.g. in PowerShell:

  ```powershell
  gst-launch-1.0 udpsrc port=5000 caps="image/jpeg,framerate=20/1" ^
    ! jpegdec ! videoconvert ! autovideosink sync=false
  ```

- Optionally verify traffic with a Windows packet sniffer (e.g. Wireshark). If video appears only with the firewall disabled, add an inbound rule for UDP port 5000 instead of leaving the firewall off.

### C. GStreamer on Pi: `v4l2src` / `libcamerasrc` errors

While experimenting with running GStreamer **on the Pi** (instead of using only `rpicam-hello` / `rpicam-vid`), the following issues were seen.

1. **`v4l2src` memory / negotiation error**

   Pipeline used:

   ```bash
   gst-launch-1.0 v4l2src device=/dev/video0 ! \
     video/x-raw,width=1280,height=720,framerate=20/1 ! \
     jpegenc ! rtpjpegpay ! \
     udpsink host=<base_station_ip> port=5000 sync=false
   ```

   Errors observed:

   - `GstV4l2Src: Failed to allocate required memory.`
   - `Buffer pool activation failed` followed by
   - `Internal data stream error` and `reason not-negotiated (-4)`.

   **Interpretation:** `v4l2src` cannot set up its buffer pool against the libcamera V4L2 compatibility device for Camera Module 3 on this Ubuntu image. This appears to be a stack/driver interaction problem rather than a HaLow or network issue.

2. **`libcamerasrc` finds no cameras**

   Pipelines such as:

   ```bash
   gst-launch-1.0 libcamerasrc ! \
     video/x-raw,width=1280,height=720,framerate=20/1 ! \
     videoconvert ! autovideosink
   ```

   produce:

   - `libcamera::CameraManager::cameras() is empty`
   - `Could not find any supported camera on this system`

   even though `rpicam-hello` and `rpicam-vid` work.

   **Interpretation:** The GStreamer libcamera plugin is installed, but libcamera (as used by GStreamer) is not enumerating the Pi camera. This may be due to version or configuration mismatches between the system libcamera used by `rpicam-*` and the one used by `libcamerasrc`.

**Current stance:**

- Camera health on the Pi is validated primarily via `rpicam-hello` / `rpicam-vid`.
- HaLow video tests for this mission focus on **base‑station GStreamer receivers** (H.264 or MJPEG) rather than running GStreamer camera sources directly on the Pi.
- Future work may revisit a pure‑GStreamer Pi pipeline if/when the libcamera + GStreamer integration on this Ubuntu image is more predictable.

### D. `rpicam-vid` H.264 codec error on Pi (and YUV → GStreamer workaround)

**Symptom:** Running an H.264 UDP test with `rpicam-vid`, e.g.

```bash
rpicam-vid -t 0 --width 1280 --height 720 --framerate 20 \
  --codec h264 --inline --profile high \
  -o udp://<base_station_ip>:5000
```

prints normal libcamera startup logs and then exits with:

- `ERROR: *** Unable to find an appropriate H.264 codec ***`

**Context / interpretation (on our Ubuntu 24.04 Pi 5 image):**

- `rpicam-apps` is built from source as described in section **2.0**, with the optional libav encoder disabled.
- On this stack, the `--codec h264` path in `rpicam-vid` cannot locate a usable H.264 encoder implementation, so the tool aborts instead of streaming.
- This is a limitation of the camera/codec stack on this image, not of the HaLow link itself.

**Impact for QWACR HaLow video:**

- The ad‑hoc "Path 2" design (Pi running `rpicam-vid --codec h264` and a matching raw H.264 receiver on the base station) is **not** reliable on this Ubuntu 24.04 Pi 5 configuration.
- A **working alternative path** is the YUV‑to‑GStreamer pipeline in section **2.5**, where `rpicam-vid --codec yuv420 -o -` feeds raw frames to a local GStreamer pipeline that performs H.264 encoding with `x264enc` and sends RTP/UDP.
- The mission‑path HaLow video design in `qwacr_comms` mirrors this idea: use a GStreamer camera source (`v4l2src` for `/dev/video*`) followed by `x264enc` + `rtph264pay` and `udpsink`.

**Workarounds / recommendations:**

- Use `rpicam-hello` and `rpicam-still` for basic camera health checks.
- For quick link checks or when debugging, use the validated YUV → GStreamer pipeline in section **2.5**; this has been confirmed to stream Camera Module 3 video over HaLow to the dev machine.
- For ROS‑integrated missions, prefer the `qwacr_comms` pipelines and treat the `rpicam-vid --codec h264` error as an environment limitation to be revisited if/when the Pi image gains a working on‑board H.264 codec.
