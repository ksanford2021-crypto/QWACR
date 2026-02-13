# ESP32 LoRa Firmware

This folder contains a shared LoRa bridge codebase for both robot and base station Heltec boards.

- `robot_side/robot_side.ino`: Robot-side firmware (receives teleop, sends telemetry).
- `base_station/base_station.ino`: Base station firmware (sends teleop, receives telemetry).
- `common/lora_bridge_common.h`: Shared logic and protocol helpers.

## Notes
- Select Arduino IDE or PlatformIO before completing RadioLib integration.
- Define LoRa parameters (frequency, SF, bandwidth) in `setup_radio()`.
- UART framing should match the ROS2 `lora_protocol.py` in this repo.

## Heltec LoRa32 V3 Defaults
This firmware is configured for Heltec LoRa32 V3 (SX1262) with the standard pin map:

- NSS/CS: GPIO 8
- DIO1: GPIO 14
- BUSY: GPIO 13
- RESET: GPIO 12
- SCK: GPIO 9
- MOSI: GPIO 10
- MISO: GPIO 11

If your board differs, override the `LORA_*` macros in the sketch before including
`lora_bridge_common.h`.
