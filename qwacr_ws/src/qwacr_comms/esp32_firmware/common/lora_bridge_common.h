#pragma once

// Common LoRa bridge logic for both robot and base stations.
// Define LORA_ROLE_BASE or LORA_ROLE_ROBOT before including.

#include <Arduino.h>

#if !defined(LORA_ROLE_BASE) && !defined(LORA_ROLE_ROBOT)
#error "Define LORA_ROLE_BASE or LORA_ROLE_ROBOT before including lora_bridge_common.h"
#endif

// RadioLib provides SX1276/SX1278 support for Heltec LoRa32 boards.
// Install it from the Arduino Library Manager.
#include <RadioLib.h>

namespace qwacr_lora {

static const uint8_t HEADER_BYTE = 0xA5;
static const uint8_t MSG_TELEMETRY = 0x01;
static const uint8_t MSG_TELEOP = 0x02;
static const uint8_t MSG_ESTOP = 0xFF;

struct TeleopCommand {
  float linear_x = 0.0f;
  float angular_z = 0.0f;
  uint8_t flags = 0;
};

struct Telemetry {
  float gps_lat = 0.0f;
  float gps_lon = 0.0f;
  uint8_t gps_fix = 0;
  float battery_v = 0.0f;
  float battery_a = 0.0f;
  uint8_t system_status = 0;
  float odom_distance = 0.0f;
  int8_t rssi = 0;
  int8_t snr = 0;
};

inline uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)(data[i] << 8);
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Heltec LoRa32 V3 (SX1262) default pins. Override in your sketch if needed.
#ifndef LORA_NSS
#define LORA_NSS 8
#endif
#ifndef LORA_DIO1
#define LORA_DIO1 14
#endif
#ifndef LORA_RST
#define LORA_RST 12
#endif
#ifndef LORA_BUSY
#define LORA_BUSY 13
#endif
#ifndef LORA_SCK
#define LORA_SCK 9
#endif
#ifndef LORA_MOSI
#define LORA_MOSI 10
#endif
#ifndef LORA_MISO
#define LORA_MISO 11
#endif

// LoRa radio configuration defaults.
#ifndef LORA_FREQUENCY_MHZ
#define LORA_FREQUENCY_MHZ 915.0
#endif
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR 7
#endif
#ifndef LORA_BANDWIDTH_KHZ
#define LORA_BANDWIDTH_KHZ 125.0
#endif
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE 5
#endif
#ifndef LORA_TX_POWER_DBM
#define LORA_TX_POWER_DBM 20
#endif

static const size_t kMaxFrameLen = 3 + 255 + 2;
static const size_t kUartBufferLen = 512;

static uint8_t uart_buffer[kUartBufferLen];
static size_t uart_len = 0;

static uint8_t rx_buffer[kMaxFrameLen];
static volatile bool radio_rx_flag = false;

// Heltec V3 uses SX1262 (ESP32-S3).
static SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

struct ParsedFrame {
  uint8_t msg_id = 0;
  uint8_t length = 0;
  uint8_t payload[255] = {0};
};

inline void on_radio_receive() {
  radio_rx_flag = true;
}

inline size_t build_frame(uint8_t msg_id, const uint8_t *payload, size_t length, uint8_t *out) {
  out[0] = HEADER_BYTE;
  out[1] = msg_id;
  out[2] = static_cast<uint8_t>(length);
  if (length > 0) {
    memcpy(out + 3, payload, length);
  }
  uint16_t crc = crc16_ccitt(out, 3 + length);
  out[3 + length] = static_cast<uint8_t>(crc & 0xFF);
  out[4 + length] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  return 3 + length + 2;
}

inline bool parse_frame(uint8_t *buffer, size_t &length, ParsedFrame &out) {
  if (length < 4) {
    return false;
  }

  size_t start = 0;
  while (start < length && buffer[start] != HEADER_BYTE) {
    ++start;
  }
  if (start >= length) {
    length = 0;
    return false;
  }
  if (start > 0) {
    memmove(buffer, buffer + start, length - start);
    length -= start;
  }
  if (length < 4) {
    return false;
  }

  uint8_t msg_id = buffer[1];
  uint8_t payload_len = buffer[2];
  size_t frame_len = 3 + payload_len + 2;
  if (length < frame_len) {
    return false;
  }

  uint16_t recv_crc = static_cast<uint16_t>(buffer[3 + payload_len]) |
                      (static_cast<uint16_t>(buffer[4 + payload_len]) << 8);
  uint16_t calc_crc = crc16_ccitt(buffer, 3 + payload_len);
  if (recv_crc != calc_crc) {
    memmove(buffer, buffer + 1, length - 1);
    length -= 1;
    return false;
  }

  out.msg_id = msg_id;
  out.length = payload_len;
  if (payload_len > 0) {
    memcpy(out.payload, buffer + 3, payload_len);
  }

  memmove(buffer, buffer + frame_len, length - frame_len);
  length -= frame_len;
  return true;
}

inline void setup_radio() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    return;
  }
  radio.setFrequency(LORA_FREQUENCY_MHZ);
  radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
  radio.setBandwidth(LORA_BANDWIDTH_KHZ);
  radio.setCodingRate(LORA_CODING_RATE);
  radio.setOutputPower(LORA_TX_POWER_DBM);

  radio.setDio1Action(on_radio_receive);
  radio.startReceive();
}

inline void poll_radio() {
  if (!radio_rx_flag) {
    return;
  }
  radio_rx_flag = false;

  int packet_len = radio.getPacketLength();
  if (packet_len <= 0 || packet_len > static_cast<int>(kMaxFrameLen)) {
    radio.startReceive();
    return;
  }

  int state = radio.readData(rx_buffer, packet_len);
  if (state == RADIOLIB_ERR_NONE) {
    uint8_t temp[kMaxFrameLen];
    size_t temp_len = static_cast<size_t>(packet_len);
    memcpy(temp, rx_buffer, temp_len);

    ParsedFrame frame;
    if (parse_frame(temp, temp_len, frame)) {
      Serial.write(rx_buffer, packet_len);
    }
  }

  radio.startReceive();
}

inline void handle_uart() {
  while (Serial.available() > 0 && uart_len < kUartBufferLen) {
    uint8_t byte = static_cast<uint8_t>(Serial.read());
    uart_buffer[uart_len++] = byte;
  }

  ParsedFrame frame;
  while (parse_frame(uart_buffer, uart_len, frame)) {
    uint8_t out[kMaxFrameLen];
    size_t out_len = build_frame(frame.msg_id, frame.payload, frame.length, out);
    
    int state = radio.transmit(out, out_len);
    radio.startReceive();
  }
}

inline void publish_status() {
  // TODO: Update OLED display and status LEDs if desired.
}

}  // namespace qwacr_lora
