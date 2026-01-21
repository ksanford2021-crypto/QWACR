/*
 * Serial Protocol for ROS 2 / Arduino Communication
 * Uses PacketSerial library for reliable packet framing with COBS encoding
 * Supports 4-motor configuration (left/right setpoints)
 * 
 * Benefits of PacketSerial:
 * - Automatic packet framing with COBS encoding
 * - No manual start/end markers needed
 * - Handles corrupted data gracefully
 * - Industry-standard approach
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <PacketSerial.h>

// Command structure from ROS 2
struct Command {
  bool valid;
  double vel_left;     // rad/s for left motors (FL + BL)
  double vel_right;    // rad/s for right motors (FR + BR)
};

// Feedback structure to ROS 2
struct Feedback {
  long encoder_fl;
  long encoder_bl;
  long encoder_fr;
  long encoder_br;
  double velocity_fl;
  double velocity_bl;
  double velocity_fr;
  double velocity_br;
};

class SerialProtocol {
private:
  PacketSerial packetSerial;
  
  // Packet type IDs
  static const byte CMD_SET_VELOCITY = 0x01;
  static const byte CMD_REQUEST_FEEDBACK = 0x02;
  static const byte FEEDBACK_DATA = 0x10;
  
  // Command storage
  bool command_ready;
  bool feedback_requested;
  double cmd_vel_left;
  double cmd_vel_right;
  
  static SerialProtocol* instance;
  
  static void onPacketReceived(const uint8_t* buffer, size_t size) {
    if (instance) {
      instance->handlePacket(buffer, size);
    }
  }
  
  void handlePacket(const uint8_t* buffer, size_t size) {
    if (size >= 9 && buffer[0] == CMD_SET_VELOCITY) {
      // Parse: [TYPE][vel_left_float][vel_right_float]
      memcpy(&cmd_vel_left, &buffer[1], 4);
      memcpy(&cmd_vel_right, &buffer[5], 4);
      command_ready = true;
    }
    else if (size >= 1 && buffer[0] == CMD_REQUEST_FEEDBACK) {
      // Request for encoder feedback: [TYPE]
      feedback_requested = true;
    }
  }
  
public:
  SerialProtocol() : command_ready(false), feedback_requested(false), cmd_vel_left(0), cmd_vel_right(0) {
    instance = this;
  }
  
  // Initialize serial communication
  void begin(HardwareSerial& serial, unsigned long baudrate) {
    serial.begin(baudrate);
    packetSerial.setStream(&serial);
    packetSerial.setPacketHandler(&onPacketReceived);
  }
  
  // Update - call frequently to process incoming packets
  void update() {
    packetSerial.update();
    
    // Check for received packets
    if (packetSerial.overflow()) {
      // Handle overflow if needed (packet too large)
    }
  }
  
  // Parse incoming command from serial
  // Call this after update() to check for new commands
  Command parseCommand(HardwareSerial& serial) {
    Command cmd;
    cmd.valid = command_ready;
    cmd.vel_left = cmd_vel_left;
    cmd.vel_right = cmd_vel_right;
    command_ready = false;  // Clear flag after reading
    return cmd;
  }
  
  // Check if feedback was requested
  bool isFeedbackRequested() {
    bool requested = feedback_requested;
    feedback_requested = false;  // Clear flag after reading
    return requested;
  }
  
  // Send feedback packet to serial
  // Format: [TYPE][enc_fl][enc_bl][enc_fr][enc_br][vel_fl][vel_bl][vel_fr][vel_br]
  void sendFeedback(HardwareSerial& serial, 
                    long encoder_fl, long encoder_bl, long encoder_fr, long encoder_br,
                    double velocity_fl, double velocity_bl, double velocity_fr, double velocity_br) {
    byte buffer[33];  // 1 type + 16 bytes encoders + 16 bytes velocities
    int idx = 0;
    
    buffer[idx++] = FEEDBACK_DATA;
    
    // Encoders (4 bytes each as int32)
    memcpy(&buffer[idx], &encoder_fl, 4); idx += 4;
    memcpy(&buffer[idx], &encoder_bl, 4); idx += 4;
    memcpy(&buffer[idx], &encoder_fr, 4); idx += 4;
    memcpy(&buffer[idx], &encoder_br, 4); idx += 4;
    
    // Velocities (4 bytes each as float)
    float vel_fl_f = (float)velocity_fl;
    float vel_bl_f = (float)velocity_bl;
    float vel_fr_f = (float)velocity_fr;
    float vel_br_f = (float)velocity_br;
    
    memcpy(&buffer[idx], &vel_fl_f, 4); idx += 4;
    memcpy(&buffer[idx], &vel_bl_f, 4); idx += 4;
    memcpy(&buffer[idx], &vel_fr_f, 4); idx += 4;
    memcpy(&buffer[idx], &vel_br_f, 4); idx += 4;
    
    packetSerial.send(buffer, idx);
  }
};

SerialProtocol* SerialProtocol::instance = nullptr;

#endif
