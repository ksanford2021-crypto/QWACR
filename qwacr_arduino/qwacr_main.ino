/*
 * QWACR Arduino Motor Control
 * 4-wheel motor PID control with 2x Pololu Dual G2 drivers
 * Full quadrature encoder decoding (6400 CPR)
 * Serial communication with ROS 2 on Raspberry Pi
 * 
 * Motor Layout:
 *   Front-Left (FL)  Front-Right (FR)
 *        |                 |
 *   Back-Left (BL)   Back-Right (BR)
 * 
 * Left motors (FL + BL) share setpoint
 * Right motors (FR + BR) share setpoint
 */

#include "motor_control.h"
#include "serial_protocol.h"
#include "quadrature_decoder.h"

// Motor Front-Left (Pololu G2 Driver 1, Motor A)
#define MOTOR_FL_PWM 9
#define MOTOR_FL_DIR 8
#define ENCODER_FL_A 2      // Interrupt pin (INT4)
#define ENCODER_FL_B 24     // GPIO pin (no interrupt needed)

// Motor Back-Left (Pololu G2 Driver 1, Motor B)
#define MOTOR_BL_PWM 10
#define MOTOR_BL_DIR 7
#define ENCODER_BL_A 18     // Interrupt pin (INT3)
#define ENCODER_BL_B 25     // GPIO pin (no interrupt needed)

// Motor Front-Right (Pololu G2 Driver 2, Motor A)
#define MOTOR_FR_PWM 11
#define MOTOR_FR_DIR 6
#define ENCODER_FR_A 20     // Interrupt pin (INT1)
#define ENCODER_FR_B 26     // GPIO pin (no interrupt needed)

// Motor Back-Right (Pololu G2 Driver 2, Motor B)
#define MOTOR_BR_PWM 12
#define MOTOR_BR_DIR 5
#define ENCODER_BR_A 3      // Interrupt pin (INT5)
#define ENCODER_BR_B 27     // GPIO pin (no interrupt needed)

// Encoder counts per output shaft revolution
// 2x quadrature decoding: 64 base CPR × 100 gear × 2 edges (A channel) = 3200
// (Only channel A has interrupts, channel B read as GPIO for direction)
#define COUNTS_PER_REV 3200

// Motor nominal speed: 90 RPM = 9.42 rad/s
#define MAX_VELOCITY_RAD_S 9.42

// Motor objects with PID parameters (Kp, Ki, Kd)
// Start with conservative tuning, adjust based on motor response
Motor motor_fl(MOTOR_FL_PWM, MOTOR_FL_DIR, COUNTS_PER_REV, 15.0, 1.0, 0.1);
Motor motor_bl(MOTOR_BL_PWM, MOTOR_BL_DIR, COUNTS_PER_REV, 15.0, 1.0, 0.1);
Motor motor_fr(MOTOR_FR_PWM, MOTOR_FR_DIR, COUNTS_PER_REV, 15.0, 1.0, 0.1);
Motor motor_br(MOTOR_BR_PWM, MOTOR_BR_DIR, COUNTS_PER_REV, 15.0, 1.0, 0.1);

// Serial communication
SerialProtocol serial_protocol;

// Timing variables (milliseconds)
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = 500;  // Stop if no command for 500ms

// Encoder counts (volatile for ISR access)
volatile long encoder_fl_count = 0;
volatile long encoder_bl_count = 0;
volatile long encoder_fr_count = 0;
volatile long encoder_br_count = 0;

// Quadrature decoders for all 4 encoders
QuadratureDecoder decoder_fl;
QuadratureDecoder decoder_bl;
QuadratureDecoder decoder_fr;
QuadratureDecoder decoder_br;

// ISR for encoder Front-Left
void encoder_fl_ISR() {
  uint8_t a = digitalRead(ENCODER_FL_A);
  uint8_t b = digitalRead(ENCODER_FL_B);
  int8_t direction = decoder_fl.updateFast(a, b);
  encoder_fl_count += direction;
}

// ISR for encoder Back-Left
void encoder_bl_ISR() {
  uint8_t a = digitalRead(ENCODER_BL_A);
  uint8_t b = digitalRead(ENCODER_BL_B);
  int8_t direction = decoder_bl.updateFast(a, b);
  encoder_bl_count += direction;
}

// ISR for encoder Front-Right
void encoder_fr_ISR() {
  uint8_t a = digitalRead(ENCODER_FR_A);
  uint8_t b = digitalRead(ENCODER_FR_B);
  int8_t direction = decoder_fr.updateFast(a, b);
  encoder_fr_count += direction;
}

// ISR for encoder Back-Right
void encoder_br_ISR() {
  uint8_t a = digitalRead(ENCODER_BR_A);
  uint8_t b = digitalRead(ENCODER_BR_B);
  int8_t direction = decoder_br.updateFast(a, b);
  encoder_br_count += direction;
}

void setup() {
  // Initialize serial communication (115200 baud with RPI)
  serial_protocol.begin(Serial, 115200);
  
  // Motor pins setup
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FL_DIR, OUTPUT);
  pinMode(MOTOR_BL_PWM, OUTPUT);
  pinMode(MOTOR_BL_DIR, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT);
  pinMode(MOTOR_FR_DIR, OUTPUT);
  pinMode(MOTOR_BR_PWM, OUTPUT);
  pinMode(MOTOR_BR_DIR, OUTPUT);
  
  // Encoder pins setup
  pinMode(ENCODER_FL_A, INPUT);
  pinMode(ENCODER_FL_B, INPUT);
  pinMode(ENCODER_BL_A, INPUT);
  pinMode(ENCODER_BL_B, INPUT);
  pinMode(ENCODER_FR_A, INPUT);
  pinMode(ENCODER_FR_B, INPUT);
  pinMode(ENCODER_BR_A, INPUT);
  pinMode(ENCODER_BR_B, INPUT);
  
  // Initialize quadrature decoders
  decoder_fl.begin(ENCODER_FL_A, ENCODER_FL_B);
  decoder_bl.begin(ENCODER_BL_A, ENCODER_BL_B);
  decoder_fr.begin(ENCODER_FR_A, ENCODER_FR_B);
  decoder_br.begin(ENCODER_BR_A, ENCODER_BR_B);
  
  // Attach interrupts for encoders (CHANGE captures all edge transitions)
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), encoder_fl_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BL_A), encoder_bl_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), encoder_fr_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BR_A), encoder_br_ISR, CHANGE);
  
  // Stop all motors initially
  motor_fl.stop();
  motor_bl.stop();
  motor_fr.stop();
  motor_br.stop();
  
  delay(100);
}

void loop() {
  unsigned long current_time = millis();
  
  // Update PacketSerial to process incoming data
  serial_protocol.update();
  
  // Check for incoming serial commands
  Command cmd = serial_protocol.parseCommand(Serial);
  
  if (cmd.valid) {
      last_command_time = current_time;
      
      // Set velocity setpoints for PID (in rad/s)
      // Clamp to motor limits (±90 RPM = ±9.42 rad/s)
      double vel_left_clamped = constrain(cmd.vel_left, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);
      double vel_right_clamped = constrain(cmd.vel_right, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);
      
      // Left motors get vel_left, right motors get vel_right
      motor_fl.setVelocity(vel_left_clamped);
      motor_bl.setVelocity(vel_left_clamped);
      motor_fr.setVelocity(vel_right_clamped);
      motor_br.setVelocity(vel_right_clamped);
  }
  
  // Timeout safety: stop motors if no command received
  if (current_time - last_command_time > COMMAND_TIMEOUT) {
    motor_fl.stop();
    motor_bl.stop();
    motor_fr.stop();
    motor_br.stop();
  }
  
  // Update PID controllers (call frequently for smooth control)
  // Each motor tracks its own encoder and computes independent PWM
  motor_fl.updatePID(encoder_fl_count);
  motor_bl.updatePID(encoder_bl_count);
  motor_fr.updatePID(encoder_fr_count);
  motor_br.updatePID(encoder_br_count);
  
  // Send feedback back to RPI at ~50Hz (every 20ms)
  static unsigned long last_feedback_time = 0;
  if (current_time - last_feedback_time >= 20) {
    last_feedback_time = current_time;
    
    // Get current velocities from PID calculations (rad/s)
    double vel_fl = motor_fl.getVelocity();
    double vel_bl = motor_bl.getVelocity();
    double vel_fr = motor_fr.getVelocity();
    double vel_br = motor_br.getVelocity();
    
    // Send feedback packet (all 4 motors)
    serial_protocol.sendFeedback(Serial, 
                                 encoder_fl_count, encoder_bl_count, 
                                 encoder_fr_count, encoder_br_count,
                                 vel_fl, vel_bl, vel_fr, vel_br);
  }
  
  // Small delay to prevent overwhelming the loop
  delayMicroseconds(100);
}
