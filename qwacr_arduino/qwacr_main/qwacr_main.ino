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

// Motor Front-Left (Pololu G2 Driver 1, Motor A)
#define MOTOR_FL_PWM 9
#define MOTOR_FL_DIR 8
// Use high-numbered GPIOs for encoder B channels to keep them physically separated
#define ENCODER_FL_A 2      // Interrupt pin (INT4)
#define ENCODER_FL_B 41     // GPIO pin (no interrupt needed)

// Motor Back-Left (Pololu G2 Driver 1, Motor B)
#define MOTOR_BL_PWM 10
#define MOTOR_BL_DIR 7
#define ENCODER_BL_A 18     // Interrupt pin (INT3)
#define ENCODER_BL_B 45     // GPIO pin (no interrupt needed)

// Motor Front-Right (Pololu G2 Driver 2, Motor A)
#define MOTOR_FR_PWM 11
#define MOTOR_FR_DIR 6
#define ENCODER_FR_A 20     // Interrupt pin (INT1)
#define ENCODER_FR_B 49     // GPIO pin (no interrupt needed)

// Motor Back-Right (Pololu G2 Driver 2, Motor B)
#define MOTOR_BR_PWM 12
#define MOTOR_BR_DIR 5
#define ENCODER_BR_A 3      // Interrupt pin (INT5)
#define ENCODER_BR_B 53     // GPIO pin (no interrupt needed)

// Motor Driver Sleep Pins (active HIGH to enable, LOW to sleep)
// Driver 1 (Left motors FL + BL)
#define DRIVER_1_M1SLP 37   // M1SLP for FL motor (Driver 1)
#define DRIVER_1_M2SLP 29   // M2SLP for BL motor (Driver 1)

// Driver 2 (Right motors FR + BR)
#define DRIVER_2_M1SLP 30   // M1SLP for FR motor (Driver 2)
#define DRIVER_2_M2SLP 31   // M2SLP for BR motor (Driver 2)

// Motor Driver Current Sense Pins (analog inputs)
// Driver 1 (Left motors FL + BL)
#define DRIVER_1_M1CS A0    // Current sense for FL motor (Driver 1 M1CS)
#define DRIVER_1_M2CS A1    // Current sense for BL motor (Driver 1 M2CS)

// Driver 2 (Right motors FR + BR)
#define DRIVER_2_M1CS A2    // Current sense for FR motor (Driver 2 M1CS)
#define DRIVER_2_M2CS A3    // Current sense for BR motor (Driver 2 M2CS)

// Encoder counts per output shaft revolution
// 2x quadrature decoding: 64 base CPR × 100 gear × 2 edges (A channel) = 3200
// (Only channel A has interrupts, channel B read as GPIO for direction)
#define COUNTS_PER_REV 3200

// Motor nominal speed: 90 RPM = 9.42 rad/s
#define MAX_VELOCITY_RAD_S 9.42

// Pololu Dual G2 24v14 current sense characteristics
// Current sense output ~20 mV/A, with ~50 mV offset
// Arduino Mega ADC: 0-5V mapped to 0-1023
const float CS_V_PER_A = 0.020f;      // 20 mV/A
const float CS_OFFSET_V = 0.05f;      // ~50 mV offset
const float ADC_REF_V = 5.0f;         // ADC reference voltage
const float CURRENT_LIMIT_A = 2.6f;   // Nominal max current per motor channel (straight driving)
const float CURRENT_LIMIT_TURN_BOOST_A = 5.0f; // Higher limit during turn boost for skid-steer turns

// Motor objects with PID parameters (Kp, Ki, Kd, Kff)
// Tuned for more torque but still smoother than original: Kff=32, Kp=18, Ki=0.2, Kd=0.05
Motor motor_fl(MOTOR_FL_PWM, MOTOR_FL_DIR, COUNTS_PER_REV, 18.0, 0.2, 0.05, 32.0);
Motor motor_bl(MOTOR_BL_PWM, MOTOR_BL_DIR, COUNTS_PER_REV, 18.0, 0.2, 0.05, 32.0);
Motor motor_fr(MOTOR_FR_PWM, MOTOR_FR_DIR, COUNTS_PER_REV, 18.0, 0.2, 0.05, 32.0);
Motor motor_br(MOTOR_BR_PWM, MOTOR_BR_DIR, COUNTS_PER_REV, 18.0, 0.2, 0.05, 32.0);

// Serial communication
SerialProtocol serial_protocol;

// Timing variables (milliseconds)
unsigned long last_command_time = 0;
// Stop and put drivers to sleep quickly if no command is received
// This reduces idle "ghost" noise from the motor drivers.
const unsigned long COMMAND_TIMEOUT = 500;  // milliseconds

// Motor enable flag - prevents PID output until first command received
boolean motors_enabled = false;

// Ramped velocity targets (rad/s) and acceleration limiting
double target_vel_left = 0.0;
double target_vel_right = 0.0;
unsigned long last_ramp_update_time = 0;
// Max change in wheel velocity per second (rad/s^2)
const double MAX_ACCEL_RAD_S2 = 8.0;       // normal accel for straight driving
const double MAX_ACCEL_TURN_RAD_S2 = 14.0; // slightly snappier accel when turning in place

// Temporary current-limit boost for skid-steer turns
bool turn_boost_active = false;
unsigned long turn_boost_start_time = 0;
const unsigned long TURN_BOOST_DURATION_MS = 250;  // Allow extra current for first 0.25 s of a turn

// Debug: count bytes received to detect serial data flow
unsigned long bytes_received = 0;
unsigned long last_byte_count = 0;

// Encoder counts (volatile for ISR access)
volatile long encoder_fl_count = 0;
volatile long encoder_bl_count = 0;
volatile long encoder_fr_count = 0;
volatile long encoder_br_count = 0;

// Helper to read motor current (in amps) from a current sense analog pin
float readMotorCurrentA(int cs_pin)
{
  int raw = analogRead(cs_pin);
  float voltage = (static_cast<float>(raw) * ADC_REF_V) / 1023.0f;
  float current = (voltage - CS_OFFSET_V) / CS_V_PER_A;
  if (current < 0.0f) {
    current = 0.0f;
  }
  return current;
}

// Quadrature decoders for all 4 encoders
// A-edge counting with direction from channel B using XOR logic:
// If A and B are different → forward, same → reverse (adjust polarity to match motor direction)
void encoder_fl_ISR() { 
  encoder_fl_count += (digitalRead(ENCODER_FL_A) == digitalRead(ENCODER_FL_B)) ? -1 : 1; 
}
void encoder_bl_ISR() { 
  encoder_bl_count += (digitalRead(ENCODER_BL_A) == digitalRead(ENCODER_BL_B)) ? -1 : 1; 
}
void encoder_fr_ISR() { 
  encoder_fr_count += (digitalRead(ENCODER_FR_A) == digitalRead(ENCODER_FR_B)) ? -1 : 1; 
}
void encoder_br_ISR() { 
  encoder_br_count += (digitalRead(ENCODER_BR_A) == digitalRead(ENCODER_BR_B)) ? -1 : 1; 
}

void setup() {
  // Built-in LED for command indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize serial communication (115200 baud with RPI)
  Serial.begin(115200);
  delay(100);
  serial_protocol.begin(Serial, 115200);
  
  // Motor driver sleep pins - keep drivers DISABLED (LOW) until first command
  pinMode(DRIVER_1_M1SLP, OUTPUT);
  pinMode(DRIVER_1_M2SLP, OUTPUT);
  pinMode(DRIVER_2_M1SLP, OUTPUT);
  pinMode(DRIVER_2_M2SLP, OUTPUT);
  digitalWrite(DRIVER_1_M1SLP, LOW);  // Sleep FL motor
  digitalWrite(DRIVER_1_M2SLP, LOW);  // Sleep BL motor
  digitalWrite(DRIVER_2_M1SLP, LOW);  // Sleep FR motor
  digitalWrite(DRIVER_2_M2SLP, LOW);  // Sleep BR motor
  
  // Motor pins setup - explicitly set all to LOW
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FL_DIR, OUTPUT);
  digitalWrite(MOTOR_FL_PWM, LOW);
  digitalWrite(MOTOR_FL_DIR, LOW);
  analogWrite(MOTOR_FL_PWM, 0);
  
  pinMode(MOTOR_BL_PWM, OUTPUT);
  pinMode(MOTOR_BL_DIR, OUTPUT);
  digitalWrite(MOTOR_BL_PWM, LOW);
  digitalWrite(MOTOR_BL_DIR, LOW);
  analogWrite(MOTOR_BL_PWM, 0);
  
  pinMode(MOTOR_FR_PWM, OUTPUT);
  pinMode(MOTOR_FR_DIR, OUTPUT);
  digitalWrite(MOTOR_FR_PWM, LOW);
  digitalWrite(MOTOR_FR_DIR, LOW);
  analogWrite(MOTOR_FR_PWM, 0);
  
  pinMode(MOTOR_BR_PWM, OUTPUT);
  pinMode(MOTOR_BR_DIR, OUTPUT);
  digitalWrite(MOTOR_BR_PWM, LOW);
  digitalWrite(MOTOR_BR_DIR, LOW);
  analogWrite(MOTOR_BR_PWM, 0);
  
  // Encoder pins setup (push-pull outputs from encoders; no pull-ups needed)
  pinMode(ENCODER_FL_A, INPUT);
  pinMode(ENCODER_FL_B, INPUT);
  pinMode(ENCODER_BL_A, INPUT);
  pinMode(ENCODER_BL_B, INPUT);
  pinMode(ENCODER_FR_A, INPUT);
  pinMode(ENCODER_FR_B, INPUT);
  pinMode(ENCODER_BR_A, INPUT);
  pinMode(ENCODER_BR_B, INPUT);

  // Current sense pins (analog inputs)
  pinMode(DRIVER_1_M1CS, INPUT);
  pinMode(DRIVER_1_M2CS, INPUT);
  pinMode(DRIVER_2_M1CS, INPUT);
  pinMode(DRIVER_2_M2CS, INPUT);
  
  // Initialize quadrature decoders
  
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
  
  // Initialize last_command_time to trigger immediate timeout safety
  last_command_time = millis();
  
  delay(100);
}

void loop() {
  unsigned long current_time = millis();
  
  // Update PacketSerial to process incoming data (don't read manually!)
  serial_protocol.update();
  
  // Check for incoming serial commands
  Command cmd = serial_protocol.parseCommand(Serial);
  
  if (cmd.valid) {
      // Blink LED to show command received
      digitalWrite(LED_BUILTIN, HIGH);
      
      // Detect sign change in setpoint to reset integral windup
      static double last_left_setpoint = 0;
      if ((cmd.vel_left >= 0 && last_left_setpoint < 0) || 
          (cmd.vel_left < 0 && last_left_setpoint >= 0)) {
        // Sign change - stop motors momentarily to reset
        motor_fl.stop();
        motor_bl.stop();
      }
      last_left_setpoint = cmd.vel_left;
      
      last_command_time = current_time;
      motors_enabled = true;  // Enable motors after first command
      
      // Wake up all motor driver channels on first command (set sleep pins HIGH)
      digitalWrite(DRIVER_1_M1SLP, HIGH);  // Wake FL motor
      digitalWrite(DRIVER_1_M2SLP, HIGH);  // Wake BL motor
      digitalWrite(DRIVER_2_M1SLP, HIGH);  // Wake FR motor
      digitalWrite(DRIVER_2_M2SLP, HIGH);  // Wake BR motor
      
      // Clamp commanded velocities to motor limits (±90 RPM = ±9.42 rad/s)
      double vel_left_clamped = constrain(cmd.vel_left, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);
      double vel_right_clamped = constrain(cmd.vel_right, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);

      // Update ramp targets instead of instantly stepping setpoints
      target_vel_left = vel_left_clamped;
      target_vel_right = vel_right_clamped;

      // Detect strong skid-steer turns (left/right wheels commanding opposite directions)
      bool turning_in_place =
        ((vel_left_clamped > 0.3 && vel_right_clamped < -0.3) ||
         (vel_left_clamped < -0.3 && vel_right_clamped > 0.3));

      if (turning_in_place) {
        turn_boost_active = true;
        turn_boost_start_time = current_time;
      }
  }

  // Apply velocity ramping toward latest targets to avoid current spikes
  if (motors_enabled) {
    if (last_ramp_update_time == 0) {
      last_ramp_update_time = current_time;
    }

    double dt = (current_time - last_ramp_update_time) / 1000.0;  // seconds
    if (dt > 0.0) {
      if (dt > 0.5) dt = 0.5;  // clamp for safety

      double accel_limit = turn_boost_active ? MAX_ACCEL_TURN_RAD_S2 : MAX_ACCEL_RAD_S2;
      double max_step = accel_limit * dt;

      auto rampToward = [max_step](double current_val, double target_val) -> double {
        double delta = target_val - current_val;
        if (delta > max_step) delta = max_step;
        else if (delta < -max_step) delta = -max_step;
        return current_val + delta;
      };

      double current_left_setpoint = motor_fl.getSetpoint();
      double current_right_setpoint = motor_fr.getSetpoint();

      double new_left = rampToward(current_left_setpoint, target_vel_left);
      double new_right = rampToward(current_right_setpoint, target_vel_right);

      motor_fl.setVelocity(new_left);
      motor_bl.setVelocity(new_left);
      motor_fr.setVelocity(new_right);
      motor_br.setVelocity(new_right);

      last_ramp_update_time = current_time;
    }
  }
  
  // Timeout safety: stop motors if no command received
  if (!motors_enabled || (current_time - last_command_time > COMMAND_TIMEOUT)) {
    motor_fl.stop();
    motor_bl.stop();
    motor_fr.stop();
    motor_br.stop();
    // Return drivers to sleep so SLP reads low when idle/no commands
    digitalWrite(DRIVER_1_M1SLP, LOW);
    digitalWrite(DRIVER_1_M2SLP, LOW);
    digitalWrite(DRIVER_2_M1SLP, LOW);
    digitalWrite(DRIVER_2_M2SLP, LOW);
    digitalWrite(LED_BUILTIN, LOW);  // Turn off LED when motors disabled
    motors_enabled = false;
    // Reset ramping and turn-boost state when motors are disabled
    target_vel_left = 0.0;
    target_vel_right = 0.0;
    last_ramp_update_time = 0;
    turn_boost_active = false;
  }
  
  // Monitor motor currents and adjust per-motor PWM scale so current stays below limit
  if (motors_enabled) {
    // Update temporary turn-boost window
    if (turn_boost_active && (current_time - turn_boost_start_time > TURN_BOOST_DURATION_MS)) {
      turn_boost_active = false;
    }

    float effective_current_limit = turn_boost_active ? CURRENT_LIMIT_TURN_BOOST_A : CURRENT_LIMIT_A;

    float current_fl = readMotorCurrentA(DRIVER_1_M1CS);
    float current_bl = readMotorCurrentA(DRIVER_1_M2CS);
    float current_fr = readMotorCurrentA(DRIVER_2_M1CS);
    float current_br = readMotorCurrentA(DRIVER_2_M2CS);

    auto computeScale = [effective_current_limit](float current) -> double {
      if (current <= 0.0f || current <= effective_current_limit) {
        return 1.0;
      }
      double scale = static_cast<double>(effective_current_limit / current);
      // Prevent scale from dropping too low so the motor can still produce torque
      if (scale < 0.4) scale = 0.4;
      if (scale > 1.0) scale = 1.0;
      return scale;
    };

    motor_fl.setOutputScale(computeScale(current_fl));
    motor_bl.setOutputScale(computeScale(current_bl));
    motor_fr.setOutputScale(computeScale(current_fr));
    motor_br.setOutputScale(computeScale(current_br));
  }

  // Update PID if motors are enabled (PWM scaling applied inside Motor)
  if (motors_enabled) {
    motor_fl.updatePID(encoder_fl_count);
    motor_bl.updatePID(encoder_bl_count);
    motor_fr.updatePID(encoder_fr_count);
    motor_br.updatePID(encoder_br_count);
  }
  
  // Handle feedback requests (request/response pattern - no blocking)
  if (serial_protocol.isFeedbackRequested()) {
    // Get current velocities from PID calculations (rad/s)
    double vel_fl = motor_fl.getVelocity();
    double vel_bl = motor_bl.getVelocity();
    double vel_fr = motor_fr.getVelocity();
    double vel_br = motor_br.getVelocity();

    // Read latest currents (amps) for each motor channel
    float cur_fl = readMotorCurrentA(DRIVER_1_M1CS);
    float cur_bl = readMotorCurrentA(DRIVER_1_M2CS);
    float cur_fr = readMotorCurrentA(DRIVER_2_M1CS);
    float cur_br = readMotorCurrentA(DRIVER_2_M2CS);
    
    // Send feedback packet immediately in response to query
    serial_protocol.sendFeedback(Serial, 
                   encoder_fl_count, encoder_bl_count, 
                   encoder_fr_count, encoder_br_count,
                   vel_fl, vel_bl, vel_fr, vel_br,
                   cur_fl, cur_bl, cur_fr, cur_br);
  }
  
  // Small delay to prevent overwhelming the loop
  delayMicroseconds(100);
}
