/*
 * Motor Control Library with PID
 * Handles PWM output and PID velocity control
 * Uses Arduino PID library for robust control
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <PID_v1.h>

class Motor {
private:
  // Motor pins
  int pwm_pin;
  int dir_pin;
  
  // Encoder configuration
  int counts_per_rev;
  
  // PID variables (must be doubles for PID library)
  double setpoint;           // Desired velocity (rad/s)
  double input;              // Measured velocity (rad/s)
  double output;             // PID output (-255 to 255)
  
  // PID controller object
  PID* pid_controller;
  
  // Timing
  unsigned long last_update_time;
  long last_encoder_count;
  
  // Constraints
  const float MAX_PWM = 255.0;
  
public:
  Motor(int pwm, int dir, int cpr, double kp_val, double ki_val, double kd_val)
    : pwm_pin(pwm), dir_pin(dir), counts_per_rev(cpr),
      setpoint(0), input(0), output(0),
      last_update_time(0), last_encoder_count(0) {
    
    // Initialize PID controller
    // PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
    pid_controller = new PID(&input, &output, &setpoint, kp_val, ki_val, kd_val, DIRECT);
    
    // Set output limits to match PWM range
    pid_controller->SetOutputLimits(-MAX_PWM, MAX_PWM);
    
    // Set sample time to 20ms (50Hz control loop)
    pid_controller->SetSampleTime(20);
    
    // Set PID to automatic mode
    pid_controller->SetMode(AUTOMATIC);
  }
  
  ~Motor() {
    delete pid_controller;
  }
  
  // Set desired velocity (rad/s)
  void setVelocity(double velocity_rad_s) {
    setpoint = velocity_rad_s;
  }
  
  // Stop the motor
  void stop() {
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin, LOW);
    setpoint = 0;
    input = 0;
    output = 0;
  }
  
  // Tune PID parameters on the fly
  void tunePID(double kp, double ki, double kd) {
    pid_controller->SetTunings(kp, ki, kd);
  }
  
  // Update PID controller and set motor PWM
  // encoder_count: current absolute encoder count
  void updatePID(long encoder_count) {
    unsigned long current_time = millis();
    
    // First call initialization
    if (last_update_time == 0) {
      last_update_time = current_time;
      last_encoder_count = encoder_count;
      return;
    }
    
    // Calculate time delta (in seconds)
    float dt = (current_time - last_update_time) / 1000.0;
    if (dt < 0.001) return;  // Skip if called too quickly
    if (dt > 0.5) dt = 0.5;  // Limit dt for stability
    
    // Calculate actual velocity from encoder change
    long count_delta = encoder_count - last_encoder_count;
    last_encoder_count = encoder_count;
    last_update_time = current_time;
    
    // Convert counts to radians per second
    // count_delta / counts_per_rev = revolutions
    // revolutions * 2*pi = radians
    // radians / dt = rad/s
    input = (count_delta / (double)counts_per_rev) * 2.0 * PI / dt;
    
    // Run PID computation
    pid_controller->Compute();
    
    // Apply motor output
    applyMotorOutput(output);
  }
  
  // Get current measured velocity (rad/s)
  double getVelocity() {
    return input;
  }
  
  // Get current setpoint (rad/s)
  double getSetpoint() {
    return setpoint;
  }
  
  // Get current PID output
  double getOutput() {
    return output;
  }

private:
  // Apply PID output to motor (direction and PWM)
  void applyMotorOutput(double pwm_value) {
    if (pwm_value >= 0) {
      digitalWrite(dir_pin, HIGH);  // Forward
      analogWrite(pwm_pin, (int)constrain(pwm_value, 0, MAX_PWM));
    } else {
      digitalWrite(dir_pin, LOW);   // Reverse
      analogWrite(pwm_pin, (int)constrain(-pwm_value, 0, MAX_PWM));
    }
  }
};

#endif
