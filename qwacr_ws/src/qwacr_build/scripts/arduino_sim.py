#!/usr/bin/env python3
"""
Arduino Uno simulator for 4-wheel differential drive robot.
Simulates PID control for each motor with encoder feedback.

Protocol:
- Receives: vel1,vel2,vel3,vel4 (target velocities in rad/s)
- Sends: pos1,pos2,pos3,pos4,vel1,vel2,vel3,vel4 (encoder positions in rad and velocities in rad/s)
"""

import os
import pty
import time
import errno
import select
import sys

# Allow serial port to be specified via environment variable or use default
MASTER_SLAVE_SYMLINK = os.environ.get('SERIAL_PORT', '/tmp/ttyV0')

# PID parameters (tuned for simulated motors)
KP = 2.0  # Proportional gain
KI = 0.5  # Integral gain
KD = 0.1  # Derivative gain

# Motor simulation parameters
MAX_ACCEL = 10.0  # rad/s^2 - maximum angular acceleration
FRICTION = 0.5    # Friction coefficient (velocity damping)
INERTIA = 0.1     # Motor+wheel inertia

class MotorPID:
    """Simple PID controller for one motor."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, setpoint, measured, dt):
        """Compute PID output (PWM equivalent as acceleration)."""
        error = setpoint - measured
        self.integral += error * dt
        # Anti-windup
        self.integral = max(-10.0, min(10.0, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

class Motor:
    """Simulates a DC motor with encoder."""
    def __init__(self, pid_controller):
        self.position = 0.0  # rad
        self.velocity = 0.0  # rad/s
        self.pid = pid_controller
        self.target_velocity = 0.0  # rad/s
        
    def update(self, dt):
        """Update motor physics simulation."""
        # PID computes desired acceleration
        pid_output = self.pid.compute(self.target_velocity, self.velocity, dt)
        
        # Limit acceleration
        acceleration = max(-MAX_ACCEL, min(MAX_ACCEL, pid_output))
        
        # Apply physics: friction and acceleration
        friction_force = -FRICTION * self.velocity
        total_accel = acceleration + friction_force / INERTIA
        
        # Update velocity and position
        self.velocity += total_accel * dt
        self.position += self.velocity * dt
        
    def set_target(self, velocity):
        """Set target velocity from ROS2 command."""
        self.target_velocity = velocity

def main():
    # Open PTY
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)

    # Create symlink
    try:
        if os.path.islink(MASTER_SLAVE_SYMLINK) or os.path.exists(MASTER_SLAVE_SYMLINK):
            os.remove(MASTER_SLAVE_SYMLINK)
        os.symlink(slave_name, MASTER_SLAVE_SYMLINK)
    except OSError as e:
        print(f"Failed to create symlink {MASTER_SLAVE_SYMLINK} -> {slave_name}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Arduino simulator running. Device: {MASTER_SLAVE_SYMLINK} -> {slave_name}")
    print(f"PID gains: Kp={KP}, Ki={KI}, Kd={KD}")
    print("Arduino simulator ready for connections", file=sys.stderr)
    sys.stderr.flush()
    
    # Wait a moment to ensure hardware is ready
    time.sleep(0.5)

    # Initialize 4 motors with PID controllers
    motors = [Motor(MotorPID(KP, KI, KD)) for _ in range(4)]
    
    # Timing
    last_time = time.time()
    update_rate = 50  # Hz (Arduino loop rate)
    dt = 1.0 / update_rate
    
    # Input buffer for commands from ROS2
    input_buffer = ""
    
    # Set master to non-blocking
    import fcntl
    flags = fcntl.fcntl(master, fcntl.F_GETFL)
    fcntl.fcntl(master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    
    try:
        while True:
            current_time = time.time()
            elapsed = current_time - last_time
            
            # Check for incoming commands from ROS2
            try:
                data = os.read(master, 1024)
                if data:
                    input_buffer += data.decode('ascii', errors='ignore')
                    
                    # Process complete lines
                    while '\n' in input_buffer:
                        line, input_buffer = input_buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Parse command: vel1,vel2,vel3,vel4
                        try:
                            velocities = [float(v) for v in line.split(',')]
                            if len(velocities) == 4:
                                for i, vel in enumerate(velocities):
                                    motors[i].set_target(vel)
                                print(f"Received commands: {velocities}", file=sys.stderr)
                        except ValueError as e:
                            print(f"Failed to parse command: {line} - {e}", file=sys.stderr)
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    raise
            
            # Update motors at fixed rate
            if elapsed >= dt:
                for motor in motors:
                    motor.update(dt)
                last_time = current_time
                
                # Send encoder data back to ROS2
                # Format: pos1,pos2,pos3,pos4,vel1,vel2,vel3,vel4
                positions = [f"{m.position:.6f}" for m in motors]
                velocities = [f"{m.velocity:.6f}" for m in motors]
                packet = ','.join(positions + velocities) + '\n'
                
                try:
                    written = 0
                    while written < len(packet):
                        try:
                            n = os.write(master, packet[written:].encode('ascii'))
                            written += n
                        except OSError as e:
                            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                                # Slave not ready, retry after small delay
                                time.sleep(0.001)
                            else:
                                raise
                except OSError as e:
                    if e.errno == errno.EIO:
                        print("Slave closed, exiting simulator", file=sys.stderr)
                        break
                    else:
                        raise
            
            # Small sleep to prevent busy-waiting
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nSimulator interrupted", file=sys.stderr)
    finally:
        try:
            os.remove(MASTER_SLAVE_SYMLINK)
        except OSError:
            pass
        os.close(master)
        os.close(slave)
        print('Arduino simulator exiting', file=sys.stderr)

if __name__ == '__main__':
    main()
