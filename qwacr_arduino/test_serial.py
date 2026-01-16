#!/usr/bin/env python3
"""
Serial Protocol Test Utility for Arduino Motor Control
Tests communication with Arduino Mega running qwacr_main.ino
"""

import serial
import struct
import time
import argparse

class ArduinoSerialClient:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1.0)
        time.sleep(2)  # Wait for Arduino to reset
        print(f"Connected to {port} at {baudrate} baud")
    
    def send_command(self, vel_motor1, vel_motor2):
        """
        Send velocity commands to Arduino
        vel_motor1, vel_motor2: velocities in rad/s
        """
        # Pack: [START][vel_m1_float][vel_m2_float][END]
        command = bytes([0xAA])  # START
        command += struct.pack('<f', vel_motor1)  # Motor 1 velocity (4 bytes, little-endian float)
        command += struct.pack('<f', vel_motor2)  # Motor 2 velocity (4 bytes, little-endian float)
        command += bytes([0x55])  # END
        
        self.serial.write(command)
        print(f"Sent: Motor1={vel_motor1:.2f} rad/s, Motor2={vel_motor2:.2f} rad/s")
    
    def read_feedback(self):
        """
        Read feedback from Arduino
        Returns: (encoder1, encoder2, velocity1, velocity2) or None if no valid packet
        """
        while self.serial.in_waiting >= 17:  # Min packet size: START(1) + enc1(4) + enc2(4) + vel1(4) + vel2(4) + END(1)
            byte = self.serial.read(1)
            
            if byte == b'\xBB':  # FEEDBACK_START
                data = self.serial.read(16)
                
                if len(data) == 16 and data[-1:] == b'\x66':  # FEEDBACK_END
                    encoder1 = struct.unpack('<i', data[0:4])[0]
                    encoder2 = struct.unpack('<i', data[4:8])[0]
                    velocity1 = struct.unpack('<f', data[8:12])[0]
                    velocity2 = struct.unpack('<f', data[12:16])[0]
                    
                    return (encoder1, encoder2, velocity1, velocity2)
        
        return None
    
    def close(self):
        self.serial.close()


def test_basic_communication():
    """Test basic send/receive"""
    client = ArduinoSerialClient()
    
    try:
        print("\n=== Testing Basic Communication ===")
        
        # Send stop command
        print("\n1. Sending STOP command (0 rad/s)...")
        client.send_command(0, 0)
        time.sleep(0.2)
        
        # Check feedback
        for i in range(5):
            feedback = client.read_feedback()
            if feedback:
                enc1, enc2, vel1, vel2 = feedback
                print(f"  Feedback: E1={enc1}, E2={enc2}, V1={vel1:.3f}, V2={vel2:.3f}")
            time.sleep(0.1)
        
        # Send forward command
        print("\n2. Sending FORWARD command (2 rad/s for both motors)...")
        client.send_command(2.0, 2.0)
        
        for i in range(10):
            feedback = client.read_feedback()
            if feedback:
                enc1, enc2, vel1, vel2 = feedback
                print(f"  Feedback: E1={enc1}, E2={enc2}, V1={vel1:.3f}, V2={vel2:.3f}")
            time.sleep(0.1)
        
        # Send stop
        print("\n3. Sending STOP...")
        client.send_command(0, 0)
        
        for i in range(3):
            feedback = client.read_feedback()
            if feedback:
                enc1, enc2, vel1, vel2 = feedback
                print(f"  Feedback: E1={enc1}, E2={enc2}, V1={vel1:.3f}, V2={vel2:.3f}")
            time.sleep(0.1)
        
        print("\n=== Test Complete ===")
        
    finally:
        client.close()


def test_ros2_integration():
    """
    Test with ROS 2 by bridging serial commands to /diff_cont/cmd_vel
    This allows testing without full ROS 2 serial hardware interface
    """
    try:
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from rclpy.node import Node
        
        class ArduinoTestBridge(Node):
            def __init__(self):
                super().__init__('arduino_test_bridge')
                self.client = ArduinoSerialClient()
                
                # Subscribe to diff_drive_controller command
                self.subscription = self.create_subscription(
                    TwistStamped,
                    '/diff_cont/cmd_vel',
                    self.cmd_vel_callback,
                    10)
                
                # Timer to read feedback
                self.create_timer(0.05, self.read_feedback_callback)
                
                self.get_logger().info("Arduino Test Bridge started")
            
            def cmd_vel_callback(self, msg):
                """Convert Twist commands to motor velocities"""
                # For differential drive:
                # v_left = (linear_vel - angular_vel * wheel_sep/2) / wheel_radius
                # v_right = (linear_vel + angular_vel * wheel_sep/2) / wheel_radius
                
                linear_vel = msg.twist.linear.x
                angular_vel = msg.twist.angular.z
                
                # Robot parameters (from config)
                wheel_separation = 0.38
                wheel_radius = 0.165
                
                # Calculate motor velocities
                vel_motor1 = (linear_vel - angular_vel * wheel_separation/2) / wheel_radius
                vel_motor2 = (linear_vel + angular_vel * wheel_separation/2) / wheel_radius
                
                self.client.send_command(vel_motor1, vel_motor2)
            
            def read_feedback_callback(self):
                """Read feedback and optionally publish"""
                feedback = self.client.read_feedback()
                if feedback:
                    enc1, enc2, vel1, vel2 = feedback
                    print(f"Feedback: E1={enc1}, E2={enc2}, V1={vel1:.3f}, V2={vel2:.3f}")
        
        rclpy.init()
        node = ArduinoTestBridge()
        rclpy.spin(node)
        
    except ImportError:
        print("ERROR: ROS 2 not available. Install rclpy or use test_basic_communication()")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arduino Motor Control Tester')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--ros2', action='store_true', help='Test with ROS 2 integration')
    
    args = parser.parse_args()
    
    if args.ros2:
        print("Starting ROS 2 integration test...")
        test_ros2_integration()
    else:
        print("Starting basic communication test...")
        test_basic_communication()
