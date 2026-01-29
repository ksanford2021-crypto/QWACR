#!/usr/bin/env python3
"""
Hardware Integration Testing Script for QWACR
Tests all sensors and systems before full navigation launch.

Usage:
    ros2 run qwacr_navigation hardware_test
    
Or for specific test:
    ros2 run qwacr_navigation hardware_test --test aurora
    ros2 run qwacr_navigation hardware_test --test gps
    ros2 run qwacr_navigation hardware_test --test ekf
    ros2 run qwacr_navigation hardware_test --test motors
    ros2 run qwacr_navigation hardware_test --test all
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
import argparse
import sys
import time
from collections import deque

class HardwareTest(Node):
    def __init__(self):
        super().__init__('hardware_test')
        
        # Test results storage
        self.test_results = {}
        self.data_received = {}
        self.data_rates = {}
        self.data_buffers = {
            'aurora_odom': deque(maxlen=50),
            'aurora_scan': deque(maxlen=50),
            'aurora_imu': deque(maxlen=50),
            'gps_odom': deque(maxlen=50),
            'gps_fix': deque(maxlen=50),
            'gps_heading': deque(maxlen=50),
            'ekf_local': deque(maxlen=50),
            'ekf_global': deque(maxlen=50),
            'wheel_odom': deque(maxlen=50),
        }
        
        # Initialize flags
        for key in self.data_buffers.keys():
            self.data_received[key] = False
        
        # Create subscribers
        self.create_subscriptions()
        
        self.get_logger().info('Hardware test node initialized')
        self.get_logger().info('Waiting for sensor data...')
    
    def create_subscriptions(self):
        """Create all sensor subscriptions"""
        # Aurora SLAM topics
        self.sub_aurora_odom = self.create_subscription(
            Odometry, '/odom', self.aurora_odom_callback, 10)
        self.sub_aurora_scan = self.create_subscription(
            LaserScan, '/scan', self.aurora_scan_callback, 10)
        self.sub_aurora_imu = self.create_subscription(
            Imu, '/imu', self.aurora_imu_callback, 10)
        
        # GPS topics
        self.sub_gps_odom = self.create_subscription(
            Odometry, '/gps/enu_odom', self.gps_odom_callback, 10)
        self.sub_gps_fix = self.create_subscription(
            NavSatFix, '/fix', self.gps_fix_callback, 10)
        self.sub_gps_heading = self.create_subscription(
            Float64, '/gps/heading', self.gps_heading_callback, 10)
        
        # EKF topics
        self.sub_ekf_local = self.create_subscription(
            Odometry, '/odometry/local', self.ekf_local_callback, 10)
        self.sub_ekf_global = self.create_subscription(
            Odometry, '/odometry/global', self.ekf_global_callback, 10)
        
        # Wheel odometry
        self.sub_wheel_odom = self.create_subscription(
            Odometry, '/diff_cont/odom', self.wheel_odom_callback, 10)
    
    def aurora_odom_callback(self, msg):
        self.data_received['aurora_odom'] = True
        self.data_buffers['aurora_odom'].append(time.time())
    
    def aurora_scan_callback(self, msg):
        self.data_received['aurora_scan'] = True
        self.data_buffers['aurora_scan'].append(time.time())
    
    def aurora_imu_callback(self, msg):
        self.data_received['aurora_imu'] = True
        self.data_buffers['aurora_imu'].append(time.time())
    
    def gps_odom_callback(self, msg):
        self.data_received['gps_odom'] = True
        self.data_buffers['gps_odom'].append(time.time())
    
    def gps_fix_callback(self, msg):
        self.data_received['gps_fix'] = True
        self.data_buffers['gps_fix'].append(time.time())
    
    def gps_heading_callback(self, msg):
        self.data_received['gps_heading'] = True
        self.data_buffers['gps_heading'].append(time.time())
    
    def ekf_local_callback(self, msg):
        self.data_received['ekf_local'] = True
        self.data_buffers['ekf_local'].append(time.time())
    
    def ekf_global_callback(self, msg):
        self.data_received['ekf_global'] = True
        self.data_buffers['ekf_global'].append(time.time())
    
    def wheel_odom_callback(self, msg):
        self.data_received['wheel_odom'] = True
        self.data_buffers['wheel_odom'].append(time.time())
    
    def calculate_rate(self, buffer):
        """Calculate Hz from timestamps"""
        if len(buffer) < 2:
            return 0.0
        time_diffs = [buffer[i] - buffer[i-1] for i in range(1, len(buffer))]
        avg_diff = sum(time_diffs) / len(time_diffs)
        return 1.0 / avg_diff if avg_diff > 0 else 0.0
    
    def test_aurora(self):
        """Test Aurora SLAM sensor"""
        self.get_logger().info('\n=== Testing Aurora SLAM Sensor ===')
        
        # Wait for data
        timeout = 10.0
        start = time.time()
        
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all([
                self.data_received['aurora_odom'],
                self.data_received['aurora_scan'],
                self.data_received['aurora_imu']
            ]):
                break
        
        # Check results
        results = {
            'Odometry (/odom)': self.data_received['aurora_odom'],
            'LaserScan (/scan)': self.data_received['aurora_scan'],
            'IMU (/imu)': self.data_received['aurora_imu'],
        }
        
        # Calculate rates if data received
        if self.data_received['aurora_odom']:
            # Collect more data for rate calculation
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            odom_rate = self.calculate_rate(self.data_buffers['aurora_odom'])
            scan_rate = self.calculate_rate(self.data_buffers['aurora_scan'])
            imu_rate = self.calculate_rate(self.data_buffers['aurora_imu'])
            
            self.get_logger().info(f'Aurora Odometry Rate: {odom_rate:.1f} Hz')
            self.get_logger().info(f'Aurora Scan Rate: {scan_rate:.1f} Hz')
            self.get_logger().info(f'Aurora IMU Rate: {imu_rate:.1f} Hz')
        
        # Print results
        all_pass = all(results.values())
        for topic, status in results.items():
            status_str = '✓ PASS' if status else '✗ FAIL'
            self.get_logger().info(f'  {topic}: {status_str}')
        
        self.test_results['aurora'] = all_pass
        return all_pass
    
    def test_gps(self):
        """Test GPS sensor"""
        self.get_logger().info('\n=== Testing GPS Sensor ===')
        
        # Wait for data
        timeout = 15.0  # GPS may take longer
        start = time.time()
        
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all([
                self.data_received['gps_odom'],
                self.data_received['gps_fix'],
                self.data_received['gps_heading']
            ]):
                break
        
        # Check results
        results = {
            'ENU Odometry (/gps/enu_odom)': self.data_received['gps_odom'],
            'NavSatFix (/fix)': self.data_received['gps_fix'],
            'Heading (/gps/heading)': self.data_received['gps_heading'],
        }
        
        # Calculate rates if data received
        if self.data_received['gps_odom']:
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            odom_rate = self.calculate_rate(self.data_buffers['gps_odom'])
            fix_rate = self.calculate_rate(self.data_buffers['gps_fix'])
            heading_rate = self.calculate_rate(self.data_buffers['gps_heading'])
            
            self.get_logger().info(f'GPS Odometry Rate: {odom_rate:.1f} Hz')
            self.get_logger().info(f'GPS Fix Rate: {fix_rate:.1f} Hz')
            self.get_logger().info(f'GPS Heading Rate: {heading_rate:.1f} Hz')
        
        # Print results
        all_pass = all(results.values())
        for topic, status in results.items():
            status_str = '✓ PASS' if status else '✗ FAIL'
            self.get_logger().info(f'  {topic}: {status_str}')
        
        self.test_results['gps'] = all_pass
        return all_pass
    
    def test_ekf(self):
        """Test EKF localization"""
        self.get_logger().info('\n=== Testing EKF Localization ===')
        
        # Wait for data
        timeout = 10.0
        start = time.time()
        
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all([
                self.data_received['ekf_local'],
                self.data_received['ekf_global']
            ]):
                break
        
        # Check results
        results = {
            'Local Odometry (/odometry/local)': self.data_received['ekf_local'],
            'Global Odometry (/odometry/global)': self.data_received['ekf_global'],
        }
        
        # Calculate rates if data received
        if self.data_received['ekf_local']:
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            local_rate = self.calculate_rate(self.data_buffers['ekf_local'])
            global_rate = self.calculate_rate(self.data_buffers['ekf_global'])
            
            self.get_logger().info(f'Local EKF Rate: {local_rate:.1f} Hz')
            self.get_logger().info(f'Global EKF Rate: {global_rate:.1f} Hz')
        
        # Print results
        all_pass = all(results.values())
        for topic, status in results.items():
            status_str = '✓ PASS' if status else '✗ FAIL'
            self.get_logger().info(f'  {topic}: {status_str}')
        
        self.test_results['ekf'] = all_pass
        return all_pass
    
    def test_motors(self):
        """Test motor control"""
        self.get_logger().info('\n=== Testing Motor Control ===')
        
        # Wait for data
        timeout = 5.0
        start = time.time()
        
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.data_received['wheel_odom']:
                break
        
        # Check results
        results = {
            'Wheel Odometry (/diff_cont/odom)': self.data_received['wheel_odom'],
        }
        
        # Calculate rate if data received
        if self.data_received['wheel_odom']:
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            odom_rate = self.calculate_rate(self.data_buffers['wheel_odom'])
            self.get_logger().info(f'Wheel Odometry Rate: {odom_rate:.1f} Hz')
        
        # Print results
        all_pass = all(results.values())
        for topic, status in results.items():
            status_str = '✓ PASS' if status else '✗ FAIL'
            self.get_logger().info(f'  {topic}: {status_str}')
        
        self.test_results['motors'] = all_pass
        return all_pass
    
    def test_tf_tree(self):
        """Test TF tree structure"""
        self.get_logger().info('\n=== Testing TF Tree ===')
        self.get_logger().info('Run: ros2 run tf2_tools view_frames')
        self.get_logger().info('Expected frames: map -> odom -> base_link')
        return True
    
    def run_all_tests(self):
        """Run all hardware tests"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('QWACR Hardware Integration Test Suite')
        self.get_logger().info('='*60)
        
        # Run tests
        aurora_pass = self.test_aurora()
        gps_pass = self.test_gps()
        ekf_pass = self.test_ekf()
        motors_pass = self.test_motors()
        self.test_tf_tree()
        
        # Summary
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('='*60)
        
        results = {
            'Aurora SLAM': aurora_pass,
            'GPS': gps_pass,
            'EKF Localization': ekf_pass,
            'Motor Control': motors_pass,
        }
        
        for test, passed in results.items():
            status = '✓ PASS' if passed else '✗ FAIL'
            self.get_logger().info(f'{test}: {status}')
        
        all_pass = all(results.values())
        
        if all_pass:
            self.get_logger().info('\n✓ ALL TESTS PASSED - Ready for navigation!')
            self.get_logger().info('Next step: ros2 launch qwacr_navigation navigation_with_robot.launch.py')
        else:
            self.get_logger().warn('\n✗ SOME TESTS FAILED - Check hardware connections')
        
        self.get_logger().info('='*60 + '\n')
        return all_pass

def main(args=None):
    parser = argparse.ArgumentParser(description='QWACR Hardware Test')
    parser.add_argument('--test', choices=['aurora', 'gps', 'ekf', 'motors', 'all'],
                        default='all', help='Specific test to run')
    
    # Parse known args (ROS2 will handle the rest)
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    node = HardwareTest()
    
    try:
        if parsed_args.test == 'all':
            success = node.run_all_tests()
        elif parsed_args.test == 'aurora':
            success = node.test_aurora()
        elif parsed_args.test == 'gps':
            success = node.test_gps()
        elif parsed_args.test == 'ekf':
            success = node.test_ekf()
        elif parsed_args.test == 'motors':
            success = node.test_motors()
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
