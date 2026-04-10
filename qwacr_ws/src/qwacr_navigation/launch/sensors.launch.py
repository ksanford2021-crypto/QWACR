#!/usr/bin/env python3
"""
QWACR Sensors Launch
Brings up: Aurora SLAM, GPS driver (no motors, no Nav2)
Use this for testing sensors without hardware motors
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    qwacr_gps_dir = get_package_share_directory('qwacr_gps')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    aurora_ip = LaunchConfiguration('aurora_ip')
    gps_serial_port = LaunchConfiguration('gps_serial_port')
    gps_baud_rate = LaunchConfiguration('gps_baud_rate')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_aurora_ip_cmd = DeclareLaunchArgument(
        'aurora_ip',
        default_value='192.168.11.1',
        description='IP address of Aurora SLAM sensor')
    
    declare_gps_serial_port_cmd = DeclareLaunchArgument(
        'gps_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for GPS module')
    
    declare_gps_baud_rate_cmd = DeclareLaunchArgument(
        'gps_baud_rate',
        default_value='460800',
        description='Baud rate for GPS module (LG580P uses 460800)')
    
    # Aurora SLAM sensor (publishes /odom, /scan, /imu)
    # Note: Aurora typically runs on its own launch system
    # User should launch separately with:
    # ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
    aurora_reminder_node = Node(
        package='qwacr_navigation',
        executable='hardware_test',
        name='aurora_reminder',
        output='screen',
        parameters=[{
            'reminder_only': True,
            'message': f'Remember to launch Aurora SLAM separately:\n'
                      f'ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:={aurora_ip}'
        }],
        # This node doesn't exist, just a reminder in launch file
        # Remove this and launch Aurora manually
        on_exit='shutdown'
    )
    
    # GPS driver (publishes /fix, /gps/enu_odom, /gps/heading)
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qwacr_gps_dir, 'launch', 'gps.launch.py')
        ),
        launch_arguments={
            'serial_port': gps_serial_port,
            'baud_rate': gps_baud_rate,
            'publish_enu': 'true',
            'publish_heading': 'true',
        }.items()
    )
    
    # Static transform: base_link to sensor frames (adjust as needed)
    # These should match your robot's physical sensor mounting
    base_to_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps',
        arguments=['0.0', '0.0', '0.15', '0', '0', '0', 'base_link', 'gps'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0.0', '0.0', '0.20', '0', '0', '0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        # IMU mounting: +Y forward, -Z down (sensor upside-down in Z).
        # Base frame follows REP-103 (+X forward, +Y left, +Z up).
        # We first rotate +90 deg about Z (yaw = +pi/2) so IMU +Y aligns with base +X,
        # then rotate 180 deg about X (roll = +pi) so IMU -Z becomes base +Z.
        # Combined roll-pitch-yaw (rpy) ≈ (pi, 0, pi/2).
        arguments=['0.0', '0.0', '0.15', '3.1416', '0', '1.5708', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_aurora_ip_cmd)
    ld.add_action(declare_gps_serial_port_cmd)
    ld.add_action(declare_gps_baud_rate_cmd)
    
    # Add nodes
    ld.add_action(gps_launch)
    ld.add_action(base_to_gps_tf)
    ld.add_action(base_to_laser_tf)
    ld.add_action(base_to_imu_tf)
    
    return ld
