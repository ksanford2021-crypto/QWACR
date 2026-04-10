#!/usr/bin/env python3
"""
QWACR Localization Launch
Brings up: Sensors + Dual EKF sensor fusion (no Nav2, no motors)
Use this for testing EKF integration without navigation
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
    qwacr_nav_dir = get_package_share_directory('qwacr_navigation')
    robot_localization_dir = get_package_share_directory('robot_localization')
    
    # File paths
    ekf_params_file = os.path.join(robot_localization_dir, 'params', 'ekf_local_global.yaml')
    
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
        description='Baud rate for GPS module')
    
    # Include sensors launch
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qwacr_nav_dir, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'aurora_ip': aurora_ip,
            'gps_serial_port': gps_serial_port,
            'gps_baud_rate': gps_baud_rate,
        }.items()
    )
    
    # Local EKF (odom frame): wheel odometry + IMU
    # Fuses continuous sensors for smooth local odometry
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_odom',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # Output topic only; inputs use /diff_cont/odom and /imu/data directly
            ('odometry/filtered', 'odometry/local'),
        ],
    )
    
    # Global EKF (map frame): Local sensors + GPS
    # Adds GPS for global positioning with discrete jumps acceptable
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_map',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # GPS already at /gps/enu_odom (correct in EKF config)
            # Output topic only; inputs use /diff_cont/odom, /imu/data, /gps/enu_odom directly
            ('odometry/filtered', 'odometry/global'),
        ],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_aurora_ip_cmd)
    ld.add_action(declare_gps_serial_port_cmd)
    ld.add_action(declare_gps_baud_rate_cmd)
    
    # Add nodes
    ld.add_action(sensors_launch)
    ld.add_action(ekf_local_node)
    ld.add_action(ekf_global_node)
    
    return ld
