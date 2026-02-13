#!/usr/bin/env python3
"""
Simple motor control launch - Just ros2_control without URDF/RViz
Use this for standalone motor testing or with Nav2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for motor controller Arduino'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
    ]

    # Get package directories
    qwacr_build_dir = get_package_share_directory('qwacr_build')
    
    # File paths
    xacro_file = os.path.join(qwacr_build_dir, 'urdf', 'qwacr.urdf.xacro')
    controller_config = os.path.join(qwacr_build_dir, 'config', 'diff_drive_controller.yaml')
    
    # Generate robot description with serial port parameters
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=false',
            ' serial_port:=', serial_port,
            ' baud_rate:=', baud_rate,
        ]),
        value_type=str,
    )

    # Robot state publisher (publishes robot description and TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
    )

    # Controller manager (ros2_control node)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_broad',
            '-c', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )

    # Differential drive controller spawner
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '-c', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )

    return LaunchDescription([
        *declare_args,
        robot_state_publisher_node,
        control_node,
        joint_broad_spawner,
        diff_cont_spawner,
    ])
