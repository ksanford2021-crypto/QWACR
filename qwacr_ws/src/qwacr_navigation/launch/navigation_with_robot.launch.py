#!/usr/bin/env python3
"""
Complete QWACR Navigation Launch with RViz
Brings up: robot_state_publisher, controllers, Nav2, and RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    qwacr_build_dir = get_package_share_directory('qwacr_build')
    qwacr_nav_dir = get_package_share_directory('qwacr_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # File paths
    xacro_file = os.path.join(qwacr_build_dir, 'urdf', 'qwacr.urdf.xacro')
    rviz_config_file = os.path.join(qwacr_build_dir, 'launch', 'qwacr_display.rviz')
    nav2_params_file = os.path.join(qwacr_nav_dir, 'config', 'nav2_params.yaml')
    controller_config = os.path.join(qwacr_build_dir, 'config', 'diff_drive_controller.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2')
    
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for motor controller')
    
    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for motor controller')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_hardware_cmd = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Use real hardware (requires serial connection)')
    
    # Robot description from URDF/xacro
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=false',
            ' serial_port:=', serial_port,
            ' baud_rate:=', baud_rate,
        ]),
        value_type=str,
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
    )
    
    # Joint state publisher (publishes joint states for visualization)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Controller manager
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Spawn joint state broadcaster
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '-c', '/controller_manager'],
        output='screen',
    )
    
    # Spawn diff drive controller
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager'],
        output='screen',
    )
    
    # Nav2 stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
        }.items()
    )
    
    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_baud_rate_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(control_node)
    ld.add_action(joint_broad_spawner)
    ld.add_action(diff_cont_spawner)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld
