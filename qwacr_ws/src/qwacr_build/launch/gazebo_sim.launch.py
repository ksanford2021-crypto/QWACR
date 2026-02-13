#!/usr/bin/env python3

"""Gazebo Simulation Launch for QWACR.

Separate launch file for Gazebo simulation that doesn't alter the hardware display.launch.py.

This launch file simulates:
- 4-wheel differential drive robot with ros2_control
- LiDAR scanner at 10Hz (360°, 12m range) -> /scan
- GPS receiver at 5Hz (±1m horizontal, ±2m vertical noise) -> /gps/fix
- IMU at 50Hz (angular velocity + linear acceleration) -> /imu/data
- Wheel odometry via diff_drive_controller -> /diff_cont/odom

The simulated sensors allow testing:
- Nav2 navigation stack with real sensor data
- EKF sensor fusion (GPS + IMU + wheel odometry)
- Costmap generation from LiDAR scans
- GPS waypoint following
- Path planning and obstacle avoidance

Usage:
  # Basic simulation with RViz
  ros2 launch qwacr_build gazebo_sim.launch.py

  # With teleop control
  ros2 launch qwacr_build gazebo_sim.launch.py use_teleop:=true

  # Custom world
  ros2 launch qwacr_build gazebo_sim.launch.py world:=/path/to/world.sdf

  # No GUI (for automated testing)
  ros2 launch qwacr_build gazebo_sim.launch.py gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_share_dir = get_package_share_directory('qwacr_build')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Launch keyboard teleop (requires interactive terminal)'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to Gazebo world file (empty string for default empty world)'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show Gazebo GUI'
    )
    
    entity_name_arg = DeclareLaunchArgument(
        'entity_name',
        default_value='qwacr',
        description='Name of robot entity in Gazebo'
    )
    
    # Configuration files
    xacro_file = os.path.join(pkg_share_dir, 'urdf', 'qwacr.urdf.xacro')
    controller_config = os.path.join(pkg_share_dir, 'config', 'diff_drive_controller.yaml')
    rviz_config_file = os.path.join(pkg_share_dir, 'qwacr_display.rviz')
    
    # Process URDF with Gazebo flag enabled
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=true',
            ' serial_port:=/tmp/ttyGazebo',  # Dummy port for Gazebo mode
            ' baud_rate:=115200',
        ]),
        value_type=str,
    )
    
    # Robot State Publisher - publishes TF tree from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )
    
    # Launch Gazebo
    def launch_gazebo(context):
        world_path = LaunchConfiguration('world').perform(context)
        gui = LaunchConfiguration('gui').perform(context)
        
        # Build Gazebo arguments
        gz_args = ['-r', '-v', '4']
        if world_path and world_path.strip():
            gz_args.append(world_path)
        
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py'
                    ])
                ),
                launch_arguments={
                    'gz_args': ' '.join(gz_args),
                    'on_exit_shutdown': 'true',
                }.items(),
            )
        ]
    
    gazebo_launcher = OpaqueFunction(function=launch_gazebo)
    
    # Bridge Gazebo topics to ROS2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Bridge LiDAR scan data
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/scan', '/scan')],
    )
    
    # Bridge GPS NavSat data
    gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Bridge IMU data
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Bridge cmd_vel commands from ROS2 to Gazebo
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Bridge odometry from Gazebo to ROS2
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/empty/model/qwacr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/empty/model/qwacr/joint_state', '/joint_states'),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Spawn robot in Gazebo from robot_description topic
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-entity', LaunchConfiguration('entity_name'),
                    '-topic', '/robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.2'  # Spawn slightly above ground
                ],
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )
    
    # RViz for visualization
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )
    
    # Keyboard teleop for manual control
    teleop_node = Node(
        condition=IfCondition(LaunchConfiguration('use_teleop')),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',  # Launch in separate terminal
        remappings=[('/cmd_vel', '/cmd_vel')],  # Gazebo diff drive listens to /cmd_vel
    )
    
    return LaunchDescription([
        # Declare arguments
        use_rviz_arg,
        use_teleop_arg,
        world_arg,
        gui_arg,
        entity_name_arg,
        
        # Launch nodes
        robot_state_publisher_node,
        clock_bridge,
        lidar_bridge,
        gps_bridge,
        imu_bridge,
        cmd_vel_bridge,
        odom_bridge,
        gazebo_launcher,
        spawn_robot,
        rviz_node,
        teleop_node,
    ])
