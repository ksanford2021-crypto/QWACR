#!/usr/bin/env python3
"""
Nav2 with Mock Robot (No Hardware)
For testing Nav2 stack without physical robot/motors
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    qwacr_nav_dir = get_package_share_directory('qwacr_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # File paths
    nav2_params_file = os.path.join(qwacr_nav_dir, 'config', 'nav2_params.yaml')
    bt_xml_file = os.path.join(qwacr_nav_dir, 'config', 'qwacr_outdoor_nav.xml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_default_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=bt_xml_file,
        description='Full path to the default Nav2 behavior tree XML file to use')
    
    # Nav2 stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
        }.items()
    )
    
    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_default_bt_xml_cmd)
    
    # Add nodes
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld
