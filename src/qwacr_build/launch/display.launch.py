#!/usr/bin/env python3

"""Display + (optional) Gazebo (Harmonic) launch for QWACR.

- Starts robot_state_publisher + optional joint_state_publisher + RViz
- Starts controller_manager with configured controllers in real/hw mode or Gazebo simulation
- Optionally launches Gazebo Harmonic via ros_gz_sim with PTY serial simulator
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    GroupAction,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import textwrap

rviz_config_file = os.path.join(
    get_package_share_directory('qwacr_build'),
    'qwacr_display.rviz'
)

def generate_launch_description():

    # Launch arguments
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_teleop = LaunchConfiguration('use_teleop')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    sdf_file = LaunchConfiguration('sdf_file')
    entity_name = LaunchConfiguration('entity_name')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    declare_args = [
        DeclareLaunchArgument('use_gazebo', default_value='false', description='Launch Gazebo (Harmonic) and spawn robot'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(["'true' if '", use_gazebo, "' == 'true' else 'false'"]), description='Use simulated time'),
        DeclareLaunchArgument('use_teleop', default_value='false', description='Launch keyboard teleop (requires interactive terminal)'),
        DeclareLaunchArgument('world', default_value='', description='Path to Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('sdf_file', default_value='', description='Path to SDF file'),
        DeclareLaunchArgument('entity_name', default_value='qwacr', description='Gazebo entity name'),
        DeclareLaunchArgument('serial_port', default_value='/tmp/ttyV0', description='Serial port'),
        DeclareLaunchArgument('baud_rate', default_value='115200', description='Baud rate'),
    ]

    pkg_share_dir = get_package_share_directory('qwacr_build')
    xacro_file = os.path.join(pkg_share_dir, 'urdf', 'qwacr.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=', use_gazebo,
            ' serial_port:=', serial_port,
            ' baud_rate:=', baud_rate,
        ]),
        value_type=str,
    )

    controller_config = os.path.join(pkg_share_dir, 'config', 'diff_drive_controller.yaml')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}],
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(use_gazebo),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    teleop_node = Node(
        condition=IfCondition(PythonExpression(["'", use_gazebo, "' == 'true' and '", use_teleop, "' == 'true'"])),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel')],
        parameters=[{'stamped': True}],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'controller_manager:=debug']
    )

    # When using Gazebo with hardware plugin, start ros2_control_node (will connect to PTY simulator)
    control_node_with_sim = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'controller_manager:=debug']
    )

    # Arduino simulator with PID control
    arduino_simulator = Node(
        package='qwacr_build',
        executable='arduino_sim.py',
        output='screen',
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )

    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # Spawners for Gazebo mode - load and activate controllers directly
    joint_broad_spawner_in_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )

    diff_cont_spawner_in_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # Delay spawners slightly to ensure control node fully initializes
    start_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(period=2.0, actions=[joint_broad_spawner, diff_cont_spawner])
            ],
        )
    )

    # In Gazebo mode with hardware plugin, start ros2_control_node after Arduino simulator is ready
    # Add a small delay to ensure simulator is fully running
    start_control_node_after_sim = RegisterEventHandler(
        OnProcessStart(
            target_action=arduino_simulator,
            on_start=[control_node_with_sim],
        )
    )

    # In Gazebo mode with hardware plugin, start spawners after control_node_with_sim is up
    # Controllers will activate automatically on load
    start_spawners_after_control = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node_with_sim,
            on_start=[joint_broad_spawner_in_sim, diff_cont_spawner_in_sim],
        )
    )

    gz_args_expr = PythonExpression(["'-r -v 4 ' + '", world, "'"])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'gz_args': gz_args_expr,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_from_sdf = Node(
        condition=IfCondition(PythonExpression(["'", use_gazebo, "' == 'true' and '", sdf_file, "' != ''"])),
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', entity_name, '-file', sdf_file],
        output='screen',
    )

    spawn_from_description = Node(
        condition=IfCondition(PythonExpression(["'", use_gazebo, "' == 'true' and '", sdf_file, "' == ''"])),
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', entity_name, '-topic', 'robot_description'],
        output='screen',
    )

    # Wait until the serial port exists and is openable before starting control node
    # Use environment variable injection to avoid complex PythonExpression string building
    wait_for_serial = ExecuteProcess(
        cmd=[
            'python3', '-c',
            'import os, sys, time; '\
            'p = os.environ.get("SERIAL_PORT", ""); start = time.time(); '\
            'timeout = 10.0; '\
            'while time.time() - start < timeout: '\
            '  try:\n' \
            '    if p and os.path.exists(p): '\
            '      fd = os.open(p, os.O_RDWR | os.O_NOCTTY); os.close(fd); sys.exit(0)\n' \
            '  except Exception:\n' \
            '    pass\n' \
            '  time.sleep(0.2)\n' \
            'sys.exit(1)'
        ],
        additional_env={'SERIAL_PORT': serial_port},
        output='screen',
    )

    start_control_after_serial = RegisterEventHandler(
        OnProcessStart(
            target_action=wait_for_serial,
            on_start=[control_node, start_spawners],
        )
    )

    # Hardware mode group - use Arduino simulator with readiness gate
    hardware_group = GroupAction(
        condition=UnlessCondition(use_gazebo),
        actions=[
            arduino_simulator,
            wait_for_serial,
            start_control_after_serial,
        ]
    )

    # Gazebo + hardware mode group (arduino_simulator starts first, then others follow via event handlers)
    gazebo_group = GroupAction(
        condition=IfCondition(use_gazebo),
        actions=[
            arduino_simulator,
            start_control_node_after_sim,
            start_spawners_after_control,
        ]
    )

    return LaunchDescription([
        *declare_args,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        teleop_node,
        hardware_group,
        gazebo_group,
        gazebo_launch,
        spawn_from_sdf,
        spawn_from_description,
    ])
