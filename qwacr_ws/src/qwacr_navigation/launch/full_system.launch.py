#!/usr/bin/env python3
"""
QWACR Full System Launch
Complete autonomous navigation system integration:
- Hardware interface (motor control + wheel odometry)
- Sensors (Aurora SLAM, GPS, IMU)
- Localization (Dual EKF sensor fusion)
- Navigation (Nav2 stack)
- Visualization (RViz)

Launch sequence:
1. Robot state publisher (URDF/TF)
2. Motor controllers (mega_diff_drive_control)
3. Sensors (Aurora, GPS)
4. EKF localization (local + global)
5. Nav2 navigation stack
6. RViz (optional)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    qwacr_build_dir = get_package_share_directory('qwacr_build')
    qwacr_nav_dir = get_package_share_directory('qwacr_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_localization_dir = get_package_share_directory('robot_localization')
    
    # File paths
    xacro_file = os.path.join(qwacr_build_dir, 'urdf', 'qwacr.urdf.xacro')
    rviz_config_file = os.path.join(qwacr_build_dir, 'launch', 'qwacr_display.rviz')
    nav2_params_file = os.path.join(qwacr_nav_dir, 'config', 'nav2_params.yaml')
    bt_xml_file = os.path.join(qwacr_nav_dir, 'config', 'qwacr_outdoor_nav.xml')
    controller_config = os.path.join(qwacr_build_dir, 'config', 'diff_drive_controller.yaml')
    ekf_params_file = os.path.join(robot_localization_dir, 'params', 'ekf_local_global.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_hardware = LaunchConfiguration('use_hardware')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    gps_serial_port = LaunchConfiguration('gps_serial_port')
    gps_baud_rate = LaunchConfiguration('gps_baud_rate')
    aurora_ip = LaunchConfiguration('aurora_ip')
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
    
    declare_use_hardware_cmd = DeclareLaunchArgument(
        'use_hardware',
        default_value='true',
        description='Use real hardware motors (requires serial connection)')
    
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='Serial port for motor controller Arduino')
    
    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for motor controller')
    
    declare_gps_serial_port_cmd = DeclareLaunchArgument(
        'gps_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for GPS module')
    
    declare_gps_baud_rate_cmd = DeclareLaunchArgument(
        'gps_baud_rate',
        default_value='460800',
        description='Baud rate for GPS module (LG580P uses 460800)')
    
    declare_aurora_ip_cmd = DeclareLaunchArgument(
        'aurora_ip',
        default_value='192.168.11.1',
        description='IP address of Aurora SLAM sensor')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_default_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=bt_xml_file,
        description='Full path to the default Nav2 behavior tree XML file to use')
    
    # ========================================================================
    # LAYER 1: ROBOT STATE PUBLISHER (URDF/TF Tree)
    # ========================================================================
    
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=false',
            ' serial_port:=', serial_port,
            ' baud_rate:=', baud_rate,
        ]),
        value_type=str,
    )
    
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
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # ========================================================================
    # LAYER 2: HARDWARE INTERFACE (MOTOR CONTROL + WHEEL ODOMETRY)
    # ========================================================================
    
    # Controller manager (if using hardware)
    control_node = Node(
        condition=IfCondition(use_hardware),
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Spawn joint state broadcaster
    joint_broad_spawner = Node(
        condition=IfCondition(use_hardware),
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '-c', '/controller_manager'],
        output='screen',
    )
    
    # Spawn diff drive controller (publishes /diff_cont/odom)
    diff_cont_spawner = Node(
        condition=IfCondition(use_hardware),
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager'],
        output='screen',
    )
    
    # ========================================================================
    # LAYER 3: SENSORS (GPS, AURORA SLAM)
    # ========================================================================
    
    # Include sensors launch (GPS + Aurora reminders)
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
    
    # ========================================================================
    # LAYER 4: LOCALIZATION (DUAL EKF SENSOR FUSION)
    # ========================================================================
    
    # Local EKF (odom frame): Wheel odom + Aurora SLAM + Aurora IMU
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_odom',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
            # Remap wheel odometry to expected topic name
            ('odometry/wheel', 'diff_cont/odom'),
        ],
    )
    
    # Global EKF (map frame): All local sensors + GPS
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_map',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
            # Remap wheel odometry
            ('odometry/wheel', 'diff_cont/odom'),
        ],
    )
    
    # ========================================================================
    # LAYER 5: NAVIGATION (NAV2 STACK)
    # ========================================================================
    
    # Nav2 stack (uses /odometry/global as input)
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
    
    # ========================================================================
    # LAYER 6: VISUALIZATION (RVIZ)
    # ========================================================================
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # ========================================================================
    # BUILD LAUNCH DESCRIPTION
    # ========================================================================
    
    ld = LaunchDescription()
    
    # Declare all launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_hardware_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_baud_rate_cmd)
    ld.add_action(declare_gps_serial_port_cmd)
    ld.add_action(declare_gps_baud_rate_cmd)
    ld.add_action(declare_aurora_ip_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_default_bt_xml_cmd)
    
    # Add nodes in order
    # Layer 1: Robot description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    
    # Layer 2: Hardware interface
    ld.add_action(control_node)
    ld.add_action(joint_broad_spawner)
    ld.add_action(diff_cont_spawner)
    
    # Layer 3: Sensors
    ld.add_action(sensors_launch)
    
    # Layer 4: Localization
    ld.add_action(ekf_local_node)
    ld.add_action(ekf_global_node)
    
    # Layer 5: Navigation
    ld.add_action(nav2_bringup_launch)
    
    # Layer 6: Visualization
    ld.add_action(rviz_node)
    
    return ld
