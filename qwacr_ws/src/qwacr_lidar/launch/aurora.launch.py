from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import launch


def generate_launch_description():
    # Find package directories
    qwacr_lidar_share = FindPackageShare('qwacr_lidar')
    
    # Configuration file
    config_file = PathJoinSubstitution([
        qwacr_lidar_share,
        'config',
        'aurora_params.yaml'
    ])
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for Aurora LiDAR'
    )
    
    scan_frequency_arg = DeclareLaunchArgument(
        'scan_frequency',
        default_value='10.0',
        description='Scan frequency in Hz (10-20 for Aurora)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # RPLidar node (Aurora uses rplidar_ros driver)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'scan_frequency': LaunchConfiguration('scan_frequency'),
            }
        ],
    )
    
    # Static TF: base_link -> laser_frame
    # Aurora mounted at front center of robot, 0.15m forward, 0.12m up
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '0.15', '0.0', '0.12',  # x, y, z (meters)
            '0', '0', '0',           # roll, pitch, yaw (radians)
            'base_link',
            'laser_frame'
        ],
    )
    
    # RViz (optional)
    rviz_config = PathJoinSubstitution([
        qwacr_lidar_share,
        'rviz',
        'aurora.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    
    return LaunchDescription([
        serial_port_arg,
        scan_frequency_arg,
        use_rviz_arg,
        rplidar_node,
        static_tf_node,
        rviz_node,
    ])
