from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('qwacr_gps')
    config_file = os.path.join(pkg_dir, 'config', 'gps_params.yaml')

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GPS device'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for GPS serial communication'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps',
        description='Frame ID for GPS messages'
    )

    publish_enu_arg = DeclareLaunchArgument(
        'publish_enu',
        default_value='true',
        description='Whether to publish ENU pose'
    )

    publish_heading_arg = DeclareLaunchArgument(
        'publish_heading',
        default_value='true',
        description='Whether to publish heading from $GPHDT'
    )

    # GPS driver node
    gps_driver_node = Node(
        package='qwacr_gps',
        executable='gps_driver_node',
        name='gps_driver',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_enu': LaunchConfiguration('publish_enu'),
                'publish_heading': LaunchConfiguration('publish_heading'),
            }
        ],
        remappings=[
            ('/fix', '/gps/fix'),
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        publish_enu_arg,
        publish_heading_arg,
        gps_driver_node,
    ])
