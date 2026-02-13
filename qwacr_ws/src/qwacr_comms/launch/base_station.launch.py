from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    role = LaunchConfiguration("role")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")

    return LaunchDescription([
        DeclareLaunchArgument("role", default_value="base"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        Node(
            package="qwacr_comms",
            executable="lora_bridge",
            name="lora_bridge",
            parameters=[{
                "role": role,
                "serial_port": serial_port,
                "baud_rate": baud_rate,
            }],
        ),
    ])
