import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = os.path.join(
        get_package_share_directory("qwacr_comms"),
        "config",
        "halow_config.yaml",
    )

    return LaunchDescription([
        Node(
            package="qwacr_comms",
            executable="video_publisher",
            name="video_publisher",
            parameters=[config_file],
            output="screen",
        ),
    ])
