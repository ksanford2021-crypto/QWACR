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
            # Reuse the main HaLow config but override the primary
            # camera index and name so this launch streams from the
            # left camera instead of the front.
            parameters=[
                config_file,
                {
                    "camera1_index": 1,
                    "camera1_name": "left",
                },
            ],
            output="screen",
        ),
    ])
