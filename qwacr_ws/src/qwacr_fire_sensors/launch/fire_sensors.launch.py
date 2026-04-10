from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="qwacr_fire_sensors",
                executable="sen54_node.py",
                name="sen54",
                output="screen",
                parameters=[{"poll_period": 1.0}],
            ),
            Node(
                package="qwacr_fire_sensors",
                executable="mlx90640_node.py",
                name="mlx90640_front",
                output="screen",
                parameters=[
                    {
                        "poll_period": 0.5,
                        "mux_port": 0x02,
                        "topic": "thermal/front/array",
                        "frame_id": "thermal_front_frame",
                    }
                ],
            ),
            Node(
                package="qwacr_fire_sensors",
                executable="mlx90640_node.py",
                name="mlx90640_left",
                output="screen",
                parameters=[
                    {
                        "poll_period": 0.5,
                        "mux_port": 0x10,
                        "topic": "thermal/left/array",
                        "frame_id": "thermal_left_frame",
                    }
                ],
            ),
            Node(
                package="qwacr_fire_sensors",
                executable="dgs2_h2_node.py",
                name="dgs2_h2",
                output="screen",
                parameters=[{"port": "/dev/ttyAMA4", "baud": 9600}],
            ),
            Node(
                package="qwacr_fire_sensors",
                executable="dgs2_co_node.py",
                name="dgs2_co",
                output="screen",
                parameters=[{"port": "/dev/ttyAMA2", "baud": 9600}],
            ),
        ]
    )
