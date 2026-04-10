#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomYawPrinter(Node):
    """Subscribe to /diff_cont/odom and print yaw in radians.

    This node converts the quaternion orientation in nav_msgs/Odometry
    to a yaw angle (around +Z) and prints both radians and degrees.
    It also maintains a simple unwrapped yaw estimate so you can see
    approximate cumulative rotation past +/-pi.
    """

    def __init__(self) -> None:
        super().__init__("odom_yaw_printer")

        self._sub = self.create_subscription(
            Odometry,
            "/diff_cont/odom",
            self._odom_callback,
            10,
        )

        self._last_yaw: Optional[float] = None
        self._unwrapped_yaw: float = 0.0

        self.get_logger().info(
            "Subscribed to /diff_cont/odom. Printing yaw (rad, deg, unwrapped_rad)."
        )

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        # Assuming roll/pitch ~ 0, use simplified yaw = 2*atan2(z, w).
        yaw = 2.0 * math.atan2(q.z, q.w)

        # Unwrap yaw to avoid jumps at +/-pi
        if self._last_yaw is None:
            self._unwrapped_yaw = yaw
        else:
            dyaw = yaw - self._last_yaw
            # Wrap into [-pi, pi]
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            self._unwrapped_yaw += dyaw

        self._last_yaw = yaw

        yaw_deg = math.degrees(yaw)
        unwrapped_deg = math.degrees(self._unwrapped_yaw)

        self.get_logger().info(
            f"yaw: {yaw: .3f} rad ({yaw_deg: .1f} deg), "
            f"unwrapped: {self._unwrapped_yaw: .3f} rad ({unwrapped_deg: .1f} deg)"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomYawPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
