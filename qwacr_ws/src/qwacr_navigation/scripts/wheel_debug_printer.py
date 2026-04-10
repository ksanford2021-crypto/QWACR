#!/usr/bin/env python3

import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelDebugPrinter(Node):
    """Print left/right wheel velocities and differences from /joint_states.

    This node is meant to be run while you command straight motion
    (non-zero linear.x, angular.z ~ 0). It helps quantify how much the
    left and right wheel speeds differ in practice.
    """

    def __init__(self) -> None:
        super().__init__("wheel_debug_printer")

        # Parameters so we don't hard-code joint names
        self.declare_parameter(
            "left_wheel_names",
            [
                "base_to_front_left_wheel",
                "base_to_back_left_wheel",
            ],
        )
        self.declare_parameter(
            "right_wheel_names",
            [
                "base_to_front_right_wheel",
                "base_to_back_right_wheel",
            ],
        )

        self.left_wheel_names: List[str] = [
            s.strip()
            for s in self.get_parameter("left_wheel_names")
            .get_parameter_value()
            .string_array_value
        ]
        self.right_wheel_names: List[str] = [
            s.strip()
            for s in self.get_parameter("right_wheel_names")
            .get_parameter_value()
            .string_array_value
        ]

        self._last_print_time: float = 0.0
        self._print_period: float = 0.2  # seconds

        self._sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            50,
        )

        self.get_logger().info(
            "Subscribed to /joint_states. Will print left/right wheel speeds "
            "every %.1f s." % self._print_period
        )
        self.get_logger().info(
            f"Left wheels:  {self.left_wheel_names}"
        )
        self.get_logger().info(
            f"Right wheels: {self.right_wheel_names}"
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        now = time.time()
        if now - self._last_print_time < self._print_period:
            return
        self._last_print_time = now

        # Build a lookup of joint name -> velocity
        vel_map: Dict[str, float] = {}
        for name, vel in zip(msg.name, msg.velocity):
            vel_map[name] = vel

        left_vals = [vel_map[n] for n in self.left_wheel_names if n in vel_map]
        right_vals = [vel_map[n] for n in self.right_wheel_names if n in vel_map]

        if not left_vals or not right_vals:
            # Not all wheels reported yet
            return

        left_avg = sum(left_vals) / len(left_vals)
        right_avg = sum(right_vals) / len(right_vals)
        diff = left_avg - right_avg

        # Also print per-wheel values for quick inspection
        parts = [
            f"L_avg={left_avg:+.4f} rad/s",
            f"R_avg={right_avg:+.4f} rad/s",
            f"L-R={diff:+.4f} rad/s",
        ]

        for n in self.left_wheel_names:
            if n in vel_map:
                parts.append(f"{n}={vel_map[n]:+.4f}")
        for n in self.right_wheel_names:
            if n in vel_map:
                parts.append(f"{n}={vel_map[n]:+.4f}")

        self.get_logger().info(" | ".join(parts))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelDebugPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
