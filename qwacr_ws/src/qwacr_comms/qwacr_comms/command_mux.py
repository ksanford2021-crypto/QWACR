"""Command multiplexer with priority and timeout handling."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


@dataclass
class SourceState:
    msg: Twist
    last_time: float


class CommandMux(Node):
    def __init__(self) -> None:
        super().__init__("command_mux")

        self.declare_parameter("timeout", 2.0)
        self.timeout = float(self.get_parameter("timeout").value)

        self._sources = {
            "lora": None,
            "teleop": None,
            "auto": None,
        }

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(Twist, "/lora/cmd_vel_in", self._on_lora, 10)
        self.create_subscription(Twist, "/cmd_vel_teleop", self._on_teleop, 10)
        self.create_subscription(Twist, "/cmd_vel_autonomous", self._on_auto, 10)

        self.create_timer(0.05, self._publish_best)

    def _update(self, key: str, msg: Twist) -> None:
        self._sources[key] = SourceState(msg=msg, last_time=self.get_clock().now().seconds_nanoseconds()[0])

    def _is_fresh(self, state: SourceState | None) -> bool:
        if state is None:
            return False
        now = self.get_clock().now().seconds_nanoseconds()[0]
        return (now - state.last_time) <= self.timeout

    def _on_lora(self, msg: Twist) -> None:
        self._update("lora", msg)

    def _on_teleop(self, msg: Twist) -> None:
        self._update("teleop", msg)

    def _on_auto(self, msg: Twist) -> None:
        self._update("auto", msg)

    def _publish_best(self) -> None:
        # Priority: LoRa > teleop > autonomous
        if self._is_fresh(self._sources["lora"]):
            self.pub.publish(self._sources["lora"].msg)
        elif self._is_fresh(self._sources["teleop"]):
            self.pub.publish(self._sources["teleop"].msg)
        elif self._is_fresh(self._sources["auto"]):
            self.pub.publish(self._sources["auto"].msg)
        else:
            self.pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
