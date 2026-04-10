"""Mission manager for high-level robot modes.

Listens for string commands (typically from the base-station GUI via
LoRa) on `/mission_command` and exposes:

- `/mission_state` (String): current mode, e.g. "idle", "manual",
  "rtb", "waypoint:default", "loiter", "estop".
- `/motors_enabled` (Bool): desired motor-enable state for Arduino or
  drive controllers (True in active mission modes, False in idle/estop).

This node does **not** talk to Nav2 directly yet; it is a thin control
surface other nodes can subscribe to when wiring up autonomous
behavior, RTB, and loiter logic.
"""

from __future__ import annotations

from typing import Literal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


MissionMode = Literal[
    "idle",
    "manual",
    "rtb",
    "loiter",
    "estop",
    "waypoint:default",
]


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")

        self._mode: MissionMode = "idle"

        self.state_pub = self.create_publisher(String, "/mission_state", 10)
        self.motors_enabled_pub = self.create_publisher(Bool, "/motors_enabled", 10)

        self.create_subscription(String, "/mission_command", self._on_command, 10)

        # Publish initial state
        self._publish_state()

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if cmd == "manual":
            self._set_mode("manual")
        elif cmd.startswith("waypoint"):
            # For now, treat any waypoint:* as a single "waypoint:default" mode.
            self._set_mode("waypoint:default")
        elif cmd == "rtb":
            self._set_mode("rtb")
        elif cmd == "loiter":
            self._set_mode("loiter")
        elif cmd == "estop":
            self._set_mode("estop")
        elif cmd == "idle":
            self._set_mode("idle")
        else:
            self.get_logger().warn(f"Unknown mission command: {cmd!r}")

    def _set_mode(self, mode: MissionMode) -> None:
        if mode == self._mode:
            return
        self._mode = mode
        self.get_logger().info(f"Mission mode -> {mode}")
        self._publish_state()

    def _publish_state(self) -> None:
        # Mission state
        self.state_pub.publish(String(data=self._mode))

        # Motors enabled policy: active in manual/rtb/loiter/waypoint, off in idle/estop.
        motors_on = self._mode in {"manual", "rtb", "loiter", "waypoint:default"}
        self.motors_enabled_pub.publish(Bool(data=motors_on))


def main() -> None:
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - manual execution
    main()
