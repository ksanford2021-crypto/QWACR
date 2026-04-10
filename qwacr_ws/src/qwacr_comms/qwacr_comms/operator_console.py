"""Terminal-based operator console for base-station Pi.

Provides a single SSH-friendly UI for:
- Viewing LoRa telemetry summaries.
- Sending teleop commands.
- Selecting which camera the robot streams over HaLow.

This is intentionally curses-based to avoid heavy GUI dependencies
while still giving an operator "one screen" control surface over SSH.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import curses

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


@dataclass
class TelemetrySnapshot:
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None
    gps_fix: Optional[int] = None
    battery_v: Optional[float] = None
    battery_a: Optional[float] = None
    odom_distance: Optional[float] = None
    system_status: Optional[int] = None
    # Placeholder fields for future fire payload summary when added to LoRa
    fire_status: Dict[str, Any] = field(default_factory=dict)


class OperatorConsole(Node):
    def __init__(self) -> None:
        super().__init__("operator_console")

        # Publishers for teleop and camera selection.
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.camera_select_pub = self.create_publisher(String, "/camera_select", 10)

        # Subscribe to LoRa telemetry JSON from lora_bridge.
        self.telemetry: TelemetrySnapshot = TelemetrySnapshot()
        self.create_subscription(String, "/lora/telemetry_in", self._on_telemetry, 10)

        # Track current teleop command to show in UI.
        self._last_cmd: Twist = Twist()

    # ROS callbacks -----------------------------------------------------

    def _on_telemetry(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"Failed to parse telemetry JSON: {exc}")
            return

        ts = TelemetrySnapshot()
        ts.gps_lat = float(data.get("gps_lat")) if "gps_lat" in data else None
        ts.gps_lon = float(data.get("gps_lon")) if "gps_lon" in data else None
        ts.gps_fix = int(data.get("gps_fix")) if "gps_fix" in data else None
        ts.battery_v = float(data.get("battery_v")) if "battery_v" in data else None
        ts.battery_a = float(data.get("battery_a")) if "battery_a" in data else None
        ts.odom_distance = (
            float(data.get("odom_distance")) if "odom_distance" in data else None
        )
        ts.system_status = (
            int(data.get("system_status")) if "system_status" in data else None
        )

        # Fire payload summary can be added later once encoded into LoRa
        fire = data.get("fire_status")
        if isinstance(fire, dict):
            ts.fire_status = fire

        self.telemetry = ts

    # Teleop helpers ----------------------------------------------------

    def send_teleop(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        self._last_cmd = msg

    def send_camera_select(self, name: str) -> None:
        self.camera_select_pub.publish(String(data=name))


# Curses UI -------------------------------------------------------------


def _draw_ui(stdscr: "curses._CursesWindow", node: OperatorConsole) -> None:
    stdscr.clear()
    max_y, max_x = stdscr.getmaxyx()

    # Header
    stdscr.addstr(0, 0, "QWACR Operator Console (base-station Pi)")
    stdscr.hline(1, 0, ord("-"), max_x)

    # Telemetry section
    stdscr.addstr(2, 0, "Telemetry (from /lora/telemetry_in):")
    t = node.telemetry
    stdscr.addstr(3, 2, f"GPS: lat={t.gps_lat:.6f if t.gps_lat is not None else '---'} "
                          f"lon={t.gps_lon:.6f if t.gps_lon is not None else '---'} "
                          f"fix={t.gps_fix if t.gps_fix is not None else '-'}")
    stdscr.addstr(4, 2, f"Battery: V={t.battery_v:.2f if t.battery_v is not None else '---'} "
                          f"A={t.battery_a:.2f if t.battery_a is not None else '---'}")
    stdscr.addstr(5, 2, f"Odom distance: {t.odom_distance:.1f if t.odom_distance is not None else '---'} m")

    # Fire payload summary placeholder (will populate once LoRa carries it)
    stdscr.addstr(7, 0, "Fire payload summary:")
    if t.fire_status:
        line = 8
        for key, value in t.fire_status.items():
            if line >= max_y - 6:
                break
            stdscr.addstr(line, 2, f"{key}: {value}")
            line += 1
    else:
        stdscr.addstr(8, 2, "(waiting for fire_status in telemetry)")

    # Teleop status
    stdscr.addstr(max_y - 6, 0, "Teleop:")
    stdscr.addstr(
        max_y - 5,
        2,
        f"linear.x={node._last_cmd.linear.x:+.2f}  angular.z={node._last_cmd.angular.z:+.2f}",
    )

    # Controls
    stdscr.hline(max_y - 4, 0, ord("-"), max_x)
    stdscr.addstr(max_y - 3, 0, "Controls:")
    stdscr.addstr(max_y - 2, 2, "W/S: forward/backward   A/D: rotate left/right   Space: stop")
    stdscr.addstr(max_y - 1, 2, "F: front camera   L: left camera   Q: quit")

    stdscr.refresh()


def run_curses(node: OperatorConsole) -> None:
    # Main curses loop; run until user presses Q.
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    try:
        last_draw = 0.0
        while rclpy.ok():
            # Pump ROS events without blocking.
            rclpy.spin_once(node, timeout_sec=0.0)

            now = time.time()
            if now - last_draw > 0.1:
                _draw_ui(stdscr, node)
                last_draw = now

            ch = stdscr.getch()
            if ch == -1:
                time.sleep(0.02)
                continue

            ch = chr(ch).lower() if 0 <= ch < 256 else ""

            if ch == "q":
                break
            elif ch == "w":
                node.send_teleop(0.4, 0.0)
            elif ch == "s":
                node.send_teleop(-0.4, 0.0)
            elif ch == "a":
                node.send_teleop(0.0, 1.2)
            elif ch == "d":
                node.send_teleop(0.0, -1.2)
            elif ch == " ":
                node.send_teleop(0.0, 0.0)
            elif ch == "f":
                node.send_camera_select("front")
            elif ch == "l":
                node.send_camera_select("left")

            time.sleep(0.02)
    finally:
        # Restore terminal state.
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()


def main() -> None:
    rclpy.init()
    node = OperatorConsole()
    try:
        run_curses(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - manual run
    main()
