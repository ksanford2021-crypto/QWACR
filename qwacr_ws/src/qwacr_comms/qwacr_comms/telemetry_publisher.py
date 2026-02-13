"""Collects telemetry and publishes JSON for LoRa transmission."""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class TelemetryPublisher(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_publisher")

        self.declare_parameter("rate_hz", 1.0)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self._battery: Optional[BatteryState] = None
        self._gps: Optional[NavSatFix] = None
        self._odom: Optional[Odometry] = None

        self.pub = self.create_publisher(String, "/lora/telemetry_out", 10)

        self.create_subscription(BatteryState, "/battery/state", self._on_battery, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._on_gps, 10)
        self.create_subscription(Odometry, "/diff_cont/odom", self._on_odom, 10)

        self.create_timer(1.0 / self.rate_hz, self._publish)

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = msg

    def _on_gps(self, msg: NavSatFix) -> None:
        self._gps = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _publish(self) -> None:
        if self._battery is None or self._gps is None or self._odom is None:
            return

        payload = {
            "gps_lat": float(self._gps.latitude),
            "gps_lon": float(self._gps.longitude),
            "gps_fix": int(self._gps.status.status >= 0),
            "battery_v": float(self._battery.voltage),
            "battery_a": float(self._battery.current),
            "system_status": 0,
            "odom_distance": float(self._odom.pose.pose.position.x),
            "rssi": 0,
            "snr": 0,
        }
        self.pub.publish(String(data=json.dumps(payload)))


def main() -> None:
    rclpy.init()
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
