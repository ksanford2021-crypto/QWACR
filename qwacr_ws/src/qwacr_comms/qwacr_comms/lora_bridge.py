"""ROS2 bridge between LoRa ESP32 UART and ROS topics."""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from qwacr_comms.lora_protocol import (
    MSG_TELEMETRY,
    MSG_TELEOP,
    TeleopCommand,
    Telemetry,
    pack_telemetry,
    pack_teleop,
    parse_frame,
    unpack_telemetry,
    unpack_teleop,
)


try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - serial import is optional at runtime
    serial = None


class LoRaBridge(Node):
    def __init__(self) -> None:
        super().__init__("lora_bridge")

        self.declare_parameter("role", "robot")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("telemetry_rate", 1.0)
        self.declare_parameter("teleop_rate", 10.0)
        self.declare_parameter("enable_serial", True)

        self.role = self.get_parameter("role").value
        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.telemetry_rate = float(self.get_parameter("telemetry_rate").value)
        self.teleop_rate = float(self.get_parameter("teleop_rate").value)
        self.enable_serial = bool(self.get_parameter("enable_serial").value)

        self._serial = None
        self._rx_buffer = bytearray()

        self.teleop_pub = self.create_publisher(Twist, "/lora/cmd_vel_in", 10)
        self.telemetry_pub = self.create_publisher(String, "/lora/telemetry_in", 10)

        self.teleop_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self.telemetry_sub = self.create_subscription(
            String, "/lora/telemetry_out", self._on_telemetry_out, 10
        )

        if self.enable_serial:
            self._open_serial()
            self.create_timer(0.02, self._poll_serial)

        self.get_logger().info(f"LoRa bridge role={self.role}, port={self.serial_port}")

    def _open_serial(self) -> None:
        if serial is None:
            self.get_logger().error("pyserial not available. Install pyserial to use LoRa UART.")
            return
        try:
            self._serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0)
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.get_logger().error(f"Failed to open serial port: {exc}")
            self._serial = None

    def _poll_serial(self) -> None:
        if self._serial is None:
            return
        try:
            data = self._serial.read(self._serial.in_waiting or 1)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f"Serial read error: {exc}")
            return

        if data:
            self._rx_buffer.extend(data)

        while True:
            parsed, self._rx_buffer = parse_frame(self._rx_buffer)
            if parsed is None:
                break
            msg_id, payload = parsed
            if msg_id == MSG_TELEOP and self.role == "robot":
                cmd = unpack_teleop(payload)
                if cmd:
                    twist = Twist()
                    twist.linear.x = cmd.linear_x
                    twist.angular.z = cmd.angular_z
                    self.teleop_pub.publish(twist)
            elif msg_id == MSG_TELEMETRY and self.role == "base":
                telemetry = unpack_telemetry(payload)
                if telemetry:
                    self.telemetry_pub.publish(String(data=json.dumps(telemetry.__dict__)))

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self.role != "base":
            return
        if self._serial is None:
            return
        cmd = TeleopCommand(linear_x=msg.linear.x, angular_z=msg.angular.z, flags=0)
        packet = pack_teleop(cmd)
        try:
            self._serial.write(packet)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f"Serial write error: {exc}")

    def _on_telemetry_out(self, msg: String) -> None:
        if self.role != "robot":
            return
        if self._serial is None:
            return
        try:
            payload = json.loads(msg.data)
            telemetry = Telemetry(**payload)
        except Exception as exc:
            self.get_logger().warn(f"Telemetry JSON parse error: {exc}")
            return
        packet = pack_telemetry(telemetry)
        try:
            self._serial.write(packet)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f"Serial write error: {exc}")


def main() -> None:
    rclpy.init()
    node = LoRaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
