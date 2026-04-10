#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Temperature, RelativeHumidity
import serial

from dgs2_decode import Dgs2Measurement, parse_measurement


class Dgs2Node(Node):
    def __init__(self, name: str, default_port: str) -> None:
        super().__init__(name)

        port = self.declare_parameter("port", default_port).get_parameter_value().string_value
        baud = self.declare_parameter("baud", 9600).get_parameter_value().integer_value

        self.get_logger().info(f"Opening DGS2 sensor on {port} @ {baud} baud")

        # Decoded outputs
        self.ppb_pub = self.create_publisher(Float32, f"gas/{name}_ppb", 10)
        self.temp_pub = self.create_publisher(Temperature, f"gas/{name}_temperature", 10)
        self.rh_pub = self.create_publisher(RelativeHumidity, f"gas/{name}_relative_humidity", 10)
        # Raw line for debugging / logging
        self.raw_pub = self.create_publisher(String, f"gas/{name}_raw", 10)

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=1.0)
        except serial.SerialException as exc:
            self.get_logger().error(f"Failed to open serial port {port}: {exc}")
            self.ser = None
            return

        # Put sensor into continuous-measurement mode ('C' command, per manual)
        try:
            self.ser.write(b"C\r")
            self.ser.flush()
            self.get_logger().info("Sent 'C' command to enable continuous measurements")
        except serial.SerialException as exc:
            self.get_logger().warn(f"Failed to send initial 'C' command: {exc}")

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _publish_measurement(self, m: Dgs2Measurement) -> None:
        # Raw string
        raw_msg = String()
        raw_msg.data = m.raw_line
        self.raw_pub.publish(raw_msg)

        # Gas concentration in ppb
        ppb_msg = Float32()
        ppb_msg.data = float(m.ppb)
        self.ppb_pub.publish(ppb_msg)

        # Temperature in °C
        temp_msg = Temperature()
        temp_msg.temperature = float(m.temperature_c)
        temp_msg.variance = 0.0
        self.temp_pub.publish(temp_msg)

        # Relative humidity as fraction [0, 1]
        rh_msg = RelativeHumidity()
        rh_msg.relative_humidity = float(m.relative_humidity_percent) / 100.0
        rh_msg.variance = 0.0
        self.rh_pub.publish(rh_msg)

    def _reader_loop(self) -> None:
        if self.ser is None:
            return
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline()
            except serial.SerialException as exc:
                self.get_logger().warn(f"Serial read error: {exc}")
                break
            if not line:
                continue

            measurement = parse_measurement(line)
            if measurement is None:
                self.get_logger().warn(f"Failed to parse DGS2 line: {line!r}")
                continue

            self._publish_measurement(measurement)

    def destroy_node(self) -> None:  # type: ignore[override]
        self._stop_event.set()
        try:
            if hasattr(self, "_thread"):
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if getattr(self, "ser", None) is not None:
                try:
                    # Toggle continuous mode off on shutdown
                    self.ser.write(b"C\r")
                    self.ser.flush()
                except serial.SerialException:
                    pass
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    # Default CO port is ttyAMA3; override with port:=... if needed
    node = Dgs2Node("co", "/dev/ttyAMA3")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
