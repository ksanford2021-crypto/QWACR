#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float32
from smbus2 import SMBus

I2C_BUS = 1
MUX_ADDR = 0x70
SEN54_ADDR = 0x69
MUX_PORT_SEN54 = 0x01  # bit 0


class Sen54Node(Node):
    def __init__(self) -> None:
        super().__init__("sen54_node")

        self.bus = SMBus(I2C_BUS)

        self.pm25_pub = self.create_publisher(Float32, "environment/pm2_5", 10)
        self.voc_pub = self.create_publisher(Float32, "environment/voc_index", 10)
        self.rh_pub = self.create_publisher(RelativeHumidity, "environment/humidity", 10)
        self.temp_pub = self.create_publisher(Temperature, "environment/temperature", 10)

        period = self.declare_parameter("poll_period", 1.0).get_parameter_value().double_value
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info("SEN54 node started (via I2C mux port 0)")

    def select_mux_channel(self) -> None:
        self.bus.write_byte(MUX_ADDR, MUX_PORT_SEN54)

    def timer_callback(self) -> None:
        try:
            self.select_mux_channel()
            time.sleep(0.002)

            # Start measurement (single-shot) - simplified; real driver would follow
            # Sensirion's recommended sequence. Here we just issue a read of the
            # "read measured values" command 0x03 0xC4 (SEN5x datasheet).
            self.bus.write_i2c_block_data(SEN54_ADDR, 0x03, [0xC4])
            time.sleep(0.1)

            # Read back 24 bytes (enough for PM values, humidity, temperature, VOC)
            data = self.bus.read_i2c_block_data(SEN54_ADDR, 0x00, 24)
        except OSError as exc:
            self.get_logger().warn(f"I2C error talking to SEN54: {exc}")
            return

        # This node only publishes simple placeholders; a proper implementation
        # should use Sensirion's SEN5x driver to decode CRC-protected words.
        # For now, scale a couple of bytes to plausible floats so the rest of the
        # system can be wired up.
        if len(data) < 8:
            return

        # Fake PM2.5 from first two data bytes
        pm25_raw = (data[0] << 8) | data[1]
        pm25 = float(pm25_raw) / 10.0

        voc_raw = (data[2] << 8) | data[3]
        voc_index = float(voc_raw) / 10.0

        rh_msg = RelativeHumidity()
        rh_msg.relative_humidity = float(((data[4] << 8) | data[5]) / 100.0)

        t_msg = Temperature()
        t_msg.temperature = float(((data[6] << 8) | data[7]) / 200.0)

        self.pm25_pub.publish(Float32(data=pm25))
        self.voc_pub.publish(Float32(data=voc_index))
        self.rh_pub.publish(rh_msg)
        self.temp_pub.publish(t_msg)

    def destroy_node(self) -> None:  # type: ignore[override]
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Sen54Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
