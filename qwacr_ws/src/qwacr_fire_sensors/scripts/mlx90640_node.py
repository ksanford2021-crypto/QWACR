#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from smbus2 import SMBus

I2C_BUS = 1
MUX_ADDR = 0x70
MLX90640_ADDR_DEFAULT = 0x33
MUX_PORT_DEFAULT = 0x02  # bit 1

ROWS = 24
COLS = 32


class Mlx90640Node(Node):
    def __init__(self) -> None:
        super().__init__("mlx90640_node")

        self.bus = SMBus(I2C_BUS)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter("mux_port", MUX_PORT_DEFAULT)
        self.declare_parameter("i2c_addr", MLX90640_ADDR_DEFAULT)
        self.declare_parameter("topic", "thermal/array")
        self.declare_parameter("frame_id", "thermal_frame")

        self.mux_port = int(self.get_parameter("mux_port").value)
        self.i2c_addr = int(self.get_parameter("i2c_addr").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        topic = str(self.get_parameter("topic").value)
        self.image_pub = self.create_publisher(Image, topic, 10)

        period = float(self.get_parameter("poll_period").value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"MLX90640 node started on topic '{topic}' via mux port 0x{self.mux_port:02x} "
            f"at I2C addr 0x{self.i2c_addr:02x}"
        )

    def select_mux_channel(self) -> None:
        self.bus.write_byte(MUX_ADDR, self.mux_port)

    def timer_callback(self) -> None:
        try:
            self.select_mux_channel()
            time.sleep(0.002)

            # This is NOT a full MLX90640 implementation. We just read a very
            # small register block to prove connectivity and publish
            # placeholder data. Linux SMBus block reads are limited to 32
            # bytes, so keep the length at or below that limit.
            reg_high = 0x24
            reg_low = 0x00
            self.bus.write_i2c_block_data(self.i2c_addr, reg_high, [reg_low])
            time.sleep(0.01)
            block_len = 32
            raw = self.bus.read_i2c_block_data(self.i2c_addr, reg_high, block_len)
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f"I2C error talking to MLX90640: {exc}")
            return

        # Build a dummy grayscale image just to push bytes through ROS.
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.frame_id
        img.height = ROWS
        img.width = COLS
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = COLS

        # Use raw bytes truncated/extended to ROWS*COLS
        flat = bytes(raw) * ((ROWS * COLS + len(raw) - 1) // len(raw))
        img.data = list(flat[: ROWS * COLS])

        self.image_pub.publish(img)

    def destroy_node(self) -> None:  # type: ignore[override]
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Mlx90640Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
