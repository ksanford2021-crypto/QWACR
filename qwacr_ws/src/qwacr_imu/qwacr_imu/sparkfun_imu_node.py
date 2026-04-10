#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Imu
from smbus2 import SMBus

try:
    import qwiic_ism330dhcx  # SparkFun Python library for ISM330DHCX
except ImportError:
    qwiic_ism330dhcx = None


I2C_BUS = 1
MUX_ADDR = 0x70
MUX_PORT_DEFAULT = 0x08  # default mux channel for IMU; can be overridden via parameter


class SparkFunImuNode(Node):
    """ROS 2 node that reads the SparkFun 9DoF ISM330DHCX over I2C and publishes sensor_msgs/Imu.

    This node uses the SparkFun qwiic_ism330dhcx Python library. Only accel + gyro are used;
    the magnetometer on the breakout is intentionally ignored per project requirements.
    """

    def __init__(self) -> None:
        super().__init__('sparkfun_imu_node')

        if qwiic_ism330dhcx is None:
            self.get_logger().error(
                'qwiic_ism330dhcx Python package not found. Install it on the Pi, e.g. "pip install sparkfun-qwiic-ism330dhcx".'
            )

        # Parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('mux_port', MUX_PORT_DEFAULT)

        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate_hz: float = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 100.0

        self.mux_port: int = int(self.get_parameter('mux_port').get_parameter_value().integer_value)

        # Local handle to I2C bus for mux selection
        self._bus: Optional[SMBus] = None
        try:
            self._bus = SMBus(I2C_BUS)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to open I2C bus {I2C_BUS}: {exc!r}')
            self._bus = None

        self._select_mux_channel()

        # Initialize IMU
        self.imu: Optional[object] = None
        if qwiic_ism330dhcx is not None:
            try:
                # Handle possible class-name differences between library versions
                imu_cls = getattr(qwiic_ism330dhcx, 'QwiicIsm330dhcx', None)
                if imu_cls is None:
                    imu_cls = getattr(qwiic_ism330dhcx, 'QwiicIsm330DHCX', None)

                # As a fallback, search for any callable whose name contains "ism330"
                if imu_cls is None:
                    for attr_name in dir(qwiic_ism330dhcx):
                        if 'ism330' in attr_name.lower():
                            candidate = getattr(qwiic_ism330dhcx, attr_name)
                            if callable(candidate):
                                imu_cls = candidate
                                break

                if imu_cls is None:
                    raise AttributeError(
                        'qwiic_ism330dhcx module does not expose an ISM330 class; '
                        'check that the correct SparkFun library is installed.'
                    )

                # We know from i2cdetect that the device responds at 0x6B on mux
                # port mux_port. Select the mux channel, then force the address.
                self._select_mux_channel()
                try:
                    self.imu = imu_cls(address=0x6B)
                except TypeError:
                    self.imu = imu_cls(0x6B)

                # Initialize without relying on is_connected(), and let runtime
                # reads report issues via warnings.
                if hasattr(self.imu, 'begin'):
                    self.imu.begin()

                # Apply a basic configuration similar to the SparkFun
                # example so the device actually streams data.
                try:
                    if hasattr(self.imu, 'device_reset') and hasattr(self.imu, 'get_device_reset'):
                        self.imu.device_reset()
                        # Give the device a moment to come out of reset.
                        import time as _time  # local import to avoid top-level dependency

                        for _ in range(10):
                            if self.imu.get_device_reset() is False:
                                break
                            _time.sleep(0.01)

                    if hasattr(self.imu, 'set_device_config'):
                        self.imu.set_device_config()
                    if hasattr(self.imu, 'set_block_data_update'):
                        self.imu.set_block_data_update()

                    # Accel configuration
                    if hasattr(self.imu, 'set_accel_data_rate') and hasattr(self.imu, 'kXlOdr104Hz'):
                        self.imu.set_accel_data_rate(self.imu.kXlOdr104Hz)
                    if hasattr(self.imu, 'set_accel_full_scale') and hasattr(self.imu, 'kXlFs4g'):
                        self.imu.set_accel_full_scale(self.imu.kXlFs4g)

                    # Gyro configuration
                    if hasattr(self.imu, 'set_gyro_data_rate') and hasattr(self.imu, 'kGyroOdr104Hz'):
                        self.imu.set_gyro_data_rate(self.imu.kGyroOdr104Hz)
                    if hasattr(self.imu, 'set_gyro_full_scale') and hasattr(self.imu, 'kGyroFs500dps'):
                        self.imu.set_gyro_full_scale(self.imu.kGyroFs500dps)

                    # Filters (optional; best-effort)
                    if hasattr(self.imu, 'set_accel_filter_lp2'):
                        self.imu.set_accel_filter_lp2()
                    if hasattr(self.imu, 'set_accel_slope_filter') and hasattr(self.imu, 'kLpOdrDiv100'):
                        self.imu.set_accel_slope_filter(self.imu.kLpOdrDiv100)
                    if hasattr(self.imu, 'set_gyro_filter_lp1'):
                        self.imu.set_gyro_filter_lp1()
                    if hasattr(self.imu, 'set_gyro_lp1_bandwidth') and hasattr(self.imu, 'kBwMedium'):
                        self.imu.set_gyro_lp1_bandwidth(self.imu.kBwMedium)
                except Exception as cfg_exc:  # noqa: BLE001
                    self.get_logger().warn(f'Failed to apply IMU configuration: {cfg_exc!r}')

                self.get_logger().info('Initialized SparkFun ISM330DHCX IMU at I2C address 0x6B.')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Failed to initialize ISM330DHCX: {exc!r}')
                self.imu = None

        # Publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 50)

        # Timer
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self) -> None:
        if self.imu is None:
            return

        try:
            self._select_mux_channel()

            # For the current SparkFun qwiic_ism330dhcx library, readings are
            # typically obtained via get_accel() and get_gyro(). Some
            # versions provide check_status() but we don't rely on it, since
            # configuration can vary.
            accel = None
            gyro = None
            if hasattr(self.imu, 'get_accel'):
                accel = self.imu.get_accel()
            if hasattr(self.imu, 'get_gyro'):
                gyro = self.imu.get_gyro()

            if accel is None or gyro is None:
                return

            # SparkFun driver returns accel in milli-g and gyro in
            # milli-deg-per-second. Convert to g and dps here.
            ax_mg = float(getattr(accel, 'xData', 0.0))
            ay_mg = float(getattr(accel, 'yData', 0.0))
            az_mg = float(getattr(accel, 'zData', 0.0))
            gx_mdps = float(getattr(gyro, 'xData', 0.0))
            gy_mdps = float(getattr(gyro, 'yData', 0.0))
            gz_mdps = float(getattr(gyro, 'zData', 0.0))

            ax_g = ax_mg / 1000.0
            ay_g = ay_mg / 1000.0
            az_g = az_mg / 1000.0
            gx_dps = gx_mdps / 1000.0
            gy_dps = gy_mdps / 1000.0
            gz_dps = gz_mdps / 1000.0
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to read IMU data: {exc!r}')
            return

        msg = Imu()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        # Orientation not estimated here; let robot_localization integrate gyro + accel.
        msg.orientation_covariance[0] = -1.0

        # Angular velocity: convert from deg/s to rad/s
        d2r = math.pi / 180.0
        msg.angular_velocity.x = gx_dps * d2r
        msg.angular_velocity.y = gy_dps * d2r
        msg.angular_velocity.z = gz_dps * d2r
        # Simple default covariances; to be tuned from data.
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        # Linear acceleration: convert from g to m/s^2
        g_const = 9.80665
        msg.linear_acceleration.x = ax_g * g_const
        msg.linear_acceleration.y = ay_g * g_const
        msg.linear_acceleration.z = az_g * g_const
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.imu_pub.publish(msg)

    def _select_mux_channel(self) -> None:
        if self._bus is None:
            return
        try:
            self._bus.write_byte(MUX_ADDR, self.mux_port)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to select IMU mux channel 0x{self.mux_port:02x}: {exc!r}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SparkFunImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
