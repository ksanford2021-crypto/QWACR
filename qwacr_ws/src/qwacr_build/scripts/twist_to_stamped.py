#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        # Use relative names so CLI remapping can change them easily
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.get_logger().info('Twist→TwistStamped bridge ready: cmd_vel -> cmd_vel_stamped')

    def cb(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        # Leave frame_id empty unless the user remaps in tf; can be set via parameter if needed
        ts.twist = msg
        self.pub.publish(ts)


def main():
    rclpy.init()
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
