import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class PiInterfaceArdNode(Node):
    def __init__(self):
        super().__init__('pi_interface_ard_node')

        # Initialize serial communication with Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=.1)
        self.ser.flush()

        # Subscriber to cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for state topic
        self.state_publisher = self.create_publisher(JointState, 'state', 10)

        # Timer to periodically read from Arduino and publish state
        self.timer = self.create_timer(0.1, self.publish_state)

    def cmd_vel_callback(self, msg):
        # Send velocity commands to Arduino
        command = f"{msg.linear.x},{msg.angular.z}\n"
        self.ser.write(command.encode('utf-8'))

    def publish_state(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            try:
                position, velocity = map(float, line.split(','))
                state_msg = JointState()
                state_msg.position = position
                state_msg.velocity = velocity
                self.state_publisher.publish(state_msg)
            except ValueError:
                self.get_logger().error(f"Invalid data received from Arduino: {line}")