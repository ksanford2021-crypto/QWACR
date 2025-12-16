#!/usr/bin/env python3
"""
Test script to verify non-zero velocity commands are reaching arduino_sim.
Sends TwistStamped messages with proper timestamps to /diff_cont/cmd_vel
"""

import sys
import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Header

def main():
    rclpy.init()
    node = rclpy.create_node('test_cmd_publisher')
    pub = node.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
    
    # Parse command line arguments
    linear_x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    angular_z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    
    print(f"Publishing TwistStamped commands:")
    print(f"  linear.x = {linear_x} m/s")
    print(f"  angular.z = {angular_z} rad/s")
    print(f"  duration = {duration} seconds")
    print(f"\nWatch /tmp/launch_debug.log for:")
    print(f"  hw_commands: [val1, val2, val3, val4]")
    print(f"  Received commands: [val1, val2, val3, val4]")
    
    msg = TwistStamped()
    msg.header.frame_id = "base_link"
    msg.twist.linear = Vector3(x=linear_x, y=0.0, z=0.0)
    msg.twist.angular = Vector3(x=0.0, y=0.0, z=angular_z)
    
    start_time = node.get_clock().now()
    count = 0
    
    def timer_callback():
        nonlocal count
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed < duration:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            count += 1
            print(f"Published #{count}: linear.x={msg.twist.linear.x:5.2f}, angular.z={msg.twist.angular.z:5.2f}")
        else:
            print(f"\nTest complete! Published {count} messages in {duration}s")
            rclpy.shutdown()
    
    node.create_timer(0.5, timer_callback)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
