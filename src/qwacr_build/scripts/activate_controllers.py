#!/usr/bin/env python3
"""
Activate controllers after they've been loaded in inactive mode.
Useful for Gazebo mode where controllers load but aren't automatically activated.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController


class ControllerActivator(Node):
    def __init__(self):
        super().__init__('controller_activator')
        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

    def activate_controllers(self, controller_names, timeout=10.0):
        """Activate a list of controllers."""
        start_time = time.time()
        
        # Wait for service to be available
        while not self.switch_client.wait_for_service(timeout_sec=1.0):
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().error(
                    f'Controller manager service not available after {timeout}s')
                return False
            self.get_logger().info('Waiting for controller manager service...')

        # Switch controllers
        request = SwitchController.Request()
        request.activate_controllers = controller_names
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT

        try:
            future = self.switch_client.call_async(request)
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = future.result()
            if result.ok:
                self.get_logger().info(f'Successfully activated: {controller_names}')
                return True
            else:
                self.get_logger().error(f'Failed to activate controllers')
                return False
        except Exception as e:
            self.get_logger().error(f'Error activating controllers: {e}')
            return False


def main():
    rclpy.init()
    node = ControllerActivator()
    
    # Parse command line arguments
    controllers = sys.argv[1:] if len(sys.argv) > 1 else ['joint_broad', 'diff_cont']
    
    node.get_logger().info(f'Attempting to activate: {controllers}')
    
    # Try up to 3 times with increasing delays
    success = False
    for attempt in range(3):
        try:
            success = node.activate_controllers(controllers, timeout=15.0)
            if success:
                break
            node.get_logger().warn(f'Activation attempt {attempt + 1} failed, retrying...')
            time.sleep(2)
        except Exception as e:
            node.get_logger().error(f'Activation attempt {attempt + 1} error: {e}')
            time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
