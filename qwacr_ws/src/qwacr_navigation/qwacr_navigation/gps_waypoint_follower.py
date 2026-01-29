#!/usr/bin/env python3
"""
GPS Waypoint Follower Node
Converts GPS waypoints to map frame goals and sends them to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import math
import json


class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        
        # Parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('loop_waypoints', False)
        
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        
        # GPS origin (should match your GPS driver)
        self.origin_set = (self.origin_lat != 0.0 or self.origin_lon != 0.0)
        
        # Action client for Nav2
        self.nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriber for GPS fix (to set origin if needed)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fix', self.gps_callback, 10)
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, '/waypoint_status', 10)
        
        # Waypoint list
        self.waypoints_gps = []
        self.current_waypoint_idx = 0
        
        # Load waypoints if file provided
        if self.waypoint_file:
            self.load_waypoints_from_file(self.waypoint_file)
        
        self.get_logger().info('GPS Waypoint Follower initialized')
        self.get_logger().info(f'Origin: ({self.origin_lat}, {self.origin_lon})')
    
    def gps_callback(self, msg):
        """Callback for GPS fix - used to set origin if not already set"""
        if not self.origin_set and msg.status.status >= 0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_set = True
            self.get_logger().info(f'GPS origin set from first fix: ({self.origin_lat}, {self.origin_lon})')
    
    def load_waypoints_from_file(self, filename):
        """Load GPS waypoints from JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
                self.waypoints_gps = data.get('waypoints', [])
            self.get_logger().info(f'Loaded {len(self.waypoints_gps)} waypoints from {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
    
    def gps_to_enu(self, lat, lon):
        """Convert GPS lat/lon to local ENU coordinates"""
        if not self.origin_set:
            self.get_logger().warn('Origin not set, cannot convert GPS to ENU')
            return None, None
        
        R = 6371000.0  # Earth radius in meters
        
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lat0_rad = math.radians(self.origin_lat)
        lon0_rad = math.radians(self.origin_lon)
        
        # ENU offsets from origin
        east = R * math.cos(lat0_rad) * (lon_rad - lon0_rad)
        north = R * (lat_rad - lat0_rad)
        
        return east, north
    
    def create_pose_stamped(self, x, y, yaw=0.0):
        """Create a PoseStamped message in map frame"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def send_waypoints(self, waypoints_gps):
        """Convert GPS waypoints to map frame and send to Nav2"""
        if not self.origin_set:
            self.get_logger().error('Cannot send waypoints - GPS origin not set')
            return False
        
        # Convert GPS waypoints to map frame
        poses = []
        for wp in waypoints_gps:
            lat = wp.get('lat')
            lon = wp.get('lon')
            yaw = wp.get('yaw', 0.0)  # Optional heading
            
            if lat is None or lon is None:
                self.get_logger().warn(f'Skipping invalid waypoint: {wp}')
                continue
            
            x, y = self.gps_to_enu(lat, lon)
            if x is not None:
                pose = self.create_pose_stamped(x, y, yaw)
                poses.append(pose)
        
        if not poses:
            self.get_logger().error('No valid waypoints to send')
            return False
        
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_through_poses_client.wait_for_server()
        
        # Create goal
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        
        # Send goal
        self.get_logger().info(f'Sending {len(poses)} waypoints to Nav2')
        self.send_goal_future = self.nav_through_poses_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Waypoint {feedback.current_waypoint + 1}/{feedback.number_of_poses_remaining + feedback.current_waypoint}',
            throttle_duration_sec=5.0)
    
    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Callback when navigation completes"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed')
        
        # If looping, restart waypoints
        if self.loop_waypoints and self.waypoints_gps:
            self.get_logger().info('Looping waypoints...')
            self.send_waypoints(self.waypoints_gps)
    
    def add_waypoint_gps(self, lat, lon, yaw=0.0):
        """Add a single GPS waypoint"""
        self.waypoints_gps.append({'lat': lat, 'lon': lon, 'yaw': yaw})
        self.get_logger().info(f'Added waypoint: ({lat}, {lon})')
    
    def start_navigation(self):
        """Start navigation through loaded waypoints"""
        if not self.waypoints_gps:
            self.get_logger().error('No waypoints loaded')
            return False
        
        return self.send_waypoints(self.waypoints_gps)


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointFollower()
    
    # Example: Add waypoints programmatically or wait for file load
    # node.add_waypoint_gps(26.3747, -80.1009)
    # node.add_waypoint_gps(26.3748, -80.1010)
    # node.start_navigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
