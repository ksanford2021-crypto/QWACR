#!/usr/bin/env python3
"""
QWACR GPS Driver Node
Reads NMEA sentences from GPS module via USB/UART and publishes NavSatFix messages.
Includes GPS→ENU (East-North-Up) conversion for local navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import serial
import math
import re


class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('publish_enu', True)
        self.declare_parameter('publish_heading', True)
        self.declare_parameter('origin_lat', 0.0)  # Set on first fix if 0
        self.declare_parameter('origin_lon', 0.0)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_enu = self.get_parameter('publish_enu').value
        self.publish_heading = self.get_parameter('publish_heading').value
        
        # Publishers
        self.navsat_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.enu_pub = self.create_publisher(PoseStamped, '/gps/enu_pose', 10) if self.publish_enu else None
        self.heading_pub = self.create_publisher(Float64, '/gps/heading', 10) if self.publish_heading else None

        # Latest heading in degrees (True), None until received
        self.latest_heading_deg = None
        
        # Origin for ENU conversion (set on first fix or from params)
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.origin_set = (self.origin_lat != 0.0 or self.origin_lon != 0.0)
        
        # Serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self.get_logger().info(f'Connected to GPS on {self.serial_port} @ {self.baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.serial_port}: {e}')
            raise
        
        # Timer for reading serial data
        self.create_timer(0.1, self.read_gps)  # 10 Hz polling
        
        self.get_logger().info('GPS driver node started')
    
    def read_gps(self):
        """Read and parse NMEA sentences from GPS"""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # Parse GPGGA (Global Positioning System Fix Data)
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    self.parse_gpgga(line)
                
                # Parse GPRMC (Recommended Minimum Specific GPS/Transit Data)
                elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    self.parse_gprmc(line)
                # Parse HDT (Heading - True). Talker ID can vary (e.g., GP, HE)
                elif re.match(r'^\$..HDT,', line):
                    self.parse_gphdt(line)
                    
        except Exception as e:
            self.get_logger().error(f'Error reading GPS: {e}')
    
    def parse_gpgga(self, sentence):
        """Parse GPGGA NMEA sentence"""
        # Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        parts = sentence.split(',')
        if len(parts) < 15:
            return
        
        try:
            # Latitude
            lat_str = parts[2]
            lat_dir = parts[3]
            if lat_str and lat_dir:
                lat = self.parse_coordinate(lat_str, lat_dir)
            else:
                return
            
            # Longitude
            lon_str = parts[4]
            lon_dir = parts[5]
            if lon_str and lon_dir:
                lon = self.parse_coordinate(lon_str, lon_dir)
            else:
                return
            
            # Fix quality (0=invalid, 1=GPS fix, 2=DGPS fix)
            fix_quality = int(parts[6]) if parts[6] else 0
            
            # Number of satellites
            num_sats = int(parts[7]) if parts[7] else 0
            
            # Altitude
            altitude = float(parts[9]) if parts[9] else 0.0
            
            # Set origin on first valid fix
            if not self.origin_set and fix_quality > 0:
                self.origin_lat = lat
                self.origin_lon = lon
                self.origin_set = True
                self.get_logger().info(f'GPS origin set: lat={lat:.6f}, lon={lon:.6f}')
            
            # Publish NavSatFix
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = altitude
            
            # Status
            msg.status.status = NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            
            # Covariance (rough estimate based on fix type)
            if fix_quality == 2:  # DGPS
                cov = 1.0  # ~1m accuracy
            elif fix_quality == 1:  # GPS
                cov = 5.0  # ~5m accuracy
            else:
                cov = 100.0  # No fix
            
            msg.position_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            self.navsat_pub.publish(msg)
            
            # Publish ENU pose if enabled
            if self.publish_enu and self.origin_set:
                enu_pose = self.gps_to_enu(lat, lon, altitude)
                self.enu_pub.publish(enu_pose)
            
            # Log periodically
            if num_sats > 0:
                self.get_logger().info(
                    f'GPS: lat={lat:.6f}, lon={lon:.6f}, alt={altitude:.1f}m, sats={num_sats}, fix={fix_quality}',
                    throttle_duration_sec=5.0
                )
                
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Error parsing GPGGA: {e}')
    
    def parse_gprmc(self, sentence):
        """Parse GPRMC NMEA sentence (for speed/course)"""
        # Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        # Can be extended to extract speed and course if needed
        pass

    def parse_gphdt(self, sentence):
        """Parse GPHDT (True heading) NMEA sentence and publish heading"""
        # Example: $GPHDT,123.456,T*hh
        parts = sentence.split(',')
        if len(parts) < 3:
            return
        try:
            heading_str = parts[1]
            ref = parts[2][:1] if parts[2] else 'T'  # 'T' for true
            if heading_str:
                heading_deg = float(heading_str)
                self.latest_heading_deg = heading_deg if ref == 'T' else None
                if self.publish_heading and self.heading_pub and self.latest_heading_deg is not None:
                    msg = Float64()
                    msg.data = self.latest_heading_deg
                    self.heading_pub.publish(msg)
        except ValueError:
            self.get_logger().debug('Error parsing GPHDT heading')
    
    def parse_coordinate(self, coord_str, direction):
        """Convert NMEA coordinate to decimal degrees"""
        # Format: DDMM.MMMM or DDDMM.MMMM
        if not coord_str:
            return 0.0
        
        # Find decimal point
        dot_idx = coord_str.index('.')
        
        # Degrees: everything before last 2 digits before decimal
        degrees = float(coord_str[:dot_idx-2])
        
        # Minutes: last 2 digits before decimal + everything after
        minutes = float(coord_str[dot_idx-2:])
        
        # Convert to decimal degrees
        decimal = degrees + minutes / 60.0
        
        # Apply direction
        if direction in ['S', 'W']:
            decimal = -decimal
        
        return decimal
    
    def gps_to_enu(self, lat, lon, alt):
        """Convert GPS lat/lon to local ENU (East-North-Up) coordinates"""
        # Use local tangent plane approximation
        R = 6371000.0  # Earth radius in meters
        
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lat0_rad = math.radians(self.origin_lat)
        lon0_rad = math.radians(self.origin_lon)
        
        # ENU offsets from origin
        east = R * math.cos(lat0_rad) * (lon_rad - lon0_rad)
        north = R * (lat_rad - lat0_rad)
        up = alt  # Simplified; proper implementation would subtract origin altitude
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # ENU frame origin
        
        pose_msg.pose.position.x = east
        pose_msg.pose.position.y = north
        pose_msg.pose.position.z = up
        
        # Orientation: use latest true heading if available
        if self.latest_heading_deg is not None:
            yaw = math.radians(self.latest_heading_deg)
            pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
            pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            pose_msg.pose.orientation.w = 1.0
        
        return pose_msg
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('GPS serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
