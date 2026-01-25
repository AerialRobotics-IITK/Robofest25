import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import NavSatFix
import numpy as np
import os
import math
from pyproj import Geod

class HomePositionNode(Node):
    def __init__(self):
        # Set namespace from environment variable, defaulting to uav2
        self.namespace = f"uav{os.environ.get('MAV_ID', 2)}"
        
        # Initialize node with namespace
        super().__init__('home_position_node', namespace=self.namespace)
        
        # Define QoS profiles
        self.gps_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.local_pos_pub = self.create_publisher(
            PointStamped, 
            f'/{self.namespace}/local_pos', 
            self.qos_profile
        )
        
        self.offset_pub = self.create_publisher(
            PointStamped, 
            f'/{self.namespace}/offset', 
            self.gps_qos_profile
        )
        
        # Subscribers
        self.uav1_global_sub = self.create_subscription(
            NavSatFix,
            '/uav1/global_position/global',
            self.uav1_global_callback,
            self.qos_profile
        )
        
        self.own_global_sub = self.create_subscription(
            NavSatFix,
            f'/{self.namespace}/global_position/global',
            self.own_global_callback,
            self.qos_profile
        )
        
        self.own_local_sub = self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/local_position/pose',
            self.own_local_callback,
            self.qos_profile
        )
        
        # Data storage
        self.uav1_global_positions = []  # [lat, lon, alt]
        self.own_global_positions = []   # [lat, lon, alt]
        self.offset_position = None
        
        # Timing
        self.global_start_time = None
        
        # State flags
        self.global_averaging_complete = False
        
        # pyproj Geod for geodesic calculations
        self.geod = Geod(ellps='WGS84')
        
        self.get_logger().info(f'Home position node started for {self.namespace}')
        self.get_logger().info('Collecting global positions for offset calculation...')

    def uav1_global_callback(self, msg):
        """Collect uav1 global positions"""
        if self.global_averaging_complete:
            return
            
        if self.global_start_time is None:
            self.global_start_time = self.get_clock().now()
            
        current_time = self.get_clock().now()
        elapsed = (current_time - self.global_start_time).nanoseconds / 1e9
        
        if elapsed <= 5.0:
            self.uav1_global_positions.append([
                msg.latitude,
                msg.longitude,
                msg.altitude
            ])

    def own_global_callback(self, msg):
        """Collect own global positions"""
        if self.global_averaging_complete:
            return
            
        if self.global_start_time is None:
            self.global_start_time = self.get_clock().now()
            
        current_time = self.get_clock().now()
        elapsed = (current_time - self.global_start_time).nanoseconds / 1e9
        
        if elapsed <= 5.0:
            self.own_global_positions.append([
                msg.latitude,
                msg.longitude,
                msg.altitude
            ])
        
        # Check if we have enough data from both drones after 5 seconds
        if elapsed > 5.0 and not self.global_averaging_complete:
            if len(self.uav1_global_positions) > 0 and len(self.own_global_positions) > 0:
                self.global_averaging_complete = True
                self.offset_position = self.calculate_offset_vector()
                self.get_logger().info(f'Offset calculation complete: {self.offset_position}')
            else:
                self.get_logger().warn('Insufficient global position data from one or both drones')

    def own_local_callback(self, msg):
        """Handle own local position - publishes position minus offset"""
        # Get current local position
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_z = msg.pose.position.z
        
        # Subtract offset if available
        if self.offset_position is not None:
            current_x += self.offset_position.point.x
            current_y += self.offset_position.point.y
            current_z += self.offset_position.point.z
        else:
            self.get_logger().debug(f"Offset Not set for {self.namespace}")
            return
        
        # Publish position (with offset subtracted)
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = current_x
        point_msg.point.y = current_y
        point_msg.point.z = current_z
        self.local_pos_pub.publish(point_msg)
        
        # Publish offset (persists due to TRANSIENT_LOCAL)
        if self.offset_position is not None:
            self.offset_pub.publish(self.offset_position)

    def calculate_offset_vector(self):
        """Calculate offset vector between average positions of two drones in meters"""
        if not self.uav1_global_positions or not self.own_global_positions:
            self.get_logger().warn('Missing global position data for offset calculation!')
            return None
            
        # Calculate average GPS positions for both drones
        avg_uav1_gps = np.mean(self.uav1_global_positions, axis=0)
        avg_own_gps = np.mean(self.own_global_positions, axis=0)
        
        # Extract coordinates [lat, lon, alt]
        lat1, lon1, alt1 = avg_uav1_gps
        lat2, lon2, alt2 = avg_own_gps
        
        # Calculate geodesic distance and azimuth from uav1 to own position
        azimuth_deg, _, distance_m = self.geod.inv(lon1, lat1, lon2, lat2)
        
        # Convert azimuth to radians for trigonometric functions
        azimuth_rad = math.radians(azimuth_deg)
        
        # Calculate x (easting) and y (northing) components
        x_offset = distance_m * math.sin(azimuth_rad)
        y_offset = distance_m * math.cos(azimuth_rad)
        
        # z offset is difference in altitude
        z_offset = alt2 - alt1
        
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = x_offset
        point_msg.point.y = y_offset
        point_msg.point.z = z_offset
        
        return point_msg

def main(args=None):
    rclpy.init(args=args)
    node = HomePositionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
