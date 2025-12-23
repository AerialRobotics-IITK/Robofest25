#!/usr/bin/env python

import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PointStamped,PoseStamped
from geographic_msgs.msg import GeoPointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy,DurabilityPolicy
import math
from pyproj import Geod

class DronePointCalculator(Node):
    def __init__(self):
        super().__init__('drone_point_calculator')
        self.g = Geod(ellps='clrk66')
        gps_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Critical!
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
        )
        self.id = os.environ.get("MAV_ID",2)
        self.drone1_topic = "/uav1/global_position/gp_origin"
        self.drone2_topic = f"/uav{self.id}/global_position/gp_origin"
        self.drone2_local_topic = f"/uav{self.id}/local_position/pose"
        self.output_topic = f"/uav{self.id}/local_pos"
        
        self.drone1_pose = None
        self.drone2_pose = None

        self.drone1_sub = self.create_subscription(
            GeoPointStamped,
            self.drone1_topic,
            self.drone1_callback,
            gps_qos_profile
        )
        self.drone2_sub = self.create_subscription(
            GeoPointStamped,
            self.drone2_topic,
            self.drone2_callback,
            gps_qos_profile
        )
        self.drone2_local_sub = self.create_subscription(
            PoseStamped,
            self.drone2_local_topic,
            self.drone2_local_callback,
            qos_profile
        )
        self.result_publisher = self.create_publisher(
            PointStamped,
            self.output_topic,
            10
        )
        
        self.get_logger().info(f"Subscribing to drone 1 pose on: {self.drone1_topic}")
        self.get_logger().info(f"Subscribing to drone 2 pose on: {self.drone2_topic}")
        self.get_logger().info(f"Publishing calculated point to: {self.output_topic}")

    def drone1_callback(self, msg):
        self.drone1_pose = msg

    def drone2_callback(self, msg):
        self.drone2_pose = msg

    def drone2_local_callback(self, msg):
        if self.drone1_pose and self.drone2_pose:
            
            pos1 = self.drone1_pose
            pos2 = self.drone2_pose

            result_point_msg = PointStamped()
            a1,_,d = self.g.inv(pos1.position.longitude,pos1.position.latitude,pos2.position.longitude,pos2.position.latitude)

            offset_x = d*math.cos(a1)
            offset_y = d*math.sin(a1)
            result_point_msg.point.x = msg.pose.position.x + offset_x
            result_point_msg.point.y = msg.pose.position.y + offset_y
            
            if result_point_msg:
                self.result_publisher.publish(result_point_msg)
        
        else:
            if not self.drone1_pose:
                self.get_logger().warn("Waiting for gps origin from drone 1...")
            if not self.drone2_pose:
                self.get_logger().warn(f"Waiting for gps origin from drone {self.id}...")

def main(args=None):
    rclpy.init(args=args)
    
    calculator_node = DronePointCalculator()
    
    try:
        rclpy.spin(calculator_node)
    except KeyboardInterrupt:
        pass
    finally:
        calculator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
