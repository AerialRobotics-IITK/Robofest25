#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from pyproj import Geod

class DronePointCalculator(Node):
    def __init__(self):
        super().__init__('drone_point_calculator')
        self.g = Geod(ellps='clrk66')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.drone1_topic = "/uav1/global_position/global"
        self.drone2_topic = "/uav2/global_position/global"
        self.output_topic = "/offset"
        
        self.drone1_pose = None
        self.drone2_pose = None

        self.drone1_sub = self.create_subscription(
            NavSatFix,
            self.drone1_topic,
            self.drone1_callback,
            qos_profile
        )
        
        self.drone2_sub = self.create_subscription(
            NavSatFix,
            self.drone2_topic,
            self.drone2_callback,
            qos_profile
        )
        self.result_publisher = self.create_publisher(
            PointStamped,
            self.output_topic,
            10
        )
        
        self.time_period = 0.1
        self.timer = self.create_timer(self.time_period, self.timer_callback)

        self.get_logger().info(f"Subscribing to drone 1 pose on: {self.drone1_topic}")
        self.get_logger().info(f"Subscribing to drone 2 pose on: {self.drone2_topic}")
        self.get_logger().info(f"Publishing calculated point to: {self.output_topic}")

    def drone1_callback(self, msg):
        self.drone1_pose = msg

    def drone2_callback(self, msg):
        self.drone2_pose = msg

    def timer_callback(self):
        if self.drone1_pose and self.drone2_pose:
            
            pos1 = self.drone1_pose
            pos2 = self.drone2_pose

            result_point_msg = PointStamped()
            a1,a2,d = self.g.inv(pos1.longitude,pos1.latitude,pos2.longitude,pos2.latitude)

            result_point_msg.point.x = d*math.cos(a1)
            result_point_msg.point.y = d*math.sin(a1)
            
            if result_point_msg:
                self.result_publisher.publish(result_point_msg)
        
        elif not self.drone1_pose:
            self.get_logger().warn("Waiting for pose from drone 1...")
        elif not self.drone2_pose:
            self.get_logger().warn("Waiting for pose from drone 2...")

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
