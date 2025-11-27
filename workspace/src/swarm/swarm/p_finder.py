#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class DronePointCalculator(Node):
    def __init__(self):
        super().__init__('drone_point_calculator')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.drone1_topic = "/uav1/local_position/pose"
        self.drone2_topic = "/uav2/local_position/pose"
        self.output_topic = "/p_point"
        
        self.declare_parameter("publish_rate", 10.0)
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        timer_period = 1.0 / publish_rate

        self.drone1_pose = None
        self.drone2_pose = None

        self.result_publisher = self.create_publisher(
            PointStamped, 
            self.output_topic, 
            qos_profile
        )

        self.drone1_sub = self.create_subscription(
            PoseStamped,
            self.drone1_topic,
            self.drone1_callback,
            qos_profile
        )
        
        self.drone2_sub = self.create_subscription(
            PoseStamped,
            self.drone2_topic,
            self.drone2_callback,
            qos_profile
        )
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Subscribing to drone 1 pose on: {self.drone1_topic}")
        self.get_logger().info(f"Subscribing to drone 2 pose on: {self.drone2_topic}")
        self.get_logger().info(f"Publishing calculated point to: {self.output_topic}")

    def drone1_callback(self, msg):
        self.drone1_pose = msg

    def drone2_callback(self, msg):
        self.drone2_pose = msg

    def calculate_point(self, pose1, pose2):
        if pose1.header.frame_id != pose2.header.frame_id:
            self.get_logger().warn(
                "Drone coordinate frames do not match! Dropping calculation. "
                f"Frame 1: '{pose1.header.frame_id}', Frame 2: '{pose2.header.frame_id}'"
            )
            return None

        calculated_point_msg = PointStamped()
        
        calculated_point_msg.header.stamp = self.get_clock().now().to_msg()
        calculated_point_msg.header.frame_id = pose1.header.frame_id

        p1 = pose1.pose.position
        p2 = pose2.pose.position

        vec_x = p2.x - p1.x
        vec_y = p2.y - p1.y

        # Fixed calculation: using vec_x and vec_y for 2D magnitude
        magnitude = math.sqrt(vec_x**2 + vec_y**2)

        if magnitude < 1e-6:
            self.get_logger().warn("Drones are at the same XY position. Calculation is undefined.")
            return None

        r = 3.0

        unit_x = vec_x / magnitude
        unit_y = vec_y / magnitude

        result_x = p1.x + r * unit_x
        result_y = p1.y + r * unit_y
        
        calculated_point_msg.point.x = result_x
        calculated_point_msg.point.y = result_y
        calculated_point_msg.point.z = p1.z

        return calculated_point_msg

    def timer_callback(self):
        if self.drone1_pose and self.drone2_pose:
            
            pose1 = self.drone1_pose
            pose2 = self.drone2_pose

            result_point_msg = self.calculate_point(pose1, pose2)
            
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
