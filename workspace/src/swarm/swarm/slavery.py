#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import numpy as np

def rotate_vector(vec,theta):
    rot_matrix = np.array([
        [np.cos(theta),np.sin(theta)],
        [-np.sin(theta),np.cos(theta)]
    ])
    return rot_matrix @ vec

def dist_sq(a,b):
    return np.sum(np.square(a-b))

class Slavery(Node):
    def __init__(self):
        super().__init__('drone_point_calculator')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.namespace=f"uav{os.environ.get('MAV_ID',2)}"

        self.drone1_topic = "/uav1/local_pos"
        self.drone2_topic = f"/{self.namespace}/local_pos"

        self.drone2_des_topic = f"/{self.namespace}/desired_pos"
        
        self.declare_parameter("publish_rate", 10.0)

        self.drone1_pose = None
        self.drone2_pose = None

        self.mutual_dist = 3

        self.drone1_sub = self.create_subscription(
            PointStamped,
            self.drone1_topic,
            self.drone1_callback,
            qos_profile
        )
        
        self.drone2_sub = self.create_subscription(
            PointStamped,
            self.drone2_topic,
            self.drone2_callback,
            qos_profile
        )


        self.drone2_pub = self.create_publisher(
            PointStamped,
            self.drone2_des_topic,
            qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Subscribing to drone 1 pose on: {self.drone1_topic}")
        self.get_logger().info(f"Subscribing to drone 2 pose on: {self.drone2_topic}")

    def drone1_callback(self, msg):
        self.drone1_pose = msg

    def drone2_callback(self, msg):
        self.drone2_pose = msg


    def timer_callback(self):
        if self.drone1_pose and self.drone2_pose:
            p1 = np.array([[self.drone1_pose.point.x],[self.drone1_pose.point.y],[self.drone1_pose.point.z]])
            p2 = np.array([[self.drone2_pose.point.x],[self.drone2_pose.point.y],[self.drone2_pose.point.z]])

            direction = (p2-p1)/np.linalg.norm(p2-p1)
            p = p1 + direction*self.mutual_dist


            msg = PointStamped()
            msg.point.x = float(p[0])
            msg.point.y = float(p[1])
            msg.point.z = float(p[2])
            self.drone2_pub.publish(msg)


        
        elif not self.drone1_pose:
            self.get_logger().warn("Waiting for pose from drone 1...")
        elif not self.drone2_pose:
            self.get_logger().warn("Waiting for pose from drone 2...")

def main(args=None):
    rclpy.init(args=args)
    
    calculator_node = Slavery()
    
    try:
        rclpy.spin(calculator_node)
    except KeyboardInterrupt:
        pass
    finally:
        calculator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


