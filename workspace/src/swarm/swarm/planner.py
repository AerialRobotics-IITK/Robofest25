#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

def rotate_vector(vec,theta):
    rot_matrix = np.array([
        [np.cos(theta),np.sin(theta)],
        [-np.sin(theta),np.cos(theta)]
    ])
    return rot_matrix @ vec

def dist_sq(a,b):
    return np.sum(np.square(a-b))

class Planner(Node):
    def __init__(self):
        super().__init__('drone_point_calculator')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.drone1_topic = "/uav1/local_pos"
        self.drone2_topic = "/uav2/local_pos"
        self.drone3_topic = "/uav3/local_pos"

        self.drone1_des_topic = "/uav1/desired_pos"
        self.drone2_des_topic = "/uav2/desired_pos"
        self.drone3_des_topic = "/uav3/desired_pos"
        
        self.declare_parameter("publish_rate", 10.0)

        self.drone1_pose = None
        self.drone2_pose = None
        self.drone3_pose = None

        self.mutual_dist = 1.5

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

        self.drone3_sub = self.create_subscription(
            PointStamped,
            self.drone3_topic,
            self.drone3_callback,
            qos_profile
        )

        self.drone1_pub = self.create_publisher(
            PointStamped,
            self.drone1_des_topic,
            qos_profile
        )

        self.drone2_pub = self.create_publisher(
            PointStamped,
            self.drone2_des_topic,
            qos_profile
        )

        self.drone3_pub = self.create_publisher(
            PointStamped,
            self.drone3_des_topic,
            qos_profile
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Subscribing to drone 1 pose on: {self.drone1_topic}")
        self.get_logger().info(f"Subscribing to drone 2 pose on: {self.drone2_topic}")
        self.get_logger().info(f"Subscribing to drone 3 pose on: {self.drone3_topic}")

    def drone1_callback(self, msg):
        self.drone1_pose = msg

    def drone2_callback(self, msg):
        self.drone2_pose = msg

    def drone3_callback(self, msg):
        self.drone3_pose = msg


    def timer_callback(self):
        if self.drone1_pose and self.drone2_pose and self.drone3_pose:
            p1 = np.array([[self.drone1_pose.point.x],[self.drone1_pose.point.y]])
            p2 = np.array([[self.drone2_pose.point.x],[self.drone2_pose.point.y]])
            p3 = np.array([[self.drone3_pose.point.x],[self.drone3_pose.point.y]])
            
            mp = (p2+p3)/2

            direction = (mp-p1)/np.linalg.norm(mp-p1)
            nmp = p1 + direction*self.mutual_dist

            np2 = nmp + rotate_vector(direction,np.radians(90))*self.mutual_dist
            np3 = nmp - rotate_vector(direction,np.radians(90))*self.mutual_dist

            msg1 = PointStamped()
            msg2 = PointStamped()
            msg3 = PointStamped()

            msg1.point.x = float(p1[0])
            msg1.point.y = float(p1[1])

            if dist_sq(np2,p2)<dist_sq(np2,p3):
                msg2.point.x = float(np2[0])
                msg2.point.y = float(np2[1])

                msg3.point.x = float(np3[0])
                msg3.point.y = float(np3[1])
            else:
                msg2.point.x = float(np3[0])
                msg2.point.y = float(np3[1])

                msg3.point.x = float(np2[0])
                msg3.point.y = float(np2[1])

            self.drone1_pub.publish(msg1)
            self.drone2_pub.publish(msg2)
            self.drone3_pub.publish(msg3)


        
        elif not self.drone1_pose:
            self.get_logger().warn("Waiting for pose from drone 1...")
        elif not self.drone2_pose:
            self.get_logger().warn("Waiting for pose from drone 2...")
        elif not self.drone3_pose:
            self.get_logger().warn("Waiting for pose from drone 3...")

def main(args=None):
    rclpy.init(args=args)
    
    calculator_node = Planner()
    
    try:
        rclpy.spin(calculator_node)
    except KeyboardInterrupt:
        pass
    finally:
        calculator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



