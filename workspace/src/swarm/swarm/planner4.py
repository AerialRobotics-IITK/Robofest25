#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from scipy.optimize import linear_sum_assignment

def match_nearest_points_unique(points_A, points_B):
    """
    Match each point in points_A to a unique closest point in points_B.
    No point in B can be assigned more than once.
    Output order follows points_A.

    Parameters:
    -----------
    points_A : numpy array of shape (n, d)
        First array of n points with d dimensions
    points_B : numpy array of shape (n, d)
        Second array of n points with d dimensions (must match length of A)

    Returns:
    --------
    numpy array of shape (n, d)
        Array where each row is the uniquely matched point from points_B
        corresponding to the point in points_A at the same index
    """
    if len(points_A) != len(points_B):
        raise ValueError("Both arrays must have the same number of points for one-to-one matching.")

    # Compute pairwise Euclidean distance matrix (n x n)
    dist_matrix = np.linalg.norm(points_A[:, np.newaxis] - points_B, axis=2)

    # Solve optimal one-to-one assignment using Hungarian algorithm
    row_ind, col_ind = linear_sum_assignment(dist_matrix)

    # Reorder matched points from B to follow the order of A
    matched_points = np.zeros_like(points_B)
    matched_points[row_ind] = points_B[col_ind]

    return matched_points
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
        self.drone4_topic = "/uav4/local_pos"

        self.declare_parameter("publish_rate", 10.0)

        self.drone1_pose = None
        self.drone2_pose = None
        self.drone3_pose = None
        self.drone4_pose = None

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
        self.drone_pubs=[]
        for i in range(4):
            self.drone_pubs.append(self.create_publisher(
                PointStamped,
                f"uav{i+1}/desired_ps",
                qos_profile
            ))
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Subscribing to drone 1 pose on: {self.drone1_topic}")
        self.get_logger().info(f"Subscribing to drone 2 pose on: {self.drone2_topic}")
        self.get_logger().info(f"Subscribing to drone 3 pose on: {self.drone3_topic}")
        self.get_logger().info(f"Subscribing to drone 4 pose on: {self.drone4_topic}")

    def drone1_callback(self, msg):
        self.drone1_pose = msg

    def drone2_callback(self, msg):
        self.drone2_pose = msg

    def drone3_callback(self, msg):
        self.drone3_pose = msg

    def drone4_callback(self, msg):
        self.drone4_pose = msg


    def timer_callback(self):
        if self.drone1_pose and self.drone2_pose and self.drone3_pose and self.drone4_pose:
            p1 = np.array([[self.drone1_pose.point.x],[self.drone1_pose.point.y]])
            p2 = np.array([[self.drone2_pose.point.x],[self.drone2_pose.point.y]])
            p3 = np.array([[self.drone3_pose.point.x],[self.drone3_pose.point.y]])
            p4 = np.array([[self.drone4_pose.point.x],[self.drone4_pose.point.y]])
            
            mp = (p2+p3+p4)/3

            direction = (mp-p1)/np.linalg.norm(mp-p1)
            nmp = p1 + direction*self.mutual_dist

            np2 = nmp + rotate_vector(direction,np.radians(90))*self.mutual_dist
            np3 = nmp - rotate_vector(direction,np.radians(90))*self.mutual_dist
            np4 = nmp

            msg1 = PointStamped()

            drone_pos = [p2,p3,p4]
            target_pos = [np2,np3,np4]
            match_nearest_points_unique(drone_pos,target_pos)
            msg1.point.x = float(p1[0])
            msg1.point.y = float(p1[1])
            self.drone_pubs[0].publish(msg1)
            for publisher,target in zip(self.drone_pubs,target_pos):
                msg = PointStamped()
                msg.point.x = float(target[0])
                msg.point.y = float(target[1])
                publisher.publish(msg)
            


        
        elif not self.drone1_pose:
            self.get_logger().warn("Waiting for pose from drone 1...")
        elif not self.drone2_pose:
            self.get_logger().warn("Waiting for pose from drone 2...")
        elif not self.drone3_pose:
            self.get_logger().warn("Waiting for pose from drone 3...")
        elif not self.drone4_pose:
            self.get_logger().warn("Waiting for pose from drone 4...")


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



