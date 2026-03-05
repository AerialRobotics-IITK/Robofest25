#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
#from tf_transformations import euler_from_quaternion
import os
from datetime import datetime

class PoseLoggerNode(Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.csv_file = 'drone_pose_bidu_plus_50per_optical.csv'
        self.file_exists = os.path.isfile(self.csv_file)
        self.writer = None
        self.create_subscription(PoseStamped, '/uav1/local_position/pose', self.pose_callback, 10)
        self.get_logger().info(f'Subscribed to /uav1/local_position/pose. Logging to {self.csv_file}')

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
#        quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
#                msg.pose.orientation.z, msg.pose.orientation.w]
#        yaw = euler_from_quaternion(quat)[2]  # yaw is the third element (z-axis)

        if self.writer is None:
            self.writer = csv.writer(open(self.csv_file, 'w', newline=''))
            if not self.file_exists:
                self.writer.writerow(['timestamp', 'x', 'y', 'z'])

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.writerow([timestamp, x, y, z])
        self.writer.writerows([])  # Flush

def main(args=None):
    rclpy.init(args=args)
    node = PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
