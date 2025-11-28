#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import HomePosition
from mavros_msgs.srv import CommandHome
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class HomeSync(Node):
    def __init__(self):
        super().__init__('home_sync')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.client = self.create_client(CommandHome, '/uav2/cmd/set_home')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /uav2/cmd/set_home...')
        self.sub = self.create_subscription(
            HomePosition,
            '/uav1/home_position/home',
            self.callback,
            qos_profile
        )
        self.get_logger().info('Listening for UAV1 home position...')

    def callback(self, msg):
        req = CommandHome.Request()
        req.current_gps = False
        req.latitude = msg.geo.latitude
        req.longitude = msg.geo.longitude
        req.altitude = msg.geo.altitude
        self.get_logger().info(
            f'Setting UAV2 home to: ({req.latitude:.6f}, {req.longitude:.6f}, {req.altitude:.2f})'
        )
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('UAV2 home position successfully updated.')
        else:
            self.get_logger().error('Failed to set UAV2 home position.')

def main():
    rclpy.init()
    node = HomeSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

