import os
from swarm import tof
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class LidarToMavrosBridge(Node):
    def __init__(self):
        super().__init__('lidar_to_mavros_bridge')
        
        # 1. Define the topic name based on your previous info
        # Change this to match your actual namespace/topic
        self.id = os.environ.get("MAV_ID",1)
        self.namespace = f"/uav{self.id}"
        self.topic_name = f'{self.namespace}/rangefinder_sub'
        
        self.publisher_ = self.create_publisher(Range, self.topic_name, 10)
        
        # 2. Timer to publish at 10Hz (ArduPilot likes consistent rates)
        self.timer_period = 1/50  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f'Bridge started. Publishing to: {self.topic_name}')

    def timer_callback(self):
        msg = Range()
        data = tof.read_tof_data()
        
        # Header info
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uav1/base_link' # Adjust to your frame
        
        # Field of View and Type
        msg.radiation_type = Range.INFRARED # or Range.ULTRASOUND
        msg.field_of_view = 0.05            # ~3 degrees in radians
        msg.min_range = 0.5                 # 10cm
        msg.max_range = 8.0                # 20m
        
        # 3. THE DATA: Replace '2.5' with your actual LiDAR variable
        msg.range = data['dis']

        self.publisher_.publish(msg)
        # Optional: uncomment to see data in terminal
        # self.get_logger().info(f'Publishing Altitude: {msg.range}m')

def main(args=None):
    rclpy.init(args=args)
    bridge = LidarToMavrosBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
