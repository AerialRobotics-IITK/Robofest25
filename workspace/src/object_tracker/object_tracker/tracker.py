import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
import numpy as np
import cv2
from object_tracker.process_frame import process_frames


class ImageViewer(Node):
    def __init__(self):
        super().__init__('x500_mono_cam')
        self.cap_topic = "/rpi_cam/image_raw"
        self.origin = (0, 0)  # Tuple for (x, y)
        self.lock = False
        self.dist = 0
        self.direction = False
        self.call_swarm = False  # Swarm command state
        self.waist_center = (-1, -1)  # New: waist center (x, y) in image coords
        self.int_publisher_ = self.create_publisher(
            Int32MultiArray, '/hand_distance', 10
        )
        self.float_publisher_ = self.create_publisher(
            Float32, '/waist_angle', 10
        )
        self.img_sub = self.create_subscription(
            Image,
            '/rpi_cam/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self,msg):
        raw_data = np.frombuffer(msg.data, dtype=np.uint8)

        # 2. Reshape the data
        # For NV21/YUV420, the buffer size is (height * 1.5) * width
        # We need to reshape it to this specific height to allow cvtColor to work
        height = msg.height
        width = msg.width
        yuv_frame = raw_data.reshape((height + height // 2, width))

        # 3. Convert from YUV (NV21) to BGR for OpenCV/MediaPipe
        frame = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2BGR_NV21)

        # Updated to unpack 7 return values including waist_center
        (
            frame_,
            self.origin,
            self.lock,
            self.dist,
            self.direction,
            self.call_swarm,
            self.waist_center
        ) = process_frames(
            frame, self.origin, self.lock, self.dist, self.direction
        )

        # Publish [distance, lock, direction, swarm_flag]
        int_msg = Int32MultiArray()
        int_msg.data = [
            int(self.dist),       # 0: Distance (px)
            int(self.lock),       # 1: Lock state (0=unlocked, 1=locked)
            int(self.direction),  # 2: Direction (0=horizontal, 1=vertical)
            int(self.call_swarm)  # 3: Swarm command (1=pulse when gesture detected)
        ]
        self.int_publisher_.publish(int_msg)

        # Log hand and swarm state
        self.get_logger().info(
            f'Publishing: {int_msg.data}'
        )
        self.get_logger().info(
            f'dist: {self.dist}, locked: {self.lock}, swarm: {self.call_swarm}'
        )

        # Log waist center
        float_msg = Float32()
        wx, wy = self.waist_center
        float_msg.data = float(wx)
        self.float_publisher_.publish(float_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
