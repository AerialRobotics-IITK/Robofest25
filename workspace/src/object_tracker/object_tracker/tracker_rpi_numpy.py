import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32
import numpy as np
import cv2
from object_tracker.process_frame import process_frames


class ImageViewer(Node):
    def __init__(self):
        super().__init__('x500_mono_cam')
        
        # --- State Variables ---
        self.origin = (0, 0)  # Tuple for (x, y)
        self.lock = False
        self.dist = 0
        self.direction = False
        self.call_swarm = False  # Swarm command state
        self.waist_center = (-1, -1)  # Waist center (x, y) in image coords

        # --- Publishers ---
        self.int_publisher_ = self.create_publisher(
            Int32MultiArray, '/hand_distance', 10
        )
        self.float_publisher_ = self.create_publisher(
            Float32, '/waist_angle', 10
        )

        # --- Subscriptions (Replaces VideoCapture) ---
        self.cam_topic = "/camera/image_raw"
        self.image_sub = self.create_subscription(
            Image, 
            self.cam_topic, 
            self.image_callback, 
            10
        )

        # GUI Window
        # cv2.namedWindow('Processed Frame', cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Subscribed to {self.cam_topic}. Waiting for frames...")
        
    # ==========================================
    # CV_BRIDGE REPLACEMENT FUNCTION
    # ==========================================
    def ros_to_cv2(self, msg):
        """Convert ROS Image to OpenCV BGR image (robust version)"""

        raw_data = np.frombuffer(msg.data, dtype=np.uint8)

        height = msg.height
        width = msg.width
        step = msg.step
        encoding = msg.encoding.lower()
 
                # NV21 total height = height * 1.5
        yuv = raw_data.reshape((height + height // 2, step))

                # Remove padding columns
        yuv = yuv[:, :width]

        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)

        self.get_logger().debug(f"Converted ROS Image to OpenCV format: {frame.shape}, encoding: {encoding}")

        return frame

    def image_callback(self, msg):
        """This runs automatically every time a new camera frame arrives"""
        print("Received new image frame. Processing...")
        # 1. Convert ROS image to OpenCV format
        frame = self.ros_to_cv2(msg)
        if frame is None:
            return
        print(f"Received frame of shape: {frame.shape}")
        # 2. Process the frame using your custom module
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
        print(f"Processed frame. Distance: {self.dist}, Lock: {self.lock}, Direction: {self.direction}, Swarm Command: {self.call_swarm}, Waist Center: {self.waist_center}")
        # 3. Publish Hand & Swarm State [distance, lock, direction, swarm_flag]
        int_msg = Int32MultiArray()
        int_msg.data = [
            int(self.dist),       # 0: Distance (px)
            int(self.lock),       # 1: Lock state (0=unlocked, 1=locked)
            int(self.direction),  # 2: Direction (0=horizontal, 1=vertical)
            int(self.call_swarm)  # 3: Swarm command (1=pulse when gesture detected)
        ]
        self.int_publisher_.publish(int_msg)
        # 4. Publish Waist Center
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
        # cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
