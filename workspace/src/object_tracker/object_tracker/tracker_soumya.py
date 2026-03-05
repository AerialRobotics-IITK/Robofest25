import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
import numpy as np
import cv2

# Make sure this import matches your file structure!
from object_tracker.process_frame_1 import process_frames 

class ImageViewer(Node):
    def __init__(self):
        super().__init__('x500_mono_cam')
        
        # --- State Variables ---
        self.origin = (0, 0)  
        self.lock = False
        self.current_active_arm = "NONE"  # Replaced dist and direction
        self.lateral_command = "HOVER"
        self.call_swarm = False  
        self.waist_center = (-1, -1)  

        # --- Publishers ---
        self.int_publisher_ = self.create_publisher(
            Int32, '/lateral_command', 10
        )
        self.float_publisher_ = self.create_publisher(
            Float32, '/waist_angle', 10
        )

        # --- Subscriptions ---
        self.cam_topic = "/camera/image_raw"
        self.image_sub = self.create_subscription(
            Image, 
            self.cam_topic, 
            self.image_callback, 
            10
        )

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
        
        # NV21 total height = height * 1.5
        yuv = raw_data.reshape((height + height // 2, step))

        # Remove padding columns
        yuv = yuv[:, :width]

        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        return frame

    def image_callback(self, msg):
        """This runs automatically every time a new camera frame arrives"""
        
        # 1. Convert ROS image to OpenCV format
        frame = self.ros_to_cv2(msg)
        if frame is None:
            return
            
        # 2. Process the frame (Fixed argument mismatch)
        (
            frame_,
            self.origin,
            self.lock,
            self.current_active_arm,
            self.call_swarm,
            self.waist_center,
            self.lateral_command
        ) = process_frames(
            frame, 
            self.origin, 
            self.lock, 
            self.current_active_arm
        )
        
        # Safe debug print
        self.get_logger().info(
            f"Lock: {self.lock} | Arm: {self.current_active_arm} | "
            f"Steer: {self.lateral_command} | Waist: {self.waist_center[0]}"
        )

        # 3. Publish Lateral Command (-1, 0, 1) (Fixed string matching)
        int_msg = Int32()
        if self.lateral_command == "MOVE LEFT":
            int_msg.data = -1
        elif self.lateral_command == "MOVE RIGHT":
            int_msg.data = 1
        else:
            int_msg.data = 0 # HOVER
            
        self.int_publisher_.publish(int_msg)

        # 4. Publish Waist Center (X-coordinate)
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
