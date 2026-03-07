import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
import cv2
from object_tracker.process_frame import process_frames


class ImageViewer(Node):
    def __init__(self):
        super().__init__('x500_mono_cam')
        self.cap = cv2.VideoCapture(0)
        self.origin = (0, 0)  # Tuple for (x, y)
        self.lock = False
        self.dist = 0
        self.direction = False
        self.call_swarm = False  # Swarm command state
        self.waist_center = (-1, -1)  # New: waist center (x, y) in image coords
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.int_publisher_ = self.create_publisher(
            Int32MultiArray, '/hand_distance', 10
        )
        self.float_publisher_ = self.create_publisher(
            Float32, '/waist_angle', 10
        )
        cv2.namedWindow('Processed Frame', cv2.WINDOW_NORMAL)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
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
                f'dist: {self.dist}, locked: {self.lock}, swarm: {self.call_swarm}, waist_center: {self.waist_center}'
            )

            

            # Log waist center
            float_msg = Float32()
            wx, wy = self.waist_center
            float_msg.data = float(wx)
            self.float_publisher_.publish(float_msg)

            cv2.imshow('Processed Frame', frame_)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()