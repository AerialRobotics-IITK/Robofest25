import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped,PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import numpy as np
import cv2
from object_tracker.hand_detection import get_state
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

def rotate_vector(vec,theta):
    rot_matrix = np.array([
        [np.cos(theta),np.sin(theta)],
        [-np.sin(theta),np.cos(theta)]
    ])
    return rot_matrix @ vec


class ImageViewer(Node):
    def __init__(self):
        super().__init__('hand_tracker')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.namespace="uav1"
        self.altitude = 0.0
        self.stage = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.state = State()
        self.cam_topic = "/rpi_cam/image_raw"
        self.cmd = "HOLD"
       
        self.img_sub = self.create_subscription(
            Image,
            self.cam_topic,
            self.listener_callback,
            10
        )
        self.vel_pub = self.create_publisher(
            TwistStamped,
            f'/{self.namespace}/setpoint_velocity/cmd_vel',
            qos
        )
        self.pos_pub = self.create_publisher(
            PoseStamped,
            f'/{self.namespace}/setpoint_position/local',
            qos
        )
        self.create_subscription(State, f'/{self.namespace}/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, f'/{self.namespace}/local_position/pose', self.pos_cb, qos)


        self.arming_client = self.create_client(CommandBool, f'/{self.namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{self.namespace}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'/{self.namespace}/cmd/takeoff')

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):

        # ---------- WAIT FOR FCU ----------
        if not self.state.connected:
            self.get_logger().info('Waiting for MAVROS...')
            return

        # ---------- STAGE 0: SET GUIDED ----------
        if self.stage == 0:
            if self.state.mode != 'GUIDED':
                req = SetMode.Request()
                req.custom_mode = 'GUIDED'
                self.mode_client.call_async(req)
                self.get_logger().info('Setting GUIDED mode...')
            else:
                self.stage = 1

        # ---------- STAGE 1: ARM ----------
        elif self.stage == 1:
            if not self.state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info('Arming vehicle...')
            else:
                self.stage = 2

        # ---------- STAGE 2: TAKEOFF ----------
        elif self.stage == 2:
            req = CommandTOL.Request()
            req.altitude = 2.0# target altitude in meters
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0
            self.takeoff_client.call_async(req)
            self.get_logger().info('Takeoff command sent...')
            self.stage = 3

        # ---------- STAGE 3: WAIT UNTIL ALTITUDE ----------
        elif self.stage == 3:

            self.get_logger().info(f'Altitude: {self.z_pos:.2f} m')
            if self.z_pos >1.9 :
                self.get_logger().info('Target altitude reached, starting hover...')
                self.stage_start_time = self.get_clock().now()
                self.stage = 4

                self.z_ref=self.z_pos
                self.x_ref=self.x_pos
                self.y_ref=self.y_pos

                self.qx_ref=self.qx
                self.qy_ref=self.qy
                self.qz_ref=self.qz
                self.qw_ref=self.qw
            
            elif self.z_pos>1:
                pose = PoseStamped()
           
                # Example: Move forward, right, and ascend
                pose.pose.position.x = self.x_pos
                pose.pose.position.y = self.y_pos
                pose.pose.position.z = 2.0
                

                # Orientation (quaternion)
                pose.pose.orientation.x=self.qx
                pose.pose.orientation.y=self.qy
                pose.pose.orientation.z=self.qz
                pose.pose.orientation.w=self.qw

                self.pos_pub.publish(pose)



        # ---------- STAGE 4: HOVER ----------
        elif self.stage == 4:
            elapsed = (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Hovering: {elapsed:.1f} s')
            if elapsed >=10.0:
                self.get_logger().info('Starting velocity control...')
                self.stage_start_time = self.get_clock().now()
                self.stage = 5

                pose = PoseStamped()
           
            # Example: Move forward, right, and ascend
            
                pose.pose.position.x = self.x_ref
                pose.pose.position.y = self.y_ref
                pose.pose.position.z = 2.0
                

                # Orientation (quaternion)
                pose.pose.orientation.x=self.qx_ref
                pose.pose.orientation.y=self.qy_ref
                pose.pose.orientation.z=self.qz_ref
                pose.pose.orientation.w=self.qw_ref                                                 # up

                self.pos_pub.publish(pose)
            
        # ---------- STAGE 4: ----------
        elif self.stage == 5:
            msg = TwistStamped()
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0

            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

            if self.cmd == "HOLD":
                pass
            if self.cmd == "LEFT":
                dir = np.array([[0],[1]])
                vec = rotate_vector(dir,self.yaw) * 0.1
                msg.twist.linear.x= float(vec[0])
                msg.twist.linear.y= float(vec[1])
            if self.cmd == "RIGHT":
                dir = np.array([[0],[-1]])
                vec = rotate_vector(dir,self.yaw) * 0.1
                msg.twist.linear.x= float(vec[0])
                msg.twist.linear.y= float(vec[1])


            self.vel_pub.publish(msg)
    def state_cb(self, msg):
        self.state = msg

    def pos_cb(self, msg):
        if self.stage==1:
            self.base_z=msg.pose.position.z
        self.z_pos = msg.pose.position.z
        self.x_pos = msg.pose.position.x
        self.y_pos = msg.pose.position.y

        # Orientation (quaternion)
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w

        self.roll,self.pitch,self.yaw=quaternion_to_euler(self.qx, self.qy, self.qz, self.qw)

    def vel_cb(self,msg):

        self.x_vel=msg.twist.linear.x
        self.y_vel=msg.twist.linear.y
        self.z_vel=msg.twist.linear.z

        # Angular velocity (rate of change of orientation)
        self.roll_rate  = msg.twist.angular.x
        self.pitch_rate = msg.twist.angular.y
        self.yaw_rate   = msg.twist.angular.z



    def listener_callback(self,msg):
        raw_data = np.frombuffer(msg.data, dtype=np.uint8)
        # 2. Reshape the data
        # For NV21/YUV420, the buffer size is (height * 1.5) * width
        # We need to reshape it to this specific height to allow cvtColor to work
        height = msg.height
        width = msg.width
        yuv_frame = raw_data.reshape((height + height // 2, width))

        # 3. Convert from YUV (NV21) to BGR for OpenCV/MediaPipe
        frame = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2RGB_NV21)
        frame = cv2.flip(frame,-1)
        # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.cmd = get_state(frame)
        print(self.cmd)


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
