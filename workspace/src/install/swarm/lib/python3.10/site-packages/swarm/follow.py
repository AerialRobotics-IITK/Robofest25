import rclpy
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,PointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import numpy as np
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


class Follower(Node):
    def __init__(self):
        super().__init__('follower')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        gps_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Critical!
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.namespace=f"uav{os.environ.get('MAV_ID')}"
        self.get_logger().info(f"namespace {self.namespace}")
        self.z_pos=0.0
        self.altitude = 0.0
        self.stage = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.offset =None

        self.state = State()
        self.pos_pub = self.create_publisher(
            PoseStamped,
            f'/{self.namespace}/setpoint_position/local',
            qos
        )
        self.pos_sub = self.create_subscription(
            PointStamped,
            f'/{self.namespace}/desired_pos',
            self.pos_callback,
            qos
        )
        self.create_subscription(State, f'/{self.namespace}/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, f'/{self.namespace}/local_position/pose', self.pos_cb, qos)

        self.offset_topic = f"/{self.namespace}/offset"
        self.create_subscription(PointStamped, self.offset_topic, self.offset_callback, gps_qos_profile)

        self.arming_client = self.create_client(CommandBool, f'/{self.namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{self.namespace}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'/{self.namespace}/cmd/takeoff')

        self.timer = self.create_timer(0.1, self.control_loop)

    def offset_callback(self,msg):
        self.offset = msg
        
    def pos_callback(self,msg):
        if self.stage==5:
            self.get_logger().info(f"Going to {msg.point}")
            pos_msg = PoseStamped()
            pos_msg.pose.position.x = msg.point.x- self.offset.point.x
            pos_msg.pose.position.y = msg.point.y- self.offset.point.y
            pos_msg.pose.position.z = 3.0
            self.pos_pub.publish(pos_msg)
        elif self.offset is None:
            self.get_logger().warn(f"{self.namespace} offset not found")
        else:
            pass
            # self.get_logger().info(f"{self.namespace} going {msg} ")


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
                self.get_logger().info('Started following')
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
            
        elif self.stage == 5:
            elapsed = (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Following: {elapsed:.1f} s')
            if elapsed >=60.0:
                self.get_logger().info('Following End')
                pass
        elif self.stage == 6:
            pass

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


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
