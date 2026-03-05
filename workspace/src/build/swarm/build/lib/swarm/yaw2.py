#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from scipy.spatial.transform import Rotation as R

def quaternion_to_euler(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

class TakeoffVelocityLand(Node):

    def __init__(self):
        super().__init__('takeoff_velocity_land')

        # ---------------- STATE ----------------
        self.state = State()
        self.altitude = 0.0
        self.altitude_received = False
        self.stage = 0
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz loop
        self.stage_start_time = None
        self.namespace = "uav1"

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Current States

        self.z_pos = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0

        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0


        self.z_vel = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0

        self.roll_rate=0.0
        self.pitch_rate=0.0
        self.yaw_rate=0.0

        #references
        self.base_z =0.0
        self.x_ref=0.0
        self.y_ref=0.0
        self.z_ref=0.0

        self.qx_ref=0.0
        self.qy_ref=0.0
        self.qz_ref=0.0
        self.qw_ref=0.0

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(State, f'/{self.namespace}/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, f'/{self.namespace}/local_position/pose', self.pos_cb, qos)

        # ---------------- PUBLISHERS ----------------
        self.vel_pub = self.create_publisher(Twist, f'/{self.namespace}/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pos_pub = self.create_publisher(PoseStamped, f'/{self.namespace}/setpoint_position/local', 10)

        # ---------------- SERVICE CLIENTS ----------------
        self.arming_client = self.create_client(CommandBool, f'/{self.namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{self.namespace}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'/{self.namespace}/cmd/takeoff')

    # ---------------- CALLBACKS ----------------
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


    # ---------------- CONTROL LOOP ----------------
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
            req.altitude = 3.0# target altitude in meters
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
            if self.z_pos >2.9 :
                self.get_logger().info('Target altitude reached, starting hover...')
                self.stage_start_time = self.get_clock().now()
                self.stage = 4

                self.z_ref=self.z_pos
                self.x_ref=self.x_pos
                self.y_ref=self.y_pos

                self.roll_ref=self.roll
                self.pitch_ref=self.pitch
                self.yaw_ref=self.yaw

                self.qx_ref=self.qx
                self.qy_ref=self.qy
                self.qz_ref=self.qz
                self.qw_ref=self.qw
            
            elif self.z_pos>2.6:
                pose = PoseStamped()
           
                # Example: Move forward, right, and ascend
                pose.pose.position.x = self.x_pos
                pose.pose.position.y = self.y_pos
                pose.pose.position.z = 3.0
                

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
                pose.pose.position.z = 3.0
                

                # Orientation (quaternion)
                pose.pose.orientation.x=self.qx_ref
                pose.pose.orientation.y=self.qy_ref
                pose.pose.orientation.z=self.qz_ref
                pose.pose.orientation.w=self.qw_ref                                                 # up

                self.pos_pub.publish(pose)
            

        # ---------- STAGE 5: YAW ----------
        elif self.stage == 5:
            elapsed = (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9   
            twist = Twist()
            
            twist.angular.z = 0.5  # yaw rotation


            if elapsed >=5.0:
                self.get_logger().info('Starting Land ')
                twist.angular.z = 0.0
                self.stage_start_time = self.get_clock().now()
                self.stage = 6
           

            # Example: Move forward, right, and ascend
            ##twist.linear.x = 1.0# forward
            ##twist.linear.y = 0.0# right

            
            self.vel_pub.publish(twist)

            self.get_logger().info(f'Yaw control running: {elapsed:.1f} s')



        # ---------- STAGE 6: LAND ----------
        elif self.stage == 6:
            if self.state.mode != 'LAND':
                req = SetMode.Request()
                req.custom_mode = 'LAND'
                self.mode_client.call_async(req)
                self.get_logger().info('Landing...')
            else:
                self.get_logger().info('LAND mode active. Node stopping...')
                self.timer.cancel()


# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = TakeoffVelocityLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
