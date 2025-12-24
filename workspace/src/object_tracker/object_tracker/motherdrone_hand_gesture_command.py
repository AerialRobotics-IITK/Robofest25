#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory 
import os 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from std_msgs.msg import Int32MultiArray                                    # for self.direction , self.lock , self.distance

from scipy.spatial.transform import Rotation as R

def quaternion_to_euler(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

class HandBasedGestureContorl(Node):

    def __init__(self):
        super().__init__("handbasedgesturecontrol")

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

        # Orientation (quaternion)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0

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

        # For Hand Gesture Based Related Task 

        self.distance = 0
        self.prev_distance = 0
        self.vel = 0
        self.lock = 0
        self.direction = 0
        

        self.takeoff_height=-5.0

        self.rmse_x=0.0
        self.rmse_y=0.0
        self.rmse_z=0.0

        self.rmse_vx=0.0
        self.rmse_vy=0.0
        self.rmse_vz=0.0

        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0


        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(State, f'/{self.namespace}/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, f'/{self.namespace}/local_position/pose', self.pos_cb, qos)
        self.create_subscription(TwistStamped, f'{self.namespace}/local_position/velocity_local', self.vel_cb, qos)

        self.subscription = self.create_subscription(Int32MultiArray,'/hand_distance',self.distance_callback,10) 

        # ---------------- PUBLISHERS ----------------
        self.vel_pub = self.create_publisher(TwistStamped, f'/{self.namespace}/setpoint_velocity/cmd_vel', 10)
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

        self.x_vel=clamp(msg.twist.linear.x,-0.5,0.5)
        self.y_vel=clamp(msg.twist.linear.y,-0.5,0.5)
        self.z_vel=clamp(msg.twist.linear.z,-0.5,0.5)

        # Angular velocity (rate of change of orientation)
        self.roll_rate  =clamp(msg.twist.angular.x,-0.5,0.5)
        self.pitch_rate =clamp(msg.twist.angular.y,-0.5,0.5)
        self.yaw_rate   =clamp(msg.twist.angular.z,-0.5,0.5)

    
    def distance_callback(self, msg):
        self.prev_distance = self.distance
        self.distance = msg.data[0]/100                                         #4*math.tan(math.asin(msg.data[0]*0.00026)) d0 = 4
        self.vel = (self.distance-self.prev_distance)*30                                                # self.vel is the speed of movement of drone
        self.lock = msg.data[1]
        self.direction = msg.data[2]

    def error(self):

        if self.direction:

            error_x=self.x_ref-self.x_pos
            error_vx=0.0-self.x_vel

            
            error_y=self.distance-self.y_pos
            error_vy=self.vel-self.y_vel

            error_z=self.z_ref-self.z_pos
            error_vz=0.0-self.z_vel

        else:
            error_x=self.distance-self.x_pos

            error_z=self.z_ref-self.z_pos
            error_y=self.y_ref-self.y_pos
        
            error_vx=self.vel-self.x_vel
            error_vy=0.0-self.y_vel
            error_vz=0.0-self.z_vel

        self.integral_error_x+=error_x*0.1
        self.integral_error_y+=error_y*0.1
        self.integral_error_z+=error_z*0.1

        self.rmse_x+=error_x**2
        self.rmse_y+=error_y**2
        self.rmse_z+=error_z**2

        self.rmse_vx+=error_vx**2
        self.rmse_vy+=error_vy**2
        self.rmse_vz+=error_vz**2

        return(error_x,error_y,error_z,error_vx,error_vy,error_vz)
    
    # applying tracking controller
    
    def tracking_pp_controller(self):


        
        l=list(self.error())

        del_ax=(5.7*l[0]+3.4*l[3])
        del_ay=(5.7*l[1]+3.4*l[4])
        del_az=(6.2*l[2]+4.0*l[5])

        # since we want to give velocity command 

        return (del_ax,del_ay,del_az)
    
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
            

        # ---------- STAGE 5: IN HAND GESTURE BASED MODE ----------
        elif self.stage == 5:
            elapsed = (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9   
            
            twist = TwistStamped()
            
            if self.lock:

                del_ax,del_ay,del_az=self.tracking_pp_controller()
            
                # Example: Move forward, right, and ascend
                
                twist.twist.linear.x = del_ax*self.dt                                       # forward
                twist.twist.linear.y = del_ay*self.dt                                       # right
                twist.twist.linear.z = del_az*self.dt                                       # up

                self.get_logger().info(f'Velocity control running: {elapsed:.1f} s')


            elif not self.lock:

                twist.twist.linear.x = 0.0                                      # forward
                twist.twist.linear.y = 0.0                                                # right
                twist.twist.linear.z = 0.0                                                 # up


            self.vel_pub.publish(twist)





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
    node = HandBasedGestureContorl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
