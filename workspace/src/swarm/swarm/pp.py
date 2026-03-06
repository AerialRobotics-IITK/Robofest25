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

from scipy.spatial.transform import Rotation as R

import csv

def quaternion_to_euler(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw


class TakeoffVelocityLand(Node):

    def __init__(self):
        super().__init__('takeoff_velocity_land')

        #0-Set Guided , 1-arm , 2-takeoff , 3-wait until altitude , 4-Hovering , 5-Velocity Control , 6-Land

        self.namespace = "uav1"
        # ---------------- STATE ----------------
        self.state = State()
        
        self.altitude_received = False
        self.stage = 0

        self.takeoff_height=3.0

        self.dt=0.02                                               # time interval between 2 loops
        self.timer = self.create_timer(self.dt, self.control_loop)  # 1000 Hz loop

        self.stage_start_time = None



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


        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Load Trajectory 
        #
        # try:
        #     package_share_directory = get_package_share_directory('ardupilot_takeoff')
        # except Exception as e:
        #     self.get_logger().error(f"Could not find package share directory: {e}")
        #     return # Cannot continue without the package path
        #
        
        # 2. Construct the full, absolute path to the file
        file_name = 'trajectory_obs.csv' 

        # 3. Pass the absolute path to your loader function
        self.data_matrix = self.loadTrajectoryDataFromCSV(file_name)

        # Ensure the path is printed for debugging
        self.get_logger().info(f'Attempting to load trajectory from: {file_name}')

        # Extract components from the data matrix
        self.t = self.data_matrix[:, 0]
        self.n_wp = len(self.t)
        
        # Position (px, py, pz), Velocity (vx, vy, vz), Acceleration (ax, ay, az)
        self.px = self.data_matrix[:, 1]
        self.py = self.data_matrix[:, 2]
        self.pz = -self.data_matrix[:, 3]
        self.vx = self.data_matrix[:, 4]
        self.vy = self.data_matrix[:, 5]
        self.vz = -self.data_matrix[:, 6]

        self.ax = self.data_matrix[:, 7]
        self.ay = self.data_matrix[:, 8]
        self.az = self.data_matrix[:, 9]

        self.index=0



        with open("state_data.csv", mode="w+", newline="") as file:
            self.writer = csv.writer(file)
            self.writer.writerow(["time","pos_x", "pos_y", "pos_z","ideal_x","ideal_y","ideal_z","error_x","error_y","error_z","vel_x", "vel_y", "vel_z" ,"ideal_vx","ideal_vy","ideal_vz","error_vx","error_vy","error_vz","roll","pitch","yaw","roll_rate","pitch_rate","yaw_rate"])

        
        
        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(State, f'{self.namespace}/state', self.state_cb, qos)
        self.create_subscription(PoseStamped, f'{self.namespace}/local_position/pose', self.pos_cb, qos)
        self.create_subscription(TwistStamped, f'{self.namespace}/local_position/velocity_local', self.vel_cb, qos)

        # ---------------- PUBLISHERS ----------------
        self.vel_pub = self.create_publisher(TwistStamped, f'{self.namespace}/setpoint_velocity/cmd_vel', 10)
        self.pos_pub = self.create_publisher(PoseStamped, f'{self.namespace}setpoint_position/local', 10)

        # ---------------- SERVICE CLIENTS ----------------
        self.arming_client = self.create_client(CommandBool, f'{self.namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'{self.namespace}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'{self.namespace}/cmd/takeoff')


    # ---------------- CALLBACKS ----------------
    def state_cb(self, msg):
        self.state = msg

    def pos_cb(self, msg):
        self.z_pos = msg.pose.position.z
        self.x_pos = msg.pose.position.x
        self.y_pos = msg.pose.position.y

        # Orientation (quaternion)
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w

        self.roll,self.pitch,self.yaw=quaternion_to_euler(self.qx, self.qy, self.qz, self.qw)

        #self.altitude_received = True

    def vel_cb(self,msg):

        self.z_vel=msg.twist.linear.z
        self.x_vel=msg.twist.linear.x
        self.y_vel=msg.twist.linear.y

        # Angular velocity (rate of change of orientation)
        self.roll_rate  = msg.twist.angular.x
        self.pitch_rate = msg.twist.angular.y
        self.yaw_rate   = msg.twist.angular.z

        #self.vel_received = True

    # ---------------- LOAD TRAJECTORY DATA ----------------

    def loadTrajectoryDataFromCSV(self, file_name):
        # Load trajectory data from CSV file into a numpy matrix
        data = np.loadtxt(file_name, delimiter=',')
        return data
    
    # calculate the error

    def error(self):

        #error_x=self.helipad_odometry.pose.pose.position.y-self.vehicle_odometry.position[0]
        #error_y=self.helipad_odometry.pose.pose.position.x-self.vehicle_odometry.position[1]
        

        print(len(self.px))
        print(self.index)

        #error_x=self.px[self.index]-self.vehicle_odometry.position[1]             #  
        #error_y=self.py[self.index]-self.vehicle_odometry.position[0]

        #error_z=self.z_ref-self.vehicle_odometry.position[2]

        error_x=self.px[self.index]-self.x_pos             #  
        error_y=self.py[self.index]-self.y_pos

        error_z=self.z_ref-self.z_pos

        error_vx=self.vx[self.index]-self.x_vel
        error_vy=self.vy[self.index]-self.y_vel
        
        error_vz=0.0-self.z_vel

        return(error_x,error_y,error_z,error_vx,error_vy,error_vz)
    
    # applying tracking controller
    
    def tracking_pp_controller(self):


        
        l=list(self.error())


        """del_ax=(5.7*l[0]+3.4*l[3])
        del_ay=(5.7*l[1]+3.4*l[4])
        del_az=(6.2*l[2]+4.0*l[5])"""

        del_ax=(5.7*l[0]+3.4*l[3])
        del_ay=(5.7*l[1]+3.4*l[4])
        del_az=(6.2*l[2]+4.0*l[5])

        # since we want to give velocity command 

        return (del_ax,del_ay,del_az)

        

    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):
        print(self.stage)

        # ---------- WAIT FOR FCU ----------
        if not self.state.connected:
            self.get_logger().info('Waiting for FCU...')
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
            req.altitude = self.takeoff_height  # target altitude in meters
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
            if self.z_pos >=2.4 :
                self.get_logger().info('Target altitude reached, starting hover...')
                self.stage_start_time = self.get_clock().now()
                self.stage = 4

                self.z_ref=self.z_pos
                self.x_ref=self.x_pos
                self.y_ref=self.y_pos

                self.roll_ref=self.roll
                self.pitch_ref=self.pitch
                self.yaw_ref=self.yaw


        # ---------- STAGE 4: HOVER ----------
        
        elif self.stage == 4:
            elapsed = (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Hovering: {elapsed:.1f} s')

            pose = PoseStamped()
           
            # Example: Move forward, right, and ascend
            
            pose.pose.position.x = self.x_ref
            pose.pose.position.y = self.y_ref
            pose.pose.position.z = self.z_ref                                                 # up

            self.pos_pub.publish(pose)


            if elapsed >= 10.0:
                self.get_logger().info('Starting velocity control...')
                self.stage_start_time = self.get_clock().now()
                self.stage = 5

        # ---------- STAGE 5: VELOCITY CONTROL ----------

        elif self.stage == 5:
            elapsed = (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9

            del_ax,del_ay,del_az=self.tracking_pp_controller()

            ax=self.ax[self.index]
            ay=self.ay[self.index]
            az=self.az[self.index]

            
            twist = TwistStamped()
           
            # Example: Move forward, right, and ascend
            
 
            twist.twist.linear.x = self.vx[self.index] + (del_ax)*self.dt                                       # forward
            twist.twist.linear.y = self.vy[self.index] + (del_ay)*self.dt                                                # right
            twist.twist.linear.z = self.vz[self.index] + (del_az)*self.dt                                                 # up

            e_x,e_y,e_z,e_vx,e_vy,e_vz=self.error()

            e_z=self.z_ref-self.z_pos

            
            self.vel_pub.publish(twist)

            self.get_logger().info(f'Velocity control running: {elapsed:.1f} s')

            # Storing the data current datas 

            position = [self.x_pos,self.y_pos, self.z_pos]
            #orientation = [self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3]]
            orientation = [self.qx,self.qy,self.qz,self.qw]
            
            roll,pitch,yaw=quaternion_to_euler(self.qx,self.qy,self.qz,self.qw)

            velocity = [self.x_vel, self.y_vel, self.z_vel]
            angular_velocity = [self.roll_rate,self.pitch_rate,self.yaw_rate]

             
            try:
                current = np.concatenate([
                    [self.t[self.index]],
                    position,
                    [self.px[self.index], self.py[self.index], 0],
                    [e_x, e_y, e_z],
                    velocity,
                    [self.vx[self.index], self.vy[self.index], 0],
                    [e_vx, e_vy, e_vz],
                    [roll, pitch, yaw],
                    angular_velocity
                ])

                print(current)

                with open("state_data.csv", mode="a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(current.tolist())   # IMPORTANT
                    print("wrote")
                    file.flush()

                self.get_logger().info("CSV written")

            except Exception as e:
                self.get_logger().error(f"CSV write failed: {e}")

            self.index+=10

            if self.index>=len(self.px):
                self.stage=6

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
