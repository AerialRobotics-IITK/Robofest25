#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import TrajectorySetpoint ,  OffboardControlMode, VehicleControlMode, VehicleStatus, VehicleCommand, VehicleOdometry
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry

import csv
import numpy as np
import signal

import math
from scipy.spatial.transform import Rotation as R


def quaternion_to_euler(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw


class Offboard(Node):

    def __init__(self):
        super().__init__("offboard")

        self.takeoff_mode=False

        self.counter=0

        self.t=0.1                  # time
        self.dt=0.1

         # QOS Profiles
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_profile_gt = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Publishers

        self.offboard_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

         # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.status_subscriber = self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.state_callback, qos_profile)
        self.subscription = self.create_subscription(
            Int32MultiArray,                         
            '/hand_distance',   
            self.distance_callback, 
            10) 

        self.vehicle_odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, qos_profile)
        
        
        self.vehicle_status = None
        self.state = None

        self.distance = 0
        self.prev_distance = 0
        self.vel = 0
        self.lock = 0
        self.direction = 0
       
        self.vehicle_odometry = None
        

        self.takeoff_height=-5.0

        self.create_timer(0.1, self.command_loop)  # call every 2 sec

        self.rmse_x=0.0
        self.rmse_y=0.0
        self.rmse_z=0.0


        self.rmse_vx=0.0
        self.rmse_vy=0.0
        self.rmse_vz=0.0

        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0

        self.acc_mode=True



        #self.state=             # exploration state ,  Go to state , Trace State , Land state 

        with open("state_data_task_4.csv", mode="w+", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["time","pos_x", "pos_y", "pos_z","ideal_x","ideal_y","ideal_z","error_x","error_y","error_z","vel_x", "vel_y", "vel_z" ,"ideal_vx","ideal_vy","ideal_vz","error_vx","error_vy","error_vz","roll","pitch","yaw","roll_rate","pitch_rate","yaw_rate"])



    # Subscriber Callback Functions
    
    def state_callback(self, status_msg):
        self.status = status_msg
    
    def vehicle_status_callback(self, vehicle_status_msg):
        self.vehicle_status = vehicle_status_msg
   
    def vehicle_odometry_callback(self, vehicle_odometry_msg):
        self.vehicle_odometry = vehicle_odometry_msg
    
    def distance_callback(self, msg):
        self.prev_distance = self.distance
        self.distance = msg.data[0]/100#4*math.tan(math.asin(msg.data[0]*0.00026)) d0 = 4
        self.vel = (self.distance-self.prev_distance)*30
        self.lock = msg.data[1]
        self.direction = msg.data[2]
        


    def offboard_control_heartbeat_signal_publisher(self):
        msg = OffboardControlMode()
        msg.position = self.takeoff_mode
        msg.velocity = False
        msg.acceleration = self.acc_mode
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False 
        msg.direct_actuator = False 
        
        """match what_control:
            case 'position':
                msg.position = True
            case 'velocity':
                msg.velocity = True
            case 'acceleration':
                msg.acceleration = True
            case 'attitude':
                msg.attitude = True
            case 'body_rate':
                msg.body_rate = True
            case 'thrust_and_torque':
                msg.thrust_and_torque = True
            case 'direct_actuator':
                msg.direct_actuator = True"""
        
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.offboard_mode_publisher.publish(msg)


    def engage_offboard_mode(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    def disarm(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    


    def takeoff(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    def publish_trajectory_wpt(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 1.57079
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.trajectory_publisher.publish(msg)

    def publish_trajectory_v(self, vx, vy, vz):
        msg = TrajectorySetpoint()
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 1.57079
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.trajectory_publisher.publish(msg)

    def publish_trajectory_a(self, ax,ay,az):
        msg = TrajectorySetpoint()
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = ax
        msg.acceleration[1] = ay
        msg.acceleration[2] = az
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 1.57079
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.trajectory_publisher.publish(msg)

    def error(self):

        if self.direction:

            error_x=self.x_ref-self.vehicle_odometry.position[0]
            #error_y=self.helipad_odometry.pose.pose.position.x-self.vehicle_odometry.position[1]
            error_z=self.z_ref-self.vehicle_odometry.position[2]
            error_y=self.distance-self.vehicle_odometry.position[1]
        
            error_vx=0.0-self.vehicle_odometry.velocity[0]
            error_vy=self.vel-self.vehicle_odometry.velocity[1]
            error_vz=0.0-self.vehicle_odometry.velocity[2]

        else:
            error_x=self.distance-self.vehicle_odometry.position[0]
            #error_y=self.helipad_odometry.pose.pose.position.x-self.vehicle_odometry.position[1]
            error_z=self.z_ref-self.vehicle_odometry.position[2]
            error_y=self.y_ref-self.vehicle_odometry.position[1]
        
            error_vx=self.vel-self.vehicle_odometry.velocity[0]
            error_vy=0.0-self.vehicle_odometry.velocity[1]
            error_vz=0.0-self.vehicle_odometry.velocity[2]

        self.integrate_x+=error_x*0.1
        self.integrate_y+=error_y*0.1
        self.integrate_z+=error_z*0.1

        self.integrate_vx+=error_vx*0.1
        self.integrate_vy+=error_vy*0.1
        self.integarte_vz+=error_vz*0.1

        self.rmse_x+=error_x**2
        self.rmse_y+=error_y**2
        self.rmse_z+=error_z**2

        self.rmse_vx+=error_vx**2
        self.rmse_vy+=error_vy**2
        self.rmse_vz+=error_vz**2

        return(error_x,error_y,error_z,error_vx,error_vy,error_vz)
    
    def tracking_pp_controller(self):


        
        l=list(self.error())


        del_ax=(0.1*l[0]+5*l[3] + 0*self.integrate_x)
        del_ay=(0.1*l[1]+5*l[4] + 0*self.integrate_y)
        del_az=(0.1*l[2]+5*l[5] + 0*self.integrate_z)

        return (del_ax,del_ay,del_az)
    
    def tracking_ppid_controller(self):

        l=list(self.error())

        """
        r_sp : desired position (np.array, shape (3,))
        r_hat: measured/estimated position (np.array, shape (3,))
        v_hat: measured/estimated velocity (np.array, shape (3,))
        return: acceleration setpoint (np.array, shape (3,))
        """

        K_r_xy=0.95
        K_p_xy=1.8                                         #1.8
        K_i_xy=0.4
        K_d_xy=0.2

        K_r_z=1.0
        K_p_z=4.0
        K_i_z=2.0
        K_d_z=0.0


        # --- Position Error ---
        delta_r_x,delta_r_y,delta_r_z=l[0],l[1],l[2]

        


        # --- P-controller for position → velocity setpoint ---
        v_sp_x = K_r_xy * delta_r_x
        v_sp_y = K_r_xy * delta_r_y
        v_sp_z = K_r_z * delta_r_z

        #v_sp = self.saturate(v_sp, self.v_max)
        
        #estimated velocity

        v_hat_x=self.vehicle_odometry.velocity[0]
        v_hat_y=self.vehicle_odometry.velocity[1]
        v_hat_z=self.vehicle_odometry.velocity[2]

        # --- Velocity Error ---
        delta_v_x = v_sp_x - v_hat_x
        delta_v_y = v_sp_y - v_hat_y
        delta_v_z = v_sp_z - v_hat_z

        # --- Integral Term ---
        self.integral_error_x += delta_v_x
        self.integral_error_y += delta_v_y
        self.integral_error_z += delta_v_z

        self.v_hat_prev_x=0.0
        self.v_hat_prev_y=0.0
        self.v_hat_prev_z=0.0


        # --- Derivative Term (backward difference on velocity) ---
        dv_dt_x = (v_hat_x - self.v_hat_prev_x) / self.dt
        dv_dt_y = (v_hat_y - self.v_hat_prev_y) / self.dt
        dv_dt_z = (v_hat_z - self.v_hat_prev_z) / self.dt

    

        """dv_dt_x=0.0
        dv_dt_y=0.0
        dv_dt_z=0.0"""

        # --- Final Acceleration Setpoint ---
        a_sp_x = (K_p_xy * delta_v_x + K_i_xy * self.integral_error_x - K_d_xy * dv_dt_x)
        
        a_sp_y = (K_p_xy * delta_v_y + K_i_xy * self.integral_error_y - K_d_xy * dv_dt_y)
        a_sp_z = (K_p_z * delta_v_z + K_i_z * self.integral_error_z - K_d_z * dv_dt_z)

        # Store velocity setpoint for next cycle (if needed elsewhere)
        self.v_hat_prev_x=v_hat_x
        self.v_hat_prev_y=v_hat_y
        self.v_hat_prev_z=v_hat_z

        a_sp_z_2=3*(self.z_ref-self.vehicle_odometry.position[2])+5*(0.0-self.vehicle_odometry.velocity[2])
        


        return a_sp_x,a_sp_y,a_sp_z


    def command_loop(self):

        
        self.offboard_control_heartbeat_signal_publisher()

        print(self.takeoff_mode)
        print(self.counter)
        self.counter+=1

        if self.counter<100:
            return
        
        elif self.counter == 100:
            self.engage_offboard_mode()
            self.arm()
            self.takeoff_mode=True


        if self.takeoff_mode:

            print("yes_okay")
            self.publish_trajectory_wpt(0,0,-5.0)
            
            if (-5.0-0.1 <= self.vehicle_odometry.position[2] <= -5.0+0.1) :
                self.z_ref=self.vehicle_odometry.position[2]
                self.y_ref=self.vehicle_odometry.position[1]
                self.x_ref=self.vehicle_odometry.position[0]
                
                self.takeoff_mode=False


        if not self.takeoff_mode and self.counter>100 and self.lock:

            ax,ay,az=self.tracking_pp_controller()
            self.publish_trajectory_a(ax,ay,az)
            #print(self.vehicle_odometry.position)

        elif not self.takeoff_mode and self.counter>100 and not self.lock:

            self.publish_trajectory_v(0,0,0)
            
            

        position = [self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]]
        orientation = [self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3]]
        
        roll,pitch,yaw=quaternion_to_euler(self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3])

        velocity = [self.vehicle_odometry.velocity[0], self.vehicle_odometry.velocity[1], self.vehicle_odometry.velocity[2]]
        angular_velocity = [self.vehicle_odometry.angular_velocity[0], self.vehicle_odometry.angular_velocity[1], self.vehicle_odometry.angular_velocity[2]]

        

            

        if self.vehicle_odometry != None and self.takeoff_mode==False and self.counter>100:
            print(self.t)
            

            e_x,e_y,e_z,e_vx,e_vy,e_vz=self.error()

            e_z=0.0-self.vehicle_odometry.position[2]


            self.t+=0.1



        print("\n")

        

def main(args=None):
    rclpy.init(args=args)
    offboard = Offboard()

    try:
        rclpy.spin(offboard)   # Runs until Ctrl+C
    except KeyboardInterrupt:
        print("Stopping with Ctrl+C...")
    finally:
        print("rmse_x = ",math.sqrt(offboard.rmse_x/offboard.t))
        print("rmse_y = ",math.sqrt(offboard.rmse_y/offboard.t))
        print("rmse_z = ",math.sqrt(offboard.rmse_z/offboard.t))

        print("rmse_vx = ",math.sqrt(offboard.rmse_vx/offboard.t))
        print("rmse_vy = ",math.sqrt(offboard.rmse_vy/offboard.t))
        print("rmse_vz = ",math.sqrt(offboard.rmse_vz/offboard.t))


        offboard.destroy_node()

if __name__ == "__main__":
    main()



