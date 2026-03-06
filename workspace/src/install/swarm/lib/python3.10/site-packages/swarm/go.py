#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn


class ArduPilotTakeoff(Node):

    def __init__(self):
        super().__init__('ardupilot_takeoff')

        # ---------------- STATE ----------------
        self.state = State()
        self.altitude = 0.0
        self.altitude_received = False
        self.stage = 0
        self.hover_start_time = None

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(State, '/uav3/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/uav3/local_position/pose', self.pos_cb, qos)

        # ---------------- SERVICE CLIENTS ----------------
        self.arming_client = self.create_client(CommandBool, '/uav3/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/uav3/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/uav3/cmd/takeoff')

        # ---------------- CONTROL LOOP ----------------
        self.timer = self.create_timer(0.5, self.control_loop)
    
    
    # ---------------- CALLBACKS ----------------

    def state_cb(self, msg):
        self.state = msg

    def pos_cb(self, msg):
        self.altitude = msg.pose.position.z
        self.altitude_received = True

    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):
        if not self.state.connected:
            self.get_logger().info('Waiting for FCU...')
            return

        # ---------- STAGE 0: GUIDED ----------
        if self.stage == 0:
            if self.state.mode != 'GUIDED':
                req = SetMode.Request()
                req.custom_mode = 'GUIDED'
                self.mode_client.call_async(req)
                self.get_logger().info('Setting GUIDED mode')
            else:
                self.stage = 1

        # ---------- STAGE 1: ARM ----------
        elif self.stage == 1:
            if not self.state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info('Arming vehicle')
            else:
                self.stage = 2

        # ---------- STAGE 2: TAKEOFF ----------
        elif self.stage == 2:
            req = CommandTOL.Request()
            req.altitude = 3.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0
            self.takeoff_client.call_async(req)
            self.get_logger().info('Takeoff command sent')
            self.stage = 3

        # ---------- STAGE 3: WAIT FOR ALTITUDE ----------
        elif self.stage == 3:
            if not self.altitude_received:
                return

            self.get_logger().info(f'Altitude: {self.altitude:.2f} m')

            if self.altitude >= 3.0:
                self.get_logger().info('Target altitude reached')
                self.hover_start_time = self.get_clock().now()
                self.stage = 4

        # ---------- STAGE 4: HOVER ----------
        elif self.stage == 4:
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Hovering {elapsed:.1f} s')

            if elapsed >= 5.0:
                self.stage = 5



        
            
        # ---------- STAGE 5: LAND ----------
        elif self.stage == 5:
            if self.state.mode != 'LAND':
                req = SetMode.Request()
                req.custom_mode = 'LAND'
                self.mode_client.call_async(req)
                self.get_logger().info('Landing...')
            else:
                self.get_logger().info('LAND mode active')
                self.timer.cancel()


# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
