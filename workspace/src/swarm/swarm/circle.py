#!/usr/bin/env python3

import rclpy
import os
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from scipy.spatial.transform import Rotation as R


class CircleFlight(Node):

    def __init__(self):
        super().__init__('circle_flight')

        # ===== Namespace from MAV_ID =====
        self.namespace = f"uav{os.environ.get('MAV_ID')}"
        self.get_logger().info(f"Using namespace: {self.namespace}")

        # ===== Parameters =====
        self.radius = 1.5
        self.omega = 1.0
        self.theta = 0.0
        self.altitude = 2.0
        

        # ===== State =====
        self.state = State()
        self.stage = 0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Current orientation (from local_position/pose)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # ===== Publishers =====
        self.pos_pub = self.create_publisher(
            PoseStamped,
            f'/{self.namespace}/setpoint_position/local',
            10
        )

        # ===== Subscribers =====
        self.create_subscription(
            State,
            f'/{self.namespace}/state',
            self.state_cb,
            10
        )

        self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/local_position/pose',
            self.pose_cb,
            10
        )

        # ===== Services =====
        self.arm_client = self.create_client(
            CommandBool,
            f'/{self.namespace}/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            f'/{self.namespace}/set_mode'
        )

        self.takeoff_client = self.create_client(
            CommandTOL,
            f'/{self.namespace}/cmd/takeoff'
        )

        # ===== Timer =====
        self.timer = self.create_timer(0.1, self.loop)

    # ================= CALLBACKS =================

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w

    # ================= MAIN LOOP =================

    def loop(self):

        if not self.state.connected:
            self.get_logger().info("Waiting for FCU...")
            return

        # ---- Stage 0: GUIDED ----
        if self.stage == 0:
            if self.state.mode != "GUIDED":
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self.mode_client.call_async(req)
                self.get_logger().info("GUIDED")
            else:
                self.stage = 1

        # ---- Stage 1: ARM ----
        elif self.stage == 1:
            if not self.state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arm_client.call_async(req)
            else:
                self.stage = 2

        # ---- Stage 2: TAKEOFF ----
        elif self.stage == 2:
            req = CommandTOL.Request()
            req.altitude = self.altitude
            self.takeoff_client.call_async(req)
            self.stage = 3
            self.get_logger().info("Takeoff")

        # ---- Stage 3: WAIT ALT ----
        elif self.stage == 3:
            self.get_logger().info("WAiting for alt")
            self.get_logger().info(f"Altitude: {self.z}")
            if abs(self.z - self.altitude) < 0.5:
                self.cx = self.x
                self.cy = self.y
                self.hover_start = self.get_clock().now()
                self.stage = 4

        # ---- Stage 4: HOVER ----
        elif self.stage == 4:
            pose = PoseStamped()
            pose.pose.position.x = self.cx
            pose.pose.position.y = self.cy
            pose.pose.position.z = self.altitude
            self.pos_pub.publish(pose)
            t = (self.get_clock().now() - self.hover_start).nanoseconds / 1e9
            if t > 5:
                self.get_logger
                self.circle_start = self.get_clock().now()
                self.stage = 5

        # ---- Stage 5: CIRCLE ----
        elif self.stage == 5:

            self.theta += self.omega * 0.1
            
            pose = PoseStamped()

            pose.pose.position.x = self.cx + self.radius * np.cos(self.theta)
            pose.pose.position.y = self.cy + self.radius * np.sin(self.theta)
            pose.pose.position.z = self.altitude
            
            # Use the current orientation reported by local_position/pose
            pose.pose.orientation.x = self.qx
            pose.pose.orientation.y = self.qy
            pose.pose.orientation.z = self.qz
            pose.pose.orientation.w = self.qw
            self.pos_pub.publish(pose)

            t = (self.get_clock().now() - self.circle_start).nanoseconds/1e9
            if t > 60:
                self.stage = 6

        # ---- Stage 6: HOLD ----
        elif self.stage == 6:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CircleFlight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
