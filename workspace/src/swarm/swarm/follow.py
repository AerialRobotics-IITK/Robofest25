#!/usr/bin/env python3
"""
Subscribe to /p_point (PointStamped) and send uav2 there.
Works both on the ground and in the air.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

class GotoPPointNode(Node):
    def __init__(self):
        super().__init__('goto_ppoint_uav2')

        self.takeoff_alt = 2.0
        self.arming_timeout = 10.0
        self.mode_timeout = 10.0
        self.loop_rate = 20

        # publishers
        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/uav2/setpoint_raw/local', qos_profile_system_default)

        # clients
        self.arming_cli = self.create_client(CommandBool, '/uav2/cmd/arming')
        self.set_mode_cli = self.create_client(SetMode, '/uav2/set_mode')

        # subscriber
        self.create_subscription(
            PointStamped, '/p_point', self.new_target_callback, qos_profile_system_default)

        # state
        self.target = None
        self.last_target_time = 0.0
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)

        self.get_logger().info("Waiting for /p_point messages...")

    # ------------------------------------------------------------------
    def new_target_callback(self, msg: PointStamped):
        self.target = msg.point
        self.last_target_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info(f'New target: x={self.target.x:.2f} y={self.target.y:.2f} z={self.target.z:.2f}')

    # ------------------------------------------------------------------
    def wait_for_service(self, cli, timeout):
        t0 = time.time()
        while not cli.wait_for_service(timeout_sec=0.5):
            if time.time() - t0 > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    # ------------------------------------------------------------------
    def set_mode(self, mode: str) -> bool:
        if not self.wait_for_service(self.set_mode_cli, self.mode_timeout):
            self.get_logger().error('SetMode service not available')
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().mode_sent

    # ------------------------------------------------------------------
    def arm(self, arm: bool) -> bool:
        if not self.wait_for_service(self.arming_cli, self.arming_timeout):
            self.get_logger().error('Arming service not available')
            return False
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().success

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.target is None:
            return

        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX |
                        PositionTarget.IGNORE_VY |
                        PositionTarget.IGNORE_VZ |
                        PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY |
                        PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = self.target.x
        sp.position.y = self.target.y
        sp.position.z = self.target.z
        sp.yaw = 0.0

        # if not self.is_armed():
        #     self.get_logger().info('Arming...')
        #     if self.arm(True):
        #         self.get_logger().info('Armed, switching to GUIDED')
        #         self.set_mode('GUIDED')
        #         self.simple_takeoff(self.takeoff_alt)
        #     else:
        #         self.get_logger().warn('Arming failed')
        #         return

        self.setpoint_pub.publish(sp)

    # ------------------------------------------------------------------
    def is_armed(self) -> bool:
        if not hasattr(self, '_armed'):
            self._armed = False
            self.create_subscription(
                State, '/uav2/state',
                lambda st: setattr(self, '_armed', st.armed),
                qos_profile_system_default)
            rclpy.spin_once(self, timeout_sec=1.0)
        return self._armed

    # ------------------------------------------------------------------
    def simple_takeoff(self, alt):
        cli = self.create_client(CommandTOL, '/uav2/cmd/takeoff')
        if not self.wait_for_service(cli, 5):
            self.get_logger().error('Takeoff service not ready')
            return
        req = CommandTOL.Request()
        req.altitude = alt
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Take-off command accepted')
        else:
            self.get_logger().warn('Take-off command failed')

# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GotoPPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
