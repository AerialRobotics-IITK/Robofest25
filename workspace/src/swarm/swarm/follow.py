import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PointStamped,PoseStamped
from mavros_msgs.srv import CommandBool, SetMode,CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.namespace = f"uav{os.environ.get('MAV_ID',2)}",
        self.set_mode_cli = self.create_client(SetMode,f'{self.namespace}/set_mode')
        self.arming_cli = self.create_client(CommandBool, f'{self.namespace}/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, f'{self.namespace}/cmd/takeoff')

        self.publisher = self.create_publisher(PoseStamped,f'{self.namespace}/setpoint_position/local',qos_profile)
        self.tookoff = False

        self.target = None
        self.loop_rate = 10
        self.sub = self.create_subscription(PointStamped,'p_point',self.target_update,10)
        # self.timer = self.create_timer(1.0/self.loop_rate,self.control_loop)

        # Setting to guided mode
        if self.set_mode('GUIDED'):
            self.get_logger().info("GUIDED mode activated")
        else:
            self.get_logger().error("GUIDED mode not activated")
            rclpy.shutdown()

        # Arming Vehicle
        if self.arm(True):
            self.get_logger().info("ARMED!")
        else:
            self.get_logger().error("not ARMED!")
            rclpy.shutdown()

       # Takeoff Vehicle
        if self.takeoff(3.0):
            self.get_logger().info("Vehicle took off")
            time.sleep(5.0)
            self.tookoff = True
        else:
            self.get_logger().error("Takeoff failed")
            rclpy.shutdown()



    def wait_for_service(self, cli, timeout):
        t0 = time.time()
        while not cli.wait_for_service(timeout_sec=0.5):
            if time.time() - t0 > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def set_mode(self, mode: str) -> bool:
        self.get_logger().info(f"Trying to set mode {mode}")
        if not self.wait_for_service(self.set_mode_cli, 5.0):
            self.get_logger().error('SetMode service not available')
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().mode_sent

    def arm(self, arm: bool) -> bool:
        self.get_logger().info("Trying to arm")
        if not self.wait_for_service(self.arming_cli, 2.0):
            self.get_logger().error('Arming service not available')
            return False
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future,timeout_sec=2.0)
        return future.result() is not None and future.result().success

    def takeoff(self,altitude:float)->bool:
        self.get_logger().info("Trying to takeoff")
        if not self.wait_for_service(self.takeoff_cli, 2.0):
            self.get_logger().error('Takeoff service not available')
            return False
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = altitude
        future = self.takeoff_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future,timeout_sec=2.0)
        return future.result() is not None and future.result().success

    def target_update(self,msg:PointStamped):
        # self.get_logger().info("Got P point")
        if self.tookoff:
            self.get_logger().info("Moving")
            msg2 = PoseStamped()
            msg2.pose.position = msg.point
            self.publisher.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


