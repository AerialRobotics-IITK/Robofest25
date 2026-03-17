import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import asyncio
import websockets
import json
import time
import threading


class FleetStateNode(Node):

    def __init__(self):

        super().__init__("fleet_state_ws_node")

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -------------------------
        # INTERNAL STATE
        # -------------------------

        self.mode = "UNKNOWN"

        self.drones = {
            1: {"x": None, "y": None, "battery":"100%"},
            2: {"x": None, "y": None, "battery":"100%"},
            3: {"x": None, "y": None, "battery":"100%"},
            4: {"x": None, "y": None, "battery":"100%"},
        }

        self.mines = []
        self.navigation_path = []

        self.ws_clients = set()
        self.last_logged_mode = None

        # -------------------------
        # SUBSCRIBERS
        # -------------------------

        self.create_subscription(
            PointStamped,
            "/uav1/local_pos",
            lambda msg: self.drone_callback(msg, 1),
            self.qos_profile,
        )

        self.create_subscription(
            PointStamped,
            "/uav2/local_pos",
            lambda msg: self.drone_callback(msg, 2),
            self.qos_profile,
        )

        self.create_subscription(
            PointStamped,
            "/uav3/local_pos",
            lambda msg: self.drone_callback(msg, 3),
            self.qos_profile,
        )

        self.create_subscription(
            PointStamped,
            "/uav4/local_pos",
            lambda msg: self.drone_callback(msg, 4),
            self.qos_profile,
        )

        self.create_subscription(
            Int32MultiArray,
            "/hand_distance",
            self.mode_callback,
            self.qos_profile,
        )

        self.create_subscription(
            PoseArray,
            "/detected_mines",
            self.mines_callback,
            self.qos_profile,
        )

        self.create_subscription(
            Path,
            "/navigation_path",
            self.path_callback,
            self.qos_profile,
        )

        self.create_subscription(
            BatteryState,
            "/uav1/battery",
            lambda msg : self.battery_callback(msg, 1),
            self.qos_profile,
        )

        self.create_subscription(
            BatteryState,
            "/uav2/battery",
            lambda msg : self.battery_callback(msg, 2),
            self.qos_profile,
        )

        self.create_subscription(
            BatteryState,
            "/uav3/battery",
            lambda msg : self.battery_callback(msg, 3),
            self.qos_profile,
        )

        self.create_subscription(
            BatteryState,
            "/uav4/battery",
            lambda msg : self.battery_callback(msg, 4),
            self.qos_profile,
        )

        # publish JSON at 20Hz
        self.timer = self.create_timer(0.05, self.broadcast_state)

        # start websocket server
        self.start_websocket_server()
        self.get_logger().info("Fleet state websocket node started")

    # ------------------------------------------------
    # ROS CALLBACKS
    # ------------------------------------------------
    def battery_callback(self, msg, drone_id):
        self.drones[drone_id]["battery"] = f"{msg.percentage}%"

    def drone_callback(self, msg, drone_id):
        print("in")
        self.drones[drone_id]["x"] = msg.point.x
        self.drones[drone_id]["y"] = msg.point.y

    def mode_callback(self, msg):
        if(msg.data[1]):
            self.mode = "Manual Override"
        else:
            self.mode = "Autonomous"

        if self.mode != self.last_logged_mode:
            self.get_logger().info(f"Fleet mode updated to: {self.mode}")
            self.last_logged_mode = self.mode

    def mines_callback(self, msg):

        mines = []

        for pose in msg.poses:
            mines.append({
                "x": pose.position.x,
                "y": pose.position.y
            })

        self.mines = mines
        self.get_logger().debug(f"Updated detected mines count: {len(self.mines)}")

    def path_callback(self, msg):

        path = []

        for pose in msg.poses:
            path.append({
                "x": pose.pose.position.x,
                "y": pose.pose.position.y
            })

        self.navigation_path = path
        self.get_logger().debug(
            f"Updated navigation path with {len(self.navigation_path)} waypoints"
        )

    # ------------------------------------------------
    # JSON BUILDING
    # ------------------------------------------------

    def build_json(self):

        drones_list = []
        for i in range(1, 5):
            if self.drones[i]["x"] == None and self.drones[i]["y"] == None:
                continue
            drones_list.append({
                "id": i,
                "pose": {
                    "x": self.drones[i]["x"],
                    "y": self.drones[i]["y"]
                },
                "battery": self.drones[i]["battery"]
            })

        data = {
            "timestamp": time.time(),
            "mode": self.mode,
            "drones": drones_list,
            "mines": self.mines,
            "navigation_path": self.navigation_path
        }

        return json.dumps(data)

    # ------------------------------------------------
    # WEBSOCKET
    # ------------------------------------------------

    async def ws_handler(self, websocket):

        self.ws_clients.add(websocket)
        self.get_logger().info(
            f"WebSocket client connected, active clients: {len(self.ws_clients)}"
        )

        try:
            await websocket.wait_closed()
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(
                f"WebSocket client disconnected, active clients: {len(self.ws_clients)}"
            )

    async def ws_server(self):

        self.get_logger().info("Starting websocket server on 0.0.0.0:8080")
        async with websockets.serve(self.ws_handler, "0.0.0.0", 8080):
            await asyncio.Future()  # run forever

    def start_websocket_server(self):

        def run():

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.ws_server())
            except Exception as exc:
                self.get_logger().error(f"Websocket server stopped: {exc}")

        thread = threading.Thread(target=run, daemon=True)
        thread.start()

    # ------------------------------------------------
    # BROADCAST STATE
    # ------------------------------------------------

    def broadcast_state(self):
        message = self.build_json()
        self.get_logger().info(f"Broadcasting websocket payload: {message}")

        asyncio.run(self.send_to_clients(message))

    async def send_to_clients(self, message):

        dead = []

        for ws in self.ws_clients:

            try:
                await ws.send(message)

            except Exception as exc:
                self.get_logger().warning(f"Failed to send websocket update: {exc}")
                dead.append(ws)

        for ws in dead:
            self.ws_clients.discard(ws)


def main():

    rclpy.init()

    node = FleetStateNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
