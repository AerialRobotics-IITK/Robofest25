from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from swarm import drone_sync

def generate_launch_description():

    # -----------------------------
    # 1. Zenoh Router
    # -----------------------------
    zenoh = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="zenoh_router",
        output="screen"
    )

    # -----------------------------
    # 2. MAVROS (apm.launch)
    # -----------------------------
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mavros"),
                "launch",
                "apm.launch"
            )
        ),
        launch_arguments={
            "namespace": "uav2",
            "gcs_url": "",
        }.items()
    )

    start_mavros_after_zenoh = RegisterEventHandler(
        OnProcessStart(
            target_action=zenoh,
            on_start=[mavros_launch]
        )
    )

    drone_sync = Node(
        package="swarm",
        executable="sync",
        name="drone_sync",
        output="screen"
    )
    # -----------------------------
    # 3. p_finder (depends on MAVROS)
    # -----------------------------
    p_finder = Node(
        package="swarm",
        executable="finder",
        name="p_finder",
        output="screen"
    )

    # -----------------------------
    # 4. follower (depends on p_finder)
    # -----------------------------
    follower = Node(
        package="swarm",
        executable="follow",
        name="follower",
        output="screen"
    )

    call_service = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/uav2/set_stream_rate",
            "mavros_msgs/srv/StreamRate",
            "{\"stream_id\": 0, \"message_rate\": 10, \"on_off\": true }"
        ],
        output="screen"
    )
    start_after_delay = TimerAction(
        period=5.0,
        actions=[call_service,drone_sync,p_finder,follower],
    )
    # -----------------------------
    # Build launch description
    # -----------------------------
    return LaunchDescription([
        zenoh,
        start_mavros_after_zenoh,
        start_after_delay
    ])
