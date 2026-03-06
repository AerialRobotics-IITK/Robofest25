from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    # -----------------------------
    # 1. Zenoh Router
    # -----------------------------
    zenoh = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="zenoh_router",
        output="screen"
    )

    ld.add_action(TimerAction(period=0.0,actions=[zenoh]))
    # -----------------------------
    # 2. MAVROS (apm.launch)
    # -----------------------------
    num = int(os.environ.get("NUM",1))
    mavros_launchs = []
    for i in range(1,num+1):
        mavros_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("mavros"),
                    "launch",
                    "apm.launch"
                )
            ),
            launch_arguments={
                "namespace": f"uav{i}",
                "tgt_system": str(i),
                "fcu_url": f'udp://:1455{i}@',
            }.items()
        )
        # mavros_launchs.append(mavros_launch)
        ld.add_action(TimerAction(period=float(i*5),actions=[mavros_launch]))

    # start_mavros_after_zenoh = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=zenoh,
    #         on_start=mavros_launchs
    #     )
    # )

    return ld
    # -----------------------------
    # Build launch description
    # -----------------------------
    # return LaunchDescription([
    #     zenoh,
    #     start_mavros_after_zenoh,
    # ])
