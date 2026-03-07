from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Camera Node (Starts immediately)
    raspi_cam = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        output="screen"
    )

    # 2. Yaw Tracking Node 
    yaw_tracker = Node(
        package="object_tracker",
        executable="tracker_sitl",
        name="tracker_sitl",
        output="screen"
    )

    delayed_yaw_tracker = TimerAction(
        period=2.0,
        actions=[yaw_tracker]
    )

    # 3. Yaw Control Node 
    yaw_controller = Node(
        package="human_tracking_controls",
        executable="mavros_yaw_body_tracking",
        name="mavros_yaw_body_tracking",
        output="screen"
    )

    delayed_yaw_controller = TimerAction(
        period=5.0,
        actions=[yaw_controller]
    )

    # Return the description so ROS 2 knows what to launch
    return LaunchDescription([
        raspi_cam,
        delayed_yaw_tracker,
        delayed_yaw_controller
    ])