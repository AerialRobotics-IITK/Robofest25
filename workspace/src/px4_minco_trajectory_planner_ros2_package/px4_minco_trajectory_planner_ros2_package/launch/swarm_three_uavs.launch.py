from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common_swarm_params = [
        {"swarm.enabled": True},
        {"swarm.peer_timeout_sec": 1.0},
        {"swarm.threshold": 1.0},
        {"swarm.ellipsoid_diag": [1.0, 1.0, 1.0]},
        {"swarm.default_segment_dt": 0.1},
    ]

    uav1_node = Node(
        package="px4_minco_trajectory_planner_ros2_package",
        executable="integrated_drone_mapping_node_with_drone_tracking",
        name="integrated_mapping_sfc_gcopter_uav1",
        output="screen",
        parameters=common_swarm_params
        + [
            {"namespace": "/uav1"},
            {"planning.start.use_odom": True},
            {"planning.goal.x": 10.0},
            {"planning.goal.y": -7.0},
            {"planning.goal.z": 1.0},
            {"swarm.self_trajectory_topic": "/swarm/uav1/trajectory"},
            {
                "swarm.peer_topics": [
                    "/swarm/uav2/trajectory",
                    "/swarm/uav3/trajectory",
                ]
            },
        ],
        remappings=[
            ("/uav1/scan", "/uav1/scan"),
        ],
    )

    uav2_node = Node(
        package="px4_minco_trajectory_planner_ros2_package",
        executable="integrated_drone_mapping_node_with_drone_tracking",
        name="integrated_mapping_sfc_gcopter_uav2",
        output="screen",
        parameters=common_swarm_params
        + [
            {"namespace": "/uav2"},
            {"planning.start.use_odom": True},
            {"planning.goal.x": 8.0},
            {"planning.goal.y": 4.0},
            {"planning.goal.z": 1.0},
            {"swarm.self_trajectory_topic": "/swarm/uav2/trajectory"},
            {
                "swarm.peer_topics": [
                    "/swarm/uav1/trajectory",
                    "/swarm/uav3/trajectory",
                ]
            },
        ],
        remappings=[
            ("/uav1/scan", "/uav2/scan"),
        ],
    )

    uav3_node = Node(
        package="px4_minco_trajectory_planner_ros2_package",
        executable="integrated_drone_mapping_node_with_drone_tracking",
        name="integrated_mapping_sfc_gcopter_uav3",
        output="screen",
        parameters=common_swarm_params
        + [
            {"namespace": "/uav3"},
            {"planning.start.use_odom": True},
            {"planning.goal.x": -6.0},
            {"planning.goal.y": -6.0},
            {"planning.goal.z": 1.0},
            {"swarm.self_trajectory_topic": "/swarm/uav3/trajectory"},
            {
                "swarm.peer_topics": [
                    "/swarm/uav1/trajectory",
                    "/swarm/uav2/trajectory",
                ]
            },
        ],
        remappings=[
            ("/uav1/scan", "/uav3/scan"),
        ],
    )

    return LaunchDescription([uav1_node, uav2_node, uav3_node])
