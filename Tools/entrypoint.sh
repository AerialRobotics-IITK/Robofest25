#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /workspace/install/setup.bash
python3 /Tools/update_router_config.py
$1
