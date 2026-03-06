#!/bin/bash

cd /workspace
colcon build --packages-select human_tracking_controls
source install/setup.bash
ros2 run human_tracking_controls mavros_yaw_body_tracking
