from docker.io/ros:humble

# Basic Utils and MAVROS
run apt-get update && apt-get install -y tmux ripgrep neovim fd-find
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-mavros-extras wget
run wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
run chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh

# Zenoh Stuff
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp
# Package stuff
run apt-get update && apt-get install -y python3-pip ros-${ROS_DISTRO}-camera-ros \
                      python3-serial python3-pyproj

# Requires python3.10
run pip install mediapipe==0.10.9 scipy opencv-python

run apt-get update && apt-get install -y libgeographic-dev pkg-config

# copy workspace/ /workspace/
workdir /workspace
run --mount=type=bind,source=./workspace/src,target=/workspace/src \
      . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# Final config modifications
env RMW_IMPLEMENTATION=rmw_zenoh_cpp
env ZENOH_ROUTER_CONFIG_URI=/root/router_config.json5
copy zenoh/ /root/


copy Tools /Tools
run chmod +x /Tools/update_router_config.py
run chmod +x /Tools/entrypoint.sh
entrypoint [ "/Tools/entrypoint.sh" ]
cmd [ "tmux" ]
