from docker.io/ros:humble

# Basic Utils and MAVROS
run apt-get update && apt-get install -y tmux ripgrep neovim
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-mavros-extras wget
run wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
run chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh

# Zenoh Stuff
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp
env RMW_IMPLEMENTATION=rmw_zenoh_cpp
env ZENOH_ROUTER_CONFIG_URI=/root/router_config.json5
copy zenoh/ /root/

# Package stuff
run apt-get update && apt-get install -y python3-pyproj
copy workspace/ /workspace/
workdir /workspace
run colcon build --symlink-install
run echo 'source /workspace/install/setup.bash' >> /root/.bashrc

# Final config modifications
copy Tools /Tools
run chmod +x /Tools/update_router_config.py && /Tools/update_router_config.py

cmd [ "tmux" ]
