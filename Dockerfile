from docker.io/ros:humble
run apt-get update && apt-get install -y tmux ripgrep neovim
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp\
                        ros-${ROS_DISTRO}-mavros-extras wget
run wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
run chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh
run echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> /root/.bashrc
run apt-get update && apt-get install -y python3-pip
run pip3 install pyproj
env RMW_IMPLEMENTATION=rmw_zenoh_cpp
copy mavros/ /root/
run cat /root/mod.bashrc >> /root/.bashrc
copy workspace/ /workspace/
workdir /workspace
run colcon build --symlink-install
run echo 'source /workspace/install/setup.bash' >> /root/.bashrc
cmd [ "tmux" ]
