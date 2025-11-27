from docker.io/ros:humble
run apt-get update && apt-get install -y tmux ripgrep neovim
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp\
                        ros-${ROS_DISTRO}-mavros-extras wget
run wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
run chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh
run echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> .bashrc
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-demo-nodes-cpp
workdir /root
copy . .
run cat mod.bashrc >> /root/.bashrc
