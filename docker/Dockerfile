from docker.io/ros:humble
run apt-get update && apt-get install -y tmux ripgrep neovim
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp\
                        ros-${ROS_DISTRO}-mavros-extras
run ros2 run mavros install_geographiclib_datasets.sh
run echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> .bashrc
