from docker.io/ros:humble

env DEBIAN_FRONTEND=noninteractive

# All apt-get installs
run apt-get update && apt-get install -y tmux ripgrep neovim fd-find wget ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-rmw-zenoh-cpp ros-${ROS_DISTRO}-camera-ros \
    python3-serial python3-pyproj libgeographic-dev pkg-config build-essential git libxml2-dev bison flex libcdk5-dev cmake python3-pip libusb-1.0-0-dev \
    libavahi-client-dev libavahi-common-dev libaio-dev python3-setuptools

# Basic Utils and MAVROS
run wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
run chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh

# Zenoh Stuff
run apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp
# Package stuff
run apt-get update && apt-get install -y python3-pip ros-${ROS_DISTRO}-camera-ros \
                      python3-serial python3-pyproj

run apt-get update && apt-get install -y \
    libgpiod-dev \
    libgpiodcxx-dev \
    libeigen-dev \
    libomp-dev \
    libxcb-cursor0 \
    libxkbcommon-x11-0 \
    libserialport-dev \
    python3-matplotlib \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
    
# Note: Removed 'pip install matplotlib' because 'python3-matplotlib' is faster on Pi

# Requires python3.10
run pip install mediapipe==0.10.9 scipy opencv-python

#installing sdr libraries
workdir /opt/sdr
run git clone --branch v0.23 https://github.com/analogdevicesinc/libiio.git && \
    cd libiio && \
    mkdir build && cd build && \
    cmake -DPYTHON_BINDINGS=ON .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig
run git clone https://github.com/analogdevicesinc/libad9361-iio.git && \
    cd libad9361-iio && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig
run git clone --branch v0.0.14 https://github.com/analogdevicesinc/pyadi-iio.git && \
    cd pyadi-iio && \
    pip3 install -r requirements.txt && \
    python3 setup.py install

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
