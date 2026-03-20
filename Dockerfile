FROM docker.io/ros:humble

# 1. Define ARG for build-time expansion, ENV for runtime
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive


# 2. Optimized and Corrected APT Installs
RUN apt-get update && apt-get install -y \
    # Build Tools
    cmake pkg-config build-essential git wget tmux ripgrep neovim fd-find bison flex libxml2-dev \
    # Python & Utils
    python3-pip python3-setuptools python3-serial python3-pyproj \
    # Hardware Libs (SDR & GPIO)
    libusb-1.0-0-dev libavahi-client-dev libavahi-common-dev libaio-dev \
    libgpiod-dev libeigen3-dev libomp-dev libserialport-dev \
    # ROS 2 Core Dependencies (Using absolute name 'humble' for stability)
    ros-humble-mavros-extras \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-camera-ros \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    # Solver & GUI Dependencies (Fixed for Ubuntu 22.04)
    python3-matplotlib python3-numpy python3-scipy python3-opencv \
    libxkbcommon-x11-0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libgpiod-dev \
    && rm -rf /var/lib/apt/lists/*

    
# 3. MAVROS Datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# 3. Pip Installs
RUN pip install mediapipe==0.10.9

# 4. SDR Libraries (Build with limited cores -j2 to prevent QEMU crash)
WORKDIR /opt/sdr
RUN git clone --branch v0.23 https://github.com/analogdevicesinc/libiio.git && \
    cd libiio && mkdir build && cd build && \
    cmake -DPYTHON_BINDINGS=ON .. && make -j2 && make install && ldconfig

RUN git clone https://github.com/analogdevicesinc/libad9361-iio.git && \
    cd libad9361-iio && mkdir build && cd build && \
    cmake .. && make -j2 && make install && ldconfig

RUN git clone --branch v0.0.14 https://github.com/analogdevicesinc/pyadi-iio.git && \
    cd pyadi-iio && pip3 install -r requirements.txt && python3 setup.py install

# 5. Workspace Build
WORKDIR /workspace
# Using COPY instead of bind-mount for better stability in ARM emulation
COPY ./workspace/src /workspace/src

# GENTLE COLCON BUILD:
# --executor sequential: Prevents OOM/Abort crashes by building 1 package at a time
# --parallel-workers 2: Limits CPU threads
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --symlink-install \
    --executor sequential \
    --parallel-workers 2 \
    --event-handlers console_direct+

# 6. Final Config
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER_CONFIG_URI=/root/router_config.json5

COPY zenoh/ /root/
COPY Tools /Tools

RUN chmod +x /Tools/update_router_config.py && \
    chmod +x /Tools/entrypoint.sh

# Automatically source ROS and Workspace in every bash session
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/Tools/entrypoint.sh" ]
CMD [ "tmux" ]