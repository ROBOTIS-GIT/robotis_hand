FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN apt-get update && apt-get install -y \
    vim \
    nano \
    python3-pip \
    ros-jazzy-rmw-zenoh-cpp

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_hand.git

RUN cd ${COLCON_WS} && \
    apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip config set global.break-system-packages true

RUN pip3 install --no-cache-dir zenoh-ros2-sdk

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc && \
    echo "export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'" >> ~/.bashrc && \
    echo "alias zenohd='ros2 run rmw_zenoh_cpp rmw_zenohd'" >> ~/.bashrc
