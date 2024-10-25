FROM ros:noetic-perception-focal as base_dev_image

# Install ROS packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    libeigen3-dev \
    libpcap-dev \
    libignition-math4-dev \
    libgoogle-glog-dev \
    ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-ddynamic-reconfigure \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-effort-controllers   \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-rqt-gui-py \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-hector-gazebo-plugins \
    ros-${ROS_DISTRO}-hector-mapping \
    ros-${ROS_DISTRO}-hector-trajectory-server \
    ros-${ROS_DISTRO}-hector-geotiff && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python dependencies
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir laspy numpy open3d argparse matplotlib tqdm scipy

# Boostrap workspace
ENV CATKIN_DIR=/catkin_ws
RUN mkdir -p $CATKIN_DIR/src/
RUN /bin/bash -c "echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc" \
    && /bin/bash -c "echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc"   \
    && /bin/bash -c "echo 'export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/catkin_ws/src/cessna_simulation/worlds' >> /root/.bashrc" \
    && /bin/bash -c "echo 'export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/catkin_ws/src/cessna_simulation/models' >> /root/.bashrc"

WORKDIR $CATKIN_DIR
COPY ./src/ $CATKIN_DIR/src/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make"
