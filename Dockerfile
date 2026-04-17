FROM osrf/ros:humble-desktop

# Install project dependencies + VNC + X11 requirements
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-realsense2-camera \
    ros-humble-cv-bridge \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-numpy \
    python3-scipy \
    python3-pip \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    xterm \
    xkb-data \
    x11-xkb-utils \
    xfonts-base \
    dbus-x11 \
    libxrandr2 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d

# Set up workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/homeplus_robot_workspace

# Build workspace
RUN bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys='tf_transformations' && \
    colcon build --symlink-install"

# Set up runtime dir
RUN mkdir -p /tmp/runtime-root

# Source everything on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export DISPLAY=:1" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
