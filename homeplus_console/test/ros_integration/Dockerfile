# Integration test image: real ROS 2 Humble + the console in ROS mode + a fake robot speaking today's topics.
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-moveit-msgs python3-pip python3-serial \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir fastapi "uvicorn[standard]" "pydantic>=2" pyyaml pillow numpy websockets
COPY homeplus_interfaces /ws/src/homeplus_interfaces
RUN bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --packages-select homeplus_interfaces >/dev/null"
