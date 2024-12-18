ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

RUN apt update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        iproute2 \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
# Install rmw_zenoh
# Latest version as of writing
RUN git clone https://github.com/ros2/rmw_zenoh.git && cd rmw_zenoh && git reset --hard dd82e84 && cd ..

# Configure Zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --event-handlers console_cohesion+\
 && rm -rf /var/lib/apt/lists/*"

# Set up entrypoint
COPY ./docker/operator/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]