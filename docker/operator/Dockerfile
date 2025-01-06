ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

RUN apt update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        iproute2 \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install rmw_zenoh
RUN mv /etc/apt/sources.list.d/ros2-latest.list /etc/ros2.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2-testing/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list
# temporarily as per https://docs.ros.org/en/jazzy/Installation/Testing.html#deb-testing-repository and
# https://discourse.ros.org/t/rmw-zenoh-binaries-for-rolling-jazzy-and-humble/41395
RUN apt update -qq && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
RUN mv /etc/ros2.list /etc/apt/sources.list.d/ros2-latest.list

# Configure Zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --event-handlers console_cohesion+\
 && rm -rf /var/lib/apt/lists/*"

# Set up entrypoint
COPY ./docker/operator/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]