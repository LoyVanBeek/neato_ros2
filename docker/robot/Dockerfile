ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

RUN apt update -qq && apt install -y --no-install-recommends \
      apt-transport-https \
      apt-utils \
      bash-completion \
      ca-certificates \
      dpkg \
      gdb \
      git \
      git-lfs \
      software-properties-common \
      tree \
      vim \
      wget \
      iproute2 \
      net-tools \
      iputils-ping \
      netcat-traditional \
    && rm -rf /var/lib/apt/lists/*

# install_ros_tools
RUN apt update -qq && apt install -y --no-install-recommends \
      curl \
      gnupg2 \
      lsb-release \
      python3-pip \
      ros-dev-tools \
      python3-argcomplete \
      tmux \
    && rm -rf /var/lib/apt/lists/*

# install ROS additional packages needed for application
RUN apt update -qq && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
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

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

COPY ./neato_description /root/ros2_ws/src/neato_description
COPY ./neato_ros2_python /root/ros2_ws/src/neato_ros2_python
COPY ./neato_bringup /root/ros2_ws/src/neato_bringup

# Configure Zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --event-handlers console_cohesion+\
 && rm -rf /var/lib/apt/lists/*"

COPY ./docker/robot/zenoh.json5 /
ENV ZENOH_ROUTER_CONFIG_URI=/zenoh.json5

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Set up entrypoint
COPY ./docker/robot/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
