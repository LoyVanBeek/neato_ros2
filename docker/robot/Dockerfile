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
    && rm -rf /var/lib/apt/lists/*

# install_ros_tools
RUN apt update -qq && apt install -y --no-install-recommends \
      curl \
      gnupg2 \
      lsb-release \
      python3-pip \
      python3-rosinstall \
      python3-rosinstall-generator \
      ros-dev-tools \
      python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# install ROS additional packages needed for application
RUN apt update -qq && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

COPY ./neato_description /root/ros2_ws/src/neato_description
COPY ./neato_ros2_python /root/ros2_ws/src/neato_ros2_python
COPY ./neato_bringup /root/ros2_ws/src/neato_bringup

# Prepare CycloneDDS config file
COPY ./docker/robot/cyclonedds.xml /cyclonedds.xml
ENV CYCLONEDDS_URI=file:///cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --event-handlers console_cohesion+\
 && rm -rf /var/lib/apt/lists/*"

# Set up entrypoint
COPY ./docker/robot/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
