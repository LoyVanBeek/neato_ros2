ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base-jammy

RUN apt update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop=0.10.0-1* \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        iproute2 \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

# Prepare CycloneDDS config file
COPY ./docker/operator/cyclonedds.xml /cyclonedds.xml
ENV CYCLONEDDS_URI=file:///cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set up entrypoint
COPY ./docker/operator/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]