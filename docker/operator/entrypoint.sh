#! /bin/bash
# Entrypoint for ROS Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Execute the command passed into this entrypoint
# exec rviz2
/bin/bash