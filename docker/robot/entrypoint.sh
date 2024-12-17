#! /bin/bash
# Entrypoint for ROS Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the overlay workspace, if built
source /root/ros2_ws/install/setup.bash

tmux new-session -d -s zenohd 'ros2 run rmw_zenoh_cpp rmw_zenohd'

# Execute the command passed into this entrypoint
exec "$@"