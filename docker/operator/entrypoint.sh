#! /bin/bash
# Entrypoint for ROS Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name} ({file_name}:{function_name}:{line_number})]: {message}"

tmux new-session -d -s zenohd 'ros2 run rmw_zenoh_cpp rmw_zenohd'
tmux new-session -d -s rviz 'rviz2 -d /root/neato.rviz'

# Execute the command passed into this entrypoint
exec  "$@"