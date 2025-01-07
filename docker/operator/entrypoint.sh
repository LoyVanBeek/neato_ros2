#! /bin/bash
# Entrypoint for ROS Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash

tmux new-session -d -s zenohd 'ros2 run rmw_zenoh_cpp rmw_zenohd'
tmux new-session -d -s joystick 'ros2 launch virtual_joystick vjoy.launch.py'

# Execute the command passed into this entrypoint
exec rviz2 -d /root/neato.rviz