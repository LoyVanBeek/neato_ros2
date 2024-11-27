![Test neato_ros2_python](https://github.com/LoyVanBeek/neato_ros2/workflows/Test%20neato_ros2_python/badge.svg)

# neato_ros2
ROS2 interface to Neato vacuum robots

# Docker
```bash
docker build -f docker/Dockerfile . -t neato

docker run -it --net=host --ipc=host --privileged \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY}:/root/.Xauthority" \
neato bash -c "ros2 launch neato_bringup neato.launch.py"
```