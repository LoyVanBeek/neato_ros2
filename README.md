![Test neato_ros2_python](https://github.com/LoyVanBeek/neato_ros2/workflows/Test%20neato_ros2_python/badge.svg)

# neato_ros2
ROS2 interface to Neato vacuum robots

# Docker
```bash
# All executed from top-level directory of repo, same as this README
docker build -f docker/operator/Dockerfile -t operator .
docker build -f docker/robot/Dockerfile -t neato .
docker compose -f docker/docker-compose-robot.yml up
docker compose -f docker/docker-compose-operator.yml up
```