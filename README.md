# neato_ros2
ROS2 interface to Neato vacuum robots

# Docker
```bash
# All executed from top-level directory of repo, same as this README
docker build -f docker/operator/Dockerfile -t operator .
docker build -f docker/robot/Dockerfile -t neato . --cpuset-cpus="0"
docker compose -f docker/docker-compose-robot.yml up
docker compose -f docker/docker-compose-operator.yml up
```

CI (GitHub Actions, `.github/workflows/docker-robot.yml`) builds the robot image
for `linux/arm64` (Raspberry Pi with 64-bit Ubuntu) on every push and pushes it to
`ghcr.io/loyvanbeek/neato_ros2`. On the robot, pull it instead of building:

```bash
docker pull ghcr.io/loyvanbeek/neato_ros2:latest
docker tag ghcr.io/loyvanbeek/neato_ros2:latest neato
docker compose -f docker/docker-compose-robot.yml up
```

Specific versions: branch names (`:main`, `:feature-build_docker_in_ci`), git
short SHAs (`:sha-<7 digits>`) and `v*` tags.

One-time setup: the first push creates the package as private. To pull it
without logging in on the robot, make it public once under your GitHub profile
→ Packages → `neato_ros2` → Package settings, or `docker login ghcr.io` on the
robot instead.