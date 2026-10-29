#!/usr/bin/env bash
# Run a ROS command inside the running fake sim (start it first with ./docker/fake_sim.sh).
#
#   ./docker/ros.sh ros2 run fake_carla check_sensors --save /repo/out/front_camera.png
#   ./docker/ros.sh ros2 topic list
#   ./docker/ros.sh ros2 topic hz /sensors/lidar
#   ./docker/ros.sh bash                          # a shell with ROS and the workspace sourced
#
# /repo is this folder, so files saved under /repo/out/ show up in ./out/ here.
set -euo pipefail
exec docker exec -it carla-fake /entrypoint.sh "$@"
