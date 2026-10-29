#!/usr/bin/env bash
# Run the fake CARLA bridge + sensors node (+ controls) in Docker. Needs Docker Desktop.
#
#   ./docker/fake_sim.sh                         fake bridge + sensors_node
#   ./docker/fake_sim.sh controls:=true          + controls_node driving the fake car
#   ./docker/fake_sim.sh cruise_speed:=0.0       any fake_sim.launch.py argument works
#
# Then, in another terminal, run commands inside it with ./docker/ros.sh. Ctrl-C to stop.
set -euo pipefail
cd "$(dirname "$0")/.."
echo "building the Docker image (slow only the first time) ..."
docker build -q -t carla-fake -f docker/Dockerfile docker > /dev/null
mkdir -p out
docker run --rm -it --name carla-fake --shm-size=512m -v "$PWD":/repo carla-fake \
    ros2 launch fake_carla fake_sim.launch.py "$@"
