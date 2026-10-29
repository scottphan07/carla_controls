#!/bin/bash
# Sources ROS, builds the repo (mounted at /repo) into /ws when the container starts, then runs
# the given command. `docker exec` commands (docker/ros.sh) skip the build and reuse it.
set -e
source /opt/ros/humble/setup.bash
source /opt/carla_msgs_ws/install/setup.bash

if [ "$$" = "1" ]; then   # main container process, not a docker exec
    echo "building controls_msgs, carla_sensors, carla_controls, fake_carla ..."
    cd /ws
    if ! colcon build --base-paths /repo \
            --packages-select controls_msgs carla_sensors carla_controls fake_carla \
            > /ws/build.log 2>&1; then
        cat /ws/build.log
        exit 1
    fi
    echo "build OK"
fi

source /ws/install/setup.bash
exec "$@"
