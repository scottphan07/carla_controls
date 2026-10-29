# ROS 2 Humble with everything needed to run this repo against the fake CARLA bridge.
# No CARLA and no GPU needed. Built and started by docker/fake_sim.sh.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-numpy python3-pytest \
        ros-humble-sensor-msgs-py ros-humble-tf2-ros ros-humble-diagnostic-msgs \
    && rm -rf /var/lib/apt/lists/*

# carla_msgs normally comes with the CARLA ROS bridge; build just the messages so controls_node
# (which publishes CarlaEgoVehicleControl) can run and drive the fake car.
RUN mkdir -p /opt/carla_msgs_ws/src \
    && git clone https://github.com/carla-simulator/ros-carla-msgs /opt/carla_msgs_ws/src/carla_msgs \
    && git -C /opt/carla_msgs_ws/src/carla_msgs checkout 081fdcdc8a4f8d4ce936b792c4ef13aba3fd7478 \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/carla_msgs_ws && colcon build && rm -rf build log

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fake_carla", "fake_sim.launch.py"]
