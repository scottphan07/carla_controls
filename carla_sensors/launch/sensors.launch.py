"""Starts the sensors node. Start the CARLA ROS bridge and the ego vehicle separately,
or use carla_controls' sim.launch.py, which starts everything."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="carla_sensors",
            executable="sensors_node",
            name="sensors",
            parameters=[{"use_sim_time": True}],  # use CARLA's clock, published by the bridge on /clock
            output="screen",
        ),
    ])
