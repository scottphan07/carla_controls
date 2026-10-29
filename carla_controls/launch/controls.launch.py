"""Starts the controls node. Start the CARLA ROS bridge separately (see README)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="carla_controls",
            executable="controls_node",
            name="controls",
            parameters=[{"use_sim_time": True}],  # use CARLA's clock, published by the bridge on /clock
            output="screen",
        ),
    ])
