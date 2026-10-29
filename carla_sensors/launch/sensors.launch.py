"""Starts the sensors node with config/sensors_params.yaml. Start the CARLA ROS bridge and the
ego vehicle separately, or use carla_controls' sim.launch.py, which starts everything.

    ros2 launch carla_sensors sensors.launch.py
    ros2 launch carla_sensors sensors.launch.py params_file:=/path/to/my_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("carla_sensors")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(share, "config", "sensors_params.yaml")),
        Node(
            package="carla_sensors",
            executable="sensors_node",
            name="sensors",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": True},  # CARLA's clock, published by the bridge on /clock
            ],
            output="screen",
        ),
    ])
