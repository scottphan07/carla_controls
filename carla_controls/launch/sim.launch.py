"""Whole pipeline in one command: CARLA ROS bridge + ego vehicle with our sensors + sensors node + controls.

CARLA itself runs as the `carla` systemd service on the shared sim box.

    ros2 launch carla_controls sim.launch.py
    ros2 launch carla_controls sim.launch.py town:=Town03
    ros2 launch carla_controls sim.launch.py controls:=false wait_for_control:=False   # sensors only

Arguments
  town                     map to load (default Town01)
  objects_definition_file  sensor rig (default: carla_sensors' config/objects.json)
  sensors                  start sensors_node, which republishes on /sensors/* (default true)
  controls                 start controls_node (default true)
  wait_for_control         bridge waits for a vehicle_control_cmd before each tick (default True).
                           Set False when running without controls, or the sim never advances.

Fixed settings, which controls_node depends on: synchronous mode and 0.05 s per tick (DT in
controls_node.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

FIXED_DELTA_SECONDS = "0.05"  # must match DT in controls_node.py


def _launch_file(package, name):
    """carla-ros-bridge installs launch files in share/<pkg>/ or share/<pkg>/launch/."""
    share = get_package_share_directory(package)
    for path in (os.path.join(share, name), os.path.join(share, "launch", name)):
        if os.path.exists(path):
            return path
    raise FileNotFoundError(f"{name} not found in package {package}")


def _include(package, name, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file(package, name)),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    share = get_package_share_directory("carla_controls")
    sensors_share = get_package_share_directory("carla_sensors")

    bridge = _include(
        "carla_ros_bridge", "carla_ros_bridge.launch.py",
        host="localhost",
        port="2000",
        timeout="20",  # loading a town takes a while
        town=LaunchConfiguration("town"),
        passive="False",
        synchronous_mode="True",
        synchronous_mode_wait_for_vehicle_control_command=LaunchConfiguration("wait_for_control"),
        fixed_delta_seconds=FIXED_DELTA_SECONDS,
    )
    ego = _include(
        "carla_spawn_objects", "carla_example_ego_vehicle.launch.py",
        objects_definition_file=LaunchConfiguration("objects_definition_file"),
        role_name="ego_vehicle",
    )
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sensors_share, "launch", "sensors.launch.py")),
        condition=IfCondition(LaunchConfiguration("sensors")),
    )
    controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "controls.launch.py")),
        condition=IfCondition(LaunchConfiguration("controls")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("town", default_value="Town01"),
        DeclareLaunchArgument(
            "objects_definition_file",
            default_value=os.path.join(sensors_share, "config", "objects.json")),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("controls", default_value="true"),
        DeclareLaunchArgument("wait_for_control", default_value="True"),
        bridge,
        ego,
        sensors,
        controls,
    ])
