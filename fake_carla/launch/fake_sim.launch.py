"""Fake CARLA bridge + sensors node (+ optionally controls), with no simulator.

    ros2 launch fake_carla fake_sim.launch.py                       # fake bridge + sensors_node
    ros2 launch fake_carla fake_sim.launch.py cruise_speed:=0.0     # car stands still
    ros2 launch fake_carla fake_sim.launch.py controls:=true        # + controls_node driving the car
    ros2 launch fake_carla fake_sim.launch.py controls:=true wait_for_control:=true   # lockstep, like sim.launch.py

Then, in another terminal:  ros2 run fake_carla check_sensors --save /tmp/front_camera.png

controls:=true needs carla_msgs, controls_msgs and carla_controls built in the workspace (the
Docker setup in docker/ does this). With controls on, controls_node brakes until something
publishes /planning/target, and its speed/steering stubs output zero, so the car will sit still
until those are written; that's expected.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def _setup(context):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")

    actions = [Node(
        package="fake_carla",
        executable="fake_bridge",
        name="fake_carla_bridge",
        parameters=[{
            "cruise_speed": float(LaunchConfiguration("cruise_speed").perform(context)),
            "wait_for_control": flag("wait_for_control"),
            "render_scale": int(LaunchConfiguration("render_scale").perform(context)),
        }],  # no use_sim_time here: this node *is* the clock
        output="screen",
    )]
    if flag("sensors"):
        actions.append(_include("carla_sensors", "sensors.launch.py"))
    if flag("controls"):
        actions.append(_include("carla_controls", "controls.launch.py"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cruise_speed", default_value="5.0", description="[m/s] speed with no controls"),
        DeclareLaunchArgument("wait_for_control", default_value="false"),
        DeclareLaunchArgument("render_scale", default_value="2", description="1 = sharp camera, slower"),
        DeclareLaunchArgument("sensors", default_value="true", description="start carla_sensors' sensors_node"),
        DeclareLaunchArgument("controls", default_value="false", description="start carla_controls' controls_node"),
        OpaqueFunction(function=_setup),
    ])
