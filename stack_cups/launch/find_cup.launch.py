"""Run find_cup with the sim or real-robot parameters (roadmap 0.2).

    ros2 launch stack_cups find_cup.launch.py config:=sim
    ros2 launch stack_cups find_cup.launch.py config:=robot
    ros2 launch stack_cups find_cup.launch.py config:=sim grasp:=true steps:=ready,align

config is required on purpose: sim values (fast speeds, sim workarounds) must never be
used on the robot by accident. It is sim, robot, or a path to a .yaml file.
Under ros2 launch the node has no keyboard, so with step_confirm each step waits for:
    ros2 topic pub --once /stack_cups/continue std_msgs/msg/Empty
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OVERRIDES = {  # launch argument: type of the node parameter it sets ("" = keep the file's)
    "grasp": bool,
    "steps": str,
    "show_windows": bool,
    "step_confirm": bool,
}


def config_path(config):
    if config.endswith(".yaml"):
        return config
    return os.path.join(get_package_share_directory("stack_cups"), "config", f"{config}.yaml")


def _node(context):
    path = config_path(LaunchConfiguration("config").perform(context))
    if not os.path.exists(path):
        raise FileNotFoundError(f"no parameter file {path}: use config:=sim, config:=robot "
                                "or a path to a .yaml file")
    overrides = {}
    for name, kind in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            overrides[name] = value.lower() in ("true", "1", "yes") if kind is bool else value
    return [Node(package="stack_cups", executable="find_cup", name="find_cup",
                 output="screen", emulate_tty=True, parameters=[path, overrides])]


def generate_launch_description():
    args = [DeclareLaunchArgument("config", description="sim, robot, or a .yaml path")]
    args += [DeclareLaunchArgument(name, default_value="",
                                   description="override the parameter file's value")
             for name in OVERRIDES]
    return LaunchDescription(args + [OpaqueFunction(function=_node)])
