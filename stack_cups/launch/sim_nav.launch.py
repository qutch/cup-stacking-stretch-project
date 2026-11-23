"""Nav2 on the task scene's map in the MuJoCo sim (roadmap 0.8), using the course starter's
Nav2 setup (parcs_stretch_nav2).

    ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false     # terminal 1
    ros2 launch stack_cups sim_nav.launch.py                           # terminal 2
    python3 sim/nav_check.py                                           # terminal 3

Why not the starter's `navigation.launch.py use_sim_time:=true`:
  - The sim publishes /clock, but stamps its scans, odometry and TF with wall time (on
    purpose, upstream), so Nav2 has to run on wall time. With use_sim_time:=true, AMCL and
    the controller look up transforms at sim time and find none.
  - With use_sim_time:=false that launch file starts the real robot's driver and lidar.
  - Its remap of Nav2's /cmd_vel to the driver's /stretch/cmd_vel is commented out, so the
    base never moves.
This file starts only the starter's Nav2 bringup, on wall time, with that remap.

The sim's base also follows cmd_vel slowly: its wheel actuators have gear 3, which the sim
driver's wheel-speed math leaves out, and heavy joint friction, so the base reaches only about
a quarter of the commanded speed (1 rad/s turns at 0.27 rad/s; top speed ~0.09 m/s). A 180
degree turn in place then takes ~12 s, longer than Nav2's 10 s progress check, and slow
commands barely move it. SIM_NAV2 adjusts the starter's parameters for that.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml

SIM_NAV2 = {  # parameter name (anywhere in the params file): sim value
    "movement_time_allowance": "30.0",  # progress check: 0.5 m within this many seconds
    "min_speed_xy": "0.05",  # DWB: don't command crawl speeds the sim barely follows
    "min_speed_theta": "0.4",
}


def generate_launch_description():
    starter = get_package_share_directory("parcs_stretch_nav2")
    ours = get_package_share_directory("stack_cups")
    args = [
        DeclareLaunchArgument(
            "map", default_value=os.path.join(ours, "maps", "sim_task_room.yaml"),
            description="map yaml (default: the task scene, made with sim/map_sim_scene.py)"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(starter, "config", "nav2_params.yaml"),
            description="Nav2 parameters (the starter's other file, nav2_voxel_params.yaml, "
                        "needs the spatio-temporal voxel layer plugin)"),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
    ]
    params = RewrittenYaml(source_file=LaunchConfiguration("params_file"),
                           param_rewrites=SIM_NAV2, convert_types=True)
    nav2 = GroupAction([
        SetRemap(src="/cmd_vel", dst="/stretch/cmd_vel"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(starter, "launch", "bringup_launch.py")),
            launch_arguments={"use_sim_time": "false", "autostart": "true", "slam": "False",
                              "map": LaunchConfiguration("map"),
                              "params_file": params}.items()),
    ])
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_bringup"), "launch", "rviz_launch.py")),
        condition=IfCondition(LaunchConfiguration("use_rviz")))
    return LaunchDescription(args + [nav2, rviz])
