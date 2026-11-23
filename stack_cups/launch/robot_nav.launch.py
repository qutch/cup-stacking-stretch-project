"""Nav2 on the real robot (roadmap 2.3), using the course starter's Nav2 setup
(parcs_stretch_nav2), for carry_to_stack and find_cup's navigate step.

Start the driver and the lidar first (README: First runs on the real robot):
    ros2 launch stretch_core stretch_driver.launch.py mode:=navigation broadcast_odom_tf:=True
    ros2 launch stretch_core rplidar.launch.py
    ros2 launch stack_cups robot_nav.launch.py use_rviz:=true      # the starter's WVH map

Why not the starter's navigation.launch.py: its remap of Nav2's /cmd_vel to the driver's
/stretch/cmd_vel is commented out, so Nav2 never moves the base (the driver listens only on
/stretch/cmd_vel), and it starts the driver and lidar itself. This file starts only the
starter's Nav2 bringup, on wall time, with that remap. Same as sim_nav.launch.py, without
the sim's changes.

params_file: the starter's nav2_params.yaml (lidar only, 0.22 m robot_radius as in our
robot.yaml), the one tested in the sim. The starter's own default, nav2_voxel_params.yaml,
adds the WVH keepout zones and the head camera's depth points (it needs the
spatio-temporal voxel layer plugin and has a 0.27 m robot_radius).

ROBOT_NAV2 slows Nav2 down for the first runs with a cup held out on the arm: the starter
allows 0.26 m/s, 1 rad/s and 2.5 m/s^2. Raise the values once carrying works.
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

ROBOT_NAV2 = {  # parameter name (anywhere in the params file): first-run value
    "max_vel_x": "0.15",  # DWB
    "max_speed_xy": "0.15",
    "max_vel_theta": "0.5",
    "acc_lim_x": "1.0",
    "decel_lim_x": "-1.0",
    "acc_lim_theta": "1.5",
    "decel_lim_theta": "-1.5",
    "max_velocity": "[0.15, 0.0, 0.5]",  # velocity smoother, if the bringup runs one
    "min_velocity": "[-0.15, 0.0, -0.5]",
    "max_accel": "[1.0, 0.0, 1.5]",
    "max_decel": "[-1.0, 0.0, -1.5]",
    "max_rotational_vel": "0.5",  # spin recovery
    "rotational_acc_lim": "1.5",
    "movement_time_allowance": "20.0",  # progress check: 0.5 m within this many seconds
}


def generate_launch_description():
    starter = get_package_share_directory("parcs_stretch_nav2")
    args = [
        DeclareLaunchArgument(
            "map", default_value=os.path.join(starter, "map", "WVH_edit.yaml"),
            description="map yaml (default: the starter's WVH map)"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(starter, "config", "nav2_params.yaml"),
            description="Nav2 parameters (nav2_voxel_params.yaml for the WVH keepout zones; "
                        "needs the spatio-temporal voxel layer plugin)"),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
    ]
    params = RewrittenYaml(source_file=LaunchConfiguration("params_file"),
                           param_rewrites=ROBOT_NAV2, convert_types=True)
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
