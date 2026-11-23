"""The MuJoCo sim with the full-task scene (roadmap 0.4): a room, a table with 6 tapered
red cups standing upright, and blue tape marking the stacking area.

    ros2 launch stack_cups sim.launch.py                       # 6 loose cups
    ros2 launch stack_cups sim.launch.py cups:=pyramid         # a finished 3-2-1 pyramid
    ros2 launch stack_cups sim.launch.py cups:=single          # one loose cup (grasp tests)
    ros2 launch stack_cups sim.launch.py cups:=touching        # two loose cups rim to rim
    ros2 launch stack_cups sim.launch.py cups:=mixed           # 3 loose, 3 placed in the tape
    ros2 launch stack_cups sim.launch.py seed:=4               # random loose-cup layout
    ros2 launch stack_cups sim.launch.py scene:=default        # stretch_mujoco's own scene
    ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false   # faster, no window

The head camera (color + depth) is on. Cup sizes come from config/sim.yaml (or config:=).
The scene is written to ~/.ros/stack_cups_scene.xml and loaded by stretch_simulation's
launch file, which gets the remaining arguments.
"""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PASS_THROUGH = {  # stretch_simulation launch arguments and our defaults
    "mode": "navigation",
    "use_cameras": "true",
    "cameras": "cam_d435i_rgb,cam_d435i_depth",
    "use_mujoco_viewer": "true",
    "use_rviz": "false",
    "broadcast_odom_tf": "True",
    # No static map->odom: like the real robot, map comes from AMCL or SLAM (roadmap 0.8)
    "broadcast_map_tf": "False",
}

# stretch_mujoco_driver.launch.py reads sys.argv (not its launch arguments) to decide
# whether to ask, on the terminal, for a RoboCasa kitchen layout. This scene never uses
# RoboCasa, so make sure it sees the flag even when included from here.
if "use_robocasa:=false" not in sys.argv:
    sys.argv.append("use_robocasa:=false")


def cup_params(config):
    path = config if config.endswith(".yaml") else os.path.join(
        get_package_share_directory("stack_cups"), "config", f"{config}.yaml")
    with open(path) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    return {k: float(v) for k, v in params.items() if k.startswith("cup_")}


def _sim(context):
    scene = LaunchConfiguration("scene").perform(context)
    args = {k: LaunchConfiguration(k).perform(context) for k in PASS_THROUGH}
    args["use_robocasa"] = "false"
    if scene == "task":
        from stack_cups import sim_scene

        seed = LaunchConfiguration("seed").perform(context)
        path = os.path.expanduser("~/.ros/stack_cups_scene.xml")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        sim_scene.write_scene(path, LaunchConfiguration("cups").perform(context),
                              int(seed) if seed else None,
                              cup_params(LaunchConfiguration("config").perform(context)))
        args["scene_xml"] = path
    elif scene != "default":
        args["scene_xml"] = scene  # a path to your own MJCF scene
    sim_launch = os.path.join(get_package_share_directory("stretch_simulation"), "launch",
                              "stretch_mujoco_driver.launch.py")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch),
                                     launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("scene", default_value="task",
                               description="task, default, or a path to an MJCF scene"),
         DeclareLaunchArgument("cups", default_value="upright",
                               choices=["upright", "pyramid", "single", "touching", "mixed"]),
         DeclareLaunchArgument("seed", default_value="",
                               description="random loose-cup layout; empty = fixed layout"),
         DeclareLaunchArgument("config", default_value="sim",
                               description="where the cup sizes come from")]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in PASS_THROUGH.items()]
        + [OpaqueFunction(function=_sim)])
