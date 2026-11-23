# Cup stacking with Stretch

ROS 2 Humble code for Hello Robot's Stretch, run in the Stretch MuJoCo simulator and on the
real robot. The `stack_cups` package finds a red cup with the head camera, lines the robot
up with it, and reaches the gripper to it, using depth to avoid hitting the table. It also
has the sim scene for the full task (a table with 6 cups and a taped stacking area),
per-platform parameter files, and real-robot safety (stop at any time, slow speeds, one
step at a time).

See the [Roadmap](#roadmap) for what is done and what is left.

## Workspace layout

Clone this repo next to Hello Robot's `stretch_ros2` and the course starter package
`parcs_stretch_nav2_WVH` (Nav2 setup and maps) in the same `src/` folder:

```
workspace/
    - src/
        - stretch_ros2/
        - parcs_stretch_nav2_WVH/
        - cup-stacking-stretch-project/
    - build/  install/  log/      (created by colcon)
```

```bash
mkdir -p ~/ws/src && cd ~/ws/src        # any workspace folder name works
git clone https://github.com/hello-robot/stretch_ros2.git -b humble
git clone https://github.com/NU-PARCS/parcs_stretch_nav2_WVH.git
git clone https://github.com/qutch/cup-stacking-stretch-project.git
```

## One-time setup

Needs Ubuntu 22.04 with ROS 2 Humble (on a Mac, an Ubuntu VM in UTM works; on Windows,
WSL 2 with Ubuntu 22.04 works). Follow the team's ROS2 setup guide (PDF), steps 8-10. The
parts that are easy to get wrong:

1. **OpenCV on ARM (Apple Silicon VMs):** pip's `opencv-python` crashes with
   `Illegal instruction`. Use Ubuntu's build:
   `sudo pip3 uninstall -y opencv-python && sudo apt install -y python3-opencv`.
   Re-check after installing `stretch_mujoco`, which pulls pip's version back in:
   `python3 -c "import cv2; print(cv2.__file__)"` must print a `/usr/lib/python3/...` path.
2. **stretch_mujoco is not on PyPI.** Install it from source with submodules:
   ```bash
   mkdir -p ~/repos && cd ~/repos
   git clone https://github.com/hello-robot/stretch_mujoco --recurse-submodules
   cd stretch_mujoco
   sudo $(which uv) pip install --system -e .
   sudo pip3 install -e third_party/robosuite -e third_party/robocasa
   ```
3. **Robot model:** `stretch_urdf_ros_update.py` finds `stretch_description` through the
   *built* workspace, so build that package once first:
   ```bash
   cd ~/ws && source /opt/ros/humble/setup.bash
   colcon build --packages-select stretch_description && source install/setup.bash
   sudo pip3 install hello-robot-stretch-urdf
   stretch_urdf_ros_update.py --model SE3 --tool eoa_wrist_dw3_tool_sg3   # answer y
   ```
4. **ROS dependencies,** including two the sim launch file uses but does not declare. This
   also installs Nav2 and slam_toolbox for `parcs_stretch_nav2`:
   ```bash
   cd ~/ws
   rosdep install --from-paths src --ignore-src -y -r --skip-keys stretch_body
   sudo apt install -y ros-humble-joint-state-publisher ros-humble-control-msgs
   sudo pip3 install ros2_numpy pyquaternion
   sudo pip3 install --force-reinstall --no-cache-dir \
       "numpy==1.23.3" "numba==0.56.4" "llvmlite==0.39.1"     # always last
   ```
5. **Simulator patches:** fixes to `stretch_mujoco`, `stretch_ros2` and
   `parcs_stretch_nav2_WVH` that make the sim run in real time, make arm moves accurate, and
   let Nav2 and SLAM run on the sim (see [Simulator patches](#simulator-patches)):
   ```bash
   ~/ws/src/cup-stacking-stretch-project/sim/patches/apply_patches.sh
   ```
6. **Build:**
   ```bash
   cd ~/ws && source /opt/ros/humble/setup.bash
   colcon build --packages-up-to stretch_simulation stack_cups parcs_stretch_nav2
   colcon build --packages-select stretch_description
   ```
   Do not run a plain `colcon build`; `stretch_deep_perception` fails to build and is not
   needed. After changing anything in `stack_cups`, rebuild it
   (`colcon build --packages-select stack_cups`): `ros2 run` and `ros2 launch` use the
   installed copy.

## Running in the sim

**Terminal 1: simulator with the task scene**

```bash
cd ~/ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch stack_cups sim.launch.py                            # MuJoCo window on
ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false   # no window
```

The [task scene](#the-task-scene): a room with a table on the robot's arm side, 6 red cups
standing on it, and blue tape marking the stacking area. Options: `cups:=pyramid` (a
finished pyramid on the tape), `seed:=4` (random loose-cup layout), `scene:=default`
(`stretch_mujoco`'s own scene: one red cylinder on a table), `config:=robot` (cup sizes
from `robot.yaml`). The head camera's color and depth are on, the robot starts in
navigation mode, and `odom` → `base_link` is published. With the MuJoCo window on, the sim
ran at about 0.8× real time in our tests; without it, at real time.

The plain simulator launch still works (default scene):

```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
    mode:=navigation use_rviz:=false use_robocasa:=false use_cameras:=true \
    cameras:=cam_d435i_rgb,cam_d435i_depth use_mujoco_viewer:=false
```

`use_cameras:=true` is required there; cameras are off by default. `cameras:=` (added by
the patches) renders only the head camera's color and depth, which is all `find_cup` uses.
Each extra camera costs render time. In a VM where depth comes out broken,
`export MUJOCO_GL=osmesa` before launching (see Troubleshooting).

**Terminal 2: find_cup**

```bash
cd ~/ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch stack_cups find_cup.launch.py config:=sim                  # reach to the cup
ros2 launch stack_cups find_cup.launch.py config:=sim grasp:=true      # also close the gripper
ros2 launch stack_cups find_cup.launch.py config:=sim steps:=aim_head,ready   # only these steps
ros2 launch stack_cups find_cup.launch.py config:=sim step_confirm:=true      # confirm each step
```

`config:=` is `sim`, `robot`, or the path of a parameter file. `ros2 run stack_cups find_cup`
also works and uses the sim values. In the task scene `find_cup` does not work yet: it goes
for the biggest red blob the head camera sees, and once the raised arm hides that cup, the
biggest blob is the next cup, so ALIGN chases a different cup after each drive and gives up
(roadmap 1.1). Use `scene:=default` (one cup) until then.

Main parameters. `grasp`, `steps`, `step_confirm` and `show_windows` can be given to
`find_cup.launch.py` as `name:=value`; `ros2 run stack_cups find_cup --ros-args -p name:=value`
sets any parameter.

| Parameter | Sim | Robot | Meaning |
|---|---|---|---|
| `steps` | `all` | `all` | Run only these steps: any of `aim_head,ready,align,reach` |
| `grasp` | `false` | `false` | Close the gripper at the end |
| `step_confirm` | `false` | `true` | Before each step, wait for Enter or `/stack_cups/continue` |
| `show_windows` | `true` | `true` | Show the camera and red-mask debug windows |
| `max_base_speed` | `0.3` | `0.1` | Base speed cap, m/s |
| `max_joint_speed_m`, `max_joint_speed_rad` | off | `0.05`, `0.3` | Lift/arm (m/s) and head/wrist (rad/s) speed caps |
| `cup_height`, `cup_rim_diameter`, ... | 16 oz cup | 16 oz cup | Cup size (placeholder until the real cups are measured) |

Every parameter, with comments, is in `stack_cups/config/sim.yaml` and `robot.yaml`; the
defaults (the sim values) are in `stack_cups/stack_cups/params.py`.

The node logs each phase: `[START]` → `[AIM_HEAD]` → `[READY]` → `[ALIGN]` → `[REACH]` →
`[DONE]`. On any problem it stops the robot and logs `[FAILED] <reason>`; when asked to
stop it logs `[STOPPED] <why>`.

## Stopping and running step by step

Any of these stops the robot at once (roadmap 0.6):

- **Ctrl+C** in the node's terminal,
- `ros2 topic pub --once /stack_cups/stop std_msgs/msg/Empty` from any terminal,
- the robot's **runstop** button (`/is_runstopped`).

A stop sets the base speed to 0, cancels the arm move in progress, calls the driver's
`/stop_the_robot`, and ends the run with `[STOPPED]`. In the sim, each of them (the topic
and Ctrl+C during ALIGN's drive, the runstop during READY) left the base still within
0.6 s, and the node exited cleanly.

To go one step at a time, use `step_confirm:=true` (the default with `config:=robot`): before
each step the node logs `[STEP] next: <step>` and waits for
`ros2 topic pub --once /stack_cups/continue std_msgs/msg/Empty`, or for Enter when it was
started with `ros2 run` (`ros2 launch` gives the node no keyboard). `steps:=` runs only some
steps (for example `steps:=aim_head,ready` to stop with the arm raised above the cup).

## First runs on the real robot

The code has not run on the real robot yet. `config:=robot` is slow and step-by-step on
purpose. Suggested order:

1. Start the driver with odometry TF, the head camera, and (for Nav2) the lidar:
   ```bash
   ros2 launch stretch_core stretch_driver.launch.py mode:=navigation broadcast_odom_tf:=True
   ros2 launch stretch_core stretch_realsense.launch.py
   ros2 launch stretch_core rplidar.launch.py
   ```
2. Check every topic, service, action, joint and TF frame the code uses is there (it only
   listens, nothing moves). Every line should say `PASS`:
   ```bash
   ros2 run stack_cups check_interfaces --nav
   ```
3. Run `find_cup` a step at a time, with the runstop button in reach. Each step waits for
   `ros2 topic pub --once /stack_cups/continue std_msgs/msg/Empty` from a second terminal:
   ```bash
   ros2 launch stack_cups find_cup.launch.py config:=robot steps:=aim_head
   ros2 launch stack_cups find_cup.launch.py config:=robot steps:=aim_head,ready
   ros2 launch stack_cups find_cup.launch.py config:=robot      # all steps, grasp off
   ```
4. Check the values marked "verify on the robot" in `config/robot.yaml` (gripper
   positions, slowest base speed, `arm_len`, `yaw_sign`; `sim/check_robot_layer.py`
   measures the last two, and it drives the base) and update the file.

Differences between the real robot and the sim (roadmap 0.3, from the two drivers' code;
`check_interfaces` passes on the sim):

| | Real robot | Sim | How the code handles it |
|---|---|---|---|
| Depth image | `16UC1`, millimeters | `32FC1`, meters | `perception.depth_in_meters` |
| Color image | `rgb8` | `bgr8` | converted to BGR when received |
| `odom` → `base_link` TF | only with `broadcast_odom_tf:=True` | on in `sim.launch.py` | launch argument (step 1) |
| `map` → `odom` TF | from AMCL or SLAM | fixed by the sim driver unless `broadcast_map_tf:=False` | `sim.launch.py` turns it off (patch) |
| Lidar | `/scan` and `/scan_filtered` | `/scan_filtered`; `/scan` added by the patch | patch |
| Gripper position | finger angle | the sim's own scale | `gripper_open`/`gripper_closed` in `robot.yaml` are marked "verify" |
| Closing on a cup | the goal aborts after ~10 s (the fingers never reach the target) | succeeds once the fingers stop (patch) | not handled yet: keep `grasp:=false` on the robot until roadmap 1.7 |
| `/stop_the_robot` | does not cancel a running arm move | | a stop cancels the move itself |
| Clock | wall time | sim time (`/clock`) | the node switches to sim time when `/clock` exists |

## How it works

| File (`stack_cups/stack_cups/`) | What it is |
|---|---|
| `cup_finder_controller.py` | The `find_cup` node: the phases below |
| `robot.py` | `CupRobot`: the starter code's `HelloNode` plus checked joint moves, base moves steered by odometry, and the stop handling above |
| `head_camera.py` | `HeadCamera`: the head camera's frames, and fresh measurements (only while stopped, median of several frames) |
| `align.py` | `align_arm`: line the arm up with a target by driving the base, then finish with the wrist yaw |
| `perception.py`, `motion.py` | The math, with no ROS (easy to test): red mask, depth to 3D, cup, table top and table edge, reach planning |
| `params.py` | Every parameter and its default; `config/*.yaml` set them per platform |
| `sim_scene.py` | The task scene for the MuJoCo sim |
| `check_interfaces.py` | The interface check for the real robot or the sim |

Phases of `find_cup` (the whole run stays in navigation mode: the base takes `cmd_vel` and
the other joints take `move_to_pose` goals):

1. **START:** wait for camera images, depth, camera info, TF and `/odom`.
2. **AIM_HEAD:** point the head camera at the table on the robot's arm side.
3. **READY:** measure the cup, raise the lift above it, straighten the wrist, open the
   gripper, and read the grasp point's offsets from TF.
4. **ALIGN:** locate the cup in 3D from depth (depth points are projected into the color
   image and only those on red pixels are kept), measuring only while stopped, then drive
   the measured offset by odometry; repeat at most `max_drives` times. With
   `use_wrist_yaw`, drive only until the wrist yaw can cover the rest, then turn the wrist
   and check from TF that the grasp point is lined up with the cup.
5. **REACH:** from depth, measure the table top and the table edge. Refuse to move if the
   gripper would be lowered into the table, onto it while retracted, or the cup is out of
   reach. Otherwise lower to cup height and extend. With the patched sim, each move lands
   within 5 mm; if the server reports a joint as blocked (the gripper against the cup), the
   node does not push past the target.

In MuJoCo tests the cup is located within about 1 cm and the table within about 1 cm, and
ALIGN lines the arm up with the cup within about 1 cm. Picking the cup up is not reliable
yet: in 3 sim runs with `grasp:=true` (default scene) the cup was never lifted. One run
stopped at the wrist check; in the other two the gripper closed without holding the cup
(roadmap 1.6-1.8).

## The task scene

`stack_cups/stack_cups/sim_scene.py` builds it (roadmap 0.4); `sim.launch.py` writes it to
`~/.ros/stack_cups_scene.xml` and starts the sim with it. The robot starts at the origin
facing +x, with its arm side (-y) toward the table:

- **Table:** 1.9 × 0.76 m, 0.74 m high, its near edge 0.55 m from the robot.
- **Cups:** 6 tapered red cups standing upright. Five are within reach from the start side;
  one is only reachable from the far side, to test going around the table. Their sizes come
  from the parameter files. Each cup collides as 20 thin wall boxes and a base disk, and is
  drawn as one mesh (drawing the boxes made camera frames about 5× slower).
- **Stacking area:** blue tape on the table's +x end.
- **Room:** walls and four boxes of furniture, so the lidar map has features to localize on.

Check the cups stay put without ROS: `python3 -m stack_cups.sim_scene --check`
(`--cups pyramid`, `--seed 4`, `--out file.xml` also work).

## Navigation in the sim

Work in progress (roadmap 0.8). The starter package `parcs_stretch_nav2` runs Nav2 with AMCL
on a saved map. With the patches its launch files run on the sim (`use_sim_time:=true` skips
the real driver and lidar), and the sim publishes `/scan` for AMCL. Still to do: make the map
of the task scene and check Nav2 on it. In the first mapping run the robot stopped making
progress on the last leg of its drive around the table, so no map was saved;
`map_sim_scene.py` now logs where the robot is and gives up on a stuck leg.

Run these from `~/ws/src/cup-stacking-stretch-project`, with the sim in terminal 1
(`ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false`).

**1. Map the task scene:**

```bash
ros2 launch parcs_stretch_nav2 offline_mapping.launch.py use_sim_time:=true use_rviz:=false
python3 sim/map_sim_scene.py      # drives once around the table
mkdir -p stack_cups/maps
ros2 run nav2_map_server map_saver_cli -f stack_cups/maps/sim_task_room \
    --ros-args -p use_sim_time:=true
```

Then rebuild `stack_cups` so the map is installed with the package.

**2. Navigate on the map:**

```bash
ros2 launch parcs_stretch_nav2 navigation.launch.py use_sim_time:=true teleop_type:=none \
    map:=$(ros2 pkg prefix --share stack_cups)/maps/sim_task_room.yaml \
    params_file:=$(ros2 pkg prefix --share parcs_stretch_nav2)/config/nav2_params.yaml
python3 sim/nav_check.py      # sets the initial pose, drives to 3 goals, checks each
```

`params_file` matters: the starter's default, `nav2_voxel_params.yaml`, needs the
spatio-temporal voxel layer plugin (`sudo apt install ros-humble-spatio-temporal-voxel-layer`).

## Tests

```bash
cd ~/ws && colcon test --packages-select stack_cups && colcon test-result --verbose
cd ~/ws/src/cup-stacking-stretch-project/stack_cups && python3 -m pytest test   # faster
```

- `test_perception_geometry.py`: `perception.py` on a synthetic color + depth image of a
  cup on a table (ray-cast, so the true answers are known): red mask, depth units,
  projection, cup location, table top and edge, and every case where reach planning must
  refuse.
- `test_motion.py`: the motion math.
- `test_config.py`: `sim.yaml` and `robot.yaml` set every parameter in `params.py` with the
  right types, and the robot file is slower and has no sim-only workarounds.
- `test_sim_scene.py`: cup geometry, layouts, and (with MuJoCo installed) that the scene
  compiles, each cup is drawn as one mesh, and the cups stay put upright and as a pyramid.
- Lint: `flake8` (settings in `test/flake8.ini`) and `pep257` (a few docstring style rules
  relaxed in `test_pep257.py`).

Not covered yet: images from the real robot (record some on its first runs), tape detection
(roadmap 2.1) and pyramid spot detection (4.2).

## Simulator patches

`sim/patches/` holds fixes to the simulator packages and the starter package.
`apply_patches.sh` applies them (setup step 5); `apply_patches.sh --revert` undoes them.
Rebuild `stretch_simulation` and `parcs_stretch_nav2` after either. They were made against
`stretch_mujoco` commit `d107e09` (main), `stretch_ros2` commit `73decc6` (humble) and
`parcs_stretch_nav2_WVH` commit `272180f`.

**`stretch_mujoco.patch`** (the simulator):

| Problem | Fix | Measured |
|---|---|---|
| Every camera frame drew the lidar's 361 site spheres | Don't draw sites in camera images or the viewer (turn them back on in the viewer under Rendering) | head camera frame: 192 ms → 8 ms |
| The 360-ray lidar was recomputed on every 2 ms physics step, but is only read at 15 Hz | Compute sensors at 30 Hz of sim time | physics step ~17× cheaper |
| Physics pacing lost every sleep overshoot, and state was exchanged with the ROS side on every step | Pace against a running schedule; exchange state at 100 Hz | with the rows above: 0.003× → 1.0× real time (headless, head camera) |
| Rendering a camera frame held the physics lock, so frames slower than the camera rate (30 Hz) stalled physics | Hold the lock only while copying the scene; render outside it | task scene: 0.55× → 1.0× real time; slow frames now lower the camera rate instead |
| The ROS side and the sim process both edit the shared command record, without a common lock | Share one lock | lost commands: 1 in 60 → 0 in 100 |
| "Move the base 0 m" (sent for 1 s after `cmd_vel` stops) drove the base backward | Treat it as a stop | no more backward creep |

**`stretch_ros2.patch`** (the ROS driver):

- **Trajectory goals** (`/stretch_controller/follow_joint_trajectory`): every goal used to end
  `ABORTED` (a crash after the move), and the server counted a joint as arrived within 5 cm.
  Now a goal succeeds only once every joint has stopped within 5 mm (or the goal's
  `goal_tolerance`). Joints that sag under gravity (lift ~1 cm, wrist pitch ~5 mrad) are
  re-commanded until they get there.
- **Honest failures:** a joint that something stops (the gripper against a cup, the gripper on
  the table) fails with `<joint> blocked at ...` and keeps pushing only as hard as it did
  before the patch; the arm is never commanded past its target, so it can't shove a cup. A
  target past a joint limit fails too. The gripper succeeds when it stops, since stopping
  short means it is holding something.
- **Base joints:** `translate_mobile_base` / `rotate_mobile_base` in a trajectory now move the
  base by that distance/angle in position mode (they always failed before), and are refused
  with a message in navigation mode.
- **`cameras:=` launch option:** render only the listed cameras (default `all`).
- **`scene_xml:=` launch option:** load your own MuJoCo scene (`sim.launch.py` uses it).
- **`/scan`:** the sim published only `/scan_filtered`; it now publishes the same scan on
  `/scan` too, like the real robot. The starter's AMCL reads `/scan`.
- **`broadcast_map_tf:=` launch option** (default `True`, as before): the sim publishes a
  fixed `map` → `odom`. Turn it off when AMCL or SLAM provides it (`sim.launch.py` does).

Unchanged upstream behavior to know about: the driver runs trajectory goals on its only
thread, so `/stretch/joint_states`, TF and `cmd_vel` handling pause while a goal runs.

**`parcs_stretch_nav2.patch`** (the course starter package):

- `offline_mapping.launch.py` skips the real driver and lidar when `use_sim_time:=true`
  (as `navigation.launch.py` already does), and runs slam_toolbox on sim time.
- `simple_commander_demo.py` imported `parcs_parcs_stretch_nav2.robot_navigator` (doubled
  prefix) and failed as shipped; it now imports `parcs_stretch_nav2.robot_navigator`.

## Helper scripts (`sim/`)

| Script | What it does |
|---|---|
| `patches/apply_patches.sh` | Apply (or `--revert`) the patches |
| `check_camera.py` | Grab one camera frame, print brightness and red-pixel count, save it to `/tmp` |
| `debug_cup_depth.py` | Run the node's depth pipeline on one frame and print every step |
| `check_measure.py` | Measure the cup the way `find_cup` does, without moving, and say why an attempt fails |
| `check_robot_layer.py` | Check `CupRobot` on the running sim and measure the values `find_cup` needs (moves the base, then undoes it) |
| `nudge_base.py` | Drive the base an exact distance (to set up a start pose) |
| `clock_rate.py` | How fast the sim runs compared with real time |
| `map_sim_scene.py` | Drive once around the task scene's table for mapping |
| `nav_check.py` | Navigate the task scene with Nav2 and check where the robot ends up |
| `stretch_table_scene.py` | Standalone MuJoCo scene (no ROS): Stretch + square table |
| `cup_approach.py` | Standalone: random red cup on the table; robot drives around the table to it. `--face arm` is not reliable yet (the gripper can clip the table) |

Run the ROS ones after `source install/setup.bash` with the sim running; run the standalone
ones from the `sim/` folder with `python3 <script>`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `[FAILED] depth image looks broken` or the cup is never found in depth | The GPU renderer is not producing depth (common in VMs). Restart the sim with `export MUJOCO_GL=osmesa`. Check with `sim/debug_cup_depth.py`. |
| Camera window all black, `red pixels: 0` | The head is not looking at the cup. Keep `aim_head:=true` or adjust `head_pan`/`head_tilt`. |
| Node gets no images at all | The sim was launched without `use_cameras:=true` (`sim.launch.py` turns it on). |
| `package 'joint_state_publisher' not found` / `No module named 'control_msgs'` | Setup step 4. |
| `Permission denied` reading files during `colcon build` from a VM shared folder | Files are readable only by their owner on the host. On the Mac: `chmod -R go+rX <workspace>/src`. If the build fails, `ros2 run` silently runs the old installed version. |
| Sim far slower than real time (moves time out, ALIGN needs many nudges, camera FPS low) | Apply the simulator patches (setup step 5), rebuild, and run without the viewer (`use_mujoco_viewer:=false`). `python3 sim/clock_rate.py` shows the speed. |
| Every trajectory goal ends `ABORTED` with an empty error message | The simulator patches are not applied, or `stretch_simulation` was not rebuilt after applying them. |
| Nav2 never becomes active, or the costmaps log a missing plugin | Pass `params_file:=.../nav2_params.yaml` (see [Navigation in the sim](#navigation-in-the-sim)). |
| Viewer black/glitchy | Log in to Ubuntu on Xorg (not Wayland), enable GPU acceleration in the VM, give the VM more CPU cores, and use `use_mujoco_viewer:=false`. |

## Roadmap

Check items off here as they are finished. Items marked **(partial)** have working
code that still needs the improvements listed under them.

**Requirements**
- Build a flipped 3-2-1 pyramid: 6 red cups, rim down.
- Loose cups start upright, outside the stacking area.
- The stacking area is marked with blue tape.
- Must work in the MuJoCo sim and on the real Stretch.
- Real cup size is still unknown, so all cup measurements are parameters.
- Navigation uses the course starter package
  [`parcs_stretch_nav2`](https://github.com/NU-PARCS/parcs_stretch_nav2_WVH): Nav2 with
  AMCL localization on a saved map of WVH (plus a keepout map) and a Python
  `BasicNavigator` API (`goToPose`, `followWaypoints`, ...).

**Open item:** real cup dimensions (height, rim and base diameter). These set the grasp
height, base-row spacing and lift range. Put them in `config/robot.yaml` (and `sim.yaml`,
which also sizes the sim's cups).

### 0. Setup and shared work

- [x] **0.1 Split `cup_finder_controller.py` into reusable pieces**: `CupRobot` (on the
      starter's `HelloNode`: checked moves, odometry drives, stops), `HeadCamera` (frames and
      fresh measurements), `align_arm` (ALIGN for any target), `perception.py`.
- [x] **0.2 Parameter files for sim and real robot**: `config/sim.yaml` and
      `config/robot.yaml`, loaded with `find_cup.launch.py config:=sim|robot`. Sim-only
      workarounds are marked in `sim.yaml` and off in `robot.yaml`; all cup measurements are
      parameters (placeholder: a 16 oz cup, 12 cm tall, 9.5 cm rim).
- [ ] **0.3 Check the real robot's topics and frames** (partial)
  - Done: the differences between the drivers are listed in
    [First runs on the real robot](#first-runs-on-the-real-robot), and
    `check_interfaces` checks every topic, service, action, joint and frame we use; it
    passes on the sim.
  - Left: run `check_interfaces --nav` on the real robot.
- [x] **0.4 Sim scene for the full task**: `ros2 launch stack_cups sim.launch.py`; see
      [The task scene](#the-task-scene).
- [ ] **0.5 Tests** (partial)
  - Done: unit tests for `perception.py` on synthetic depth images, the config files, the
    sim scene, and pyramid spot math (the sim's pyramid layout); `colcon test` lint passes
    (lint settings relaxed for our style).
  - Left: tests on images saved from the sim and the real robot; tape detection tests
    come with 2.1.
- [x] **0.6 Real-robot safety**: speed caps, stop at any time (Ctrl+C, a topic, the
      runstop), step-by-step runs; see [Stopping and running step by step](#stopping-and-running-step-by-step).
- [x] **0.7 Add `parcs_stretch_nav2` to the workspace**: in the setup steps above; its
      `simple_commander_demo.py` import is fixed by the patch.
- [ ] **0.8 Navigation in the sim** (partial)
  - Done: the sim publishes `/scan` for AMCL, the starter's mapping launch runs on the sim
    (patch), and `sim/map_sim_scene.py` and `sim/nav_check.py` drive the mapping route and
    check Nav2 goals; see [Navigation in the sim](#navigation-in-the-sim).
  - Left: make the map of the task scene (the first run got stuck on the last leg of the
    drive) and check Nav2 on it.

### 1. Find the cup and pick it up

**Owner:** Hutch

- [ ] **1.1 Detect loose cups** (partial: red detection works)
  - Make the red thresholds parameters and tune them for real lighting.
  - Ignore red cups inside the taped area (already placed).
  - Use depth to separate cups that touch.
  - Pick the closest loose cup, not the biggest red blob.
  - Keep tracking the same cup through ALIGN (the one nearest its last position): in the
    task scene the raised arm hides the target and the next cup becomes the biggest blob.
- [ ] **1.2 Search when no cup is in view**: pan the head, or turn and drive.
- [ ] **1.3 Drive to the cup's general area** (partial: Nav2 from the starter package)
  - Save a pose near the cup table in the map frame and drive there with
    `BasicNavigator.goToPose`, with the arm side facing the table.
  - Stow the arm first (the starter recommends it, and Nav2's footprint assumes it).
  - `sim/cup_approach.py` is no longer needed for this; keep it only as a standalone
    sim test.
- [ ] **1.3b Hand off from Nav2 to ALIGN**
  - Nav2 stops within 25 cm and 0.25 rad (about 14°) of the goal
    (`xy_goal_tolerance`, `yaw_goal_tolerance`), so the camera has to finish the job.
  - ALIGN only drives forward and back; add a step that turns the robot square to the
    table edge (seen in depth) before aligning.
  - Switch the driver from navigation mode to position mode for arm moves, and back.
- [x] **1.4 Center the cup and line it up with the arm** (about ±1 cm; slow in the sim)
- [x] **1.5 Measure the cup and table, check the reach is safe**
  - [ ] Add a cup height measurement as a cross-check against the parameters.
- [ ] **1.6 Move the gripper to the cup** (partial: works)
  - Grasp at a set height on the cup (from parameters), not the cup's center.
- [ ] **1.7 Close the gripper by effort** (partial: closes to a fixed position)
  - Hold firmly without crushing real cups.
- [ ] **1.8 Check the grasp worked**: gripper position/effort, or the cup is gone from
      the table.
- [ ] **1.9 Lift and retract into the carry pose**: clear the table, arm in, wrist
      tucked, cup upright.
- [ ] **1.10 Retry on a failed grasp**: open, back off, repeat ALIGN and REACH.

### 2. Drive to the stacking area with the cup

- [ ] **2.1 Detect the blue tape**: color threshold, depth for 3D corners, then the
      area's center and direction. Used only for the final approach; Nav2 gets the robot
      close.
- [ ] **2.2 Know where the robot is** (partial: AMCL in the starter package)
  - Set the initial pose in code (`setInitialPose`) instead of clicking in RViz.
  - Save named poses (cup table, stacking area) in the map frame in a config file.
- [ ] **2.3 Plan and follow a path to the area** (partial: Nav2 `goToPose`)
- [ ] **2.4 Avoid obstacles** (partial: Nav2 costmaps with lidar and the keepout map)
  - Nav2 uses `robot_radius: 0.22`, which assumes the arm is stowed. Make sure the
    carry pose keeps the cup inside that footprint, or enlarge it while carrying.
- [ ] **2.5 Check the cup is still held while driving**: watch gripper effort, stop if
      it drops.
- [ ] **2.6 Park lined up with the next pyramid spot** (partial: reuse ALIGN with the
      spot as the target)
  - Same Nav2-to-ALIGN hand-off as 1.3b, squaring up to the tape instead of the table
    edge.

### 3. Flip the cup

- [ ] **3.1 Check the wrist can turn 180°**: check `joint_wrist_roll` limits in the URDF
      and on the real robot. If not, combine pitch and roll, or grasp at a roll angle
      that leaves room to turn.
- [ ] **3.2 Flip at the stacking area**: arm retracted and raised clear of the pyramid,
      just before placing.
- [ ] **3.3 Check the grip after flipping**: a flipped cup can slide toward its narrow
      end and fall out.
- [ ] **3.4 Test flipping on the real robot early**: riskiest step; the sim may not
      model a plastic cup slipping well.

### 4. Place the cup in the pyramid

**Owner:** Dan

- [ ] **4.1 Define the 3-2-1 layout relative to the tape**
  - 3 base cups at fixed spots along one tape edge (spacing is a parameter).
  - 2 middle cups, each centered between two base cups.
  - 1 top cup, centered between the two middle cups.
- [ ] **4.2 See the current pyramid**: find every cup inside the tape (upside down
      confirms it's placed), sort into rows by height, compare against the 6 spots.
- [ ] **4.3 Choose the next spot**
  - Fill base, then middle, then top.
  - Base cups go on the table; upper cups go between the measured positions of the two
    cups below, at the measured height of their tops.
- [ ] **4.4 Plan the placement** (partial: extend `plan_reach`)
  - Add the grasp-point-to-rim distance and a clearance margin.
  - Check the top cup is within the lift's range.
  - Upper rows need about ±1 cm accuracy.
- [ ] **4.5 Move above the spot and lower slowly**: keep the arm above placed cups while
      extending; reuse `move_joint_precise`.
- [ ] **4.6 Release and back away**: open gently, raise straight up, retract.
- [ ] **4.7 Check the placement**: new cup in place, cups below unmoved; stop and report
      if something fell.
- [ ] **4.8 Loop for the next cup** until all 6 are placed.
