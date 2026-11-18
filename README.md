# Cup stacking with Stretch

ROS 2 Humble code for Hello Robot's Stretch, run in the Stretch MuJoCo simulator.
The `stack_cups` package finds a red cup with the head camera, lines the robot up
with it, and reaches the gripper to it, using depth to avoid hitting the table.

## Workspace layout

Clone this repo next to Hello Robot's `stretch_ros2` in the same `src/` folder:

```
workspace/
    - src/
        - stretch_ros2/
        - cup-stacking-stretch-project/
    - build/  install/  log/      (created by colcon)
```

The important part is having the 3rd party stretch robot package in the same source
for building and working with nodes.

```bash
mkdir -p ~/ws/src && cd ~/ws/src        # any workspace folder name works
git clone https://github.com/hello-robot/stretch_ros2.git -b humble
git clone https://github.com/qutch/cup-stacking-stretch-project.git
```

## One-time setup

Needs Ubuntu 22.04 with ROS 2 Humble (on a Mac, an Ubuntu VM in UTM works). Follow the
team's ROS2 setup guide (PDF), steps 8-10. The parts that are easy to get wrong:

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
4. **ROS dependencies,** including two the sim launch file uses but does not declare:
   ```bash
   cd ~/ws
   rosdep install --from-paths src --ignore-src -y -r --skip-keys stretch_body
   sudo apt install -y ros-humble-joint-state-publisher ros-humble-control-msgs
   sudo pip3 install ros2_numpy pyquaternion
   sudo pip3 install --force-reinstall --no-cache-dir \
       "numpy==1.23.3" "numba==0.56.4" "llvmlite==0.39.1"     # always last
   ```
5. **Build:**
   ```bash
   cd ~/ws && source /opt/ros/humble/setup.bash
   colcon build --packages-up-to stretch_simulation stack_cups
   colcon build --packages-select stretch_description
   ```
   Do not run a plain `colcon build`; `stretch_deep_perception` fails to build and is not needed.

## Running

**Terminal 1: simulator**

```bash
cd ~/ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export MUJOCO_GL=osmesa      # see "Depth" below; egl is faster if your GPU gives real depth
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
    mode:=navigation use_rviz:=false use_robocasa:=false use_cameras:=true
```

`use_cameras:=true` is required; cameras are off by default. Add
`use_mujoco_viewer:=false` to save a lot of CPU. The default scene (with
`use_robocasa:=false`) has a red cylinder on a table to the robot's right.

**Terminal 2: the node**

```bash
cd ~/ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run stack_cups find_cup                                  # reach to the cup
ros2 run stack_cups find_cup --ros-args -p grasp:=true        # also close the gripper
```

Parameters (`--ros-args -p name:=value`):

| Parameter | Default | Meaning |
|---|---|---|
| `aim_head` | `true` | Point the head at the table before starting |
| `head_pan`, `head_tilt` | `-1.57`, `-0.6` | Head pose used by `aim_head` (radians) |
| `grasp` | `false` | Close the gripper at the end |
| `show_windows` | `true` | Show the camera and red-mask debug windows |

The node logs each phase: `[START]` → `[AIM_HEAD]` → `[CENTER]` → `[READY]` →
`[ALIGN]` → `[REACH]` → `[DONE]`. On any problem it stops the robot and logs
`[FAILED] <reason>`.

## How it works

`stack_cups/cup_finder_controller.py` (the ROS node) and `stack_cups/perception.py`
(pure numpy/OpenCV, no ROS, easy to test):

1. **CENTER:** drive forward/back until the red blob is centered in the head image.
2. **READY:** raise the lift above the cup, straighten the wrist, open the gripper, and
   measure the grasp point's offsets from TF.
3. **ALIGN:** locate the cup in 3D from depth (depth points are projected into the color
   image and only those on red pixels are kept), then drive until it lines up with the
   arm. Close to the target it uses stop-and-measure nudges, because camera frames lag
   when the sim is slow.
4. **REACH:** from depth, measure the table top and the table edge. Refuse to move if the
   gripper would be lowered into the table, onto it while retracted, or the cup is out of
   reach. Otherwise lower to cup height and extend, correcting until each joint is within
   1 cm (the sim's trajectory server accepts anything within 5 cm).

In MuJoCo tests the cup is located within about 1 cm and the table within about 1 cm.
A grasp works when the cup is within about ±1 cm of the arm line, which is what ALIGN
aims for.

## Helper scripts (`sim/`)

| Script | What it does |
|---|---|
| `check_camera.py` | Grab one camera frame, print brightness and red-pixel count, save it to `/tmp` |
| `debug_cup_depth.py` | Run the node's depth pipeline on one frame and print every step |
| `stretch_table_scene.py` | Standalone MuJoCo scene (no ROS): Stretch + square table |
| `cup_approach.py` | Standalone: random red cup on the table; robot drives around the table to it. `--face arm` is not reliable yet (the gripper can clip the table) |

Run the ROS ones after `source install/setup.bash` with the sim running; run the standalone
ones from the `sim/` folder with `python3 <script>`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `[FAILED] depth image looks broken` or the cup is never found in depth | The GPU renderer is not producing depth (common in VMs). Restart the sim with `export MUJOCO_GL=osmesa`. Check with `sim/debug_cup_depth.py`. |
| Camera window all black, `red pixels: 0` | The head is not looking at the cup. Keep `aim_head:=true` or adjust `head_pan`/`head_tilt`. |
| Node gets no images at all | The sim was launched without `use_cameras:=true`. |
| `package 'joint_state_publisher' not found` / `No module named 'control_msgs'` | Setup step 4. |
| `Permission denied` reading files during `colcon build` from a VM shared folder | Files are readable only by their owner on the host. On the Mac: `chmod -R go+rX <workspace>/src`. If the build fails, `ros2 run` silently runs the old installed version. |
| Viewer black/glitchy or very slow | Log in to Ubuntu on Xorg (not Wayland), enable GPU acceleration in the VM, give the VM more CPU cores, and use `use_mujoco_viewer:=false`. |
| `colcon test` lint failures | The ROS lint config wants single quotes and Google import order; the code currently uses double quotes (black style). Either reformat or relax the lint tests. |
