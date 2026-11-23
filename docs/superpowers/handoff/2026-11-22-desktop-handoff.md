# Handoff: fast cup pick-up, continued on the Windows desktop (WSL2)

Written 2026-11-22 for a new Claude Code session on Hutch's Windows desktop. Read this
first, then the spec and plan it links. It replaces the conversation that built this
branch on the Mac. It records what was learned, what is verified and what is not.

## 1. The goal

Make the Stretch robot's red-cup detection and pick-up (`ros2 run stack_cups find_cup`)
fast and precise enough for a real-world run, built on the course starter code's
`hello_helpers.HelloNode`. The old node took minutes and failed in ALIGN.

- Spec: `docs/superpowers/specs/2026-11-22-fast-cup-pickup-design.md`
- Plan (8 tasks): `docs/superpowers/plans/2026-11-22-fast-cup-pickup.md`
- Baseline timings of the old code: `docs/superpowers/plans/2026-11-22-baseline-timing.md`
- Branch: `fast-pickup` (commit `6e10cac` and later). `main` is untouched.

## 2. Working agreements (from the user)

- **Do not modify Hello Robot's `stretch_ros2`** (including the sim driver). The user
  declined patching it. Work around its bugs in our code.
- Commit only when the user asks. Push only with their approval.
- Keep the user's personal details out of anything sent to outside services.
- The user prefers clear, plain explanations and step-by-step commands.
- Run long sim runs in the background and read their logs; a full run with cameras can take
  tens of minutes of real time on a slow machine.

## 3. Environment on the desktop

- Windows + WSL2 + Ubuntu 22.04 + ROS 2 Humble. GPU: RTX 2080 SUPER. In WSL, `nvidia-smi`
  and `glxinfo -B` (renderer `D3D12 (NVIDIA GeForce RTX 2080 SUPER)`) were confirmed.
- Workspace: `~/ws` with `src/stretch_ros2` (branch `humble`) and
  `src/cup-stacking-stretch-project`. stretch_mujoco is installed from source in
  `~/repos/stretch_mujoco` (README "One-time setup" steps 2-5).
- **Rebuild after every code change:**
  `cd ~/ws && source /opt/ros/humble/setup.bash && colcon build --packages-select stack_cups --symlink-install && source install/setup.bash`
  (Never a plain `colcon build`: `stretch_deep_perception` fails and is not needed.)

### First job on the desktop: is the GPU path usable?

Terminal 1 (sim):

```bash
cd ~/ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export MUJOCO_GL=egl
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
    mode:=navigation use_rviz:=false use_robocasa:=false use_cameras:=true use_mujoco_viewer:=false
```

Terminal 2, after `Changed to mode = navigation`:

```bash
python3 ~/ws/src/cup-stacking-stretch-project/sim/clock_rate.py
python3 ~/ws/src/cup-stacking-stretch-project/sim/debug_cup_depth.py | head -4
```

- **Speed:** the Mac VM ran at **1.0%** of real time with osmesa (CPU), and 5.5-6.8% with
  its GPU.
- **Depth must be real:** `depth values: min=0.000 max=9.999 median≈1` (head at its default
  pose the numbers differ, but there must be no negatives and few NaNs). In the Mac VM every
  GPU mode (egl, glfw, ANGLE Metal and OpenGL) gave broken depth
  `min=-0.017 max=0.000 zeros~35% nan=38%`, so it had to use osmesa.
- If egl errors, try `MUJOCO_GL=glfw` (WSLg has a display). If depth is broken, use
  `MUJOCO_GL=osmesa`. Check `env | grep LIBGL`: in the VM `LIBGL_ALWAYS_SOFTWARE=1` was set
  and forced software GL. It must be unset for any GPU mode.
- Cameras off (`use_cameras:=false`) runs ~50% of real time on the Mac; useful for testing
  base and arm code, not perception.

## 4. Running find_cup

```bash
ros2 run stack_cups find_cup --ros-args -p grasp:=true 2>&1 | tee ~/runB.log
grep -vE "move_to_pose" ~/runB.log | grep -E "\[|Error|Traceback|WARN"     # summary
```

- Restart the sim before every run so the robot starts at the same pose.
- Phases: `[START]` → `[AIM_HEAD]` → `[READY]` → `[ALIGN]` → `[REACH]` → `[DONE]` or
  `[FAILED] <reason>`. Each logs `[timing] X took <sim s> s (wall <real s> s)`.
- Every "cup not found" now names why, e.g. `no red blob ... (N red pixels)`,
  `TF ... unavailable`, `red blob at (...) but too few depth points on it`.
- Parameters are in `PARAMS` at the top of `cup_finder_controller.py` (`-p name:=value`).
  Main ones: `grasp`, `show_windows` (OpenCV windows; set false with no display),
  `use_wrist_yaw`, `v_min`/`v_max`/`decel`, `samples`, `settle_s`, `grasp_height`,
  `odom_topic`.

## 5. Code map

| File | What it is |
|---|---|
| `stack_cups/stack_cups/robot.py` | `CupRobot(hm.HelloNode)`: startup, `param()`, sim-time clock, `move()` (checks results; works around the sim's status-6 bug), `move_joint_precise`, `wait_joint_still`, `lookup()` (TF), odometry, cmd_vel stream, `drive_by()` |
| `stack_cups/stack_cups/cup_finder_controller.py` | `FindCup(CupRobot)`: the phases, `measure()`, `measure_fresh()` (median, in the odom frame), `align()`, `_align_wrist()`, `reach()` |
| `stack_cups/stack_cups/motion.py` | Pure numpy math: `base_speed`, `progress_along`, `yaw_reach`, `yaw_for_offset`, `robust_center`, `base_to_odom`, `odom_to_base` |
| `stack_cups/stack_cups/perception.py` | Unchanged so far: `find_red_blob`, `locate_cup`, `estimate_table_top`, `table_edge_distance`, `plan_reach`, `quat_to_mat` |
| `stack_cups/test/test_motion.py` | 7 unit tests (`python3 -m pytest stack_cups/test/test_motion.py`) |
| `sim/clock_rate.py` | Sim speed as % of real time |
| `sim/debug_cup_depth.py` | Every step of the depth pipeline on one frame (standalone) |
| `sim/check_measure.py` | Measures the cup with the real `FindCup` class, moves nothing, prints miss reasons |
| `sim/check_robot_layer.py` | Checks `move`, precise lift, TF, invalid moves, `drive_by`, wrist yaw |
| `sim/check_camera.py` | Saves one head-camera frame to `/tmp/camera_check.png` |
| `sim/nudge_base.py <m>` | Drives the base by a distance (to set up a start offset) |

## 6. Hard-won facts about the sim (all verified unless marked)

1. **Trajectory server reports ABORTED (status 6) after every goal** even when the joints
   arrive (it raises "Please use get_position() for left_wheel_vel" in its wait). `move()`
   checks `joint_states` instead and warns once.
2. Its `wait_until_at_setpoint` gives up after **5 s of wall time**, tolerance 0.05. At low
   sim speed that fires mid-move, so `move()` keeps waiting while the joints still move.
3. It ignores `time_from_start`, doesn't check the driving mode (arm moves work in
   navigation mode), and **does not support `translate_mobile_base`**, which is why base
   moves use cmd_vel + odometry (`drive_by`).
4. **Base drift (root cause found):** the driver times cmd_vel in **wall** time. If the last
   twist is 0.5-1.5 s old it runs `sim.move_by(base_translate, 0.0)`, which **drives the
   base backward**. Test: no twists → odom x steady (-0.06806 ×3); a zero twist every ~3 s
   → x went -0.19 → -0.91 in 15 s; zero twists at 10 Hz → steady (0.2 mm in 15 s). Our
   stream was a sim-time timer (one message per ~10 wall s at 1% speed), so the base crept
   back 3-4 cm/s while "stopped". **Fixed** in `6e10cac`: a 10 Hz wall-time thread
   (`_stream_loop`). **Verified in run B3** (no drift warnings; odom x identical from
   READY to ALIGN).
5. `/odom` is MuJoCo ground truth; `/clock` is sim time; cmd_vel topic is
   `/stretch/cmd_vel`.
6. Cameras render at 10 Hz sim time; between renders the last frame is **re-sent**, so
   counting messages does not mean fresh frames. `measure_fresh` waits `settle_s` and
   samples `sample_dt` apart in sim time.
7. **`hm.get_p1_to_p2_matrix` returned wrong rotations** (positions right): its
   `ros2_numpy` read the quaternion as (w,x,y,z). Verified numerically (max diff 0.001).
   `lookup()` now uses `tf2_buffer.lookup_transform` + `perception.quat_to_mat`. Probably
   the same on the desktop (same pip package); teammates using that helper for rotations
   will hit it too.
8. With cameras on, everything is slow in wall time; all waits use the ROS clock (sim time
   when `/clock` exists). The only wall-time limit is `clock_stalled()`: the ROS clock not
   moving for 60 s.
9. `New publisher ... '/camera/depth/color/points' ... incompatible QoS` warnings and
   `terminate called without an active exception` at script exit are harmless.

## 7. Measured values

- Wrist: `yaw_sign` +1, `arm_len` 0.270 m (yaw axis to grasp center).
- Base: `v_min` 0.06 m/s moves the base; `drive_by` errors ≤ 2.6 mm (±10 cm in 4.7 s sim,
  ±2 cm in 1.3 s sim); a +32.7 cm drive ended at +32.4 cm.
- Default scene, from a fresh sim: head pan -1.57 / tilt -0.6 sees the cup. Cup at about
  base (0.13-0.15, -0.556, 0.528) m; cup in odom ≈ (+0.08, -0.553). Grasp point
  x ≈ -0.021, 0.109 m above the lift, 0.415 m beyond the extension; lift raises to ~0.670.
- Sim-time phase costs in the last run: AIM_HEAD 0.4 s, READY 2.6 s, ALIGN 14.9 s (incl.
  a 32 cm drive). These are close to what the real robot would take.

## 8. Run history (the last two on the new code)

- **Run B1:** READY found the cup; ALIGN failed: a 600 s wall cap killed `drive_by` mid-drive
  (removed since; replaced by `clock_stalled()`).
- **Run B2:** cup found every time, odom-frame measuring stable (cup odom within 5 mm,
  spread ~1 mm); drive accurate; wrist turned to +0.141 rad as asked; **FAILED**
  "wrist yaw left +0.174 m" because the base drifted back ~17 cm during the wrist turn and
  re-measure (fact 4, now fixed).
- **Run B3 (`-p grasp:=true`, after the drift fix): DONE.** No drift warnings; odom x
  -0.013 in both READY and ALIGN. Cup +12.2 cm → one drive (asked +12.2, got +11.9) → cup
  -0.9 cm → wrist yaw -0.033 rad (reached -0.026) → grasp point **-0.1 cm** from the cup.
  REACH: lift 0.418 (target 0.420), extension 0.137 (target 0.141; first try stopped 3.5 cm
  short and `move_joint_precise` corrected it), gripper closed. Sim time: AIM_HEAD 0.4 s,
  READY 2.4 s, ALIGN 8.5 s, REACH 4.4 s, **total 15.7 s** (wall 1875 s at 1% speed).
  Not checked: whether the cup is actually held (grasp check is roadmap 1.8).
- **Possible speed-up seen in B3:** the 12 cm drive took about 6 s of sim time (~2 cm/s
  average) although `v_max` is 0.15 m/s; worth checking `drive_by`'s speed profile and
  the sim base's acceleration on the desktop.

## 9. What is left (plan tasks)

Update 2026-11-22 (desktop, after PR #6 merged; results in the timing file):

- Tasks 1-4: complete. Tasks 5-6: **verified** in the patched sim with the viewer on:
  run A passed, run B passed earlier (B3), D1 passed (now fails in 5 s), D2 passed.
  Run C found a false DONE (extension blocked short); a blocked reach now fails.
- Task 7: several cups. The 6-cup task scene now works (the raised gripper used to hide
  the target; `ready_above_cup` 0.05). Still: pick the closest loose cup, red HSV ranges
  and `min_area` as parameters, `test_perception.py`.
- Task 8: README update, final runs from about 5, 20 and 40 cm offsets.
- Then a whole-branch review, and the list of rulings below goes to the user.
- Superseded by PR #6: the "do not modify stretch_ros2" agreement (patches are fine now),
  and facts 1-4 in section 6 (the patches fix them at the source).

## 10. Decisions made on the user's behalf (rulings) and their cost if wrong

1. `move()` accepts a non-success status when every joint is within 0.05 of target
   (gripper not checked) — on the real robot an aborted-but-arrived move is accepted.
2. All timeouts and timings use the ROS clock; `use_sim_time` is turned on automatically
   when `/clock` has a publisher — if something publishes `/clock` on the real robot, times
   follow it.
3. Sim timings are judged in sim time (logged with wall time) because the sim is slow.
4. Navigation mode is entered before AIM_HEAD; the gripper opens with the wrist-straighten
   move, not the lift move.
5. Wrist yaw is set with `move_joint_precise` (tol 0.01 rad) because `move()` alone
   accepts 0.05 rad (13 mm at 0.27 m).
6. `measure_fresh` waits `settle_s` 0.3 s and samples 0.1 s apart instead of counting
   frames — on the real robot that is 0.3 s instead of ~0.07 s.
7. `lookup()` doesn't use `hm.get_p1_to_p2_matrix` (fact 7).
8. The 600 s wall cap was replaced by `clock_stalled()` — a sim that crawls but never
   freezes is only bounded by the ROS-time timeouts.
9. Measurements are combined in the odom frame and converted back with the latest pose —
   measurements inherit any odometry error (none in the sim; wheel odometry on the robot).
10. cmd_vel is streamed at 10 Hz wall time — when the node exits, the driver passes
    through its "move by 0" window once (~1 wall s of backward creep: negligible at 1% sim
    speed, a few cm in a fast sim). Consider switching to position mode on exit if that
    shows up on the desktop.
11. Fewer verification runs than the plan listed (each camera run cost 10-40 minutes on the
    Mac) — less run-to-run variance data.

## 11. Debugging playbook

1. Cup not found → read the reason in the log; run `sim/check_measure.py` (same class, no
   motion) and `sim/debug_cup_depth.py` (independent pipeline); compare the camera
   rotation printed by both (fact 7 was found this way).
2. Base moving when it shouldn't → watch `ros2 topic echo /odom --field pose.pose.position.x`
   with and without our node; check what is publishing on `/stretch/cmd_vel`
   (`ros2 topic info /stretch/cmd_vel -v`).
3. A joint "failed" → the sim reports status 6 anyway; compare `joint_states` with the
   target (`move()` logs which joints missed and by how much).
4. Anything slow → `sim/clock_rate.py` first; compare sim-time vs wall-time in the
   `[timing]` lines.
5. The sim driver's source is in `~/ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/`
   (read it, don't edit it); stretch_mujoco is in `~/repos/stretch_mujoco`.
