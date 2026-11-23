# Faster, more precise cup pick-up, built on the starter code

Date: 2026-11-22 · Owner: Hutch · Roadmap: 0.1, 1.1 (part), 1.4, 1.6 (part)

## Why

The `find_cup` node lines the robot up with a red cup and reaches to it, but it is far too
slow to be usable on the real robot. The robot ends up in the right place eventually, but the
approach would not work in a real-world run. The course also gives starter code
(`parcs_stretch_nav2`, which uses Hello Robot's `hello_helpers.HelloNode`). Our node uses none
of it, which makes it harder to share code with the rest of the team and to move to the real
robot.

## Where the time goes today (from reading `cup_finder_controller.py`)

- **ALIGN, fine stage:** up to 25 rounds of "drive a short burst, stop, wait 1 s, average 3
  frames". The sim's base only reaches about 20–25% of a small commanded speed, so near the
  target it moves at about 8 mm/s, and bursts can last 6 s.
- **CENTER and ALIGN steer the same thing twice:** both drive forward/back. CENTER uses
  image pixels, then ALIGN does it again with depth.
- **Arm:** joints move one at a time. Every move is followed by a 0.3 s wait, and every
  correction round by a 1.5 s wait. `move_joint_precise` can do up to 5 rounds per joint.
- **Mode switches:** the node switches to position mode and back to navigation mode several
  times, with a wait each time.

The proportional "big steps far, small steps near" control already exists in ALIGN. It is
slow because it steers by the camera, which lags, not because of the step sizes.

## What we found in the starter code and the sim

- `HelloNode` (from `hello_helpers`, already a dependency in `package.xml`) gives us:
  `move_to_pose`, `switch_to_*_mode`, `stop_the_robot`, `stow_the_robot`, `tf2_buffer`,
  `joint_state`, and `hm.get_p1_to_p2_matrix`. The starter's `move_robot.py` is the pattern:
  `node.move_to_pose({joint: value}, blocking=True)`.
- `move_to_pose` has gaps:
  - On invalid input it logs and returns `None` instead of raising.
  - A blocking call waits forever and never checks whether the move succeeded.
  - `main()` starts ROS itself and starts its own spinning thread.
  - It always subscribes to the full point cloud, which costs CPU in Python.
  - It warns on every call made outside position mode.
- **The sim cannot do a base position move.** Its trajectory server passes
  `translate_mobile_base` to `sim.move_to`, which raises "Cannot set an absolute position
  for a continuous joint". The real robot supports it; the sim does not.
- **The sim's trajectory server never checks the driving mode,** so arm, lift, wrist, head
  and gripper moves work in navigation mode. The real driver does the same: in navigation
  mode only the base behaves differently.
- **The sim publishes `/odom`** with the base pose straight from MuJoCo, so it is exact. The
  real driver publishes `/odom` from wheel encoders.
- **The sim ignores a move's `time_from_start`** and reports success once a joint is within
  5 cm. The correct-until-within-1-cm loop is therefore still needed in the sim.

## Goals

1. Get from start to the gripper at the cup much faster. Targets:
   - at most 3 base moves in ALIGN (usually 1)
   - no fixed wait longer than 0.3 s
   - a timing log line for every phase
2. Stay as precise or better: grasp point within ±1 cm of the cup along the base's x axis,
   checked through the robot's frame transforms after the final move.
3. Build on `HelloNode` and its calls, in the style of the starter code, so the same helpers
   work on the real robot and can be shared with task 4.
4. When several red cups are in view, pick the one closest to the arm line, not the biggest
   red blob.

## Non-goals

- Searching for a cup that is not in view (roadmap 1.2)
- Nav2 driving to the table (1.3)
- Squaring the base to the table edge (1.3b)
- Closing the gripper by effort (1.7)
- Checking the grasp worked (1.8)
- Lifting and carrying the cup (1.9)
- Ignoring cups inside the taped area (1.1)

## Anti-goals (a failure even if it is faster)

- Any change that can lower the gripper into the table or onto it. The existing
  `plan_reach` safety checks stay and still gate every reach.
- A second, separate way of controlling the base for the sim and for the real robot. One code
  path must work in both.
- Copying `hello_helpers` code into our package. We call it.

## Approaches considered

1. **Base position moves through `move_to_pose({'translate_mobile_base': dx})`.** This is the
   cleanest starter-style call. **Rejected for now:** it raises an error in the sim's driver
   (see above), so it cannot be tested there.
2. **Drive by odometry (chosen).** Measure the cup once while stopped. Then drive in
   navigation mode with `cmd_vel`, steered by `/odom` many times a second (not by the camera)
   until the base has moved `dx`. Speed comes from the remaining distance, with a top speed
   and a minimum speed that actually moves the base. Re-measure once; correct at most twice.
   This works the same on the sim and the real robot, and the camera's lag no longer
   matters.
3. **Steer continuously by the camera (today's method, tuned).** Rejected: it is slow
   because of camera lag, whatever the gains.

The wrist-yaw correction is added on top of approach 2. After the base move, any remaining
x error of up to a few cm is covered by turning the wrist: yaw = asin(dx / L), where L is the
distance from the wrist's yaw axis to the grasp center, read from TF. This means the base
rarely needs a second move.

## Design

### Node structure

`FindCup` becomes a subclass of `hm.HelloNode`.

- **Startup:** `main()` creates the node with `HelloNode.main(...)` and
  `wait_for_first_pointcloud=False`. We do not call `rclpy.init()` ourselves. After startup,
  destroy the point-cloud subscription.
- **Parameters:** HelloNode declares parameters automatically from overrides, so we read
  them through a helper that declares a parameter only if it does not already exist.
- **What gets replaced:**

  | Ours | HelloNode version |
  |---|---|
  | `move_joints` | a thin wrapper `move(pose, timeout)` around `move_to_pose(blocking=False)` |
  | `switch_mode` | `switch_to_navigation_mode()` |
  | `lookup` | `self.tf2_buffer.lookup_transform` + `perception.quat_to_mat` (not `hm.get_p1_to_p2_matrix`: its rotations are wrong in our VM, see robot.py) |
  | `self.joints` | `self.joint_state` |
  | `stop` | zero `cmd_vel` plus `stop_the_robot()` on failure |

- **The `move` wrapper** waits for the goal to be accepted and for the result, with a
  timeout. It raises if `move_to_pose` returned `None`, if the goal was rejected, or if the
  result status is not SUCCEEDED.
- **Driving mode:** the whole pick-up runs in navigation mode, so the base takes `cmd_vel`
  and the arm still moves through `move_to_pose`. This removes all mode switching. We switch
  modes once at the start, and suppress or accept HelloNode's "intended for position mode"
  warning.

### Phases

1. **START.** Wait for color, depth, camera info, TF and `/odom`.
2. **AIM_HEAD.** Unchanged, using `move`.
3. **READY.**
   - Measure the cup while stopped.
   - Raise the lift with the arm retracted and open the gripper in one move.
   - Straighten the wrist in a second move (yaw 0, pitch 0, roll 0).
   - Read the grasp offsets from TF.
   - CENTER is removed: the cup only has to be visible in depth.
4. **ALIGN.**
   - Measure dx (cup x minus grasp x in `base_link`) while stopped, using fresh frames.
   - If |dx| is more than the wrist can cover, call `drive_by(dx)`. Re-measure. Repeat at
     most 3 times.
   - Then set the wrist yaw to cover the remaining error, re-read the offsets from TF, and
     check the grasp point is within ±1 cm of the cup's x. Fail clearly if not.
5. **REACH.**
   - Unchanged `plan_reach` safety checks, using the re-read offsets.
   - Lift and extension use `move_joint_precise`, rebuilt on `move`. It waits until the
     joint has stopped moving (from `joint_state`) instead of sleeping 1.5 s.
   - An optional `grasp_height` parameter grasps at a set height above the table top
     instead of the cup's measured center.
6. **DONE / FAILED.** Each phase logs how long it took, and the total.

### `drive_by(distance)`, base move by odometry

- Record the start pose from `/odom`.
- At 20 Hz:
  - progress = displacement projected onto the start heading
  - remaining = distance − progress
  - stop when |remaining| < `drive_tol` (default 3 mm)
- The speed command comes from a pure function, `base_speed(remaining, v_max, v_min,
  decel)` = sign · clip(√(2 · decel · |remaining|), v_min, v_max). It may reverse if the
  base overshoots.
- Fail if the base stops making progress for 2 s, or after a timeout.
- `cmd_vel` keeps streaming at 10 Hz, as now, because the driver brakes if commands stop.
- `v_min` is a parameter. The sim needs a larger value than the real robot (friction).

### Fresh measurements (precision)

Replace `SETTLE_S` sleeps with a counter of depth frames received. After stopping, wait
for N new frames (default 2), then take the **median** of K measurements. If the samples
spread more than 1 cm, take more.

### Detection

`perception.find_red_blobs` returns all red blobs above the minimum area, plus the mask.
The node locates each blob in 3D and picks the one with the smallest |dx| that is
within reach. The red HSV thresholds and the minimum area become node parameters, with
today's values as defaults.

### Testing

- **Unit tests** (pure Python, in `stack_cups/test/`, run with `python3 -m pytest` in the
  VM):
  - `base_speed`
  - `yaw_for_offset`
  - `find_red_blobs` and closest-cup choice, on synthetic images
  - the fresh-measurement median and spread rule
- **In the sim:**
  - Run `find_cup` from at least 3 start offsets (cup about 5, 20 and 40 cm from the arm
    line). Record the per-phase times before and after.
  - Confirm each run ends within ±1 cm, with `grasp:=true` picking up the cup.
  - Add a second red cup for the closest-cup check.

## Left to the builder

- The exact sign of the wrist-yaw-to-x relation: measure it in the sim with a small yaw
  move, then encode it.
- The value of `v_min` for the sim.
- Whether HelloNode's position-mode warning is silenced or left alone.
- Default for the wrist-yaw limit: 0.25 rad, about ±5 cm at L ≈ 0.2 m. Lower it if the
  fingers hit the cup at that angle.
