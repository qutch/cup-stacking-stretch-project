# Fast Cup Pick-up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `find_cup` line the gripper up with a red cup and reach to it in a few
moves instead of dozens. It is rebuilt on the starter code's `HelloNode`, and the grasp
point stays within ±1 cm.

**Architecture:**
- A new reusable layer, `CupRobot(hm.HelloNode)`, wraps the starter's `move_to_pose` (with
  timeouts and errors) and adds `drive_by`, which moves the base an exact distance using
  `/odom`.
- `FindCup` is rewritten on that layer. It runs entirely in navigation mode, measures the
  cup only while stopped and on fresh frames, moves the base once or twice, and covers the
  last few cm with the wrist yaw.
- Math stays in ROS-free modules (`motion.py`, `perception.py`) so it can be unit-tested.

**Tech Stack:** ROS 2 Humble, rclpy, `hello_helpers.hello_misc.HelloNode`, numpy, OpenCV,
pytest, Stretch MuJoCo sim.

**Spec:** `docs/superpowers/specs/2026-11-22-fast-cup-pickup-design.md`

## Global Constraints

- Everything runs in the Ubuntu VM. Work from the workspace root (`~/ws`). Build with
  `colcon build --packages-select stack_cups --symlink-install`, then
  `source install/setup.bash`. Never run a plain `colcon build`.
- Sim command (Terminal 1):
  `export MUJOCO_GL=osmesa && ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation use_rviz:=false use_robocasa:=false use_cameras:=true use_mujoco_viewer:=false`
- Unit tests: `cd ~/ws/src/cup-stacking-stretch-project/stack_cups && python3 -m pytest test/<file> -v`.
- Call `hello_helpers`; never copy its code into our package.
- One way of controlling the base for both the sim and the real robot. Do not use
  `translate_mobile_base`: the sim driver raises an error on it.
- Every reach still goes through `perception.plan_reach`'s safety checks.
- Accuracy: grasp point within ±0.010 m of the cup's base-x, read through TF after the
  final move.
- Speed: at most 3 base moves in ALIGN, and no fixed `time.sleep` longer than 0.3 s in the
  node.
- Every phase logs its duration, and DONE/FAILED logs the total.
- Code style: match the existing files (double quotes, numpy docstrings kept short).
- Work on branch `fast-pickup`. Commit only when the user has asked for commits.

## Review Focus

1. **The cup leaves the camera view after a base move.** Expect `[FAILED] lost the cup …`
   with the base stopped, not a crash or a robot that keeps driving. Pinned in Task 5,
   Step 5.
2. **The base oscillates around the target** (sim friction plus a large `v_min`). Expect
   `drive_by` to give up after `max_reversals` and return the distance reached, not loop
   forever. Pinned in Task 2 (`base_speed` is 0 inside `tol`) and Task 4, Step 4.
3. **`/odom` missing or on another topic.** Expect START to fail within 30 s with a message
   naming the topic. Pinned in Task 5, Step 5.
4. **An exception or Ctrl+C during `drive_by`.** Expect the streamed `cmd_vel` to be 0
   afterwards, so the base stops within 0.5 s. Pinned in Task 4, Step 4.
5. **`move_to_pose` returns `None` or the goal is rejected** (bad joint name, or a second
   goal within 0.2 s). Expect `move` to raise `RuntimeError` naming the pose. Pinned in
   Task 3, Step 4.

---

## File structure

| File | Responsibility |
|---|---|
| `stack_cups/stack_cups/motion.py` (new) | Pure math: base speed profile, progress along heading, wrist yaw for an x offset, robust median of samples, base↔odom point transform |
| `stack_cups/stack_cups/robot.py` (new) | `CupRobot(hm.HelloNode)`: startup, params, `move`, joint helpers, TF lookup, `/odom`, `cmd_vel` streaming, `drive_by`. Reusable by task 4 (placing) |
| `stack_cups/stack_cups/perception.py` | Add `find_red_blobs`, threshold arguments, `choose_cup`, `nearest_cup` |
| `stack_cups/stack_cups/cup_finder_controller.py` | `FindCup(CupRobot)`: phases only |
| `stack_cups/test/test_motion.py`, `test_perception.py` (new) | Unit tests |
| `sim/check_robot_layer.py` (new) | Sim check of `CupRobot`: drive accuracy, `v_min`, wrist-yaw sign and arm length |
| `README.md` | Updated "How it works", parameters, roadmap ticks |

---

### Task 1: Phase timing and a baseline run

**Files:**
- Modify: `stack_cups/stack_cups/cup_finder_controller.py` (`set_phase`, `run`)

**Interfaces:**
- Produces: log lines `[timing] <PHASE> took <s> s` and `[timing] total <s> s`. Later tasks keep this format.

- [ ] **Step 1:** `git checkout -b fast-pickup`
- [ ] **Step 2:** In `FindCup.__init__`, add `self.t_start = self.t_phase = None`. In
  `set_phase`:
  - when a phase changes and the previous one is not `START`, log
    `[timing] {prev} took {now - t_phase:.1f} s`;
  - set `t_phase = now`;
  - on the first call, set `t_start`.

  In `run`, after DONE or FAILED, log `[timing] total {now - t_start:.1f} s`.
- [ ] **Step 3:** Build, start the sim, and run `ros2 run stack_cups find_cup` 3 times from
  the default start pose.
  - Expected: DONE each time, with a timing line for every phase.
  - Copy the times into a table in `docs/superpowers/plans/2026-11-22-baseline-timing.md`
    (phase × run). This is the "before" number.
- [ ] **Step 4:** Commit: `feat(find_cup): log per-phase timing; record baseline`

---

### Task 2: Pure motion math

**Files:**
- Create: `stack_cups/stack_cups/motion.py`
- Test: `stack_cups/test/test_motion.py`

**Interfaces:**
- Produces:
  - `base_speed(remaining: float, v_max: float, v_min: float, decel: float, tol: float) -> float`
  - `progress_along(start: tuple[float, float, float], xy: tuple[float, float]) -> float`
    (start = odom x, y, yaw)
  - `yaw_reach(arm_len: float, max_yaw: float) -> float`
  - `yaw_for_offset(dx: float, arm_len: float, max_yaw: float, sign: float) -> float | None`
  - `robust_center(points: np.ndarray, max_spread: float) -> tuple[np.ndarray, float, bool]`
    returns (median xyz, spread, ok)
  - `base_to_odom(p_xy: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray`

- [ ] **Step 1: Write the failing tests**

```python
import numpy as np
import pytest
from stack_cups import motion as M

def test_base_speed_zero_inside_tol():
    assert M.base_speed(0.002, 0.15, 0.06, 0.1, tol=0.003) == 0.0
    assert M.base_speed(-0.002, 0.15, 0.06, 0.1, tol=0.003) == 0.0

def test_base_speed_sign_and_limits():
    assert M.base_speed(1.0, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(0.15)   # capped
    assert M.base_speed(0.005, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(0.06)  # floored
    assert M.base_speed(-1.0, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(-0.15)
    # sqrt(2*a*d) in between: d=0.05 -> 0.1
    assert M.base_speed(0.05, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(0.1)

def test_progress_along_heading():
    assert M.progress_along((1.0, 2.0, 0.0), (1.3, 2.4)) == pytest.approx(0.3)
    assert M.progress_along((0.0, 0.0, np.pi / 2), (0.1, -0.2)) == pytest.approx(-0.2)

def test_yaw_for_offset():
    assert M.yaw_reach(0.2, 0.25) == pytest.approx(0.2 * np.sin(0.25))
    assert M.yaw_for_offset(0.02, 0.2, 0.25, sign=1.0) == pytest.approx(np.arcsin(0.1))
    assert M.yaw_for_offset(0.02, 0.2, 0.25, sign=-1.0) == pytest.approx(-np.arcsin(0.1))
    assert M.yaw_for_offset(0.10, 0.2, 0.25, sign=1.0) is None   # beyond reach
    assert M.yaw_for_offset(0.0, 0.2, 0.25, sign=1.0) == 0.0

def test_robust_center_rejects_spread():
    pts = np.array([[0.50, -0.6, 0.8]] * 4 + [[0.53, -0.6, 0.8]])
    c, spread, ok = M.robust_center(pts, max_spread=0.01)
    assert c == pytest.approx([0.50, -0.6, 0.8]) and not ok and spread == pytest.approx(0.03)
    c, spread, ok = M.robust_center(pts[:4], max_spread=0.01)
    assert ok and spread == pytest.approx(0.0)

def test_base_to_odom():
    out = M.base_to_odom(np.array([1.0, 0.0]), (2.0, 3.0, np.pi / 2))
    assert out == pytest.approx([2.0, 4.0])
```

- [ ] **Step 2:** Run `python3 -m pytest test/test_motion.py -v`. Expected: FAIL, because
  there is no module `stack_cups.motion`.
- [ ] **Step 3: Implement `motion.py`.** Definitions the tests leave open:
  - `base_speed` = sign(r) · clip(√(2·decel·|r|), v_min, v_max), and 0 when |r| < tol.
  - `progress_along` = (xy − start_xy) · (cos yaw, sin yaw).
  - `yaw_for_offset` = sign · arcsin(dx / arm_len), or `None` when
    |dx| > `yaw_reach(arm_len, max_yaw)`.
  - `robust_center`: spread is the largest distance in the xy plane from the median.
- [ ] **Step 4:** Run the tests. Expected: 6 passed.
- [ ] **Step 5:** Commit: `feat: add pure motion math for odometry moves and wrist yaw`

---

### Task 3: `CupRobot` layer on `HelloNode`: startup, params, joint moves

**Files:**
- Create: `stack_cups/stack_cups/robot.py`

**Interfaces:**
- Consumes: `hello_helpers.hello_misc` (`HelloNode`, `get_p1_to_p2_matrix`).
- Produces `class CupRobot(hm.HelloNode)` with:
  - `start(name: str, odom_topic: str = "/odom") -> None`
  - `param(name: str, default)`: returns the value; declares the parameter only if
    `not self.has_parameter(name)`
  - `move(pose: dict[str, float], timeout: float = 30.0) -> None`: raises `RuntimeError`
  - `joint_pos(name: str) -> float | None`: reads `self.joint_state`
  - `wait_joint_still(name: str, still_m: float = 0.001, window_s: float = 0.2, timeout: float = 3.0) -> float | None`
  - `move_joint_precise(joint: str, target: float, limits: tuple[float, float], tol: float = 0.01, tries: int = 5) -> None`
  - `lookup(target: str, source: str) -> tuple[np.ndarray, np.ndarray] | None` → (R, t)
  - `enter_navigation_mode() -> None`

- [ ] **Step 1: Implement the class.** Fixed decisions:
  - **Construction.** `__init__` calls only `hm.HelloNode.__init__(self)`. `HelloNode`
    does not create the ROS node until `main()`.
  - **`start`:**
    - Call `self.main(name, name, wait_for_first_pointcloud=False)`. It runs
      `rclpy.init()` and the spinning thread itself; callers must not call `rclpy.init()`.
    - Then `self.destroy_subscription(self.point_cloud_subscriber)` to save CPU.
    - Then create the `/odom` subscription, the `cmd_vel` publisher on
      `/stretch/cmd_vel`, and the 10 Hz streaming timer (moved from today's node; the
      driver brakes after 0.5 s without commands).
  - **`move`:**
    - Call `h = self.move_to_pose(pose, blocking=False)`; raise if `h is None`.
    - Wait for the goal handle with `timeout`; raise if it is not accepted.
    - Wait for `get_result_async()` with `timeout`; raise unless
      `status == GoalStatus.STATUS_SUCCEEDED`.
    - Every error message includes the pose.
  - **`move_joint_precise`:** today's algorithm, but call `move`, and use
    `wait_joint_still(joint)` instead of `sleep(1.5)` and `sleep(0.3)`.
  - **`lookup`:** `M, _ = hm.get_p1_to_p2_matrix(source, target, self.tf2_buffer, timeout_s=0.5)`;
    return `(M[:3, :3], M[:3, 3])`, or `None`.
  - **`enter_navigation_mode`:** call `self.switch_to_navigation_mode()`, set
    `nav_active = True` and `cmd_v = 0`. HelloNode logs "intended for position mode" on each
    `move`; leave it.
- [ ] **Step 2: Create `sim/check_robot_layer.py`.** It is a ROS script; this step covers
  its first half.
  - `r = CupRobot(); r.start("check_robot_layer"); r.enter_navigation_mode()`
  - Time `move({"joint_lift": 0.8, "wrist_extension": 0.0})`.
  - Time `move_joint_precise("joint_lift", 0.6, (0.0, 1.1))`.
  - Print `lookup("base_link", "link_grasp_center")`.
- [ ] **Step 3:** Run it with the sim running.
  - Expected: both moves finish.
  - The precise lift ends within 1 cm, as shown in the log.
  - Each move takes noticeably less than today's 3 s + 1.5 s per round.
- [ ] **Step 4:** Pin Review Focus 5. In the script, call
  `move({"joint_not_real": 0.1})` inside `try` and print whether `RuntimeError` was raised.
  Expected: raised, and the message names `joint_not_real`.
- [ ] **Step 5:** Commit: `feat: add CupRobot layer on HelloNode with checked moves`

---

### Task 4: `drive_by`, base moves steered by odometry

**Files:**
- Modify: `stack_cups/stack_cups/robot.py`, `sim/check_robot_layer.py`

**Interfaces:**
- Consumes: `motion.base_speed`, `motion.progress_along`.
- Produces:
  - `CupRobot.odom_pose() -> tuple[float, float, float] | None` (x, y, yaw from the latest
    `/odom`)
  - `CupRobot.drive(v: float) -> None`
  - `CupRobot.drive_by(distance: float, *, v_max: float, v_min: float, decel: float, tol: float = 0.003, stall_s: float = 2.0, timeout_s: float = 30.0, max_reversals: int = 3) -> float`:
    returns the progress reached; raises `RuntimeError` on a stall or timeout.

- [ ] **Step 1: Implement `drive_by`.**
  - The loop runs at 20 Hz on `progress_along(start, xy)`. Each tick it calls
    `drive(base_speed(distance - progress, ...))`.
  - It returns once `base_speed` gives 0.
  - It counts sign changes of the command. Once more than `max_reversals` happen, it stops
    and returns.
  - A stall means progress changed by less than 1 mm in `stall_s` while the command was
    not 0.
  - `try/finally: self.drive(0.0)` so an exception or Ctrl+C always leaves the streamed
    command at 0.
- [ ] **Step 2:** Extend `check_robot_layer.py`. Call `drive_by(+0.10)`, then `drive_by(-0.10)`,
  then `drive_by(+0.02)`. For each, print the target, the progress reached, and the time
  taken.
  - Start with `v_max=0.15, v_min=0.06, decel=0.1`.
  - If a move stalls, raise `v_min` in steps of 0.02 until it does not.
  - Record the final `v_min`; it becomes the node default in Task 5.
- [ ] **Step 3:** Run it. Expected: each move ends within 3 mm of its target, in a few
  seconds. The 2 cm move must not stall.
- [ ] **Step 4:** Pin Review Focus 2 and 4.
  - Call `drive_by(0.05, v_min=0.25, tol=0.001, ...)`. Expected: it returns after at most
    3 reversals, without hanging.
  - Start `threading.Timer(1.0, _thread.interrupt_main).start()`, then call `drive_by(0.5)`
    in the main thread inside `try/except KeyboardInterrupt`. Afterwards check
    `r.cmd_v == 0.0`. Expected: 0.0, and the base stops (odom stays still within 0.5 s).
- [ ] **Step 5: Measure the wrist-yaw geometry** in the same script.
  - Read the grasp point x with `lookup("base_link", "link_grasp_center")` at yaw 0, then
    `move({"joint_wrist_yaw": 0.2})` and read it again.
  - Print `sign = np.sign(dx_grasp)` and `arm_len = |dx_grasp| / sin(0.2)`.
  - Record both; they become the defaults for `yaw_sign` and `arm_len` in Task 6.
- [ ] **Step 6:** Commit: `feat: add odometry-steered drive_by to CupRobot`

---

### Task 5: Rewrite `FindCup` on `CupRobot` (single cup, no wrist yaw yet)

**Files:**
- Modify: `stack_cups/stack_cups/cup_finder_controller.py` (rewrite)

**Interfaces:**
- Consumes: everything from Tasks 2–4, plus `perception.locate_cup`, `estimate_table_top`,
  `table_edge_distance`, `plan_reach`.
- Produces:
  - `FindCup(CupRobot)`
  - `measure(full: bool) -> dict | None` (same keys as today)
  - `measure_fresh(samples: int, full: bool) -> dict | None`: adds a `"spread"` key and
    uses `robust_center`
  - `arm_offsets() -> dict | None` (`h`, `c`, `x` as today)
  - `main()` entry point (unchanged name in `setup.py`)

- [ ] **Step 1: Rewrite the node.** Fixed decisions:
  - **Parameters** (`self.param`, with these defaults): `aim_head` True, `head_pan` −1.57,
    `head_tilt` −0.6, `grasp` False, `show_windows` True, `v_max` 0.15, `v_min` (value
    from Task 4), `decel` 0.1, `drive_tol` 0.003, `align_tol` 0.008, `max_drives` 3,
    `fresh_frames` 2, `samples` 5, `max_spread` 0.01, `grasp_height` −1.0 (off), and
    `odom_topic` "/odom".
  - **Frame counter.** The `_on_depth` callback increments `self.depth_count`.
  - **`measure_fresh`:**
    - Wait until `depth_count` ≥ the count at call time + `fresh_frames` (3 s timeout).
    - Collect `samples` measurements, each on a new depth frame.
    - Take `robust_center` of the cup centers. If it is not ok, collect `samples` more once
      and take it over all of them.
    - Table top is the median, edge is the minimum (as today).
  - **Phases:**
    - START waits for color, depth, both camera infos, TF and one `/odom` message. After
      30 s it raises with a message naming the missing items, including
      `odom_topic`'s value.
    - AIM_HEAD as today, using `move`.
    - READY:
      - Call `enter_navigation_mode()` once; no more mode switches after this.
      - `measure_fresh` the cup.
      - `move({"joint_lift": lift, "wrist_extension": 0.0, "joint_gripper_finger_left": GRIPPER_OPEN})`.
      - `move({"joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0, "joint_wrist_roll": 0.0})`.
      - Read `arm_offsets()`.
    - ALIGN, up to `max_drives` times:
      - `dx = measure_fresh(...)["cup"][0] - offs["x"]`.
      - If |dx| < `align_tol`, stop.
      - Otherwise `drive_by(dx, ...)`.
      - After the loop, re-measure. If still ≥ `align_tol`, raise
        `alignment: still {dx:+.3f} m off after {n} drives`.
    - REACH:
      - `measure_fresh(8, full=True)`, then `plan_reach` (unchanged), then
        `move_joint_precise` for lift and then extension.
      - If `grasp_height ≥ 0` and the table top is known, the target grasp z is
        `table_top + grasp_height`.
      - If `grasp`, close the gripper with `move`.
  - **Removed:** CENTER, `switch_mode`, `move_joints`, the old `lookup`, `measure_avg`,
    `_align_coarse`, `_align_fine`, every `SETTLE_S` sleep, and every `time.sleep` > 0.3.
  - **`main()`:** `node = FindCup(); node.start("find_cup"); node.run()`, with today's
    `finally` cleanup (`node.drive(0.0)`, `cv2.destroyAllWindows()`, `rclpy.try_shutdown()`).
    No `rclpy.init()` and no executor of our own.
  - **Failure handling.** `run`'s `except` calls `self.drive(0.0)` and then
    `self.stop_the_robot()` before logging `[FAILED]`.
- [ ] **Step 2:** Check: `grep -n "time.sleep" stack_cups/stack_cups/cup_finder_controller.py`.
  Expected: every value is ≤ 0.3.
- [ ] **Step 3:** Build, then run `find_cup` 3 times from the default pose.
  - Expected: DONE each time, and ALIGN logs at most 3 drives.
  - The timing lines show a total clearly below the Task 1 baseline. Add the times as an
    "after Task 5" column in the baseline timing file.
- [ ] **Step 4:** Run with `grasp:=true`. Expected: the gripper closes on the cup.
- [ ] **Step 5:** Pin Review Focus 1 and 3.
  - Run with `-p odom_topic:=/nope`. Expected: `[FAILED]` within 30 s, with a message that
    contains `/nope`.
  - Run with `-p head_pan:=0.0`, so the cup is out of view. Expected: `[FAILED]` with
    "lost the cup" or "cup not found", and `/stretch/cmd_vel` shows 0
    (`ros2 topic echo --once`).
- [ ] **Step 6:** Commit: `feat(find_cup): rebuild on CupRobot; odometry ALIGN; no CENTER`

---

### Task 6: Wrist yaw covers the last few centimetres

**Files:**
- Modify: `stack_cups/stack_cups/cup_finder_controller.py` (ALIGN only)

**Interfaces:**
- Consumes: `motion.yaw_reach`, `motion.yaw_for_offset`; `arm_len` and `yaw_sign` from
  Task 4, Step 5.
- Produces: new parameters `use_wrist_yaw` (True), `max_yaw` (0.25), `arm_len` (measured),
  `yaw_sign` (measured).

- [ ] **Step 1: Change ALIGN.**
  - When `use_wrist_yaw`, the drive threshold becomes `0.8 * yaw_reach(arm_len, max_yaw)`
    instead of `align_tol`.
  - After the drives, `yaw = yaw_for_offset(dx, arm_len, max_yaw, yaw_sign)`. If `None`,
    do one more `drive_by(dx)` and recompute.
  - `move({"joint_wrist_yaw": yaw})`, then re-read `arm_offsets()`. REACH uses the new
    offsets, because yaw shortens the reach in y.
  - Check `abs(cup_x - offs["x"]) < 0.010`. Otherwise raise
    `wrist yaw left {err:+.3f} m`.
- [ ] **Step 2:** Run with the robot's start pose moved so the cup is about 3 cm off the
  arm line (use `--ros-args -p use_wrist_yaw:=true`).
  - Expected: 0 or 1 drives, then a yaw move.
  - The final logged error is within ±1 cm, and the run reaches DONE.
- [ ] **Step 3:** Run with `grasp:=true` at that offset. Expected: the cup is grasped and
  the fingers do not hit the cup on the way in. If they do, lower the `max_yaw` default
  and record it.
- [ ] **Step 4:** Run with `use_wrist_yaw:=false`. Expected: Task 5 behaviour, still DONE.
- [ ] **Step 5:** Commit: `feat(find_cup): use wrist yaw for the final alignment`

---

### Task 7: Several cups: pick the one nearest the arm line and keep tracking it

**Files:**
- Modify: `stack_cups/stack_cups/perception.py`, `cup_finder_controller.py` (`measure`)
- Test: `stack_cups/test/test_perception.py`

**Interfaces:**
- Produces:
  - `red_mask(bgr, ranges=RED_RANGES) -> np.ndarray`
  - `find_red_blobs(bgr, min_area=MIN_AREA, ranges=RED_RANGES) -> tuple[list[tuple[tuple[int, int, int, int], np.ndarray]], np.ndarray]`:
    a list of (bbox, the blob's own filled mask), largest first, plus the full mask
  - `find_red_blob` stays, returning the first blob's bbox (as today)
  - `choose_cup(centers: list[np.ndarray], grasp_x: float, reach: tuple[float, float], arm_c: float) -> int | None`:
    the index of the smallest |x − grasp_x| among cups whose extension `-y - arm_c` is
    inside `reach`
  - `nearest_cup(centers_odom: list[np.ndarray], target_odom: np.ndarray, max_jump: float = 0.05) -> int | None`
  - New node parameters: `red_ranges` (12 ints: lo/hi HSV × 2 ranges, today's values) and
    `min_area` (100)

- [ ] **Step 1: Write the failing tests**

```python
import numpy as np
from stack_cups import perception as P

def _img_with_squares(*squares):
    img = np.zeros((240, 320, 3), np.uint8)
    for x, y, s in squares:
        img[y:y + s, x:x + s] = (0, 0, 255)  # pure red in BGR
    return img

def test_find_red_blobs_returns_each_blob_largest_first():
    blobs, mask = P.find_red_blobs(_img_with_squares((10, 10, 20), (200, 100, 40)))
    assert [b[0] for b in blobs] == [(200, 100, 40, 40), (10, 10, 20, 20)]
    assert blobs[1][1][15, 15] > 0 and blobs[1][1][110, 210] == 0   # per-blob mask

def test_find_red_blobs_ignores_small():
    blobs, _ = P.find_red_blobs(_img_with_squares((10, 10, 5)))
    assert blobs == []

def test_choose_cup_nearest_arm_line_within_reach():
    centers = [np.array([0.30, -0.70, 0.8]), np.array([0.05, -0.60, 0.8]),
               np.array([0.01, -1.50, 0.8])]          # last one out of reach
    assert P.choose_cup(centers, grasp_x=0.0, reach=(0.0, 0.5), arm_c=0.25) == 1
    assert P.choose_cup([centers[2]], 0.0, (0.0, 0.5), 0.25) is None

def test_nearest_cup_tracks_and_rejects_jumps():
    c = [np.array([1.0, 1.0, 0.8]), np.array([1.2, 1.0, 0.8])]
    assert P.nearest_cup(c, np.array([1.19, 1.01, 0.8])) == 1
    assert P.nearest_cup(c, np.array([2.0, 2.0, 0.8])) is None
```

- [ ] **Step 2:** Run `python3 -m pytest test/test_perception.py -v`. Expected: FAIL, with
  no attribute `find_red_blobs`.
- [ ] **Step 3: Implement the four functions.**
  - Per-blob mask: `cv2.drawContours(np.zeros_like(mask), [c], -1, 255, cv2.FILLED)`.
  - `nearest_cup` compares xy only.
- [ ] **Step 4:** Run the tests. Expected: 4 passed.
- [ ] **Step 5: Change `measure` in the node.**
  - Call `locate_cup` once per blob, with that blob's mask.
  - The first time (`self.target_odom is None`), pick with `choose_cup` and store the
    pick's position in odom (`base_to_odom` with `odom_pose()`).
  - After that, pick with `nearest_cup` against `target_odom`, and update `target_odom`
    each time.
  - Use `nearest_cup` returning `None` as "lost the cup".
  - Read the thresholds from the `red_ranges` and `min_area` parameters.
- [ ] **Step 6:** Add a second red cylinder to the sim table about 15 cm from the first.
  Use the robocasa-free default scene; add a body in `sim/` or describe the edit in the
  README. Run `find_cup`.
  - Expected: it picks the cup nearer the arm line, keeps the same cup through ALIGN, and
    reaches DONE.
- [ ] **Step 7:** Commit: `feat: choose and track the cup nearest the arm line`

---

### Task 8: README and final check

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update the README.**
  - "How it works": phases START, AIM_HEAD, READY, ALIGN (odometry drives plus wrist yaw),
    REACH. Name `CupRobot` as the shared layer built on the starter's `HelloNode`.
  - Parameter table: every new parameter with its default.
  - Helper scripts: add `check_robot_layer.py`.
  - Roadmap:
    - tick 0.1;
    - tick 1.4, with "(~±1 cm, odometry + wrist yaw)";
    - note the 1.1 progress (thresholds as parameters, closest cup);
    - note the 1.6 progress (`grasp_height`);
    - add a line to 0.2 that `v_min` for the sim goes in `sim.yaml`.
- [ ] **Step 2:** Run all unit tests: `python3 -m pytest test/test_motion.py test/test_perception.py -v`.
  Expected: 10 passed.
- [ ] **Step 3:** Do 3 sim runs with `grasp:=true`, with the cup about 5, 20 and 40 cm
  from the arm line.
  - Expected: all DONE, with the cup in the gripper.
  - Add the final timing column to the baseline timing file, and state the speed-up in the
    commit message.
- [ ] **Step 4:** Commit: `docs: describe the faster pick-up; final timings`
