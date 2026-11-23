"""Find the red cup, line the robot up with it, and reach the arm out to it.

Built from the shared pieces, which the other tasks (e.g. placing, task 4) reuse:
  - CupRobot (robot.py): the course starter's HelloNode plus checked moves and
    odometry-steered base moves
  - HeadCamera (head_camera.py): camera frames and fresh, median-of-samples measurements
  - align_arm (align.py): line the arm up with a target by odometry and the wrist yaw
  - perception.py, motion.py: the math, without ROS
The whole run stays in navigation mode: the base takes cmd_vel and every other joint
still takes move_to_pose goals.

Phases (each logged as it starts, with its duration when it ends):
  1. START     - wait for camera images, depth, camera info, TF and /odom
  2. AIM_HEAD  - point the head camera at the table on the robot's arm side
  0. NAVIGATE  - with navigate: stow the arm, Nav2 to nav_goal near the table (map frame)
  2b. SQUARE   - with square_up: fit the table's near edge in depth, turn in place until
                 the base is parallel to it (Nav2 stops within 0.25 rad of the goal)
  3. READY     - measure the closest loose cup (SEARCH first if none is in view: turn
                 the head, then drive along the table), raise the lift above it,
                 straighten the wrist, open the gripper, read the grasp offsets from TF
  4. ALIGN     - measure (stopped, fresh frames), drive exactly the measured offset by
                 odometry, repeat at most `max_drives` times; with use_wrist_yaw, drive
                 only until the wrist yaw can cover the rest, then turn the wrist and
                 check the grasp point (from TF) is within yaw_check_tol of the cup
  5. REACH     - measure the cup, table top and table edge, check the move is safe,
                 lower to cup height, then extend the arm to the cup; with grasp,
                 close the gripper and check the cup comes along
  6. CARRY     - with grasp: lift the cup clear, pull the arm in (a failed grasp or a
                 drop goes to RETRY: open, back off, start again from READY)
  7. DONE / FAILED

Times are on the node's clock: sim time in the MuJoCo sim (which can run much slower
than real time), wall time on the robot. The wall time is logged alongside.

Parameters: stack_cups/params.py, set per platform in config/sim.yaml and config/robot.yaml.
    ros2 launch stack_cups find_cup.launch.py config:=sim             # or config:=robot
    ros2 run stack_cups find_cup --ros-args -p grasp:=true            # also close the gripper
    ros2 run stack_cups find_cup --ros-args -p steps:=aim_head,ready  # only these steps
Stop at any time: Ctrl+C, ros2 topic pub --once /stack_cups/stop std_msgs/msg/Empty, or the
robot's runstop button. With step_confirm, each step waits for Enter or /stack_cups/continue.
"""

import math
import sys
import time

import cv2
import numpy as np
import rclpy

from stack_cups import motion as M
from stack_cups import params
from stack_cups import perception as P
from stack_cups.align import align_arm, odom_note
from stack_cups.head_camera import HeadCamera
from stack_cups.robot import CupRobot, JointBlocked, StopRequested

BASE_FRAME = "base_link"
GRASP_FRAME = "link_grasp_center"
STEPS = ("navigate", "aim_head", "square", "ready", "align", "reach", "carry")


class GraspFailed(RuntimeError):
    """The gripper did not get or keep the cup; worth backing off and trying again."""


class FindCup(CupRobot):
    def __init__(self):
        super().__init__()
        self.phase = "START"
        self.t_start = self.t_phase = None  # (ros time, wall time)
        self.target_odom = None  # xy (odom) of the cup being picked, once measured

    def start(self, name="find_cup"):
        super().start(name)
        self.cfg = {k: self.param(k, v) for k, v in {**params.CUP, **params.FIND_CUP}.items()}
        steps = str(self.cfg["steps"]).replace(" ", "")
        self.steps = set(STEPS) if steps in ("", "all") else set(steps.split(","))
        if self.cfg["show_windows"]:
            self.on_idle = lambda: cv2.waitKey(1)
        self.camera = HeadCamera(self)

    def step(self, name, what):
        """True if step `name` should run (it is in `steps`); with step_confirm, wait for
        the operator first."""
        if name not in self.steps:
            self.log(f"[SKIP] {name} (not in steps)")
            return False
        self.confirm(f"{name}: {what}")
        return True

    # ------------------------------------------------------------------ helpers
    def log(self, text):
        self.get_logger().info(text)

    def set_phase(self, phase, text=""):
        now = (self.now(), time.time())
        if self.t_start is None:
            self.t_start = now
        if self.t_phase is not None and self.phase != "START" and phase != self.phase:
            self.log(f"[timing] {self.phase} took {now[0] - self.t_phase[0]:.1f} s "
                     f"(wall {now[1] - self.t_phase[1]:.1f} s)")
        self.t_phase = now
        self.phase = phase
        self.log(f"[{phase}] {text}" if text else f"[{phase}]")

    def measure(self, full=True):
        """One cup measurement (see perception.measure_cup); {"why": ...} without data."""
        snap, why = self.camera.snapshot(BASE_FRAME)
        if snap is None:
            return {"why": why}
        c, target = self.cfg, None
        pose = self.odom_pose()
        if self.target_odom is not None and pose is not None:
            target = M.odom_to_base(self.target_odom, pose)
        opts = {
            "red": {"hue_width": c["red_hue_width"], "sat_min": c["red_sat_min"],
                    "val_min": c["red_val_min"]},
            "min_area": c["red_min_area"],
            "ignore_taped": c["ignore_taped"],
            "tape": {"hue_min": c["tape_hue_min"], "hue_max": c["tape_hue_max"],
                     "sat_min": c["tape_sat_min"], "val_min": c["tape_val_min"]},
            "track_radius": c["track_radius"],
            "split": {"min_rise": c["split_min_rise"]},
        }
        return P.measure_cup(snap["img"], snap["depth"], snap["K_color"], snap["K_depth"],
                             snap["T_base"], snap["T_cd"], full, target=target, opts=opts)

    def measure_fresh(self, samples=None, full=False):
        """The cup measured while stopped (median of `samples`), with the median radius,
        and with `full` also the median table top and the nearest table edge."""
        c = self.cfg
        m = self.camera.measure_fresh(lambda: self.measure(full), samples or c["samples"],
                                      c["settle_s"], c["sample_dt"], c["max_spread"],
                                      key="cup", on_sample=self.show)
        if m is None:
            return None
        if "odom_point" in m:
            # track this cup from now on (roadmap 1.1): later measurements take the cup
            # nearest here, not the closest one, so another cup cannot take its place
            self.target_odom = np.asarray(m["odom_point"][:2], float)
        found = m["samples"]
        m["radius"] = float(np.median([s["radius"] for s in found]))
        if full:
            heights = [s["cup_height"] for s in found if s.get("cup_height") is not None]
            m["cup_height"] = float(np.median(heights)) if heights else None
            tops = [s["table_top"] for s in found if s.get("table_top") is not None]
            edges = [s["edge"] for s in found if s.get("edge") is not None]
            m["table_top"] = float(np.median(tops)) if tops else None
            m["edge"] = float(np.min(edges)) if edges else None
        return m

    def show(self, m):
        if not self.cfg["show_windows"] or m is None or "img" not in m:
            return
        img = m["img"].copy()
        for other in m.get("cups", []):
            x, y, w, h = other["bbox"]
            cv2.rectangle(img, (x, y), (x + w, y + h), (160, 160, 160), 1)
        if m.get("bbox"):
            x, y, w, h = m["bbox"]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, self.phase, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
        cv2.imshow("red cup", img)
        cv2.imshow("mask", m["mask"])
        cv2.waitKey(1)

    # ------------------------------------------------------------------- phases
    def wait_for_data(self):
        self.set_phase("START",
                       "waiting for camera images, depth, camera info, TF and odometry ...")
        t0, w0 = self.now(), time.time()
        while rclpy.ok():
            missing = self.camera.missing(BASE_FRAME)
            if self.odom_pose() is None:
                missing.append(f"odometry on {self.odom_topic}")
            if not missing:
                # Zeros are normal (nothing within range). Negative values or NaNs mean
                # the GPU renderer is not producing real depth.
                d = self.camera.latest()[1][0]
                nan_frac = float(np.mean(~np.isfinite(d)))
                if nan_frac > 0.05 or np.nanmin(d) < -0.01:
                    raise RuntimeError(
                        f"depth image looks broken ({nan_frac * 100:.0f}% NaN, "
                        f"min {np.nanmin(d):.2f} m). The sim's GPU renderer is not producing "
                        "depth; restart the sim with: export MUJOCO_GL=osmesa")
                return
            odom_missing = self.odom_pose() is None
            if odom_missing and len(missing) == 1 and self.now() - t0 > 5:
                # odometry comes at 30 Hz as soon as the driver runs: don't wait 30 s for it
                raise RuntimeError(f"no odometry on {self.odom_topic} after 5 s (the camera is "
                                   "fine) - is the driver running, and is odom_topic right?")
            if self.now() - t0 > 30 or time.time() - w0 > 120:
                hint = (" - was the sim launched with use_cameras:=true?"
                        if len(missing) > odom_missing else "")
                raise RuntimeError(f"no {', '.join(missing)} after 30 s{hint}")
            self._idle(0.2)  # honours stop requests, keeps the OpenCV windows alive

    def aim_head(self):
        if not self.cfg["aim_head"]:
            return
        pan, tilt = self.cfg["head_pan"], self.cfg["head_tilt"]
        self.set_phase("AIM_HEAD", f"pan={pan:.2f} tilt={tilt:.2f}")
        self.move({"joint_head_pan": pan, "joint_head_tilt": tilt})

    def navigate(self):
        """Roadmap 1.3: stow the arm (Nav2's footprint assumes it), then drive to the pose
        near the cup table (nav_goal_x/y/yaw, map frame; the arm side, base -y, faces the
        table) with Nav2 (BasicNavigator from parcs_stretch_nav2). Nav2 has to be running.
        With nav_set_initial_pose, AMCL is first told the robot is at nav_init_x/y/yaw
        (the sim's map origin is its start pose; on the robot, set it in RViz)."""
        from geometry_msgs.msg import PoseStamped
        from parcs_stretch_nav2.robot_navigator import BasicNavigator, TaskResult

        c = self.cfg
        goal = (c["nav_goal_x"], c["nav_goal_y"], c["nav_goal_yaw"])
        self.set_phase("NAVIGATE", f"stowing the arm, then Nav2 to map ({goal[0]:+.2f}, "
                                   f"{goal[1]:+.2f}, yaw {goal[2]:+.2f})")
        self.stow_the_robot()
        end = self.now() + 30.0
        while not (self.joint_pos("joint_lift") < 0.35
                   and abs(self.joint_pos("wrist_extension")) < 0.02):
            if self.now() > end:
                raise RuntimeError("the arm did not stow in 30 s")
            self._idle(0.2)

        def pose(nav, x, y, yaw):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = nav.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            return p

        self.set_driver_mode("nav2")  # Nav2 drives the base; we stop sending cmd_vel
        nav = BasicNavigator()
        driving = False
        try:
            if not nav.nav_to_pose_client.wait_for_server(timeout_sec=c["nav_timeout"]):
                raise RuntimeError(f"no Nav2 (navigate_to_pose) after {c['nav_timeout']:.0f} s"
                                   " - start it (sim: ros2 launch stack_cups sim_nav.launch.py)")
            if c["nav_set_initial_pose"]:
                nav.setInitialPose(pose(nav, c["nav_init_x"], c["nav_init_y"], c["nav_init_yaw"]))
            nav.waitUntilNav2Active()
            t0 = time.time()
            if not nav.goToPose(pose(nav, *goal)):
                raise RuntimeError("Nav2 rejected the goal")
            driving = True
            while not nav.isTaskComplete():
                if self.stop_reason is not None or time.time() - t0 > c["nav_timeout"]:
                    self.check_stop()
                    raise RuntimeError(f"Nav2 did not arrive in {c['nav_timeout']:.0f} s")
            driving = False
            result = nav.getResult()
            if result != TaskResult.SUCCEEDED:
                raise RuntimeError(f"Nav2 failed to reach the goal ({result})")
            self.log(f"[NAVIGATE] arrived after {time.time() - t0:.1f} s")
        finally:
            if driving:
                # Leaving before Nav2 arrived (a stop, the time limit, an error, Ctrl+C):
                # cancel the goal, or Nav2 keeps driving the base after we exit
                try:
                    nav.cancelTask()
                except Exception:  # noqa: BLE001 - ROS may already be shutting down
                    pass
            nav.destroy_node()
            self.enter_navigation_mode()

    def square_up(self):
        """Roadmap 1.3b: Nav2 stops within 0.25 m and 0.25 rad of its goal. Turn in place
        until the base is parallel to the table's near edge (fitted in depth), so ALIGN's
        straight drives run along it; then, if the edge is more than edge_tol from
        edge_distance (where the cups are in reach), turn to face the table, drive the
        difference, turn back and square up again."""
        c = self.cfg
        self.set_phase("SQUARE", "fitting the table edge in depth")
        edge = self.turn_square()
        off = edge["distance"] - c["edge_distance"]
        if c["edge_distance"] > 0 and abs(off) > c["edge_tol"]:
            self.log(f"[SQUARE] the edge is {off * 100:+.1f} cm from edge_distance "
                     f"{c['edge_distance']:.2f} m: turning to the table to drive it")
            quarter = -math.pi / 2  # the arm side is base -y: turn clockwise to face it
            self.turn_by(quarter, w_max=c["w_max"], w_min=c["w_min"])
            moved = self.drive_by(off, v_max=c["v_max"], v_min=c["v_min"], decel=c["decel"],
                                  tol=c["drive_tol"])
            self.turn_by(-quarter, w_max=c["w_max"], w_min=c["w_min"])
            self.log(f"[SQUARE] drove {moved * 100:+.1f} cm toward the table")
            edge = self.turn_square()
        return edge

    def turn_square(self):
        c = self.cfg
        for i in range(int(c["square_tries"])):
            self._idle(c["settle_s"])  # let the base settle, then use frames from after it
            self.camera.wait_new_depth(2)
            snap, why = self.camera.snapshot(BASE_FRAME)
            if snap is None:
                raise RuntimeError(f"no camera frame to find the table edge: {why}")
            R, t = snap["T_base"]
            pts = P.to_base(P.backproject(snap["depth"], snap["K_depth"], step=2), R, t)
            edge = P.table_edge_line(pts)
            if edge is None:
                raise RuntimeError("could not find the table edge in depth")
            self.log(f"[SQUARE] table edge {edge['distance']:.3f} m away, at "
                     f"{math.degrees(edge['angle']):+.1f} deg (top {edge['top']:.3f} m, "
                     f"{edge['bins']} slices)")
            if abs(edge["angle"]) < c["square_tol"]:
                return edge
            turned = self.turn_by(edge["angle"], w_max=c["w_max"], w_min=c["w_min"])
            self.log(f"[SQUARE] turned {math.degrees(turned):+.1f} deg")
        raise RuntimeError(f"still {math.degrees(edge['angle']):+.1f} deg off the table edge "
                           f"after {int(c['square_tries'])} turns")

    def look(self, pan, tilt):
        self.move({"joint_head_pan": pan, "joint_head_tilt": tilt})
        return self.measure_fresh(self.cfg["search_samples"])

    def find_target(self):
        """Roadmap 1.2: measure the cup; if none is in view, search for one. First turn
        the head (pan steps both ways, then the same farther out with the head tilted
        up), then drive along the heading in steps of search_drive_step, up to
        search_drive_max each way, looking with the default head pose at each stop. A cup
        found off to the side is driven beside the arm before measuring it again with
        the default head pose. Returns that measurement; raises if no cup is found."""
        c = self.cfg
        m = self.measure_fresh()
        if m is not None:
            return m
        first_miss = self.camera.last_miss
        pan0, tilt0 = c["head_pan"], c["head_tilt"]
        step, n = c["search_pan_step"], int(c["search_pan_steps"])
        pans = [pan0 + sgn * k * step for k in range(1, n + 1) for sgn in (1, -1)]
        poses = [(p, tilt0) for p in pans] + [(p, tilt0 + c["search_tilt_up"])
                                              for p in [pan0] + pans]
        if n > 0 or c["search_drive_max"] > 0:
            self.set_phase("SEARCH", f"no cup in view ({first_miss}): turning the head")
        found = None
        for pan, tilt in poses if n > 0 else []:
            found = self.look(pan, tilt)
            if found is not None:
                self.log(f"[SEARCH] cup seen with the head at pan={pan:+.2f} tilt={tilt:+.2f}"
                         f": at base ({found['cup'][0]:+.3f}, {found['cup'][1]:+.3f})")
                break
        if found is None and n > 0:
            self.move({"joint_head_pan": pan0, "joint_head_tilt": tilt0})
        if found is None:
            d, dmax = c["search_drive_step"], c["search_drive_max"]
            k = int(dmax / d + 1e-6) if d > 0 else 0
            stops = [i * d for i in range(1, k + 1)] + [-i * d for i in range(1, k + 1)]
            here = 0.0
            for x in stops:
                moved = self.drive_by(x - here, v_max=c["v_max"], v_min=c["v_min"],
                                      decel=c["decel"], tol=c["drive_tol"])
                here += moved
                self.log(f"[SEARCH] drove to {here:+.2f} m from the start, looking")
                found = self.measure_fresh(c["search_samples"])
                if found is not None:
                    break
        if found is None:
            raise RuntimeError(f"cup not found: {first_miss}; no cup seen while searching")
        # bring the cup beside the arm and look at it with the usual head pose
        self.move({"joint_head_pan": pan0, "joint_head_tilt": tilt0})
        offs = self.arm_offsets(BASE_FRAME, GRASP_FRAME)
        arm_x = offs["x"] if offs else 0.0
        dx = float(found["cup"][0] - arm_x)
        if abs(dx) > c["align_tol"]:
            moved = self.drive_by(dx, v_max=c["v_max"], v_min=c["v_min"], decel=c["decel"],
                                  tol=c["drive_tol"])
            self.log(f"[SEARCH] drove {moved * 100:+.1f} cm to bring the cup beside the arm")
        m = self.measure_fresh()
        if m is None:
            raise RuntimeError(f"cup seen while searching but not from beside it: "
                               f"{self.camera.last_miss}")
        return m

    def ready_arm(self):
        self.set_phase("READY", "measuring the cup, raising the lift, straightening the wrist")
        m = self.find_target()
        last = m["samples"][-1]
        placed = sum(P.in_area(c["cup"][:2], last.get("tape_area")) for c in last["cups"])
        self.log(f"cup at base ({m['cup'][0]:.3f}, {m['cup'][1]:.3f}, {m['cup'][2]:.3f}) m"
                 f"{odom_note(m)}; {len(last['cups'])} red cups in view, {placed} inside the "
                 "tape: picking the closest loose one")
        offs = self.arm_offsets(BASE_FRAME, GRASP_FRAME)
        h = offs["h"] if offs else 0.115
        lift = float(np.clip(m["cup"][2] + self.cfg["ready_above_cup"] - h, 0.3, 1.05))
        # Lift first (arm stays retracted), then straighten the wrist up there
        self.move({"joint_lift": lift, "wrist_extension": 0.0})
        self.move({"joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0,
                   "joint_wrist_roll": 0.0, "joint_gripper_finger_left": self.cfg["gripper_open"]})
        for joint in ("joint_lift", "joint_wrist_yaw", "joint_wrist_pitch"):
            self.wait_joint_still(joint)
        offs = self.arm_offsets(BASE_FRAME, GRASP_FRAME)
        if offs is None:
            raise RuntimeError(f"could not read {GRASP_FRAME} from TF")
        # The lift above was set with the grasp height before the wrist was straight. A
        # wrist pitched down (it droops while runstopped) puts the grasp point ~0.24 m
        # lower, so the straightened gripper ended that much too high: right between the
        # head camera and the cup, and ALIGN lost the cup (real robot). Set it again.
        fixed = float(np.clip(m["cup"][2] + self.cfg["ready_above_cup"] - offs["h"], 0.3, 1.05))
        if abs(fixed - lift) > 0.01:
            self.log(f"[READY] lift {lift:.3f} -> {fixed:.3f} m now that the wrist is straight")
            self.move({"joint_lift": fixed})
            self.wait_joint_still("joint_lift")
            offs = self.arm_offsets(BASE_FRAME, GRASP_FRAME) or offs
        self.log(f"arm offsets: grasp is {offs['h']:.3f} m above the lift, "
                 f"{offs['c']:.3f} m beyond the extension, at x={offs['x']:.3f}")
        return offs

    def align(self, offs):
        """Returns the grasp offsets to reach with (changed if the wrist was turned)."""
        self.set_phase("ALIGN", "measure, drive the measured offset by odometry, repeat")
        return align_arm(self, self.measure_fresh, offs, self.cfg, key="cup",
                         why_lost=lambda: self.camera.last_miss)

    def reach(self, offs):
        self.set_phase("REACH", "measuring cup and table with depth")
        m = self.measure_fresh(8, full=True)
        if m is None:
            raise RuntimeError(f"lost the cup before reaching: {self.camera.last_miss}")
        cup, top, edge = m["cup"], m["table_top"], m["edge"]
        self.log(f"cup center ({cup[0]:.3f}, {cup[1]:.3f}, {cup[2]:.3f}) m, "
                 f"radius {m['radius'] * 100:.1f} cm; table top "
                 f"{'unknown' if top is None else f'{top:.3f} m'}; table edge "
                 f"{'unknown' if edge is None else f'{edge:.3f} m'} from base center")
        self.check_cup_height(m.get("cup_height"))
        target = np.array(cup, dtype=float)
        if self.cfg["grasp_height"] >= 0 and top is not None:
            target[2] = top + self.cfg["grasp_height"]
            self.log(f"grasping {self.cfg['grasp_height']:.3f} m above the table top")
        past = self.cfg["grasp_reach_past"]
        plan, why = P.plan_reach(target, m["radius"], top, edge, offs, approach_gap=-past)
        if plan is None:
            raise RuntimeError(f"not safe to reach: {why}")
        self.log(f"plan: lift {plan['lift']:.3f} m (grasp height {plan['grasp_z']:.3f} m), "
                 f"extend {plan['extension']:.3f} m ({past * 100:.1f} cm past the measured "
                 f"center)")
        opening = self.fit_opening(m["radius"])
        if opening is not None:
            self.move({"joint_gripper_finger_left": opening})
        try:
            self.move_joint_precise("joint_lift", plan["lift"], (0.0, 1.1))
            self.move_joint_precise("wrist_extension", plan["extension"], (0.0, 0.52))
        except JointBlocked as e:
            # e.g. a finger hit the cup: closing here would knock it over or miss it
            raise GraspFailed(f"the arm was blocked before reaching the cup: {e}") from None
        if self.cfg["grasp"]:
            c = self.cfg
            finger, contact = self.close_gripper(c["gripper_closed"], c["gripper_squeeze"],
                                                 c["gripper_max_effort"])
            self.check_grasp(cup, finger, contact)
            return {"cup": cup, "table_top": top}
        return None

    def check_cup_height(self, measured):
        """Roadmap 1.5: compare the measured cup height (highest red point above the
        table top) with cup_height. Off by more than cup_height_tol: a warning (wrong
        cup_height, a cup lying down or stacked, or a cup partly hidden); with
        cup_height_strict, a failure."""
        c = self.cfg
        if measured is None:
            self.log("[REACH] cup height not measured (no table top)")
            return
        err = measured - c["cup_height"]
        text = (f"cup height measured {measured * 100:.1f} cm, cup_height says "
                f"{c['cup_height'] * 100:.1f} cm ({err * 100:+.1f} cm)")
        if abs(err) <= c["cup_height_tol"]:
            self.log(f"[REACH] {text}")
            return
        if c["cup_height_strict"]:
            raise RuntimeError(f"{text}: more than cup_height_tol "
                               f"({c['cup_height_tol'] * 100:.1f} cm)")
        self.get_logger().warn(f"[REACH] {text}: check cup_height, or is the cup lying "
                               "down, stacked or partly hidden?")

    def fit_opening(self, radius):
        """Roadmap 1.6: open the fingers only as wide as the cup needs (its measured width
        plus grasp_open_margin each side), at most gripper_open, so they clear things
        next to the cup. Finger position for a width w: gripper_fit_offset +
        gripper_fit_per_m * w (fitted from where the sim fingers stopped on a 4 cm
        cylinder and a 7.75 cm cup). None = keep gripper_open (gripper_fit_per_m 0)."""
        c = self.cfg
        if c["gripper_fit_per_m"] <= 0:
            return None
        width = 2.0 * radius + 2.0 * c["grasp_open_margin"]
        pos = c["gripper_fit_offset"] + c["gripper_fit_per_m"] * width
        pos = float(np.clip(pos, c["gripper_closed"], c["gripper_open"]))
        self.log(f"[REACH] opening the gripper to {pos:+.3f} for a {radius * 200:.1f} cm cup "
                 f"(+{c['grasp_open_margin'] * 100:.1f} cm each side)")
        return pos

    def pick_attempt(self):
        """READY, ALIGN, REACH and (with grasp) CARRY, as far as `steps` allows."""
        self.target_odom = None  # READY picks the closest loose cup afresh
        offs = None
        if self.step("ready", "raise the lift above the cup, straighten the wrist, "
                              "open the gripper"):
            offs = self.ready_arm()
        offs = offs or self.arm_offsets(BASE_FRAME, GRASP_FRAME)
        if offs is None:
            raise RuntimeError(f"could not read {GRASP_FRAME} from TF")
        if self.step("align", "drive along the table until the arm lines up with the cup"):
            offs = self.align(offs)
        grasp = None
        if self.step("reach", "lower to the cup and extend the arm"
                              + (", then close the gripper" if self.cfg["grasp"] else "")):
            grasp = self.reach(offs)
        if grasp is not None and self.step("carry", "lift the cup clear and pull the arm in"):
            self.carry(grasp)

    def check_grasp(self, cup, finger, contact):
        """Roadmap 1.8: raise (so the run ends FAILED) if the gripper is not holding the cup.

        1. The fingers must have stopped on something (not closed on nothing).
        2. Lift by grasp_check_lift, which is more than the distance from the grasp point
           to the cup's top: a cup that stayed on the table is then below the fingers, so
           they close further. A shorter lift fails: the fingers slide up a cup that
           stays behind and still touch it (seen in the sim).
        The head camera's view is only logged: the arm hides part of the cup, which moves
        its measured center up even when the cup stays put (seen in the sim)."""
        c = self.cfg
        joint = "joint_gripper_finger_left"
        if not contact:
            raise GraspFailed(f"the gripper closed on nothing (fingers at {finger:+.3f})")
        lift0 = self.joint_pos("joint_lift")
        self.move({"joint_lift": lift0 + c["grasp_check_lift"]})
        after = self.wait_joint_still(joint) or self.joint_pos(joint)
        m = self.measure_fresh()
        seen = (f"camera: {self.camera.last_miss}" if m is None else
                f"camera: cup center rose {float(m['cup'][2] - cup[2]) * 100:+.1f} cm")
        self.log(f"[REACH] grasp check after a {c['grasp_check_lift'] * 100:.0f} cm lift: "
                 f"fingers {finger:+.3f} -> {after:+.3f} ({seen})")
        if finger - after > c["grasp_slip_tol"]:
            raise GraspFailed(f"the cup was not lifted: the fingers closed from {finger:+.3f} "
                              f"to {after:+.3f} while lifting (it slipped or stayed behind)")

    def carry(self, grasp):
        """Roadmap 1.9: lift the cup clear of the table, pull the arm in, keep it upright.

        The lift rises until the cup's bottom is carry_clearance above the table top (the
        cup's bottom is estimated from its measured center and cup_height). The wrist stays
        level (pitch and roll 0), so the cup stays upright, and turns to carry_wrist_yaw.
        Raises GraspFailed if the fingers close further on the way (the cup dropped)."""
        c = self.cfg
        joint = "joint_gripper_finger_left"
        self.set_phase("CARRY", "lifting the cup clear of the table and pulling the arm in")
        finger = self.wait_joint_still(joint) or self.joint_pos(joint)
        lift = self.joint_pos("joint_lift")
        top = grasp["table_top"]
        if top is not None:
            # the cup stood on the table; the lift has risen grasp_check_lift since
            bottom = top + c["grasp_check_lift"]
            lift += max(0.0, top + c["carry_clearance"] - bottom)
        lift = min(lift, 1.05)
        self.move({"joint_lift": lift})
        self.move({"wrist_extension": 0.0})
        self.move({"joint_wrist_yaw": c["carry_wrist_yaw"], "joint_wrist_pitch": 0.0,
                   "joint_wrist_roll": 0.0})
        after = self.wait_joint_still(joint) or self.joint_pos(joint)
        self.log(f"[CARRY] lift {self.joint_pos('joint_lift'):.3f} m, extension "
                 f"{self.joint_pos('wrist_extension'):.3f} m, "
                 f"fingers {finger:+.3f} -> {after:+.3f}")
        if finger - after > c["grasp_slip_tol"]:
            raise GraspFailed(f"dropped the cup while carrying: the fingers closed from "
                              f"{finger:+.3f} to {after:+.3f}")

    def back_off(self):
        """Roadmap 1.10: after a failed grasp, open the gripper and pull the arm in so the
        next attempt starts clear of the cup (READY then sets the lift and wrist)."""
        self.move({"joint_gripper_finger_left": self.cfg["gripper_open"]})
        self.move({"wrist_extension": 0.0})

    def run(self):
        try:
            unknown = self.steps - set(STEPS)
            if unknown:
                raise RuntimeError(f"unknown steps {sorted(unknown)}: use all, or some of "
                                   f"{','.join(STEPS)}")
            self.wait_for_data()
            self.enter_navigation_mode()
            c = self.cfg
            if c["navigate"] and self.step("navigate", "stow the arm, Nav2 to the table"):
                self.navigate()
            if c["aim_head"] and self.step("aim_head", "point the head at the table"):
                self.aim_head()
            if c["square_up"] and self.step("square", "turn square to the table edge"):
                self.square_up()
            # Retry a failed grasp from READY (roadmap 1.10); READY re-measures the cup
            tries = (max(1, int(self.cfg["grasp_tries"]))
                     if self.cfg["grasp"] and "ready" in self.steps else 1)
            for attempt in range(1, tries + 1):
                try:
                    self.pick_attempt()
                    break
                except GraspFailed as e:
                    if attempt == tries:
                        raise GraspFailed(f"{e} (attempt {attempt} of {tries})") from None
                    self.set_phase("RETRY", f"attempt {attempt} of {tries} failed: {e}; "
                                            "opening the gripper and backing off")
                    self.back_off()
            ran = [s for s in STEPS if s in self.steps]
            done = "gripper is at the cup"
            if "reach" in self.steps and self.cfg["grasp"]:
                done = ("carrying the cup" if "carry" in self.steps else "holding the cup")
            self.set_phase("DONE", done if "reach" in self.steps else f"ran {','.join(ran)}")
        except StopRequested as e:
            self.safe_stop()
            self.set_phase("STOPPED", str(e))
        except Exception as e:  # noqa: BLE001 - report any failure and stop safely
            self.safe_stop()
            self.set_phase("FAILED", str(e))
        self.log(f"[timing] total {self.now() - self.t_start[0]:.1f} s "
                 f"(wall {time.time() - self.t_start[1]:.1f} s)")
        # Keep the windows alive until Ctrl+C; with no windows, exit (so launch ends too)
        while self.cfg["show_windows"] and rclpy.ok():
            self.show(self.measure(full=False))
            time.sleep(0.1)


def main():
    node = FindCup()
    try:
        node.start("find_cup")
        node.run()
    except KeyboardInterrupt:
        if hasattr(node, "cmd_vel_pub"):
            node.get_logger().error("STOP: Ctrl+C")
    finally:
        if hasattr(node, "cmd_vel_pub"):
            if node.phase == "DONE":
                # Nothing is moving. Not safe_stop: the driver's /stop_the_robot also tells
                # the gripper to stay where it is (sim and real driver), which drops the
                # squeeze on a held cup, so the cup could slip out after the run
                try:
                    node.drive(0.0)
                except Exception:  # noqa: BLE001 - ROS may already be shutting down
                    pass
            else:
                node.safe_stop()  # ROS is still up here (see CupRobot.start)
        cv2.destroyAllWindows()
        rclpy.try_shutdown()
    # exit code for scripts: 0 only when every requested step finished
    sys.exit(0 if node.phase == "DONE" else 1)


if __name__ == "__main__":
    main()
