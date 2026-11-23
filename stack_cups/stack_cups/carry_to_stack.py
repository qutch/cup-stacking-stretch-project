"""Carry the cup in the gripper to the stacking area and park lined up with the taped area's
center (roadmap task 2).

Starts with a cup held in the carry pose, where task 1 leaves it (roadmap 1.9; in the sim,
sim/handoff_cup.py stands in for task 1's pick-up). Phases (each logged as it starts):
  1. START     - camera, TF, odometry and joint states; is a cup in the gripper?
  2. LOCALIZE  - (2.2) if AMCL has no pose yet, give it start_pose (where the robot was when
                 its driver started) combined with the odometry since then
  3. NAVIGATE  - (2.3-2.5) Nav2 to stacking_pose, with an outline that includes the arm and
                 the cup; stops at once if the grip changes (the cup slipped or fell)
  4. FIND_TAPE - (2.1) point the head where the tape should be and find it
  5. SQUARE    - (2.6) turn in place until the base is parallel to the tape's edge; if the
                 taped area's center is not about reach_target of arm extension away
                 (Nav2 stops up to ~25 cm off), sidestep and square up again
  6. ALIGN     - (2.6) drive along the table until the cup is within the wrist yaw's reach
                 of the line through the taped area's center, then turn the wrist to put
                 it on the line (find_cup's align_arm), measuring the tape fresh (stopped,
                 head on it) before each drive and after the turn
  7. DONE / FAILED / STOPPED; exit code 0 only on DONE
The robot ends where task 4 places from: the cup on the line through the area's center
that the arm reaches along, the base parallel to the tape, the cup still in the carry pose
except for the wrist yaw (at most max_yaw, the cup still upright).

Needs Nav2 for LOCALIZE and NAVIGATE (in the sim: ros2 launch stack_cups sim_nav.launch.py).
Parameters: stack_cups/params.py (CARRY), set per platform in config/sim.yaml and robot.yaml.
    ros2 launch stack_cups carry_to_stack.launch.py config:=sim
    ros2 launch stack_cups carry_to_stack.launch.py config:=sim carry_steps:=find_tape,square,align
Stop at any time: Ctrl+C, ros2 topic pub --once /stack_cups/stop std_msgs/msg/Empty, or the
robot's runstop button. With step_confirm, each step waits for /stack_cups/continue.
"""

import math
import sys
import time

import numpy as np
import rclpy

from stack_cups import carry as C
from stack_cups import motion as M
from stack_cups import params
from stack_cups import tape as TP
from stack_cups.align import align_arm, odom_note
from stack_cups.head_camera import HeadCamera
from stack_cups.nav import Nav2
from stack_cups.robot import CupRobot, StopRequested

BASE_FRAME = "base_link"
GRASP_FRAME = "link_grasp_center"
EYE_FRAME = "camera_color_optical_frame"  # where the head camera looks from
GRIPPER = "joint_gripper_finger_left"
STEPS = ("localize", "navigate", "find_tape", "square", "align")
PAN_LIMITS, TILT_LIMITS = (-3.9, 1.5), (-1.4, 0.4)  # inside the head joints' ranges


def deg(rad):
    return math.degrees(rad)


class CarryToStack(CupRobot):
    def __init__(self):
        super().__init__()
        self.phase = "START"
        self.t_start = self.t_phase = None  # (ros time, wall time)
        self.grip0 = None  # finger position holding the cup
        self.tape_at = None  # last tape center, odom xyz (to point the head at)

    def start(self, name="carry_to_stack"):
        super().start(name)
        defaults = {**params.CUP, **params.FIND_CUP, **params.CARRY}
        self.cfg = {k: self.param(k, v) for k, v in defaults.items()}
        steps = str(self.cfg["carry_steps"]).replace(" ", "")
        self.steps = set(STEPS) if steps in ("", "all") else set(steps.split(","))
        self.camera = HeadCamera(self)
        self.nav = Nav2(self)

    # ------------------------------------------------------------------ helpers
    def log(self, text):
        self.get_logger().info(text)

    def step(self, name, what):
        """True if step `name` should run; with step_confirm, wait for the operator first."""
        if name not in self.steps:
            self.log(f"[SKIP] {name} (not in carry_steps)")
            return False
        self.confirm(f"{name}: {what}")
        return True

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

    # ----------------------------------------------------------------- the cup
    def safe_stop(self):
        """Stop the base and any arm move, but keep squeezing the cup: the driver's
        /stop_the_robot also tells the gripper to hold where it is, which drops the grip
        force (seen in the sim) and could let the cup fall."""
        if self.grip0 is None:
            super().safe_stop()
            return
        try:
            self.drive(0.0)
        except Exception:  # noqa: BLE001 - ROS may already be shutting down
            pass
        goal = self._goal
        if goal is not None:
            try:
                goal.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    def check_cup(self, where):
        """Raise if the cup fell out of the gripper or the gripper let go of it."""
        if not self.cfg["require_cup"] or self.grip0 is None:
            return
        now = self.joint_pos(GRIPPER)
        c = self.cfg
        if now is not None and C.grip_lost(self.grip0, now, c["gripper_closed"], c["held_min"],
                                           c["grip_tol"]):
            raise RuntimeError(f"lost the cup {where}: the fingers went from {self.grip0:+.3f} "
                               f"to {now:+.3f} (fully closed is {c['gripper_closed']:+.3f})")

    def cup_xy(self):
        """The held cup's center in base_link xy (the grasp point), or None."""
        T = self.lookup(BASE_FRAME, GRASP_FRAME)
        return None if T is None else T[1][:2]

    # -------------------------------------------------------------------- tape
    def measure_tape(self):
        snap, why = self.camera.snapshot(BASE_FRAME)
        if snap is None:
            return {"why": why}
        c = self.cfg
        colour = {"hue_min": c["tape_hue_min"], "hue_max": c["tape_hue_max"],
                  "sat_min": c["tape_sat_min"], "val_min": c["tape_val_min"]}
        return TP.locate_tape(snap["img"], snap["depth"], snap["K_color"], snap["K_depth"],
                              snap["T_base"], snap["T_cd"], colour=colour)

    def look_at_tape(self):
        """Point the head at the tape's last known center, or the first look from params."""
        pan, tilt = self.cfg["tape_head_pan"], self.cfg["tape_head_tilt"]
        eye = self.lookup(BASE_FRAME, EYE_FRAME)
        if self.tape_at is not None and eye is not None:
            xy = M.odom_to_base(self.tape_at[:2], self.odom_pose())
            pan, tilt = C.look_at(eye[1], (xy[0], xy[1], self.tape_at[2]))
        self.move({"joint_head_pan": float(np.clip(pan, *PAN_LIMITS)),
                   "joint_head_tilt": float(np.clip(tilt, *TILT_LIMITS))})

    def tape_fresh(self):
        """The tape measured while stopped, with the head on it: median center ("tape",
        base_link) and median yaw over the samples, or None."""
        self.look_at_tape()
        c = self.cfg
        m = self.camera.measure_fresh(self.measure_tape, c["samples"], c["settle_s"],
                                      c["sample_dt"], c["max_spread"], key="tape")
        if m is None:
            return None
        m["yaw"] = float(np.median([s["yaw"] for s in m["samples"]]))
        if "odom_point" in m:
            self.tape_at = np.array(m["odom_point"], dtype=float)
        return m

    def describe(self, m, offs):
        t = m["tape"]
        return (f"tape center at base ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:.3f}) m, "
                f"{(t[0] - offs['x']) * 100:+.1f} cm from the cup's line; its edge "
                f"{deg(m['yaw']):+.1f} deg from the base; area {m['length']:.2f} x "
                f"{m['width']:.2f} m{odom_note(m, 'tape')}")

    # ------------------------------------------------------------------- phases
    def wait_for_data(self):
        self.set_phase("START", "waiting for camera, TF, odometry and joint states ...")
        t0, w0 = self.now(), time.time()
        while rclpy.ok():
            missing = self.camera.missing(BASE_FRAME)
            if self.odom_pose() is None:
                missing.append(f"odometry on {self.odom_topic}")
            if self.joint_pos(GRIPPER) is None:
                missing.append("joint states")
            if self.lookup(BASE_FRAME, GRASP_FRAME) is None:
                missing.append(f"TF {GRASP_FRAME}")
            if not missing:
                break
            if self.now() - t0 > 30 or time.time() - w0 > 120:
                raise RuntimeError(f"no {', '.join(missing)} after 30 s - was the sim launched "
                                   "with its head camera (ros2 launch stack_cups sim.launch.py)?")
            time.sleep(0.2)
        grip = self.wait_joint_still(GRIPPER, still_m=0.002, window_s=0.3)
        closed = self.cfg["gripper_closed"]
        if not self.cfg["require_cup"]:
            self.log(f"[START] not checking for a cup (require_cup false); fingers at {grip:+.3f}")
            return
        if not C.is_holding(grip, closed, self.cfg["held_min"]):
            raise RuntimeError(f"no cup in the gripper (fingers at {grip:+.3f}, fully closed is "
                               f"{closed:+.3f}). Task 1 picks one up; in the sim run "
                               "sim/handoff_cup.py first")
        self.grip0 = grip
        xy = self.cup_xy()
        self.log(f"[START] holding a cup: fingers at {grip:+.3f}; the cup is at base "
                 f"({xy[0]:+.3f}, {xy[1]:+.3f})")

    def localize(self):
        # Nav2's planner only finishes starting once AMCL has a pose (its costmap waits for
        # map -> base_link), so this must not wait for the rest of Nav2 first
        self.set_phase("LOCALIZE", "making sure AMCL knows where the robot is")
        pose, given = self.nav.localize(self.cfg["start_pose"])
        how = (f"gave AMCL start_pose {list(self.cfg['start_pose'])} + odometry"
               if given else "AMCL already had a pose")
        self.log(f"[LOCALIZE] {how}; robot at map ({pose[0]:+.3f}, {pose[1]:+.3f}, "
                 f"{deg(pose[2]):+.1f} deg)")

    def navigate(self):
        c = self.cfg
        goal = [float(v) for v in c["stacking_pose"]]
        if not c["poses_checked"]:
            raise RuntimeError("stacking_pose is not measured on this map: measure it, then "
                               "set poses_checked:=true (config/robot.yaml)")
        self.set_phase("NAVIGATE", f"Nav2 to the stacking area at map {goal}")
        if self.nav.map_pose() is None:
            raise RuntimeError("AMCL has no pose: run the localize step first")
        if not self.nav.wait_ready(60.0):
            raise RuntimeError("Nav2 is not accepting goals (in the sim: ros2 launch "
                               "stack_cups sim_nav.launch.py)")
        cup = self.cup_xy() if self.cfg["require_cup"] else None
        if cup is not None:
            outline = C.carry_outline(c["robot_radius"], cup, c["cup_rim_diameter"] / 2.0,
                                      c["carry_margin"])
            if self.nav.set_outline(outline):
                self.log(f"[NAVIGATE] Nav2's outline now includes the arm and the cup "
                         f"(out to y={min(y for _, y in outline):+.2f} m)")
            else:
                self.get_logger().warn("Nav2's costmaps did not confirm the carry outline")
        try:
            self.nav.go_to(goal, c["nav_timeout"], check=lambda: self.check_cup("while driving"),
                           name="the stacking area")
        finally:
            if cup is not None:
                try:  # back to the stowed outline for whoever uses Nav2 next
                    self.nav.set_outline(C.circle(0.0, 0.0, c["robot_radius"]), check_s=1.0)
                except Exception:  # noqa: BLE001 - e.g. a stop: it was published anyway
                    pass
        pose = self.nav.map_pose()
        if pose is not None:
            off = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
            self.log(f"[NAVIGATE] arrived: map ({pose[0]:+.3f}, {pose[1]:+.3f}, "
                     f"{deg(pose[2]):+.1f} deg), {off * 100:.0f} cm from the goal")
        self.check_cup("after driving")

    def find_tape(self, offs):
        self.set_phase("FIND_TAPE", "pointing the head at the stacking area")
        m = self.tape_fresh()
        if m is None:
            raise RuntimeError(f"no tape found: {self.camera.last_miss}")
        m = self.tape_fresh() or m  # a second look with the head on the center
        self.log(f"[FIND_TAPE] {self.describe(m, offs)}")
        return m

    def square(self, m, offs):
        self.set_phase("SQUARE", "turning parallel to the tape's edge")
        c = self.cfg
        tries = int(c["tape_square_tries"])
        for turns in range(tries + 1):
            yaw = m["yaw"]
            self.log(f"[SQUARE] the tape's edge is {deg(yaw):+.1f} deg from the base "
                     f"({turns} turns so far)")
            if abs(yaw) < c["tape_square_tol"]:
                return m
            if turns == tries:
                break
            turned = self.turn_by(yaw, w_max=c["w_max"], w_min=c["w_min"], decel=c["w_decel"],
                                  tol=c["tape_square_tol"] / 4.0)
            self.log(f"[SQUARE] turn {turns + 1}: asked {deg(yaw):+.1f} deg, odometry says "
                     f"{deg(turned):+.1f} deg")
            self.check_cup("while turning")
            m = self.tape_fresh()
            if m is None:
                raise RuntimeError(f"lost the tape while squaring up: {self.camera.last_miss}")
        raise RuntimeError(f"still {deg(m['yaw']):+.1f} deg off the tape's edge after "
                           f"{tries} turns")

    def sidestep(self, dist):
        """Move the base `dist` meters toward its arm side (-y; negative: away from it)
        and keep its heading: a differential-drive base cannot move sideways, so turn 90
        degrees, drive, and turn back. The cup swings through a quarter circle behind the
        base on the arm side."""
        c = self.cfg
        turn = dict(w_max=c["w_max"], w_min=c["w_min"], decel=c["w_decel"],
                    tol=c["tape_square_tol"] / 4.0)
        heading = self.odom_pose()[2]
        self.turn_by(-math.pi / 2.0, **turn)
        self.check_cup("while turning")
        moved = self.drive_by(dist, v_max=c["v_max"], v_min=c["v_min"], decel=c["decel"],
                              tol=c["drive_tol"])
        self.check_cup("while sidestepping")
        self.turn_by(C.wrap(heading - self.odom_pose()[2]), **turn)
        self.check_cup("while turning")
        return moved

    def fix_reach(self, m, offs):
        """Sidestep if the tape's center is not about reach_target of arm extension away.
        Returns the tape measured afterwards (or m) and whether the base moved."""
        c = self.cfg
        ext = C.reach_for(m["tape"][1], offs["c"])
        off = ext - c["reach_target"]
        if abs(off) <= c["reach_tol"]:
            return m, False
        self.log(f"[SQUARE] the tape's center needs {ext:.2f} m of arm extension (want "
                 f"{c['reach_target']:.2f} +/- {c['reach_tol']:.2f}): sidestepping "
                 f"{off * 100:+.0f} cm")
        moved = self.sidestep(off)
        self.log(f"[SQUARE] sidestepped {moved * 100:+.1f} cm")
        m = self.tape_fresh()
        if m is None:
            raise RuntimeError(f"lost the tape after sidestepping: {self.camera.last_miss}")
        return m, True

    def check_reach(self, m, offs):
        """The arm extension the taped area's center needs; raises if it is out of reach."""
        ext = C.reach_for(m["tape"][1], offs["c"])
        if not 0.0 <= ext <= self.cfg["max_reach"]:
            raise RuntimeError(f"the tape's center is {-m['tape'][1]:.2f} m out on the arm side: "
                               f"the arm would need {ext:+.2f} m of extension "
                               f"(0 to {self.cfg['max_reach']:.2f})")
        return ext

    def align(self, offs):
        """Returns the grasp offsets after aligning (they change when the wrist turns)."""
        self.set_phase("ALIGN", "driving along the table until the cup lines up with the "
                                "tape's center")
        # Like find_cup: the base gets within the wrist yaw's reach and the wrist covers the
        # last few cm. Base drives alone did not settle: at the sim's full base speed, short
        # drives ended 1-3 cm from their target and 3 drives were not enough (2 of 2 runs).
        # The wrist turns about a vertical axis, so the cup stays upright.
        return align_arm(self, self.tape_fresh, offs, self.cfg, key="tape",
                         why_lost=lambda: self.camera.last_miss)

    def run(self):
        """Returns True on DONE."""
        try:
            unknown = self.steps - set(STEPS)
            if unknown:
                raise RuntimeError(f"unknown carry_steps {sorted(unknown)}: use all, or some "
                                   f"of {','.join(STEPS)}")
            self.wait_for_data()
            self.enter_navigation_mode()
            if self.step("localize", "give AMCL the robot's pose if it has none"):
                self.localize()
            if self.step("navigate", "drive to the stacking area with Nav2"):
                self.navigate()
            offs = self.arm_offsets(BASE_FRAME, GRASP_FRAME)
            if offs is None:
                raise RuntimeError(f"could not read {GRASP_FRAME} from TF")
            m = None
            if self.step("find_tape", "point the head at the stacking area and find the tape"):
                m = self.find_tape(offs)
            if self.step("square", "turn in place until the base is parallel to the tape, "
                                   "and sidestep if the tape is not at a comfortable reach"):
                m = self.square(m or self.find_tape(offs), offs)
                m, moved = self.fix_reach(m, offs)
                if moved:
                    m = self.square(m, offs)
            if self.step("align", "drive along the table until the cup lines up with the "
                                  "tape's center"):
                self.check_reach(m or self.find_tape(offs), offs)
                offs = self.align(offs)
            if self.steps & {"find_tape", "square", "align"}:
                m = self.tape_fresh()
                if m is None:
                    raise RuntimeError(f"lost the tape at the end: {self.camera.last_miss}")
                ext = self.check_reach(m, offs)
                self.check_cup("at the end")
                self.log(f"[FINAL] {self.describe(m, offs)}; the center is {ext:.3f} m of "
                         f"arm extension away")
                dx = m["tape"][0] - offs["x"]
                self.set_phase("DONE", f"parked: the cup is {dx * 100:+.1f} cm from the tape "
                                       f"center's line, the base {deg(m['yaw']):+.1f} deg "
                                       f"from the tape's edge")
            else:
                self.set_phase("DONE", f"ran {','.join(s for s in STEPS if s in self.steps)}")
            return True
        except StopRequested as e:
            self.safe_stop()
            self.set_phase("STOPPED", str(e))
        except Exception as e:  # noqa: BLE001 - report any failure and stop safely
            self.safe_stop()
            self.set_phase("FAILED", str(e))
        finally:
            if self.t_start is not None:
                self.log(f"[timing] total {self.now() - self.t_start[0]:.1f} s "
                         f"(wall {time.time() - self.t_start[1]:.1f} s)")
        return False


def main():
    node = CarryToStack()
    done = False
    try:
        node.start("carry_to_stack")
        done = node.run()
    except KeyboardInterrupt:
        if hasattr(node, "cmd_vel_pub"):
            node.get_logger().error("STOP: Ctrl+C")
            node.safe_stop()  # ROS is still up here (see CupRobot.start)
    finally:
        if hasattr(node, "cmd_vel_pub"):
            node.drive(0.0)
        rclpy.try_shutdown()
    sys.exit(0 if done else 1)


if __name__ == "__main__":
    main()
