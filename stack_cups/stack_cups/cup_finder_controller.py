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
  3. READY     - measure the cup, raise the lift above it, straighten the wrist,
                 open the gripper, read the grasp point's offsets from TF
  4. ALIGN     - measure (stopped, fresh frames), drive exactly the measured offset by
                 odometry, repeat at most `max_drives` times; with use_wrist_yaw, drive
                 only until the wrist yaw can cover the rest, then turn the wrist and
                 check the grasp point (from TF) is within yaw_check_tol of the cup
  5. REACH     - measure the cup, table top and table edge, check the move is safe,
                 lower to cup height, then extend the arm to the cup
  6. DONE / FAILED

Times are on the node's clock: sim time in the MuJoCo sim (which can run much slower
than real time), wall time on the robot. The wall time is logged alongside.

Parameters: stack_cups/params.py, set per platform in config/sim.yaml and config/robot.yaml.
    ros2 launch stack_cups find_cup.launch.py config:=sim             # or config:=robot
    ros2 run stack_cups find_cup --ros-args -p grasp:=true            # also close the gripper
    ros2 run stack_cups find_cup --ros-args -p steps:=aim_head,ready  # only these steps
Stop at any time: Ctrl+C, ros2 topic pub --once /stack_cups/stop std_msgs/msg/Empty, or the
robot's runstop button. With step_confirm, each step waits for Enter or /stack_cups/continue.
"""

import time

import cv2
import numpy as np
import rclpy

from stack_cups import params
from stack_cups import perception as P
from stack_cups.align import align_arm, odom_note
from stack_cups.head_camera import HeadCamera
from stack_cups.robot import CupRobot, StopRequested

BASE_FRAME = "base_link"
GRASP_FRAME = "link_grasp_center"
STEPS = ("aim_head", "ready", "align", "reach")


class FindCup(CupRobot):
    def __init__(self):
        super().__init__()
        self.phase = "START"
        self.t_start = self.t_phase = None  # (ros time, wall time)

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
        return P.measure_cup(snap["img"], snap["depth"], snap["K_color"], snap["K_depth"],
                             snap["T_base"], snap["T_cd"], full)

    def measure_fresh(self, samples=None, full=False):
        """The cup measured while stopped (median of `samples`), with the median radius,
        and with `full` also the median table top and the nearest table edge."""
        c = self.cfg
        m = self.camera.measure_fresh(lambda: self.measure(full), samples or c["samples"],
                                      c["settle_s"], c["sample_dt"], c["max_spread"],
                                      key="cup", on_sample=self.show)
        if m is None:
            return None
        found = m["samples"]
        m["radius"] = float(np.median([s["radius"] for s in found]))
        if full:
            tops = [s["table_top"] for s in found if s.get("table_top") is not None]
            edges = [s["edge"] for s in found if s.get("edge") is not None]
            m["table_top"] = float(np.median(tops)) if tops else None
            m["edge"] = float(np.min(edges)) if edges else None
        return m

    def show(self, m):
        if not self.cfg["show_windows"] or m is None or "img" not in m:
            return
        img = m["img"].copy()
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
            if self.now() - t0 > 30 or time.time() - w0 > 120:
                raise RuntimeError(f"no {', '.join(missing)} after 30 s - was the sim launched "
                                   "with use_cameras:=true?")
            self._idle(0.2)  # honours stop requests, keeps the OpenCV windows alive

    def aim_head(self):
        if not self.cfg["aim_head"]:
            return
        pan, tilt = self.cfg["head_pan"], self.cfg["head_tilt"]
        self.set_phase("AIM_HEAD", f"pan={pan:.2f} tilt={tilt:.2f}")
        self.move({"joint_head_pan": pan, "joint_head_tilt": tilt})

    def ready_arm(self):
        self.set_phase("READY", "measuring the cup, raising the lift, straightening the wrist")
        m = self.measure_fresh()
        if m is None:
            raise RuntimeError(f"cup not found: {self.camera.last_miss}")
        self.log(f"cup at base ({m['cup'][0]:.3f}, {m['cup'][1]:.3f}, {m['cup'][2]:.3f}) m"
                 f"{odom_note(m)}")
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
        target = np.array(cup, dtype=float)
        if self.cfg["grasp_height"] >= 0 and top is not None:
            target[2] = top + self.cfg["grasp_height"]
            self.log(f"grasping {self.cfg['grasp_height']:.3f} m above the table top")
        plan, why = P.plan_reach(target, m["radius"], top, edge, offs)
        if plan is None:
            raise RuntimeError(f"not safe to reach: {why}")
        self.log(f"plan: lift {plan['lift']:.3f} m (grasp height {plan['grasp_z']:.3f} m), "
                 f"extend {plan['extension']:.3f} m")
        self.move_joint_precise("joint_lift", plan["lift"], (0.0, 1.1))
        self.move_joint_precise("wrist_extension", plan["extension"], (0.0, 0.52))
        if self.cfg["grasp"]:
            self.move({"joint_gripper_finger_left": self.cfg["gripper_closed"]})

    def run(self):
        try:
            unknown = self.steps - set(STEPS)
            if unknown:
                raise RuntimeError(f"unknown steps {sorted(unknown)}: use all, or some of "
                                   f"{','.join(STEPS)}")
            self.wait_for_data()
            self.enter_navigation_mode()
            if self.cfg["aim_head"] and self.step("aim_head", "point the head at the table"):
                self.aim_head()
            offs = None
            if self.step("ready", "raise the lift above the cup, straighten the wrist, "
                                  "open the gripper"):
                offs = self.ready_arm()
            offs = offs or self.arm_offsets(BASE_FRAME, GRASP_FRAME)
            if offs is None:
                raise RuntimeError(f"could not read {GRASP_FRAME} from TF")
            if self.step("align", "drive along the table until the arm lines up with the cup"):
                offs = self.align(offs)
            if self.step("reach", "lower to the cup and extend the arm"
                                  + (", then close the gripper" if self.cfg["grasp"] else "")):
                self.reach(offs)
            ran = [s for s in STEPS if s in self.steps]
            self.set_phase("DONE", "gripper is at the cup" if "reach" in self.steps
                           else f"ran {','.join(ran)}")
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
            node.safe_stop()  # ROS is still up here (see CupRobot.start)
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
