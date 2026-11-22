"""Find the red cup, line the robot up with it, and reach the arm out to it.

Built on CupRobot (stack_cups/robot.py), which is the course starter's HelloNode plus
checked moves and odometry-steered base moves. The whole run stays in navigation mode:
the base takes cmd_vel and every other joint still takes move_to_pose goals.

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

Run with the sim launched with use_cameras:=true:
    ros2 run stack_cups find_cup
    ros2 run stack_cups find_cup --ros-args -p grasp:=true      # also close the gripper
    ros2 run stack_cups find_cup --ros-args -p aim_head:=false  # keep current head pose
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from stack_cups import motion as M
from stack_cups import perception as P
from stack_cups.robot import CupRobot

IMAGE_TOPIC = "/camera/color/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
BASE_FRAME = "base_link"
GRASP_FRAME = "link_grasp_center"

READY_ABOVE_CUP = 0.25  # how far above the cup the grasp point sits while driving
GRIPPER_OPEN = 0.5
GRIPPER_CLOSED = -0.1

# name: default. Override with --ros-args -p name:=value
PARAMS = {
    "aim_head": True,
    "head_pan": -1.57,
    "head_tilt": -0.6,
    "grasp": False,
    "show_windows": True,
    "v_max": 0.15,  # m/s, base speed far from the target
    "v_min": 0.06,  # m/s, slowest command that still moves the base (checked in the sim)
    "decel": 0.1,  # m/s^2, how hard the base slows down near the target
    "drive_tol": 0.003,  # m, a drive stops this close to its target
    "align_tol": 0.008,  # m, grasps tested reliable within +/-1 cm of the arm line
    "max_drives": 3,
    # The sim renders a new camera image only every 0.1 s of *sim* time and re-sends the
    # last one in between, so "fresh" is decided on the ROS clock, not by counting messages
    "settle_s": 0.3,  # s to wait after stopping before measuring (~3 sim renders)
    "sample_dt": 0.1,  # s between samples (one sim render)
    "samples": 5,  # measurements per position (median)
    "max_spread": 0.01,  # m, if samples spread more, take more
    "grasp_height": -1.0,  # m above the table top to grasp at; < 0 = cup center
    # Wrist yaw covers the last few cm, so the base rarely needs a second drive.
    # arm_len and yaw_sign were measured in the sim with sim/check_robot_layer.py.
    "use_wrist_yaw": True,
    "max_yaw": 0.25,  # rad (~14 deg): about +/-6.7 cm of sideways reach
    "arm_len": 0.270,  # m, wrist yaw axis to grasp center
    "yaw_sign": 1.0,  # +yaw moves the grasp point toward +x of base_link
    "yaw_check_tol": 0.010,  # m, grasp point must end this close to the cup's x
}


class FindCup(CupRobot):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color = self.depth = None
        self.color_info = self.depth_info = None
        self.depth_count = 0
        self.last_miss = ""
        self.phase = "START"
        self.t_start = self.t_phase = None  # (ros time, wall time)

    def start(self, name="find_cup"):
        super().start(name)
        self.cfg = {k: self.param(k, v) for k, v in PARAMS.items()}
        if self.cfg["show_windows"]:
            self.on_idle = lambda: cv2.waitKey(1)
        cb = self.cb_group
        self.create_subscription(Image, IMAGE_TOPIC, self._on_color, qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self._on_color_info,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._on_depth_info,
                                 qos_profile_sensor_data, callback_group=cb)

    # ---------------------------------------------------------------- callbacks
    # Images are stored as messages and only converted when measuring: the sim re-sends
    # frames far more often than it renders them, and converting each one takes CPU
    # the sim needs.
    def _on_color(self, msg):
        with self.lock:
            self.color = msg

    def _on_depth(self, msg):
        with self.lock:
            self.depth = msg
            self.depth_count += 1

    def _on_color_info(self, msg):
        self.color_info = msg

    def _on_depth_info(self, msg):
        self.depth_info = msg

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

    def latest(self):
        """(color image, frame id), (depth image, frame id); each None if not received."""
        with self.lock:
            cmsg, dmsg = self.color, self.depth
        color = depth = None
        if cmsg is not None:
            color = (self.bridge.imgmsg_to_cv2(cmsg, "bgr8"), cmsg.header.frame_id)
        if dmsg is not None:
            depth = (np.array(self.bridge.imgmsg_to_cv2(dmsg, "32FC1"), dtype=np.float32),
                     dmsg.header.frame_id)
        return color, depth

    def wait_new_depth(self, n, timeout=3.0):
        """Wait until `n` more depth frames have arrived (ROS time). False on timeout."""
        with self.lock:
            target = self.depth_count + n
        t0 = self.now()
        while self.now() - t0 < timeout:
            with self.lock:
                if self.depth_count >= target:
                    return True
            time.sleep(0.02)
        return False

    def measure(self, full=True):
        """One depth measurement: dict with cup center/radius, and (if full) table top
        and table edge distance, all in base_link, plus the image and mask; or None."""
        color, depth = self.latest()
        if color is None or depth is None or self.depth_info is None or self.color_info is None:
            return None
        (img, color_frame), (dimg, depth_frame) = color, depth
        bbox, mask = P.find_red_blob(img)
        out = {"img": img, "mask": mask, "bbox": bbox}
        if bbox is None:
            out["why"] = f"no red blob in the image ({int(np.count_nonzero(mask))} red pixels)"
            return out
        T_base = self.lookup(BASE_FRAME, depth_frame)
        T_cd = self.lookup(color_frame, depth_frame)
        if T_base is None or T_cd is None:
            out["why"] = f"TF {depth_frame} -> {BASE_FRAME} or {color_frame} unavailable"
            return out
        R, t = T_base
        cup, radius = P.locate_cup(dimg, self.depth_info.k, R, t, mask,
                                   self.color_info.k, T_cd[0], T_cd[1], bbox)
        if cup is None:
            out["why"] = f"red blob at {bbox} but too few depth points on it"
            return out
        out.update(cup=cup, radius=radius)
        pose = self.odom_pose()
        if pose is not None:
            # The cup in the fixed odom frame, so samples taken while the base drifts
            # still agree (the base was seen creeping while "stopped" in the sim)
            out.update(pose=pose, cup_odom=np.array([*M.base_to_odom(cup[:2], pose), cup[2]]))
        if not full:
            return out
        pts = P.to_base(P.backproject(dimg, self.depth_info.k, step=2), R, t)
        top = P.estimate_table_top(pts, cup, radius)
        edge = P.table_edge_distance(pts, cup[0], 0.3, (top if top is not None else cup[2]) + 0.02)
        out.update(table_top=top, edge=edge)
        return out

    def measure_fresh(self, samples=None, full=False):
        """Measure while stopped: wait `settle_s` (ROS time) so no image used was
        rendered while moving, take `samples` measurements `sample_dt` apart, and use
        their median. If they spread more than `max_spread`, take `samples` more once.
        None if the cup was not found in any of them."""
        n = int(samples or self.cfg["samples"])
        if not self.wait_new_depth(1):
            raise RuntimeError("no depth images arriving")
        self.sleep(self.cfg["settle_s"])
        found = []

        def collect(k):
            tries = 0
            while len(found) < k and tries < 3 * k:
                tries += 1
                if tries > 1:
                    self.sleep(self.cfg["sample_dt"])
                m = self.measure(full)
                if m:
                    self.show(m)
                if m and "cup" in m:
                    found.append(m)
                else:
                    why = (m or {}).get("why", "no camera data")
                    misses[why] = misses.get(why, 0) + 1

        misses = {}
        collect(n)
        if not found:
            self.last_miss = "; ".join(f"{k} (x{v})" for k, v in misses.items())
            self.get_logger().warn(f"cup not found: {self.last_miss}")
            return None
        key = "cup_odom" if all("cup_odom" in s for s in found) else "cup"
        center, spread, ok = M.robust_center([s[key] for s in found], self.cfg["max_spread"])
        if not ok:
            collect(2 * n)
            key = "cup_odom" if all("cup_odom" in s for s in found) else "cup"
            center, spread, ok = M.robust_center([s[key] for s in found],
                                                 self.cfg["max_spread"])
        out = dict(found[-1])
        if key == "cup_odom":
            # back into base_link with the base's pose now, i.e. where the cup is
            # relative to the robot right before the next move
            pose = self.odom_pose()
            base_xy = M.odom_to_base(center[:2], pose)
            out.update(cup_odom=center, pose=pose,
                       cup=np.array([base_xy[0], base_xy[1], center[2]]))
            drift = abs(found[-1]["pose"][0] - found[0]["pose"][0])
            if drift > 0.002:
                self.get_logger().warn(f"the base moved {drift * 1000:.0f} mm while "
                                       f"measuring (it should be still)")
        else:
            out["cup"] = center
        out.update(spread=spread, radius=float(np.median([s["radius"] for s in found])))
        if full:
            tops = [s["table_top"] for s in found if s.get("table_top") is not None]
            edges = [s["edge"] for s in found if s.get("edge") is not None]
            out["table_top"] = float(np.median(tops)) if tops else None
            out["edge"] = float(np.min(edges)) if edges else None
        return out

    @staticmethod
    def _odom_note(m):
        """'; base at odom x=..., cup at odom x=...' for logs (empty if no odom)."""
        if "pose" not in m or "cup_odom" not in m:
            return ""
        return (f"; base at odom x={m['pose'][0]:+.3f}, "
                f"cup at odom ({m['cup_odom'][0]:+.3f}, {m['cup_odom'][1]:+.3f})")

    def arm_offsets(self):
        """Grasp point relative to lift/extension, measured from TF + joint states."""
        T = self.lookup(BASE_FRAME, GRASP_FRAME)
        lift, ext = self.joint_pos("joint_lift"), self.joint_pos("wrist_extension")
        if T is None or lift is None or ext is None:
            return None
        g = T[1]
        return {"h": g[2] - lift, "c": -g[1] - ext, "x": g[0]}

    def show(self, m):
        if not self.cfg["show_windows"] or m is None:
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
        self.set_phase("START", "waiting for camera images, depth, camera info, TF and odometry ...")
        t0, w0 = self.now(), time.time()
        while rclpy.ok():
            color, depth = self.latest()
            have = {
                "color image": color is not None,
                "depth image": depth is not None,
                "camera info": self.color_info is not None and self.depth_info is not None,
                "TF": depth is not None and self.lookup(BASE_FRAME, depth[1]) is not None,
                f"odometry on {self.odom_topic}": self.odom_pose() is not None,
            }
            if all(have.values()):
                # Zeros are normal (nothing within range). Negative values or NaNs mean
                # the GPU renderer is not producing real depth.
                d = depth[0]
                nan_frac = float(np.mean(~np.isfinite(d)))
                if nan_frac > 0.05 or np.nanmin(d) < -0.01:
                    raise RuntimeError(
                        f"depth image looks broken ({nan_frac * 100:.0f}% NaN, "
                        f"min {np.nanmin(d):.2f} m). The sim's GPU renderer is not producing "
                        "depth; restart the sim with: export MUJOCO_GL=osmesa")
                return
            if self.now() - t0 > 30 or time.time() - w0 > 120:
                missing = ", ".join(k for k, ok in have.items() if not ok)
                raise RuntimeError(f"no {missing} after 30 s - was the sim launched with "
                                   "use_cameras:=true?")
            time.sleep(0.2)

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
            raise RuntimeError(f"cup not found: {self.last_miss}")
        self.log(f"cup at base ({m['cup'][0]:.3f}, {m['cup'][1]:.3f}, {m['cup'][2]:.3f}) m"
                 f"{self._odom_note(m)}")
        offs = self.arm_offsets()
        h = offs["h"] if offs else 0.115
        lift = float(np.clip(m["cup"][2] + READY_ABOVE_CUP - h, 0.3, 1.05))
        # Lift first (arm stays retracted), then straighten the wrist up there
        self.move({"joint_lift": lift, "wrist_extension": 0.0})
        self.move({"joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0,
                   "joint_wrist_roll": 0.0, "joint_gripper_finger_left": GRIPPER_OPEN})
        for joint in ("joint_lift", "joint_wrist_yaw", "joint_wrist_pitch"):
            self.wait_joint_still(joint)
        offs = self.arm_offsets()
        if offs is None:
            raise RuntimeError(f"could not read {GRASP_FRAME} from TF")
        self.log(f"arm offsets: grasp is {offs['h']:.3f} m above the lift, "
                 f"{offs['c']:.3f} m beyond the extension, at x={offs['x']:.3f}")
        return offs

    def align(self, offs):
        """Returns the grasp offsets to reach with (changed if the wrist was turned)."""
        self.set_phase("ALIGN", "measure, drive the measured offset by odometry, repeat")
        c = self.cfg
        use_yaw = bool(c["use_wrist_yaw"])
        # with the wrist yaw, the base only has to get within (most of) its reach
        good_enough = (0.8 * M.yaw_reach(c["arm_len"], c["max_yaw"]) if use_yaw
                       else c["align_tol"])
        max_drives = int(c["max_drives"])
        for drives in range(max_drives + 1):
            m = self.measure_fresh()
            if m is None:
                raise RuntimeError(f"lost the cup during alignment: {self.last_miss}")
            dx = float(m["cup"][0] - offs["x"])
            self.log(f"[ALIGN] cup is {dx * 100:+.1f} cm from the arm line "
                     f"(spread {m['spread'] * 1000:.1f} mm, {drives} drives so far)"
                     f"{self._odom_note(m)}")
            if abs(dx) < c["align_tol"]:
                self.log(f"[ALIGN] aligned after {drives} drives")
                return offs
            if use_yaw and abs(dx) < good_enough:
                return self._align_wrist(dx)
            if drives == max_drives:
                break
            moved = self.drive_by(dx, v_max=c["v_max"], v_min=c["v_min"], decel=c["decel"],
                                  tol=c["drive_tol"])
            self.log(f"[ALIGN] drive {drives + 1}: asked {dx * 100:+.1f} cm, "
                     f"odometry says {moved * 100:+.1f} cm")
        raise RuntimeError(f"alignment: still {dx:+.3f} m off after {max_drives} drives")

    def _align_wrist(self, dx):
        """Turn the wrist so the grasp point moves `dx` along base x, then check it
        against a fresh measurement of the cup. Returns the new grasp offsets."""
        c = self.cfg
        yaw = M.yaw_for_offset(dx, c["arm_len"], c["max_yaw"], c["yaw_sign"])
        if yaw is None:
            raise RuntimeError(f"wrist yaw cannot cover {dx * 100:+.1f} cm "
                               f"(max_yaw {c['max_yaw']:.2f} rad)")
        self.log(f"[ALIGN] turning the wrist to {yaw:+.3f} rad to cover {dx * 100:+.1f} cm")
        self.move_joint_precise("joint_wrist_yaw", yaw, (-1.0, 1.0), tol=0.01)
        offs = self.arm_offsets()
        m = self.measure_fresh()
        if offs is None or m is None:
            raise RuntimeError("lost the cup or the grasp frame after turning the wrist")
        err = float(m["cup"][0] - offs["x"])
        self.log(f"[ALIGN] after the wrist turn the grasp point is {err * 100:+.1f} cm "
                 f"from the cup (limit {c['yaw_check_tol'] * 100:.1f} cm)")
        if abs(err) >= c["yaw_check_tol"]:
            raise RuntimeError(f"wrist yaw left {err:+.3f} m")
        return offs

    def reach(self, offs):
        self.set_phase("REACH", "measuring cup and table with depth")
        m = self.measure_fresh(8, full=True)
        if m is None:
            raise RuntimeError(f"lost the cup before reaching: {self.last_miss}")
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
            self.move({"joint_gripper_finger_left": GRIPPER_CLOSED})

    def run(self):
        try:
            self.wait_for_data()
            self.enter_navigation_mode()
            self.aim_head()
            offs = self.ready_arm()
            offs = self.align(offs)
            self.reach(offs)
            self.set_phase("DONE", "gripper is at the cup")
        except Exception as e:  # noqa: BLE001 - report any failure and stop safely
            self.drive(0.0)
            try:
                self.stop_the_robot()
            except Exception:  # noqa: BLE001 - already failing; the base is stopped above
                pass
            self.set_phase("FAILED", str(e))
        self.log(f"[timing] total {self.now() - self.t_start[0]:.1f} s "
                 f"(wall {time.time() - self.t_start[1]:.1f} s)")
        # Keep the windows alive until Ctrl+C
        while rclpy.ok():
            self.show(self.measure(full=False))
            time.sleep(0.1)


def main():
    node = FindCup()
    try:
        node.start("find_cup")
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "cmd_vel_pub"):
            node.drive(0.0)
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
