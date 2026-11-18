"""Find the red cup, line the robot up with it, and reach the arm out to it.

Phases (each logged as it starts):
  1. AIM_HEAD  - point the head camera at the table on the robot's arm side
  2. CENTER    - (original find_cup logic) drive forward/back until the cup is
                 centered in the head camera image
  3. READY     - raise the lift well above the cup, straighten the wrist, open the gripper
  4. ALIGN     - using depth, drive until the cup lines up with the arm's reach line
  5. REACH     - using depth, measure the cup and the table, check the move is safe,
                 lower to cup height, then extend the arm to the cup
  6. DONE / FAILED

Depth is used to (a) find the cup's 3D position, (b) measure the table top so the
gripper is never lowered into it, and (c) measure the table edge so the gripper is
not lowered on top of the table while retracted.

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
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from stack_cups import perception as P

IMAGE_TOPIC = "/camera/color/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
JOINT_STATES_TOPIC = "/stretch/joint_states"
CMD_VEL_TOPIC = "/stretch/cmd_vel"
TRAJ_ACTION = "/stretch_controller/follow_joint_trajectory"
BASE_FRAME = "base_link"
GRASP_FRAME = "link_grasp_center"

# Phase 2 (original find_cup tuning)
CENTER_THRESH = 2  # pixels of dead zone
KP = 0.009  # m/s per pixel of error
MAX_SPEED = 0.22
CENTER_FRAMES = 5  # consecutive centered frames before moving on
# 120 is half the height of the camera image; the D435i is mounted rotated, so the
# image's vertical axis runs along the robot's driving direction
IMAGE_CENTER_ROW = 120

# Phase 4
ALIGN_TOL = 0.008  # m; grasps tested reliable within +/-1 cm of the arm line
ALIGN_ACCEPT = 0.012  # if fine alignment stalls, still accept this much error
FINE_ZONE = 0.04  # below this error, switch to stop-and-measure nudges
FINE_MAX_NUDGES = 25
PULSE_MAX_S = 6.0
SETTLE_S = 1.0  # wait after stopping so the next camera frames are fresh
# The sim's base only reaches ~20-25% of small commanded speeds (wheel friction),
# so these commands are larger than the motion they actually produce.
ALIGN_KP = 3.0  # (m/s commanded) per m of error
ALIGN_MAX = 0.12
ALIGN_MIN = 0.04
ALIGN_STALL_S = 30.0  # fail if the error has not improved by 5 mm in this long
ALIGN_MAX_S = 240.0
# The driver brakes if cmd_vel stops arriving for 0.5 s, so the current command
# is re-published at this rate while in navigation mode.
CMD_VEL_HZ = 10.0

# Arm
READY_ABOVE_CUP = 0.25  # how far above the cup the grasp point sits while driving
GRIPPER_OPEN = 0.5
GRIPPER_CLOSED = -0.1


class FindCup(Node):
    def __init__(self):
        super().__init__("FindCup")
        self.declare_parameter("aim_head", True)
        self.declare_parameter("head_pan", -1.57)
        self.declare_parameter("head_tilt", -0.6)
        self.declare_parameter("grasp", False)
        self.declare_parameter("show_windows", True)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color = self.depth = None
        self.color_info = self.depth_info = None
        self.joints = {}

        self.create_subscription(Image, IMAGE_TOPIC, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self._on_color_info,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._on_depth_info,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joints, 10)
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.cmd_v = 0.0
        self.nav_active = False
        self.create_timer(1.0 / CMD_VEL_HZ, self._stream_cmd_vel)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.traj = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION)
        self.to_position = self.create_client(Trigger, "/switch_to_position_mode")
        self.to_navigation = self.create_client(Trigger, "/switch_to_navigation_mode")

        self.phase = "START"
        self.status = ""

    # ---------------------------------------------------------------- callbacks
    def _on_color(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.color = (img, msg.header.frame_id)

    def _on_depth(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        with self.lock:
            self.depth = (np.array(img, dtype=np.float32), msg.header.frame_id)

    def _on_color_info(self, msg):
        self.color_info = msg

    def _on_depth_info(self, msg):
        self.depth_info = msg

    def _stream_cmd_vel(self):
        if self.nav_active:
            msg = Twist()
            msg.linear.x = float(self.cmd_v)
            self.pub.publish(msg)

    def _on_joints(self, msg):
        with self.lock:
            self.joints.update(dict(zip(msg.name, msg.position)))

    # ------------------------------------------------------------------ helpers
    def log(self, text):
        self.status = text
        self.get_logger().info(text)

    def set_phase(self, phase, text=""):
        self.phase = phase
        self.log(f"[{phase}] {text}" if text else f"[{phase}]")

    def drive(self, v):
        """Set the base speed; the timer keeps re-sending it to the driver."""
        self.cmd_v = float(v)
        msg = Twist()
        msg.linear.x = self.cmd_v
        self.pub.publish(msg)

    def stop(self):
        for _ in range(3):
            self.drive(0.0)
            time.sleep(0.05)

    @staticmethod
    def wait_future(future, timeout):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.02)
        return future.result()

    def switch_mode(self, client, name):
        if name != "navigation":
            self.stop()
            self.nav_active = False
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service for {name} mode not available")
        res = self.wait_future(client.call_async(Trigger.Request()), 10.0)
        if res is None or not res.success:
            raise RuntimeError(f"could not switch to {name} mode")
        time.sleep(0.3)
        if name == "navigation":
            self.cmd_v = 0.0
            self.nav_active = True

    def move_joints(self, positions, seconds=3.0):
        """positions: {joint_name: target}. Blocks until the trajectory finishes."""
        if not self.traj.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("trajectory action server not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(positions)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions.values()]
        point.time_from_start = Duration(seconds=seconds).to_msg()
        goal.trajectory.points = [point]
        handle = self.wait_future(self.traj.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            raise RuntimeError(f"trajectory goal rejected: {positions}")
        result = self.wait_future(handle.get_result_async(), 60.0)
        if result is None:
            raise RuntimeError(f"trajectory timed out: {positions}")
        time.sleep(0.3)

    def move_joint_precise(self, joint, target, limits, tol=0.01, tries=5):
        """The sim's trajectory server reports success once a joint is within 5 cm,
        and the arm/lift settle slowly. Re-command with the remaining error added
        until the joint (per /stretch/joint_states) is within `tol`, then send the
        true target so it does not creep past."""
        cmd = target
        for i in range(tries):
            self.move_joints({joint: float(np.clip(cmd, *limits))}, 3.0)
            time.sleep(1.5)
            with self.lock:
                actual = self.joints.get(joint)
            if actual is None:
                self.get_logger().warn(f"no joint state for {joint}; skipping correction")
                return
            err = target - actual
            self.get_logger().info(f"{joint}: target {target:.3f}, actual {actual:.3f} "
                                   f"({err * 100:+.1f} cm)")
            if abs(err) < tol:
                if cmd != target:
                    self.move_joints({joint: target}, 1.0)
                return
            cmd += err
        self.get_logger().warn(f"{joint} did not get within {tol * 100:.0f} cm of {target:.3f}")

    def lookup(self, target, source):
        """(R, t) mapping points in `source` into `target`, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                                 timeout=Duration(seconds=0.5))
        except Exception as e:  # noqa: BLE001 - tf2 raises several exception types
            self.get_logger().warn(f"TF {source} -> {target} unavailable: {e}",
                                   throttle_duration_sec=2.0)
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return P.quat_to_mat(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])

    def latest(self):
        with self.lock:
            return self.color, self.depth

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
            return out
        T_base = self.lookup(BASE_FRAME, depth_frame)
        T_cd = self.lookup(color_frame, depth_frame)
        if T_base is None or T_cd is None:
            return out
        R, t = T_base
        cup, radius = P.locate_cup(dimg, self.depth_info.k, R, t, mask,
                                   self.color_info.k, T_cd[0], T_cd[1], bbox)
        if cup is None:
            return out
        out.update(cup=cup, radius=radius)
        if not full:
            return out
        pts = P.to_base(P.backproject(dimg, self.depth_info.k, step=2), R, t)
        top = P.estimate_table_top(pts, cup, radius)
        edge = P.table_edge_distance(pts, cup[0], 0.3, (top if top is not None else cup[2]) + 0.02)
        out.update(cup=cup, radius=radius, table_top=top, edge=edge)
        return out

    def measure_avg(self, n=5, full=True):
        """Average several measurements while standing still."""
        samples = []
        t0 = time.time()
        while len(samples) < n and time.time() - t0 < 10.0:
            m = self.measure(full)
            if m and "cup" in m:
                samples.append(m)
            time.sleep(0.15)
        if not samples:
            return None
        out = dict(samples[-1])
        out["cup"] = np.mean([s["cup"] for s in samples], axis=0)
        out["radius"] = float(np.mean([s["radius"] for s in samples]))
        tops = [s["table_top"] for s in samples if s.get("table_top") is not None]
        edges = [s["edge"] for s in samples if s.get("edge") is not None]
        out["table_top"] = float(np.median(tops)) if tops else None
        out["edge"] = float(np.min(edges)) if edges else None
        return out

    def arm_offsets(self):
        """Grasp point relative to lift/extension, measured from TF + joint states."""
        T = self.lookup(BASE_FRAME, GRASP_FRAME)
        with self.lock:
            lift = self.joints.get("joint_lift")
            ext = self.joints.get("wrist_extension")
        if T is None or lift is None or ext is None:
            return None
        g = T[1]
        return {"h": g[2] - lift, "c": -g[1] - ext, "x": g[0]}

    def show(self, m):
        if not self.get_parameter("show_windows").value or m is None:
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
        self.set_phase("START", "waiting for camera images, depth, camera info and TF ...")
        t0 = time.time()
        while rclpy.ok():
            color, depth = self.latest()
            if color and depth and self.color_info and self.depth_info and \
                    self.lookup(BASE_FRAME, depth[1]) is not None:
                # Zeros are normal (nothing within range, e.g. sky). Negative values or
                # NaNs mean the GPU renderer is not producing real depth.
                d = depth[0]
                nan_frac = float(np.mean(~np.isfinite(d)))
                if nan_frac > 0.05 or np.nanmin(d) < -0.01:
                    raise RuntimeError(
                        f"depth image looks broken ({nan_frac * 100:.0f}% NaN, "
                        f"min {np.nanmin(d):.2f} m). The sim's GPU renderer is not producing "
                        "depth; restart the sim with: export MUJOCO_GL=osmesa")
                return
            if time.time() - t0 > 30:
                raise RuntimeError(
                    "no camera/depth data after 30 s - "
                    "was the sim launched with use_cameras:=true?")
            time.sleep(0.2)

    def aim_head(self):
        if not self.get_parameter("aim_head").value:
            return
        pan = self.get_parameter("head_pan").value
        tilt = self.get_parameter("head_tilt").value
        self.set_phase("AIM_HEAD", f"pan={pan:.2f} tilt={tilt:.2f}")
        self.switch_mode(self.to_position, "position")
        self.move_joints({"joint_head_pan": pan, "joint_head_tilt": tilt}, 2.0)

    def center(self):
        """Original find_cup behavior: drive until the cup is centered in the image."""
        self.set_phase("CENTER", "driving to center the cup in the head camera image")
        self.switch_mode(self.to_navigation, "navigation")
        centered = 0
        while rclpy.ok():
            color, _ = self.latest()
            img = color[0]
            bbox, mask = P.find_red_blob(img)
            self.show({"img": img, "mask": mask, "bbox": bbox})
            if bbox is None:
                self.stop()
                self.get_logger().info("no red cup in view", throttle_duration_sec=2.0)
                centered = 0
                time.sleep(0.05)
                continue
            x, y, w, h = bbox
            cx, cy = x + w // 2, y + h // 2
            error = cy - IMAGE_CENTER_ROW
            self.get_logger().info(f"Cup at pixel ({cx}, {cy}), error {error}",
                                   throttle_duration_sec=1.0)
            if abs(error) > CENTER_THRESH:
                centered = 0
                self.drive(np.clip(KP * error, -MAX_SPEED, MAX_SPEED))
            else:
                self.drive(0.0)
                centered += 1
                if centered >= CENTER_FRAMES:
                    self.stop()
                    return
            time.sleep(0.05)

    def ready_arm(self):
        self.set_phase("READY", "raising the lift above the cup and straightening the wrist")
        time.sleep(0.5)
        m = self.measure_avg()
        if m is None:
            raise RuntimeError("cup not found in depth after centering")
        self.log(f"cup at base ({m['cup'][0]:.3f}, {m['cup'][1]:.3f}, {m['cup'][2]:.3f}) m")
        self.switch_mode(self.to_position, "position")
        offs = self.arm_offsets()
        h = offs["h"] if offs else 0.115
        lift = float(np.clip(m["cup"][2] + READY_ABOVE_CUP - h, 0.3, 1.05))
        # Lift first (arm stays retracted), then straighten the wrist up there
        self.move_joints({"joint_lift": lift, "wrist_extension": 0.0}, 3.0)
        self.move_joints({"joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0,
                          "joint_wrist_roll": 0.0, "joint_gripper_finger_left": GRIPPER_OPEN}, 3.0)
        time.sleep(0.5)
        offs = self.arm_offsets()
        if offs is None:
            raise RuntimeError(f"could not read {GRASP_FRAME} from TF")
        self.log(f"arm offsets: grasp is {offs['h']:.3f} m above the lift, "
                 f"{offs['c']:.3f} m beyond the extension, at x={offs['x']:.3f}")
        return offs

    def align(self, offs):
        self.set_phase("ALIGN", "driving until the cup lines up with the arm (depth)")
        self.switch_mode(self.to_navigation, "navigation")
        self._align_coarse(offs)
        self._align_fine(offs)

    def _align_coarse(self, offs):
        """Drive continuously until the cup is within FINE_ZONE of the arm line."""
        t0 = t_best = time.time()
        best = None
        while rclpy.ok():
            now = time.time()
            if now - t0 > ALIGN_MAX_S or now - t_best > ALIGN_STALL_S:
                self.get_logger().warn("coarse alignment is not converging; "
                                       "switching to stop-and-measure nudges")
                break
            m = self.measure(full=False)
            self.show(m)
            if not m or "cup" not in m:
                self.drive(0.0)
                time.sleep(0.1)
                continue
            dx = m["cup"][0] - offs["x"]
            if best is None or abs(dx) < best - 0.005:
                best, t_best = abs(dx), now
            self.get_logger().info(f"cup is {dx * 100:+.1f} cm from the arm line",
                                   throttle_duration_sec=1.0)
            if abs(dx) < FINE_ZONE:
                break
            v = np.clip(ALIGN_KP * dx, -ALIGN_MAX, ALIGN_MAX)
            self.drive(np.sign(v) * max(abs(v), ALIGN_MIN))
            time.sleep(0.05)
        self.stop()
        time.sleep(SETTLE_S)

    def _dx_still(self, offs):
        m = self.measure_avg(3, full=False)
        if m is None:
            raise RuntimeError("lost the cup in depth during alignment")
        self.show(m)
        return float(m["cup"][0] - offs["x"])

    def _align_fine(self, offs):
        """Nudge-and-look: short drive pulse, stop, wait for fresh frames, re-measure.
        Camera latency cannot cause overshoot because we only measure when stopped.

        How far a pulse moves the base depends on how fast the sim is running (it
        can be far slower than real time with software rendering), so the achieved
        speed is learned from each nudge. If a nudge falls short, the next pulse is
        longer; once pulses are at their maximum length, the command is raised."""
        cmd = ALIGN_MIN
        speed = 0.008  # m/s actually achieved at `cmd`; learned from each nudge
        dx = self._dx_still(offs)
        best = abs(dx)
        for i in range(FINE_MAX_NUDGES):
            if abs(dx) < ALIGN_TOL:
                self.log(f"[ALIGN] aligned: cup is {dx * 100:+.1f} cm from the arm line")
                return
            want = 0.8 * abs(dx)
            pulse = float(np.clip(want / speed, 0.2, PULSE_MAX_S))
            self.drive(np.sign(dx) * cmd)
            time.sleep(pulse)
            self.stop()
            time.sleep(SETTLE_S)
            new_dx = self._dx_still(offs)
            moved = float((dx - new_dx) * np.sign(dx))
            self.log(f"[ALIGN] nudge {i + 1}: {cmd:.3f} m/s for {pulse:.1f} s moved "
                     f"{moved * 100:+.1f} cm; cup now {new_dx * 100:+.1f} cm from the arm line")
            if moved > 0.0005:
                speed = float(np.clip(moved / pulse, 1e-4, 0.05))
            else:
                speed = max(speed / 2, 1e-4)
            if moved < want / 3 and pulse >= PULSE_MAX_S:
                cmd = min(cmd * 1.5, ALIGN_MAX)
            dx = new_dx
            best = min(best, abs(dx))
        if abs(dx) < ALIGN_ACCEPT:
            self.get_logger().warn(f"accepting alignment {dx * 100:+.1f} cm off "
                                   f"(target {ALIGN_TOL * 100:.1f} cm)")
            return
        raise RuntimeError(f"alignment stalled: still {dx * 100:+.1f} cm off "
                           f"after {FINE_MAX_NUDGES} nudges (best {best * 100:.1f} cm)")

    def reach(self, offs):
        self.set_phase("REACH", "measuring cup and table with depth")
        time.sleep(0.5)
        m = self.measure_avg(8)
        if m is None:
            raise RuntimeError("lost the cup in depth before reaching")
        self.show(m)
        cup, top, edge = m["cup"], m["table_top"], m["edge"]
        self.log(f"cup center ({cup[0]:.3f}, {cup[1]:.3f}, {cup[2]:.3f}) m, "
                 f"radius {m['radius'] * 100:.1f} cm; table top "
                 f"{'unknown' if top is None else f'{top:.3f} m'}; table edge "
                 f"{'unknown' if edge is None else f'{edge:.3f} m'} from base center")
        plan, why = P.plan_reach(cup, m["radius"], top, edge, offs)
        if plan is None:
            raise RuntimeError(f"not safe to reach: {why}")
        self.log(f"plan: lift {plan['lift']:.3f} m (grasp height {plan['grasp_z']:.3f} m), "
                 f"extend {plan['extension']:.3f} m")
        self.switch_mode(self.to_position, "position")
        self.move_joint_precise("joint_lift", plan["lift"], (0.0, 1.1))
        self.move_joint_precise("wrist_extension", plan["extension"], (0.0, 0.52))
        if self.get_parameter("grasp").value:
            self.move_joints({"joint_gripper_finger_left": GRIPPER_CLOSED}, 2.0)

    def run(self):
        try:
            self.wait_for_data()
            self.aim_head()
            self.center()
            offs = self.ready_arm()
            self.align(offs)
            self.reach(offs)
            self.set_phase("DONE", "gripper is at the cup")
        except Exception as e:  # noqa: BLE001 - report any failure and stop safely
            self.stop()
            self.set_phase("FAILED", str(e))
        # Keep the windows alive until Ctrl+C
        while rclpy.ok():
            self.show(self.measure())
            time.sleep(0.1)


def main():
    rclpy.init()
    node = FindCup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
