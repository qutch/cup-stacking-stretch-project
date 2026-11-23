"""CupRobot: the starter code's HelloNode plus the few things our tasks need on top.

HelloNode (hello_helpers, used by the course starter's move_robot.py) gives us
move_to_pose, mode switching, stop/stow, tf2_buffer and joint_state. This layer adds:
  - move(): move_to_pose with a timeout that raises if the move is invalid, rejected
    or does not reach its target (move_to_pose itself only logs, and a blocking call
    can wait forever). If something stops a joint (the gripper against a cup), it
    raises JointBlocked.
  - move_joint_precise(): the unpatched sim reports a joint move done once it is within
    5 cm, so re-command until the joint is within `tol`. With the sim patches
    (sim/patches) and on the real robot one move is enough. A blocked joint is never
    re-commanded past its target: that shoves whatever blocks it.
  - drive_by(): move the base an exact distance in navigation mode, steered by /odom.
    The same code works on the sim and the robot (the unpatched sim's driver cannot do
    base position moves with translate_mobile_base).
  - Safety (roadmap 0.6): a stop from any of
      * Ctrl+C in the node's terminal,
      * ros2 topic pub --once /stack_cups/stop std_msgs/msg/Empty,
      * the robot's runstop button (/is_runstopped),
    sets the base speed to 0 at once, cancels the arm move in progress, calls the
    driver's /stop_the_robot, and makes every wait raise StopRequested. Base speeds are
    capped at max_base_speed; joint moves can carry speed limits (max_joint_speed_*);
    confirm() pauses before a step when step_confirm is set.
Parameters: stack_cups/params.py (ROBOT), set per platform in config/*.yaml.

Time: every timeout, stall check and timing uses the node's ROS clock (now()). If the
sim publishes /clock, start() switches the node to sim time, so a sim that runs far
slower than real time (software rendering) does not trip timeouts. On the real robot
there is no /clock and this is wall time.

Usage (no rclpy.init(): HelloNode.main does it and starts its own spin thread):
    robot = CupRobot()
    robot.start("my_node")
    robot.enter_navigation_mode()
    robot.move({"joint_lift": 0.8})
    robot.drive_by(0.1, v_max=0.15, v_min=0.06, decel=0.1)
"""

import sys
import threading
import time

import hello_helpers.hello_misc as hm
import numpy as np
import rclpy
import rclpy.signals
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, Empty

from stack_cups import motion, params
from stack_cups.perception import quat_to_mat

CMD_VEL_TOPIC = "/stretch/cmd_vel"
STOP_TOPIC = "/stack_cups/stop"
CONTINUE_TOPIC = "/stack_cups/continue"
RUNSTOP_TOPIC = "/is_runstopped"
# The current command is re-published at this rate, in REAL (wall) time, while in
# navigation mode. The sim driver times cmd_vel in wall time: if the last command is
# 0.5-1.5 s old it runs a "move the base by 0" position command that drives the base
# backward (measured 2026-11-22: ~70 cm in 20 s with a zero command every ~3 s; still
# with none at all; the sim patches fix that). A sim-time timer at <1% sim speed sends
# one every ~10 wall s. 10 Hz wall keeps every command fresh, so the driver always
# applies velocity 0.
CMD_VEL_HZ = 10.0
DRIVE_HZ = 20.0
# Safety net if the clock stops (e.g. the sim froze): a wait gives up once the ROS clock
# has not moved for this much wall time. (Not a cap on total wall time: the sim can run
# at under 1% of real time, so a long move legitimately takes many wall minutes.)
CLOCK_STALL_S = 60.0
# Joints whose reported position cannot be compared with the command (gripper units
# differ, and a gripper closing on a cup stops short of its target on purpose)
GRIPPER_JOINTS = ("joint_gripper_finger_left", "joint_gripper_finger_right",
                  "gripper_aperture")
LINEAR_JOINTS = ("joint_lift", "wrist_extension", "joint_arm")


class JointBlocked(RuntimeError):
    """A joint stopped short because something is in its way (e.g. the gripper against a
    cup). Reported by the patched sim server (sim/patches) as "<joint> blocked at ..."."""


class StopRequested(RuntimeError):
    """Someone asked the robot to stop (stop topic, runstop button or Ctrl+C)."""


def _cancel(handle):
    """Cancel an accepted goal; never raises (ROS may already be shutting down)."""
    try:
        handle.cancel_goal_async()
    except Exception:  # noqa: BLE001
        pass


def _cancel_when_accepted(future):
    """Cancel the goal of a pending send_goal future as soon as it is accepted."""
    def done(f):
        try:
            handle = f.result()
        except Exception:  # noqa: BLE001 - the send itself failed: nothing to cancel
            return
        if handle is not None and handle.accepted:
            _cancel(handle)
    future.add_done_callback(done)


class CupRobot(hm.HelloNode):
    def __init__(self):
        # HelloNode creates the actual ROS node in main(); see start()
        hm.HelloNode.__init__(self)
        self.cmd_v = 0.0
        self.nav_active = False
        self._odom = None
        self.odom_count = 0
        self.warned_abort = False
        self.stop_reason = None
        self._goal = None  # goal handle of the arm move in progress
        self._continue = threading.Event()
        self._stdin_reader = None
        self._pub_lock = threading.Lock()
        self.rcfg = dict(params.ROBOT)
        # Called from every wait loop; a node with OpenCV windows sets it to keep them
        # responsive (they freeze and the desktop offers "Force Quit" otherwise)
        self.on_idle = None
        self._clk_last, self._clk_wall = None, time.time()

    # ------------------------------------------------------------------ startup
    def start(self, name, odom_topic=None):
        """Create the node (HelloNode.main), then add /odom, cmd_vel streaming and the
        stop inputs."""
        self.main(name, name, wait_for_first_pointcloud=False)
        # rclpy's Ctrl+C handler shuts ROS down at once, so the base could no longer be
        # told to stop. Give Ctrl+C back to Python: it raises KeyboardInterrupt in our
        # code, which stops the robot (safe_stop) while ROS is still up.
        rclpy.signals.uninstall_signal_handlers()
        # HelloNode always subscribes to the full point cloud; we use the depth image
        # instead, and converting clouds we never read costs CPU
        self.destroy_subscription(self.point_cloud_subscriber)
        self.cb_group = ReentrantCallbackGroup()
        defaults = dict(params.ROBOT, **({"odom_topic": odom_topic} if odom_topic else {}))
        self.rcfg = {k: self.param(k, v) for k, v in defaults.items()}
        self.odom_topic = self.rcfg["odom_topic"]
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10,
                                 callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Empty, STOP_TOPIC,
                                 lambda _: self.request_stop(f"{STOP_TOPIC} received"), 1,
                                 callback_group=self.cb_group)
        self.create_subscription(Bool, RUNSTOP_TOPIC, self._on_runstop, 1,
                                 callback_group=self.cb_group)
        self.create_subscription(Empty, CONTINUE_TOPIC, lambda _: self._continue.set(), 1,
                                 callback_group=self.cb_group)
        threading.Thread(target=self._stream_loop, daemon=True).start()
        self._use_sim_time_if_simulated()

    def _use_sim_time_if_simulated(self, wait_s=2.0):
        """Switch to sim time if something publishes /clock (the MuJoCo sim does; the
        real robot does not). Then wait for the first clock message."""
        if self.get_parameter("use_sim_time").value:
            sim = True
        else:
            t0 = time.time()
            while self.count_publishers("/clock") == 0 and time.time() - t0 < wait_s:
                time.sleep(0.1)
            sim = self.count_publishers("/clock") > 0
            if sim:
                self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        if sim:
            t0 = time.time()
            while self.now() == 0.0 and time.time() - t0 < 10.0:
                time.sleep(0.05)
            self.get_logger().info("using sim time from /clock")

    # ------------------------------------------------------------------ safety
    def request_stop(self, why):
        """Stop as soon as possible (any thread): base speed 0 now, cancel the arm move in
        progress, and make the main thread's next wait raise StopRequested."""
        if self.stop_reason is None:
            self.stop_reason = why
            self.get_logger().error(f"STOP: {why}")
        self.drive(0.0)
        goal = self._goal
        if goal is not None:
            goal.cancel_goal_async()

    def _on_runstop(self, msg):
        if msg.data:
            self.request_stop(f"the runstop is on ({RUNSTOP_TOPIC})")

    def check_stop(self):
        if self.stop_reason is not None:
            raise StopRequested(self.stop_reason)

    def safe_stop(self):
        """Leave the robot stopped: base speed 0, arm move cancelled, driver told to stop.
        Safe to call at any time, also while handling another error."""
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
        try:
            self.stop_the_robot()
        except Exception:  # noqa: BLE001
            pass

    def confirm(self, what):
        """With step_confirm, wait before a step until the operator presses Enter in this
        terminal or publishes CONTINUE_TOPIC (raises StopRequested on a stop)."""
        if not self.rcfg["step_confirm"]:
            return
        self._continue.clear()
        how = f"ros2 topic pub --once {CONTINUE_TOPIC} std_msgs/msg/Empty"
        if sys.stdin is not None and sys.stdin.isatty():
            how = "press Enter, or " + how
            if self._stdin_reader is None:
                self._stdin_reader = threading.Thread(target=self._read_stdin, daemon=True)
                self._stdin_reader.start()
        self.get_logger().warn(f"[STEP] next: {what}. To go on: {how}")
        while not self._continue.is_set():
            self._idle(0.05)

    def _read_stdin(self):
        for _ in sys.stdin:
            self._continue.set()

    # ------------------------------------------------------------------- waits
    def _idle(self, seconds):
        """Sleep a short wall time inside a wait loop, letting on_idle run. Every wait
        goes through here, so a stop request interrupts any of them."""
        self.check_stop()
        if self.on_idle is not None:
            self.on_idle()
        time.sleep(seconds)

    def clock_stalled(self):
        """True if the ROS clock has not advanced for CLOCK_STALL_S of wall time."""
        now, wall = self.now(), time.time()
        if now != self._clk_last:
            self._clk_last, self._clk_wall = now, wall
        return wall - self._clk_wall > CLOCK_STALL_S

    def now(self):
        """Seconds on the node's clock (sim time in the sim, wall time on the robot)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, seconds):
        """Sleep `seconds` of ROS time (sim time in the sim, which can run far slower)."""
        t0 = self.now()
        while self.now() - t0 < seconds and not self.clock_stalled():
            self._idle(0.02)

    def wait_future(self, future, timeout):
        """Poll a future (the executor runs in HelloNode's thread) for up to `timeout`
        seconds of ROS time. None on timeout."""
        t0 = self.now()
        while not future.done():
            if self.now() - t0 > timeout or self.clock_stalled():
                return None
            self._idle(0.02)
        return future.result()

    def param(self, name, default):
        """Parameter value. HelloNode's node auto-declares parameters passed with -p, so
        only declare the ones that were not passed."""
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            return default if value is None else value
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------- joints
    def _speed_limited(self, pose):
        """The pose with (position, speed, acceleration) per joint if joint speed limits
        are set (max_joint_speed_m and _rad both > 0), else unchanged."""
        lin, rot = self.rcfg["max_joint_speed_m"], self.rcfg["max_joint_speed_rad"]
        if lin <= 0.0 or rot <= 0.0:
            return pose
        out = {}
        for joint, x in pose.items():
            v = lin if joint in LINEAR_JOINTS else rot
            out[joint] = (float(x), v, 2.0 * v)
        return out

    def move(self, pose, timeout=30.0):
        """move_to_pose that raises RuntimeError unless every joint reaches its target.

        - Success: the server reports SUCCEEDED (the real driver, and the patched sim
          server, which only does so once every joint is within 5 mm).
        - A failure the server explains (an error message) raises; "<joint> blocked at"
          raises JointBlocked.
        - The unpatched MuJoCo sim server reports ABORTED with no message after every
          move (it checks the wheels with a call its stretch_mujoco version does not
          support). With aborted_ok_tol > 0 the move then counts as done if joint_states
          shows every joint within that distance of its target.
        With joint speed limits, gripper joints move in a goal of their own: HelloNode
        gives every joint without a speed 0.0 once one joint in a goal has one."""
        self.check_stop()
        limited = self._speed_limited(pose) is not pose
        grip = {j: x for j, x in pose.items() if j in GRIPPER_JOINTS}
        if limited and grip and len(grip) < len(pose):
            self.move({j: x for j, x in pose.items() if j not in GRIPPER_JOINTS}, timeout)
            self.move(grip, timeout)
            return
        goal_pose = pose if grip else self._speed_limited(pose)
        future = self.move_to_pose(goal_pose, blocking=False)
        if future is None:
            raise RuntimeError(f"invalid move (see the move_to_pose error above): {pose}")
        try:
            handle = self.wait_future(future, timeout)
        except BaseException:
            # stopped before the goal was even accepted: cancel it as soon as it is
            _cancel_when_accepted(future)
            raise
        if handle is None:
            # not answered in time, but it may still be accepted later: don't let it run
            _cancel_when_accepted(future)
            raise RuntimeError(f"move not answered in {timeout:.0f} s: {pose}")
        if not handle.accepted:
            raise RuntimeError(f"move rejected: {pose}")
        self._goal = handle
        try:
            result = self.wait_future(handle.get_result_async(), timeout)
            if result is None:
                _cancel(handle)  # timed out: stop the joints instead of leaving them moving
        except BaseException:
            _cancel(handle)
            raise
        finally:
            self._goal = None
        if result is None:
            raise RuntimeError(f"move did not finish in {timeout:.0f} s: {pose}")
        self.check_stop()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return
        why = result.result.error_string if result.result is not None else ""
        if why:
            error = JointBlocked if "blocked" in why else RuntimeError
            raise error(f"move failed (status {result.status}): {why}; pose {pose}")
        tol = self.rcfg["aborted_ok_tol"]
        if tol <= 0.0:
            raise RuntimeError(f"move failed (status {result.status}); pose {pose}")
        missed = self._wait_joints_reach(pose, timeout, tol)
        if missed:
            raise RuntimeError(f"move failed (status {result.status}); "
                               f"not at target: {missed}; pose {pose}")
        if not self.warned_abort:
            self.warned_abort = True
            self.get_logger().warn(
                f"the trajectory server reported status {result.status}, but the joints "
                "reached their targets; accepting (unpatched sim driver, warned once; "
                "see sim/patches)")

    def _wait_joints_reach(self, pose, timeout, tol, still_s=1.0, still_m=0.001):
        """The sim's server gives up after 5 s of *wall* time, so a slow sim reports a
        move as done (or aborted) while the joints are still on their way. Keep waiting
        (ROS time) while any joint is still moving; return the joints that stopped short
        of their target ({} if all reached), or those missing at the timeout."""
        t0 = self.now()
        last_move_t = t0
        prev = {j: self.joint_pos(j) for j in pose}
        while True:
            missed = self._joints_missed(pose, tol)
            now = self.now()
            if not missed or now - t0 > timeout or self.clock_stalled():
                return missed
            cur = {j: self.joint_pos(j) for j in pose}
            if any(cur[j] is not None and prev[j] is not None and abs(cur[j] - prev[j]) > still_m
                   for j in missed):
                last_move_t, prev = now, cur
            elif now - last_move_t > still_s:
                return missed  # stopped short
            self._idle(0.05)

    def _joints_missed(self, pose, tol):
        """{joint: (target, actual)} for joints in `pose` not within `tol`."""
        missed = {}
        for joint, target in pose.items():
            if joint in GRIPPER_JOINTS:
                continue
            actual = self.joint_pos(joint)
            if actual is None:
                continue  # not reported in joint_states: cannot check
            if abs(float(target) - actual) > tol:
                missed[joint] = (round(float(target), 3), round(actual, 3))
        return missed

    def joint_pos(self, name):
        js = self.joint_state
        if name in js.name:
            return float(js.position[js.name.index(name)])
        return None

    def wait_joint_still(self, name, still_m=0.001, window_s=0.2, timeout=3.0):
        """Wait until the joint moved less than `still_m` over `window_s` (ROS time);
        returns its position."""
        t0 = self.now()
        history = []
        while self.now() - t0 < timeout and not self.clock_stalled():
            now, pos = self.now(), self.joint_pos(name)
            if pos is None:
                return None
            history.append((now, pos))
            old = [p for t, p in history if now - t >= window_s]
            if old and abs(pos - old[-1]) < still_m:
                return pos
            self._idle(0.05)
        return self.joint_pos(name)

    def move_joint_precise(self, joint, target, limits, tol=0.01, tries=None):
        """Move one joint to within `tol` of `target`.

        The unpatched sim reports success once a joint is within 5 cm, so re-command with
        the remaining error added until it is within `tol` (at most `tries` moves,
        default the joint_tries parameter), then send the true target so it does not
        creep past. If something blocks the joint (JointBlocked), stop correcting:
        commanding past the target would only shove what is in the way."""
        tries = int(tries or self.rcfg["joint_tries"])
        cmd = target
        for _ in range(tries):
            blocked = None
            try:
                self.move({joint: float(np.clip(cmd, *limits))})
            except JointBlocked as e:
                blocked = e
            actual = self.wait_joint_still(joint)
            if actual is None:
                self.get_logger().warn(f"no joint state for {joint}; skipping correction")
                return
            err = target - actual
            self.get_logger().info(f"{joint}: target {target:.3f}, actual {actual:.3f} "
                                   f"({err * 100:+.1f} cm)")
            if abs(err) < tol:
                if cmd != target:
                    self.move({joint: target})
                return
            if blocked is not None:
                self.get_logger().warn(f"{joint} is blocked {err * 100:+.1f} cm short of "
                                       f"{target:.3f}; not pushing harder ({blocked})")
                if cmd != target:
                    try:
                        self.move({joint: target})
                    except JointBlocked:
                        pass  # still blocked; it now pushes only toward the true target
                return
            cmd += err
        self.get_logger().warn(f"{joint} did not get within {tol * 100:.0f} cm of {target:.3f}")

    # ----------------------------------------------------------------------- TF
    def lookup(self, target, source):
        """(R, t) mapping points in `source` into `target`, or None."""
        if not self.tf2_buffer.can_transform(target, source, Time(),
                                             timeout=Duration(seconds=0.5)):
            self.get_logger().warn(f"TF {source} -> {target} unavailable",
                                   throttle_duration_sec=2.0)
            return None
        # Not hm.get_p1_to_p2_matrix: in our VM its ros2_numpy reads the quaternion as
        # (w, x, y, z) instead of (x, y, z, w), so its rotations are wrong (positions are
        # right). Found 2026-11-22 when every cup measurement failed.
        try:
            tf = self.tf2_buffer.lookup_transform(target, source, Time())
        except Exception as e:  # noqa: BLE001 - tf2 raises several exception types
            self.get_logger().warn(f"TF {source} -> {target}: {e}", throttle_duration_sec=2.0)
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return quat_to_mat(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])

    def arm_offsets(self, base_frame="base_link", grasp_frame="link_grasp_center"):
        """Grasp point relative to the lift and arm, from TF and joint states:
        h = grasp z - lift, c = -grasp y - extension (the arm reaches along -y),
        x = grasp x, all in base_frame. None if TF or joint states are missing."""
        T = self.lookup(base_frame, grasp_frame)
        lift, ext = self.joint_pos("joint_lift"), self.joint_pos("wrist_extension")
        if T is None or lift is None or ext is None:
            return None
        g = T[1]
        return {"h": g[2] - lift, "c": -g[1] - ext, "x": g[0]}

    # --------------------------------------------------------------------- base
    def enter_navigation_mode(self):
        """Navigation mode: the base takes cmd_vel, every other joint still takes
        move_to_pose goals (HelloNode logs a 'position mode' warning for each one)."""
        self.switch_to_navigation_mode()
        self.cmd_v = 0.0
        self.nav_active = True

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (float(p.x), float(p.y), float(yaw))
        self.odom_count += 1

    def odom_pose(self):
        return self._odom

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.cmd_v)
        self.cmd_vel_pub.publish(msg)

    def _stream_loop(self):
        """Re-send the current command every 1/CMD_VEL_HZ wall seconds (see CMD_VEL_HZ)."""
        while rclpy.ok():
            if self.nav_active:
                try:
                    with self._pub_lock:  # so a stale speed is never sent after a new one
                        self._publish_cmd()
                except Exception:  # noqa: BLE001 - publisher gone during shutdown
                    return
            time.sleep(1.0 / CMD_VEL_HZ)

    def drive(self, v):
        """Set the base speed, capped at max_base_speed; the stream thread keeps re-sending
        it to the driver. Speed 0 is always allowed; any other speed raises StopRequested
        after a stop."""
        if v != 0.0:
            self.check_stop()
        cap = float(self.rcfg["max_base_speed"])
        with self._pub_lock:
            self.cmd_v = float(np.clip(v, -cap, cap))
            self._publish_cmd()

    def drive_by(self, distance, *, v_max, v_min, decel, tol=0.003, stall_s=2.0,
                 timeout_s=30.0, max_reversals=3):
        """Drive `distance` meters along the current heading, steered by /odom.
        Returns the progress reached; raises RuntimeError on a stall or timeout
        (both in ROS time). The command is always left at 0, even on an exception,
        a stop request or Ctrl+C."""
        if not self.nav_active:
            raise RuntimeError("drive_by needs navigation mode")
        start = self.odom_pose()
        if start is None:
            raise RuntimeError(f"no odometry on {self.odom_topic}")
        t0 = stall_t = self.now()
        progress = stall_ref = 0.0
        last_sign, reversals = 0.0, 0
        try:
            while rclpy.ok():
                now = self.now()
                progress = motion.progress_along(start, self.odom_pose()[:2])
                v = motion.base_speed(distance - progress, v_max, v_min, decel, tol)
                if v == 0.0:
                    break
                sign = float(np.sign(v))
                if last_sign and sign != last_sign:
                    reversals += 1
                    if reversals > max_reversals:
                        self.get_logger().warn(
                            f"drive_by: stopping after {reversals - 1} reversals, "
                            f"{(distance - progress) * 100:+.1f} cm from the target")
                        break
                last_sign = sign
                if abs(progress - stall_ref) > 0.001:
                    stall_ref, stall_t = progress, now
                elif now - stall_t > stall_s:
                    raise RuntimeError(
                        f"base stalled at {progress:+.3f} of {distance:+.3f} m "
                        f"(command {v:+.3f} m/s; try a larger v_min)")
                if now - t0 > timeout_s:
                    raise RuntimeError(
                        f"drive_by timed out at {progress:+.3f} of {distance:+.3f} m")
                if self.clock_stalled():
                    raise RuntimeError(f"the ROS clock stopped for {CLOCK_STALL_S:.0f} s "
                                       f"during drive_by (did the sim freeze?)")
                self.drive(v)
                self._idle(1.0 / DRIVE_HZ)
        finally:
            self.drive(0.0)
        return progress
