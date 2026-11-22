"""CupRobot: the starter code's HelloNode plus the few things our tasks need on top.

HelloNode (hello_helpers, used by the course starter's move_robot.py) gives us
move_to_pose, mode switching, stop/stow, tf2_buffer and joint_state. This layer adds:
  - move(): move_to_pose with a timeout that raises if the move is invalid, rejected
    or does not reach its target (move_to_pose itself only logs, and a blocking call
    can wait forever)
  - move_joint_precise(): the sim reports a joint move done once it is within 5 cm,
    so re-command until the joint is within `tol`
  - drive_by(): move the base an exact distance in navigation mode, steered by /odom
    (the sim's driver cannot do base position moves with translate_mobile_base)

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

import threading
import time

import hello_helpers.hello_misc as hm
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time

from stack_cups import motion
from stack_cups.perception import quat_to_mat

CMD_VEL_TOPIC = "/stretch/cmd_vel"
# The current command is re-published at this rate, in REAL (wall) time, while in
# navigation mode. The sim driver times cmd_vel in wall time: if the last command is
# 0.5-1.5 s old it runs a "move the base by 0" position command that drives the base
# backward (measured 2026-11-22: ~70 cm in 20 s with a zero command every ~3 s; still
# with none at all). A sim-time timer at <1% sim speed sends one every ~10 wall s, so
# the base crept back several cm/s while "stopped". 10 Hz wall keeps every command
# fresh, so the driver always applies velocity 0.
CMD_VEL_HZ = 10.0
DRIVE_HZ = 20.0
# Safety net if the clock stops (e.g. the sim froze): a wait gives up once the ROS clock
# has not moved for this much wall time. (Not a cap on total wall time: the sim can run
# at under 1% of real time, so a long move legitimately takes many wall minutes.)
CLOCK_STALL_S = 60.0
# The sim's trajectory server accepts a joint within 5 cm (or 0.05 rad) of its target
REACHED_TOL = 0.05
# Joints whose reported position cannot be compared with the command (gripper units
# differ, and a gripper closing on a cup stops short of its target on purpose)
UNVERIFIED_JOINTS = ("joint_gripper_finger_left", "joint_gripper_finger_right",
                     "gripper_aperture")


class CupRobot(hm.HelloNode):
    def __init__(self):
        # HelloNode creates the actual ROS node in main(); see start()
        hm.HelloNode.__init__(self)
        self.cmd_v = 0.0
        self.nav_active = False
        self._odom = None
        self.odom_count = 0
        self.warned_abort = False
        # Called from every wait loop; a node with OpenCV windows sets it to keep them
        # responsive (they freeze and the desktop offers "Force Quit" otherwise)
        self.on_idle = None
        self._clk_last, self._clk_wall = None, time.time()

    # ------------------------------------------------------------------ startup
    def start(self, name, odom_topic="/odom"):
        """Create the node (HelloNode.main), then add /odom and cmd_vel streaming."""
        self.main(name, name, wait_for_first_pointcloud=False)
        # HelloNode always subscribes to the full point cloud; we use the depth image
        # instead, and converting clouds we never read costs CPU
        self.destroy_subscription(self.point_cloud_subscriber)
        self.cb_group = ReentrantCallbackGroup()
        self.odom_topic = self.param("odom_topic", odom_topic)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10,
                                 callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
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

    def _idle(self, seconds):
        """Sleep a short wall time inside a wait loop, letting on_idle run."""
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
    def move(self, pose, timeout=30.0):
        """move_to_pose that raises RuntimeError unless every joint reaches its target.

        A move counts as done if the server reports success, or if it reports another
        status but joint_states shows every joint within REACHED_TOL of its target: the
        MuJoCo sim's trajectory server raises after every move (it checks the wheels
        with a call its stretch_mujoco version does not support) and so reports
        ABORTED for moves that did finish."""
        future = self.move_to_pose(pose, blocking=False)
        if future is None:
            raise RuntimeError(f"invalid move (see the move_to_pose error above): {pose}")
        handle = self.wait_future(future, timeout)
        if handle is None or not handle.accepted:
            raise RuntimeError(f"move rejected or not answered in {timeout:.0f} s: {pose}")
        result = self.wait_future(handle.get_result_async(), timeout)
        if result is None:
            raise RuntimeError(f"move did not finish in {timeout:.0f} s: {pose}")
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return
        missed = self._wait_joints_reach(pose, timeout)
        if missed:
            raise RuntimeError(f"move failed (status {result.status}); "
                               f"not at target: {missed}; pose {pose}")
        if not self.warned_abort:
            self.warned_abort = True
            self.get_logger().warn(
                f"the trajectory server reported status {result.status}, but the joints "
                "reached their targets; accepting (known sim driver bug, warned once)")

    def _wait_joints_reach(self, pose, timeout, still_s=1.0, still_m=0.001):
        """The sim's server gives up after 5 s of *wall* time, so a slow sim reports a
        move as done (or aborted) while the joints are still on their way. Keep waiting
        (ROS time) while any joint is still moving; return the joints that stopped short
        of their target ({} if all reached), or those missing at the timeout."""
        t0 = self.now()
        last_move_t = t0
        prev = {j: self.joint_pos(j) for j in pose}
        while True:
            missed = self._joints_missed(pose)
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

    def _joints_missed(self, pose):
        """{joint: (target, actual)} for joints in `pose` not within REACHED_TOL."""
        missed = {}
        for joint, target in pose.items():
            if joint in UNVERIFIED_JOINTS:
                continue
            actual = self.joint_pos(joint)
            if actual is None:
                continue  # not reported in joint_states: cannot check
            if abs(float(target) - actual) > REACHED_TOL:
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

    def move_joint_precise(self, joint, target, limits, tol=0.01, tries=5):
        """The sim's trajectory server reports success once a joint is within 5 cm.
        Re-command with the remaining error added until the joint is within `tol`,
        then send the true target so it does not creep past."""
        cmd = target
        for _ in range(tries):
            self.move({joint: float(np.clip(cmd, *limits))})
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

    def _stream_loop(self):
        """Re-send the current command every 1/CMD_VEL_HZ wall seconds (see CMD_VEL_HZ)."""
        while rclpy.ok():
            if self.nav_active:
                msg = Twist()
                msg.linear.x = float(self.cmd_v)
                try:
                    self.cmd_vel_pub.publish(msg)
                except Exception:  # noqa: BLE001 - publisher gone during shutdown
                    return
            time.sleep(1.0 / CMD_VEL_HZ)

    def drive(self, v):
        """Set the base speed; the timer keeps re-sending it to the driver."""
        self.cmd_v = float(v)
        msg = Twist()
        msg.linear.x = self.cmd_v
        self.cmd_vel_pub.publish(msg)

    def drive_by(self, distance, *, v_max, v_min, decel, tol=0.003, stall_s=2.0,
                 timeout_s=30.0, max_reversals=3):
        """Drive `distance` meters along the current heading, steered by /odom.
        Returns the progress reached; raises RuntimeError on a stall or timeout
        (both in ROS time). The command is always left at 0, even on an exception
        or Ctrl+C."""
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
                    raise RuntimeError(f"drive_by timed out at {progress:+.3f} of {distance:+.3f} m")
                if self.clock_stalled():
                    raise RuntimeError(f"the ROS clock stopped for {CLOCK_STALL_S:.0f} s "
                                       f"during drive_by (did the sim freeze?)")
                self.drive(v)
                self._idle(1.0 / DRIVE_HZ)
        finally:
            self.drive(0.0)
        return progress
