"""Nav2 from a CupRobot node (roadmap 2.2-2.4): AMCL's initial pose, NavigateToPose goals,
and the robot's outline in Nav2's costmaps.

It runs on the node's own executor (HelloNode's spin thread), like CupRobot's arm moves, so
a stop (Ctrl+C, /stack_cups/stop, the runstop) cancels the Nav2 goal at once. Use it with
Nav2 from the course starter package (in the sim: ros2 launch stack_cups sim_nav.launch.py).

    nav = Nav2(robot)
    nav.wait_ready(30.0)
    pose = nav.localize(start_pose)        # the robot's map pose, giving AMCL one if needed
    nav.set_outline(carry.carry_outline(...))
    nav.go_to((x, y, yaw), timeout=120.0, check=lambda: ...)

Goal and initial-pose stamps are 0 ("the latest transform"): in the sim the node runs on
sim time but the sim's data and Nav2 run on wall time (see sim_nav.launch.py).
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon, PolygonStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from stack_cups import carry as C

COSTMAPS = ("/local_costmap", "/global_costmap")
STATUS = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
          GoalStatus.STATUS_CANCELED: "canceled"}


class Nav2:
    def __init__(self, robot, map_frame="map", base_frame="base_link"):
        """robot: a started CupRobot."""
        self.robot = robot
        self.map_frame, self.base_frame = map_frame, base_frame
        cb = robot.cb_group
        self.client = ActionClient(robot, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.bt_state = robot.create_client(GetState, "/bt_navigator/get_state",
                                            callback_group=cb)
        self.initial_pose_pub = robot.create_publisher(PoseWithCovarianceStamped,
                                                       "/initialpose", 10)
        self.outline_pubs = [robot.create_publisher(Polygon, f"{c}/footprint", 10)
                             for c in COSTMAPS]
        self.local_outline = None
        robot.create_subscription(PolygonStamped, f"{COSTMAPS[0]}/published_footprint",
                                  lambda m: setattr(self, "local_outline", m), 10,
                                  callback_group=cb)

    def wait_ready(self, timeout):
        """Wait (ROS time) until Nav2 accepts goals: its navigator is active (its action
        server exists earlier, but rejects goals until then; Nav2 finishes starting only
        once AMCL has a pose). False on timeout."""
        t0 = self.robot.now()
        while True:
            if self.client.server_is_ready() and self.bt_state.service_is_ready():
                reply = self.robot.wait_future(self.bt_state.call_async(GetState.Request()), 2.0)
                if reply is not None and reply.current_state.label == "active":
                    return True
            if self.robot.now() - t0 > timeout or self.robot.clock_stalled():
                return False
            self.robot._idle(0.5)

    def map_pose(self):
        """The robot's (x, y, yaw) in the map frame from TF, or None (no localization yet)."""
        buf = self.robot.tf2_buffer
        if not buf.can_transform(self.map_frame, self.base_frame, Time(),
                                 timeout=Duration(seconds=0.1)):
            return None
        T = self.robot.lookup(self.map_frame, self.base_frame)
        if T is None:
            return None
        R, t = T
        return (float(t[0]), float(t[1]), float(math.atan2(R[1, 0], R[0, 0])))

    def localize(self, start_pose, timeout=30.0, std_xy=0.1, std_yaw=0.1):
        """The robot's map pose. If AMCL has none yet, give it one first: start_pose is
        where the robot was in the map when odometry started (its driver started), so the
        robot is now at compose(start_pose, odometry). Returns (pose, given) where `given`
        says whether an initial pose was sent."""
        pose = self.map_pose()
        if pose is not None:
            return pose, False
        odom = self.robot.odom_pose()
        if odom is None:
            raise RuntimeError("no odometry to place the robot on the map with")
        guess = C.compose(start_pose, odom)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame  # stamp 0: AMCL uses the latest transforms
        msg.pose.pose.position.x, msg.pose.pose.position.y = guess[0], guess[1]
        msg.pose.pose.orientation.z = math.sin(guess[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(guess[2] / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = std_xy ** 2
        cov[35] = std_yaw ** 2
        msg.pose.covariance = cov
        t0 = last = None
        while pose is None:
            now = self.robot.now()
            if t0 is None or now - last > 2.0:  # AMCL may not be up yet: send again
                self.initial_pose_pub.publish(msg)
                t0, last = t0 if t0 is not None else now, now
            if now - t0 > timeout or self.robot.clock_stalled():
                raise RuntimeError(f"AMCL did not take the initial pose {guess} in "
                                   f"{timeout:.0f} s (is Nav2 running? in the sim: ros2 "
                                   "launch stack_cups sim_nav.launch.py)")
            self.robot._idle(0.1)
            pose = self.map_pose()
        return pose, True

    def set_outline(self, points, check_s=3.0):
        """Give Nav2's costmaps a new robot outline (list of base_link (x, y)). Returns True
        once the local costmap reports it (it pads it slightly), False if it did not in
        `check_s` seconds of ROS time."""
        poly = Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in points])
        want_y = min(y for _, y in points)
        t0 = last = self.robot.now()
        self.local_outline = None
        for p in self.outline_pubs:
            p.publish(poly)
        while self.robot.now() - t0 < check_s:
            m = self.local_outline
            if m is not None and abs(min(q.y for q in m.polygon.points) - want_y) < 0.03:
                return True
            if self.robot.now() - last > 0.5:
                for p in self.outline_pubs:
                    p.publish(poly)
                last = self.robot.now()
            self.robot._idle(0.05)
        return False

    def go_to(self, pose, timeout, check=None, name="the goal"):
        """Drive to (x, y, yaw) in the map with Nav2. check(): called ~10 times a second
        while driving; raising from it cancels the goal (as does a stop). Raises
        RuntimeError if Nav2 fails or takes longer than `timeout` (ROS time)."""
        robot = self.robot
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        # Nav2 sends the base commands now: the driver in navigation mode, and CupRobot's
        # own cmd_vel stream off (it would fight Nav2's)
        robot.set_driver_mode("nav2")
        try:
            handle = robot.wait_future(self.client.send_goal_async(goal), 10.0)
            if handle is None or not handle.accepted:
                raise RuntimeError(f"Nav2 did not accept {name} {tuple(pose)}")
            result = handle.get_result_async()
            t0 = robot.now()
            try:
                while not result.done():
                    if check is not None:
                        check()
                    if robot.now() - t0 > timeout:
                        raise RuntimeError(f"Nav2 did not reach {name} in {timeout:.0f} s")
                    if robot.clock_stalled():
                        raise RuntimeError("the ROS clock stopped while Nav2 was driving")
                    robot._idle(0.1)
            except BaseException:  # a stop, Ctrl+C, a failed check: stop Nav2 driving
                handle.cancel_goal_async()
                robot.drive(0.0)
                raise
        finally:
            robot.set_driver_mode("navigation")
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 {STATUS.get(status, f'status {status}')} on the way "
                               f"to {name}")
