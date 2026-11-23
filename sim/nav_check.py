#!/usr/bin/env python3
"""Navigate the sim with Nav2 on the saved sim map (roadmap 0.8), using the starter's
BasicNavigator, and check where the robot really ends up (/odom is ground truth in the
sim, and the sim map's origin is the robot's start pose).

    ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false
    ros2 launch stack_cups sim_nav.launch.py
    python3 sim/nav_check.py
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from parcs_stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# (name, x, y, yaw) in the map frame. The arm reaches toward the robot's -y, so a yaw of 0
# along the table's near edge puts the arm side at the table.
GOALS = [("stacking area, arm toward the tape", 1.15, 0.0, 0.0),
         ("far side of the table", 0.5, -1.85, math.pi),
         ("back to the start", 0.0, 0.0, 0.0)]
# A goal passes if Nav2 reports SUCCEEDED and the robot really (by /odom) ended within Nav2's
# goal tolerance (0.25 m, 0.25 rad in parcs_stretch_nav2's configs) plus AMCL's error: Nav2
# stops once AMCL's estimate is within 0.25 m, and AMCL was 13-18 cm off in the sim's room.
XY_TOL, YAW_TOL = 0.25 + 0.20, 0.25


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stow(node):
    """Tuck the arm in first: Nav2's footprint (robot_radius 0.22) assumes it, and in the
    sim's start pose the gripper sticks out 0.5 m, just under the table top."""
    joints = {}
    node.create_subscription(JointState, "/stretch/joint_states",
                             lambda m: joints.update(zip(m.name, m.position)), 10)
    cli = node.create_client(Trigger, "/stow_the_robot")
    cli.wait_for_service(timeout_sec=10.0)
    rclpy.spin_until_future_complete(node, cli.call_async(Trigger.Request()), timeout_sec=10.0)
    end = time.time() + 30.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if joints.get("joint_lift", 1.0) < 0.3 and joints.get("joint_wrist_yaw", 0.0) > 3.0:
            return True
    return False


def main():
    rclpy.init()
    nav = BasicNavigator()
    truth, amcl = {}, {}
    nav.create_subscription(Odometry, "/odom", lambda m: truth.update(p=m.pose.pose), 10)
    nav.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                            lambda m: amcl.update(p=m.pose.pose), 10)
    if not stow(nav):
        print("WARNING: the arm did not stow", flush=True)
    nav.setInitialPose(pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()
    ok_all = True
    for name, x, y, yaw in GOALS:
        t0 = time.time()
        nav.goToPose(pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
        result = nav.getResult()
        for _ in range(10):
            rclpy.spin_once(nav, timeout_sec=0.05)
        p = truth["p"]
        dxy = math.hypot(p.position.x - x, p.position.y - y)
        dyaw = abs(math.atan2(math.sin(yaw_of(p.orientation) - yaw), math.cos(yaw_of(p.orientation) - yaw)))
        loc = ""
        if "p" in amcl:
            a = amcl["p"]
            loc = (f"; AMCL vs truth {math.hypot(a.position.x - p.position.x, a.position.y - p.position.y) * 100:.1f} cm")
        ok = result == TaskResult.SUCCEEDED and dxy <= XY_TOL and dyaw <= YAW_TOL
        ok_all &= ok
        print(f"{'PASS' if ok else 'FAIL'} {name}: {result.name if hasattr(result, 'name') else result} "
              f"in {time.time() - t0:.0f} s; ends {dxy * 100:.1f} cm, {math.degrees(dyaw):.1f} deg "
              f"from the goal{loc}", flush=True)
    print("RESULT", "all goals reached" if ok_all else "some goals missed", flush=True)
    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
