#!/usr/bin/env python3
"""Navigate the sim with Nav2 on the saved sim map (roadmap 0.8), using the starter's
BasicNavigator, and check where the robot really ends up (/odom is ground truth in the
sim, and the sim map's origin is the robot's start pose).

    ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false
    ros2 launch parcs_stretch_nav2 navigation.launch.py use_sim_time:=true teleop_type:=none \
        use_rviz:=false map:=$(ros2 pkg prefix --share stack_cups)/maps/sim_task_room.yaml \
        params_file:=$(ros2 pkg prefix --share parcs_stretch_nav2)/config/nav2_params.yaml
    python3 sim/nav_check.py
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from parcs_stretch_nav2.robot_navigator import BasicNavigator, TaskResult

# (name, x, y, yaw) in the map frame. The arm reaches toward the robot's -y, so a yaw of 0
# along the table's near edge puts the arm side at the table.
GOALS = [("stacking area, arm toward the tape", 1.15, 0.0, 0.0),
         ("far side of the table", 0.5, -1.85, math.pi),
         ("back to the start", 0.0, 0.0, 0.0)]
XY_TOL, YAW_TOL = 0.25, 0.25  # Nav2's goal checker in parcs_stretch_nav2's configs


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    nav = BasicNavigator()
    truth, amcl = {}, {}
    nav.create_subscription(Odometry, "/odom", lambda m: truth.update(p=m.pose.pose), 10)
    nav.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                            lambda m: amcl.update(p=m.pose.pose), 10)
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
