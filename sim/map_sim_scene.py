#!/usr/bin/env python3
"""Drive the robot once around the cup table in the sim, so slam_toolbox can map the
task scene (roadmap 0.8). The saved map is stack_cups/maps/sim_task_room.yaml.

    ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false
    ros2 launch parcs_stretch_nav2 offline_mapping.launch.py use_sim_time:=true use_rviz:=false
    python3 sim/map_sim_scene.py
    ros2 run nav2_map_server map_saver_cli -f stack_cups/maps/sim_task_room \
        --ros-args -p use_sim_time:=true
    python3 sim/draw_table_on_map.py stack_cups/maps/sim_task_room.yaml

The route (odom = world frame in the sim; the robot starts at the origin facing +x) keeps
the robot's center at least 0.4 m from the table, walls and furniture of sim_scene.py.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

ROUTE = [(2.0, 0.0), (2.0, -1.85), (-1.2, -1.85), (-1.2, 0.0), (0.0, 0.0)]
V_MAX, W_MAX = 0.25, 0.6  # m/s, rad/s


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Driver(Node):
    def __init__(self):
        super().__init__("map_sim_scene")
        self.pose = None
        self.joints = {}
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(JointState, "/stretch/joint_states",
                                 lambda m: self.joints.update(zip(m.name, m.position)), 10)
        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stow(self):
        """Tuck the arm in (lift down, gripper over the base). In the sim's start pose the
        gripper sticks out 0.5 m on the arm side, just under the table top, and catches the
        table legs."""
        cli = self.create_client(Trigger, "/stow_the_robot")
        cli.wait_for_service(timeout_sec=10.0)
        rclpy.spin_until_future_complete(self, cli.call_async(Trigger.Request()),
                                         timeout_sec=10.0)
        end = time.time() + 30.0
        while time.time() < end:
            self.spin_for(0.2)
            if self.joints.get("joint_lift", 1.0) < 0.3 and \
                    self.joints.get("joint_wrist_yaw", 0.0) > 3.0:
                self.spin_for(1.0)  # let it settle
                return True
        return False

    def turn_to(self, heading, tol=0.03):
        while True:
            self.spin_for(0.05)
            err = wrap(heading - self.pose[2])
            if abs(err) < tol:
                break
            self.cmd(0.0, math.copysign(max(0.15, min(W_MAX, 1.5 * abs(err))), err))
        self.cmd(0.0, 0.0)

    def go_to(self, gx, gy, tol=0.05):
        """Drive to (gx, gy). False if it is not there in a generous time (stuck)."""
        self.spin_for(0.05)
        x, y, _ = self.pose
        limit = 20.0 + 3.0 * math.hypot(gx - x, gy - y) / 0.08
        t0 = t_log = time.time()
        self.turn_to(math.atan2(gy - y, gx - x))
        reached = True
        while True:
            self.spin_for(0.05)
            x, y, th = self.pose
            dist = math.hypot(gx - x, gy - y)
            err = wrap(math.atan2(gy - y, gx - x) - th)
            if dist < tol or abs(err) > math.pi / 2:
                break
            if time.time() - t_log > 2.0:
                t_log = time.time()
                print(f"   at ({x:+.2f}, {y:+.2f}), heading {math.degrees(th):+.0f} deg, "
                      f"{dist:.2f} m to go", flush=True)
            if time.time() - t0 > limit:
                print(f"STUCK: not at ({gx:+.2f}, {gy:+.2f}) after {limit:.0f} s; "
                      f"at ({x:+.2f}, {y:+.2f})", flush=True)
                reached = False
                break
            self.cmd(min(V_MAX, max(0.08, dist)), max(-W_MAX, min(W_MAX, 2.0 * err)))
        self.cmd(0.0, 0.0)
        return reached


def main():
    rclpy.init()
    d = Driver()
    nav = d.create_client(Trigger, "/switch_to_navigation_mode")
    nav.wait_for_service(timeout_sec=10.0)
    rclpy.spin_until_future_complete(d, nav.call_async(Trigger.Request()), timeout_sec=10.0)
    print("stowing the arm", flush=True)
    if not d.stow():
        print("WARNING: the arm did not stow; the gripper may catch the table", flush=True)
    while d.pose is None:
        d.spin_for(0.1)
    t0 = time.time()
    stuck = 0
    for x, y in ROUTE:
        print(f"driving to ({x:+.2f}, {y:+.2f})", flush=True)
        stuck += not d.go_to(x, y)
    d.turn_to(0.0)
    x, y, th = d.pose
    print(f"RESULT {'route driven' if not stuck else f'stuck on {stuck} legs'}; "
          f"ends at ({x:+.3f}, {y:+.3f}, {math.degrees(th):+.1f} deg) "
          f"after {time.time() - t0:.0f} s", flush=True)
    d.cmd(0.0, 0.0)
    d.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
