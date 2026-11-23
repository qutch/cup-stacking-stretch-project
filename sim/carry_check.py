#!/usr/bin/env python3
"""Check where carry_to_stack (roadmap task 2) really left the robot in the task scene,
against the scene's ground truth: the tape's position from sim_scene.py, and the robot's
true pose from /odom (the sim's odometry starts at the scene's origin and does not drift).

    python3 sim/carry_check.py          # after: ros2 launch stack_cups carry_to_stack.launch.py config:=sim

Prints the held cup's distance from the line through the taped area's center (the line the
arm reaches along when the base is parallel to the tape), the base's angle to the tape's
edge, the arm extension that reaches the center, and whether the fingers still hold the cup.
RESULT PASS if within --line-tol and --yaw-tol and still holding (exit code 0).
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from stack_cups import sim_scene as S
from stack_cups.perception import quat_to_mat
from tf2_ros import Buffer, TransformListener

CLOSED, HELD_MIN = -0.1, 0.05  # config/sim.yaml: gripper_closed, held_min


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--line-tol", type=float, default=0.015, help="m (default 1.5 cm)")
    ap.add_argument("--yaw-tol", type=float, default=math.radians(2.0), help="rad (2 deg)")
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("carry_check")
    buf = Buffer()
    TransformListener(buf, node)
    got = {}
    node.create_subscription(Odometry, "/odom", lambda m: got.update(odom=m), 10)
    node.create_subscription(JointState, "/stretch/joint_states",
                             lambda m: got.update(js=m), 10)
    end = time.time() + 15.0
    while time.time() < end and not ("odom" in got and "js" in got and buf.can_transform(
            "base_link", "link_grasp_center", Time(), timeout=Duration(seconds=0.0))):
        rclpy.spin_once(node, timeout_sec=0.1)
    if "odom" not in got or "js" not in got:
        print("RESULT FAIL: no /odom or joint states")
        sys.exit(1)
    p, q = got["odom"].pose.pose.position, got["odom"].pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    g = buf.lookup_transform("base_link", "link_grasp_center", Time()).transform.translation
    R = quat_to_mat(0.0, 0.0, q.z, q.w)[:2, :2]
    cup = np.array([p.x, p.y]) + R @ np.array([g.x, g.y])
    t = S.TAPE
    center = np.array([(t["x0"] + t["x1"]) / 2.0, (t["y0"] + t["y1"]) / 2.0])
    # the tape's edges run along the scene's x axis
    line_err = cup[0] - center[0]
    yaw_err = math.atan2(math.sin(yaw), math.cos(yaw))
    js = got["js"]
    finger = js.position[js.name.index("joint_gripper_finger_left")]
    held = finger > CLOSED + HELD_MIN
    ok = abs(line_err) <= args.line_tol and abs(yaw_err) <= args.yaw_tol and held
    print(f"cup at ({cup[0]:+.3f}, {cup[1]:+.3f}); tape center ({center[0]:+.3f}, "
          f"{center[1]:+.3f}); base at ({p.x:+.3f}, {p.y:+.3f}, {math.degrees(yaw):+.1f} deg)")
    print(f"RESULT {'PASS' if ok else 'FAIL'}: cup {line_err * 100:+.1f} cm from the center's "
          f"line, base {math.degrees(yaw_err):+.1f} deg from the tape's edge, center "
          f"{abs(center[1] - cup[1]):.3f} m beyond the cup, fingers {finger:+.3f} "
          f"({'holding' if held else 'NOT holding'})", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
