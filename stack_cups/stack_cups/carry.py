"""Math for carrying a cup to the stacking area (roadmap task 2). No ROS, so it is easy to
test: poses in the map, the robot's outline while it holds a cup out on its arm, the grip
check, and pointing the head.

Poses are (x, y, yaw) with yaw in radians; xy points are in base_link unless noted.
"""

import math

import cv2
import numpy as np


def wrap(angle):
    """An angle in [-pi, pi)."""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def compose(a, b):
    """Pose b, given relative to pose a, in the frame a is given in. E.g. the robot's map
    pose = compose(where odometry started in the map, the odometry pose)."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], wrap(yaw + b[2]))


def circle(cx, cy, radius, n=16):
    """n points on a circle, counterclockwise."""
    return [(cx + radius * math.cos(2.0 * math.pi * k / n),
             cy + radius * math.sin(2.0 * math.pi * k / n)) for k in range(n)]


def carry_outline(robot_radius, cup_xy, cup_radius, margin, n=16):
    """The robot's outline (for Nav2's costmaps) while it holds a cup out on its arm: the
    convex hull of the base (the circle of robot_radius Nav2 uses with the arm stowed) and
    the cup, grown by `margin`. The hull also covers the arm between them. Counterclockwise
    list of (x, y) in base_link."""
    pts = np.array(circle(0.0, 0.0, robot_radius, n)
                   + circle(cup_xy[0], cup_xy[1], cup_radius + margin, n), np.float32)
    hull = cv2.convexHull(pts, clockwise=False)[:, 0, :]
    return [(float(x), float(y)) for x, y in hull]


def is_holding(finger, closed, min_open):
    """The fingers stopped at least `min_open` short of fully closed: something is held."""
    return finger > closed + min_open


def grip_lost(start, now, closed, min_open, open_tol):
    """Is the cup gone? Yes if the fingers closed to within `min_open` of fully closed
    (nothing between them any more: it fell out) or opened more than `open_tol` from
    `start`, where they held it (the gripper let go). A tapered cup squeezed by the fingers
    slowly creeps toward its wide end, so the fingers close a little while it is still
    held; that is not a loss."""
    return now <= closed + min_open or now - start > open_tol


def look_at(eye, target):
    """Head pan and tilt (rad) that point the camera from `eye` at `target` (xyz in
    base_link). Pan 0 looks along +x, -pi/2 to the arm side (-y); tilt < 0 looks down."""
    dx, dy, dz = (float(v) for v in np.asarray(target, float) - np.asarray(eye, float))
    return math.atan2(dy, dx), math.atan2(dz, math.hypot(dx, dy))


def reach_for(target_y, arm_c):
    """Arm extension that puts the grasp point at base_link y = target_y (< 0: the arm
    side). arm_c: grasp point distance beyond the extension (CupRobot.arm_offsets()["c"])."""
    return -target_y - arm_c
