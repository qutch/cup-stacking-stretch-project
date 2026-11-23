"""Pure-numpy motion math for the cup task (no ROS imports, so it is easy to unit test).

Conventions: odom poses are (x, y, yaw) with yaw in radians; distances in meters.
"""

import numpy as np


def base_speed(remaining, v_max, v_min, decel, tol):
    """Speed command for a base move with `remaining` meters to go (signed).

    Slows down as sqrt(2 * decel * distance) so the base can stop in time, but never
    below v_min (the slowest command that actually moves the base) or above v_max.
    Returns 0 once within `tol` of the target.
    """
    d = abs(remaining)
    if d < tol:
        return 0.0
    return float(np.sign(remaining) * np.clip(np.sqrt(2.0 * decel * d), v_min, v_max))


def progress_along(start, xy):
    """How far the base has moved from `start` (x, y, yaw) along its starting heading."""
    x0, y0, yaw = start
    return float((xy[0] - x0) * np.cos(yaw) + (xy[1] - y0) * np.sin(yaw))


def yaw_reach(arm_len, max_yaw):
    """Largest sideways shift of the grasp point the wrist yaw can make."""
    return float(arm_len * np.sin(max_yaw))


def yaw_for_offset(dx, arm_len, max_yaw, sign):
    """Wrist yaw that shifts the grasp point by `dx` along the base's x axis, or None if
    that needs more than `max_yaw`. `sign` is +1 or -1: which way a positive yaw shifts it."""
    if abs(dx) > yaw_reach(arm_len, max_yaw):
        return None
    return float(sign * np.arcsin(dx / arm_len))


def robust_center(points, max_spread):
    """Median of Nx3 points, the largest xy distance of any point from it, and whether
    that spread is within `max_spread`."""
    points = np.asarray(points, dtype=float)
    center = np.median(points, axis=0)
    spread = float(np.max(np.linalg.norm(points[:, :2] - center[:2], axis=1)))
    return center, spread, spread <= max_spread


def base_to_odom(p_xy, pose):
    """A point's xy in base_link -> its xy in odom, given the base pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    px, py = float(p_xy[0]), float(p_xy[1])
    return np.array([x + c * px - s * py, y + s * px + c * py])


def odom_to_base(p_xy, pose):
    """A point's xy in odom -> its xy in base_link, given the base pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = float(p_xy[0]) - x, float(p_xy[1]) - y
    return np.array([c * dx + s * dy, -s * dx + c * dy])
