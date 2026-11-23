"""Find the blue tape that marks the stacking area (roadmap 2.1). Pure numpy/OpenCV, no ROS,
so it is easy to test.

The stacking area is a rectangle outlined with blue tape on the table. From one color +
depth frame: the tape-coloured pixels (perception.tape_mask, the same colour that find_cup
uses to skip placed cups; the tape_* parameters), the depth points that land on them (moved
into base_link), only those at the tape's height, then the smallest rectangle around them
(cv2.minAreaRect). That gives the area's center and the direction of its edges, also when
part of the tape is hidden (e.g. by the cup in the gripper): the rectangle only needs the
tape's corners.
"""

import cv2
import numpy as np

from stack_cups import perception as P

MIN_POINTS = 50  # depth points on the tape needed for a measurement
PLANE_TOL = 0.02  # m: keep tape points this close to their median height


def fold_direction(angle):
    """A direction (a line, so angle and angle + pi are the same) as an angle in
    [-pi/2, pi/2)."""
    return float((angle + np.pi / 2) % np.pi - np.pi / 2)


def rectangle_edges(corners):
    """For a rectangle's 4 corners (in order around it): the direction of the edge closest
    to the x axis (yaw, in [-pi/4, pi/4]), that edge's length and the other edge's length."""
    c = np.asarray(corners, dtype=float)
    e1, e2 = c[1] - c[0], c[2] - c[1]
    a1, a2 = (fold_direction(np.arctan2(e[1], e[0])) for e in (e1, e2))
    l1, l2 = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
    if abs(a1) <= abs(a2):
        return a1, l1, l2
    return a2, l2, l1


def tape_points(img, depth, K_color, K_depth, T_base, T_cd, step=2, colour=None):
    """Nx3 base_link points of the depth image that land on tape-coloured pixels of the
    color image, and the tape mask. T_base: (R, t) depth optical -> base_link; T_cd: depth
    optical -> color optical (the two cameras sit a little apart). colour: tape_mask's
    hue_min, hue_max, sat_min, val_min (default: perception.TAPE)."""
    mask = P.tape_mask(img, **(colour or {}))
    pts = P.backproject(depth, K_depth, step=step)
    if len(pts) == 0:
        return np.zeros((0, 3)), mask
    uv = np.round(P.project(P.to_base(pts, *T_cd), K_color)).astype(int)
    h, w = mask.shape[:2]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    on_blue = np.zeros(len(pts), bool)
    on_blue[inside] = mask[uv[inside, 1], uv[inside, 0]] > 0
    return P.to_base(pts[on_blue], *T_base), mask


def locate_tape(img, depth, K_color, K_depth, T_base, T_cd, step=2, min_points=MIN_POINTS,
                colour=None):
    """The taped area from one color + depth frame (colour: see tape_points).

    Returns a dict with img and mask, and when the tape was found: tape (the area's center
    in base_link; z = the tape's height), yaw (direction of the edge closest to base_link's
    x axis, in [-pi/4, pi/4]; turning the base by yaw makes it parallel to that edge),
    length (of that edge, m), width (of the other edge, m), corners (4x2 base_link xy) and
    n (points used). Otherwise "why" says what went wrong."""
    pts, mask = tape_points(img, depth, K_color, K_depth, T_base, T_cd, step, colour)
    out = {"img": img, "mask": mask}
    if len(pts) < min_points:
        out["why"] = (f"too little blue tape in view ({int(np.count_nonzero(mask))} blue pixels, "
                      f"{len(pts)} depth points on them)")
        return out
    z = float(np.median(pts[:, 2]))
    pts = pts[np.abs(pts[:, 2] - z) < PLANE_TOL]
    if len(pts) < min_points:
        out["why"] = f"only {len(pts)} blue points at the tape's height ({z:.3f} m)"
        return out
    rect = cv2.minAreaRect(pts[:, :2].astype(np.float32))
    corners = cv2.boxPoints(rect).astype(float)
    yaw, length, width = rectangle_edges(corners)
    out.update(tape=np.array([rect[0][0], rect[0][1], z]), yaw=yaw, length=length,
               width=width, corners=corners, n=len(pts))
    return out
