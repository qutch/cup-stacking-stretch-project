"""Pure-numpy perception helpers for the cup task (no ROS imports, so they are
easy to unit test).

Conventions:
- Depth images are float32 meters (the Stretch sim publishes 32FC1); 0 / nan = no reading.
- K is the 3x3 intrinsic matrix from CameraInfo.k.
- Points are Nx3 arrays. "Optical frame" = x right, y down, z forward.
- (R, t) maps optical-frame points into base_link: p_base = R @ p_opt + t.
"""

import cv2
import numpy as np

# Same HSV red thresholds the original find_cup node used
RED_RANGES = (
    ((0, 120, 70), (10, 255, 255)),
    ((170, 120, 70), (180, 255, 255)),
)
MIN_AREA = 100

MIN_DEPTH = 0.1
MAX_DEPTH = 3.0


def red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in RED_RANGES:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def find_red_blob(bgr, min_area=MIN_AREA):
    """Bounding box (x, y, w, h) of the biggest red blob, or None. Also returns the mask."""
    mask = red_mask(bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) <= min_area:
        return None, mask
    return cv2.boundingRect(biggest), mask


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def backproject(depth, K, step=1, region=None):
    """Depth pixels -> Nx3 optical-frame points. region=(x, y, w, h) limits it to a box."""
    K = np.asarray(K, dtype=float).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = 0, 0, w, h
    if region is not None:
        rx, ry, rw, rh = region
        x0, y0 = max(0, rx), max(0, ry)
        x1, y1 = min(w, rx + rw), min(h, ry + rh)
    vs, us = np.mgrid[y0:y1:step, x0:x1:step]
    z = depth[y0:y1:step, x0:x1:step].astype(float)
    ok = np.isfinite(z) & (z > MIN_DEPTH) & (z < MAX_DEPTH)
    z, us, vs = z[ok], us[ok], vs[ok]
    return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)


def to_base(points, R, t):
    return points @ np.asarray(R).T + np.asarray(t)


def project(points, K):
    """Nx3 optical-frame points -> Nx2 pixel coordinates (u, v)."""
    K = np.asarray(K, dtype=float).reshape(3, 3)
    z = np.maximum(points[:, 2], 1e-6)
    return np.stack([K[0, 0] * points[:, 0] / z + K[0, 2],
                     K[1, 1] * points[:, 1] / z + K[1, 2]], axis=1)


def locate_cup(depth, K_depth, R, t, mask, K_color, R_cd, t_cd, bbox, pad=15):
    """3D cup center in base_link.

    Depth points near the cup's bounding box are moved into the color camera's frame
    (R_cd, t_cd: depth optical -> color optical) and projected into the color image;
    only points landing on red pixels of `mask` are kept. That handles the small
    offset between the depth and color cameras and ignores the table around the cup.
    The visible surface is pushed back by the cup's estimated radius to get its
    center. Returns (center_xyz, radius) or (None, None).
    """
    x, y, w, h = bbox
    pts = backproject(depth, K_depth, region=(x - pad, y - pad, w + 2 * pad, h + 2 * pad))
    if len(pts) < 10:
        return None, None
    uv = np.round(project(to_base(pts, R_cd, t_cd), K_color)).astype(int)
    H, W = mask.shape[:2]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    on_red = np.zeros(len(pts), bool)
    on_red[inside] = mask[uv[inside, 1], uv[inside, 0]] > 0
    pts = pts[on_red]
    if len(pts) < 8:
        return None, None
    base_pts = to_base(pts, R, t)
    # Radius from the cup's horizontal extent seen across the image
    horiz = base_pts[:, :2] - base_pts[:, :2].mean(axis=0)
    cam_dir = base_pts[:, :2].mean(axis=0) - np.asarray(t, dtype=float)[:2]
    cam_dir /= max(np.linalg.norm(cam_dir), 1e-6)
    across = np.array([-cam_dir[1], cam_dir[0]])
    widths = horiz @ across
    spread = np.percentile(widths, 95) - np.percentile(widths, 5)
    radius = float(np.clip(spread / 2, 0.015, 0.08))
    surface = base_pts.mean(axis=0)
    cam = np.asarray(t, dtype=float)
    away = surface[:2] - cam[:2]
    away /= max(np.linalg.norm(away), 1e-6)
    center = surface.copy()
    center[:2] += away * radius
    return center, radius


def estimate_table_top(points_base, cup_center, cup_radius, ring=(0.03, 0.20), bin_m=0.01):
    """Height of the surface the cup stands on.

    Takes points in a ring around the cup (not the cup itself, not above its center)
    and returns the most common height: a flat table top piles many points into one
    height bin, while the table's side face or nearby objects spread across many.
    None if there is not enough data.
    """
    d = np.linalg.norm(points_base[:, :2] - cup_center[:2], axis=1)
    sel = (d > cup_radius + ring[0]) & (d < cup_radius + ring[1])
    sel &= (points_base[:, 2] > 0.05) & (points_base[:, 2] < cup_center[2])
    z = points_base[sel, 2]
    if len(z) < 30:
        return None
    edges = np.arange(z.min(), z.max() + 2 * bin_m, bin_m)
    counts, edges = np.histogram(z, bins=edges)
    i = int(np.argmax(counts))
    in_bin = z[(z >= edges[i]) & (z <= edges[i + 1])]
    return float(np.median(in_bin))


def table_edge_distance(points_base, x_center, half_width, z_max, side=-1):
    """Distance from the base center to the nearest obstacle on the arm side
    (base_link -y when side=-1), within a corridor around x_center, from just above
    the floor up to z_max. None if nothing is seen."""
    p = points_base
    sel = (np.abs(p[:, 0] - x_center) < half_width) & (p[:, 2] > 0.05) & (p[:, 2] < z_max)
    sel &= (p[:, 1] * side) > 0.22  # ignore the robot's own base (~0.17 m half width)
    if np.count_nonzero(sel) < 20:
        return None
    return float(np.percentile(p[sel, 1] * side, 3))


def plan_reach(cup, cup_radius, table_top, edge_dist, offsets, *,
               finger_clearance=0.035, approach_gap=0.0, max_ext=0.50,
               lift_range=(0.05, 1.05), retracted_tip=0.03, edge_margin=0.03):
    """Lift height and arm extension that put the grasp center at the cup.

    offsets: dict measured from TF with the wrist straight out:
      h = grasp_z - lift, c = -grasp_y - extension (arm reaches along base -y).
    Safety rules (all from depth):
      - grasp height >= table top + finger_clearance, so the fingers clear the table
      - the retracted gripper must be outside the table edge before it is lowered
      - the extension must be within the arm's range
    Returns (plan_dict, None) or (None, reason).
    """
    if table_top is None:
        return None, "could not measure the table top from depth"
    grasp_z = max(cup[2], table_top + finger_clearance)
    lift = grasp_z - offsets["h"]
    if not lift_range[0] <= lift <= lift_range[1]:
        return None, f"needed lift {lift:.2f} m is outside {lift_range}"
    ext = (-cup[1] - approach_gap) - offsets["c"]
    if ext < 0:
        return None, f"cup is too close to the base (extension {ext:.2f} m)"
    if ext > max_ext:
        return None, f"cup is out of reach (needs {ext:.2f} m, max {max_ext:.2f} m)"
    tip = offsets["c"] + retracted_tip
    if edge_dist is not None and tip > edge_dist - edge_margin:
        return None, (f"table edge is {edge_dist:.2f} m away but the retracted gripper "
                      f"reaches {tip:.2f} m; lowering it would hit the table")
    return {"lift": lift, "extension": ext, "grasp_z": grasp_z}, None
