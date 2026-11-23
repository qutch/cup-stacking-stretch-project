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

# HSV thresholds (OpenCV hue 0-179). Red wraps around hue 0: hue <= width or >= 180 -
# width. These defaults are the original find_cup node's; params.py makes them settable.
RED = {"hue_width": 10, "sat_min": 120, "val_min": 70}
# The tape around the stacking area (blue in the sim)
TAPE = {"hue_min": 95, "hue_max": 130, "sat_min": 120, "val_min": 60}
MIN_AREA = 100

MIN_DEPTH = 0.1
MAX_DEPTH = 3.0


def depth_in_meters(depth, encoding):
    """A depth image as float32 meters. The real D435i sends 16UC1 millimeters; the sim
    sends 32FC1 meters. Zero (no reading) stays zero."""
    d = np.asarray(depth)
    if encoding in ("16UC1", "mono16"):
        return d.astype(np.float32) / 1000.0
    return d.astype(np.float32)


def _in_range(hsv, lo, hi):
    return cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))


def red_mask(bgr, hue_width=RED["hue_width"], sat_min=RED["sat_min"], val_min=RED["val_min"]):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(_in_range(hsv, (0, sat_min, val_min), (hue_width, 255, 255)),
                          _in_range(hsv, (180 - hue_width, sat_min, val_min), (180, 255, 255)))


def tape_mask(bgr, hue_min=TAPE["hue_min"], hue_max=TAPE["hue_max"], sat_min=TAPE["sat_min"],
              val_min=TAPE["val_min"]):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return _in_range(hsv, (hue_min, sat_min, val_min), (hue_max, 255, 255))


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
    pts, _ = points_on_mask(depth, K_depth, mask, K_color, R_cd, t_cd,
                            region=(x - pad, y - pad, w + 2 * pad, h + 2 * pad))
    if len(pts) < 8:
        return None, None
    return cup_from_points(to_base(pts, R, t), t)


def points_on_mask(depth, K_depth, mask, K_color, R_cd, t_cd, step=1, region=None):
    """Depth points (depth optical frame) that land on nonzero pixels of `mask` (a color
    image mask) once moved into the color camera's frame, and their color pixels (u, v)."""
    pts = backproject(depth, K_depth, step=step, region=region)
    if len(pts) == 0:
        return pts, np.zeros((0, 2), int)
    uv = np.round(project(to_base(pts, R_cd, t_cd), K_color)).astype(int)
    H, W = mask.shape[:2]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    keep = np.zeros(len(pts), bool)
    keep[inside] = mask[uv[inside, 1], uv[inside, 0]] > 0
    return pts[keep], uv[keep]


def cup_from_points(base_pts, t):
    """(center, radius) of a cup from its visible red points in base_link (camera at t)."""
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


def table_edge_line(points_base, side=-1, z_range=(0.4, 1.2), bin_x=0.02, x_half=0.6,
                    min_bins=6):
    """The table's near edge on the arm side (base_link -y when side=-1), from depth
    (roadmap 1.3b). The table top is the most common height in z_range; for every
    bin_x slice along x, the edge is the nearest table-top point; a line is fitted to
    those, dropping outliers. Returns {"angle": rad to turn the base (counterclockwise
    +) to be square with the edge, "distance": edge distance from the base center at
    x=0, "top": table top height, "bins": slices used}, or None."""
    p = np.asarray(points_base)
    sel = ((p[:, 1] * side) > 0.22) & (p[:, 2] > z_range[0]) & (p[:, 2] < z_range[1])
    sel &= np.abs(p[:, 0]) < x_half
    p = p[sel]
    if len(p) < 50:
        return None
    edges = np.arange(z_range[0], z_range[1] + 0.01, 0.01)
    counts, edges = np.histogram(p[:, 2], bins=edges)
    i = int(np.argmax(counts))
    top = float(np.median(p[(p[:, 2] >= edges[i]) & (p[:, 2] <= edges[i + 1]), 2]))
    p = p[np.abs(p[:, 2] - top) < 0.015]
    near = p[:, 1] * side  # distance out from the base on the arm side
    xb = np.floor(p[:, 0] / bin_x).astype(int)
    xs, es = [], []
    for b in np.unique(xb):
        k = xb == b
        if np.count_nonzero(k) >= 5:
            xs.append((b + 0.5) * bin_x)
            es.append(float(near[k].min()))
    if len(xs) < min_bins:
        return None
    xs, es = np.array(xs), np.array(es)
    # start from the slices near the median edge distance: a few slices can see
    # something nearer at table height (the robot's own arm sticking out, a cup)
    keep = np.abs(es - np.median(es)) < 0.03
    for _ in range(3):
        if np.count_nonzero(keep) < min_bins:
            return None
        a, b = np.polyfit(xs[keep], es[keep], 1)
        keep = np.abs(es - (a * xs + b)) < 0.015
    if np.count_nonzero(keep) < min_bins:
        return None
    a, b = np.polyfit(xs[keep], es[keep], 1)
    # the edge runs along (1, side * a) in base_link: turn by that angle to face along it
    return {"angle": float(np.arctan(side * a)), "distance": float(b), "top": top,
            "bins": int(np.count_nonzero(keep))}


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


def split_cups(pts, cam_xy, bin_m=0.01, min_rise=0.015, gap=0.03, min_pts=8,
               min_height=0.03):
    """Split one red blob's points (base_link) into one group per cup.

    Seen from the camera, each cup's front surface is an arc: nearest at the cup's middle,
    farther toward its sides. Across the image (perpendicular to the view direction) the
    front-surface distance of two cups standing side by side has two dips with a bump of
    about a radius where they touch; split at bumps more than `min_rise` above the line
    joining the two dips (seen at an angle, one cup is farther than the other). Also
    split where the points jump more than `gap` along the view (a cup seen above or
    behind another) or across it (two blobs the image joined). Groups under `min_pts`
    points or less than `min_height` tall are dropped: red-edged pixels can carry the
    depth of the table behind a cup, and those points are flat. Limit: a cup hidden
    straight behind another is not separated."""
    if len(pts) < min_pts:
        return []
    cam_xy = np.asarray(cam_xy, float)[:2]
    view = pts[:, :2].mean(axis=0) - cam_xy
    view /= max(np.linalg.norm(view), 1e-6)
    rel = pts[:, :2] - cam_xy
    along = rel @ view
    across = rel @ np.array([-view[1], view[0]])
    out = []

    def by_gaps(idx, values):
        order = idx[np.argsort(values[idx])]
        cuts = np.where(np.diff(values[order]) > gap)[0] + 1
        return np.split(order, cuts)

    def by_bumps(idx):
        lo = across[idx].min()
        b = ((across[idx] - lo) / bin_m).astype(int)
        nb = b.max() + 1
        front = np.full(nb, np.inf)
        np.minimum.at(front, b, along[idx])
        if nb >= 5:
            best, cut = min_rise, None
            for i in range(2, nb - 2):
                li = int(np.argmin(front[:i]))
                ri = i + 1 + int(np.argmin(front[i + 1:]))
                if not (np.isfinite(front[i]) and np.isfinite(front[li])
                        and np.isfinite(front[ri])):
                    continue
                chord = front[li] + (front[ri] - front[li]) * (i - li) / (ri - li)
                rise = front[i] - chord
                if rise > best:
                    best, cut = rise, i
            if cut is not None:
                return by_bumps(idx[b < cut]) + by_bumps(idx[b >= cut])
        return [idx]

    for g in by_gaps(np.arange(len(pts)), along):
        for h in by_gaps(g, across):
            for k in by_bumps(h):
                z = pts[k, 2]
                if len(k) >= min_pts and np.percentile(z, 95) - np.percentile(z, 5) >= min_height:
                    out.append(k)
    return [pts[k] for k in out]


def tape_area(depth, K_depth, img, K_color, T_base, T_cd, opts=None, min_pts=30):
    """The taped stacking area as a convex polygon (Nx2 float32, base_link xy), or None
    when too little tape is in view."""
    mask = tape_mask(img, **(opts or {}))
    if np.count_nonzero(mask) < MIN_AREA:
        return None
    pts, _ = points_on_mask(depth, K_depth, mask, K_color, *T_cd, step=2)
    if len(pts) < min_pts:
        return None
    xy = to_base(pts, *T_base)[:, :2].astype(np.float32)
    hull = cv2.convexHull(xy).reshape(-1, 2)
    return hull if len(hull) >= 3 and cv2.contourArea(hull) > 1e-3 else None


def in_area(xy, area):
    return area is not None and cv2.pointPolygonTest(
        area.reshape(-1, 1, 2), (float(xy[0]), float(xy[1])), False) >= 0


def find_cups(img, depth, K_color, K_depth, T_base, T_cd, red=None, min_area=MIN_AREA,
              split=None):
    """Every red cup in view: a list of dicts with cup (center xyz, base_link), radius,
    bbox (in the color image) and n (points). Also returns the red mask."""
    mask = red_mask(img, **(red or {}))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    R, t = T_base
    cups = []
    for k in range(1, n):
        x, y, w, h, area = stats[k]
        if area <= min_area:
            continue
        pad = 15
        blob = (labels == k).astype(np.uint8)
        region = (x - pad, y - pad, w + 2 * pad, h + 2 * pad)
        pts, _ = points_on_mask(depth, K_depth, blob, K_color, *T_cd, region=region)
        if len(pts) < 8:
            continue
        base = to_base(pts, R, t)
        for group in split_cups(base, t, **(split or {})):
            center, radius = cup_from_points(group, t)
            g_uv = np.round(project(_base_to_color(group, R, t, T_cd), K_color)).astype(int)
            gx, gy = g_uv.min(axis=0)
            gw, gh = g_uv.max(axis=0) - (gx, gy) + 1
            # its highest red point (the rim, for an upright cup; roadmap 1.5)
            cups.append({"cup": center, "radius": radius, "n": len(group),
                         "top": float(np.percentile(group[:, 2], 98)),
                         "bbox": (int(gx), int(gy), int(gw), int(gh))})
    return cups, mask


def _base_to_color(pts_base, R, t, T_cd):
    """base_link points -> color optical frame (inverse of depth->base, then depth->color)."""
    depth_opt = (np.asarray(pts_base) - t) @ R
    return to_base(depth_opt, *T_cd)


def choose_cup(cups, area=None, target=None, track_radius=0.08):
    """The cup to pick: not inside the tape `area`; the one nearest `target` (xy in
    base_link, its last position) if given, else the one closest to the robot.
    Returns (cup or None, why)."""
    loose = [c for c in cups if not in_area(c["cup"][:2], area)]
    placed = len(cups) - len(loose)
    note = f" ({placed} inside the tape)" if placed else ""
    if not loose:
        return None, (f"no loose cup: all {len(cups)} red cups are inside the tape" if cups
                      else "no red cup found")
    if target is not None:
        d = [np.hypot(*(c["cup"][:2] - target[:2])) for c in loose]
        i = int(np.argmin(d))
        if d[i] > track_radius:
            return None, (f"lost the target cup: the nearest loose cup is {d[i] * 100:.0f} cm "
                          f"from where it was{note}")
        return loose[i], None
    return min(loose, key=lambda c: np.hypot(*c["cup"][:2])), None


def measure_cup(img, depth, K_color, K_depth, T_base, T_cd, full=True, target=None,
                opts=None):
    """One cup measurement from one color + depth frame pair.

    T_base: (R, t) depth optical frame -> base_link; T_cd: depth optical -> color optical.
    Returns a dict with img, mask and bbox, plus cup (center xyz in base_link) and
    radius when the cup was found, and with `full` also table_top and edge (see
    estimate_table_top and table_edge_distance). If the cup was not found, "why" says
    why."""
    o = opts or {}
    R, t = T_base
    cups, mask = find_cups(img, depth, K_color, K_depth, T_base, T_cd, red=o.get("red"),
                           min_area=o.get("min_area", MIN_AREA), split=o.get("split"))
    out = {"img": img, "mask": mask, "bbox": None, "cups": cups}
    if not cups:
        red = int(np.count_nonzero(mask))
        out["why"] = (f"no red blob in the image ({red} red pixels)" if red <= o.get(
            "min_area", MIN_AREA) else f"red pixels ({red}) but too few depth points on them")
        return out
    area = (tape_area(depth, K_depth, img, K_color, T_base, T_cd, o.get("tape"))
            if o.get("ignore_taped", True) else None)
    chosen, why = choose_cup(cups, area, target, o.get("track_radius", 0.08))
    out["tape_area"] = area
    if chosen is None:
        out["why"] = why
        return out
    cup, radius = chosen["cup"], chosen["radius"]
    out.update(cup=cup, radius=radius, bbox=chosen["bbox"], cup_top=chosen["top"])
    if full:
        pts = to_base(backproject(depth, K_depth, step=2), R, t)
        top = estimate_table_top(pts, cup, radius)
        edge = table_edge_distance(pts, cup[0], 0.3,
                                   (top if top is not None else cup[2]) + 0.02)
        out.update(table_top=top, edge=edge)
        if top is not None:
            out["cup_height"] = chosen["top"] - top
    return out
