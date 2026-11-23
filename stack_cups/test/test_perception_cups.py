"""Choosing the cup to pick (roadmap 1.1): several cups, touching cups, the taped area,
tracking, and the red thresholds. Synthetic ray-cast scenes like test_perception_geometry:
a camera 1.2 m up looking along base_link +x, tilted down 30 deg, at a table at 0.75 m."""

import math

import numpy as np
import pytest

from stack_cups import perception as P

W, H = 480, 360
K = [300.0, 0.0, W / 2, 0.0, 300.0, H / 2, 0.0, 0.0, 1.0]
CAM_T = np.array([0.0, 0.0, 1.2])
TABLE_Z = 0.75
EYE = (np.eye(3), np.zeros(3))
RED_BGR = (20, 20, 200)
TAPE_BGR = (230, 89, 25)  # the sim's tape colour (0.10, 0.35, 0.90)


def camera_rotation(tilt=math.radians(30)):
    look = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    c, s = math.cos(tilt), math.sin(tilt)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]) @ look


def render(cups, tape=None, color=RED_BGR):
    """cups: list of (x, y, r) solid cylinders 10 cm tall on the table. tape: (x0, x1, y0,
    y1, width), a taped rectangle outline on the table. Returns (img, depth, R, t)."""
    R = camera_rotation()
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    vs, us = np.mgrid[0:H, 0:W]
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], -1) @ R.T
    depth = np.zeros((H, W), np.float32)
    img = np.zeros((H, W, 3), np.uint8)
    with np.errstate(divide="ignore", invalid="ignore"):
        s_table = (TABLE_Z - CAM_T[2]) / rays[..., 2]
    table = (rays[..., 2] < 0) & (s_table > 0)
    depth[table] = s_table[table]
    img[table] = (120, 120, 120)
    if tape is not None:
        x0, x1, y0, y1, w = tape
        p = CAM_T + s_table[..., None] * rays
        outer = (p[..., 0] > x0 - w) & (p[..., 0] < x1 + w) & (p[..., 1] > y0 - w) & (
            p[..., 1] < y1 + w)
        inner = (p[..., 0] > x0 + w) & (p[..., 0] < x1 - w) & (p[..., 1] > y0 + w) & (
            p[..., 1] < y1 - w)
        img[table & outer & ~inner] = TAPE_BGR
    best = np.where(table, s_table, np.inf)
    for x, y, r in cups:
        ox, oy = CAM_T[0] - x, CAM_T[1] - y
        a = rays[..., 0] ** 2 + rays[..., 1] ** 2
        b = 2 * (ox * rays[..., 0] + oy * rays[..., 1])
        c = ox ** 2 + oy ** 2 - r ** 2
        disc = b ** 2 - 4 * a * c
        with np.errstate(invalid="ignore"):
            s = (-b - np.sqrt(disc)) / (2 * a)
        z = CAM_T[2] + s * rays[..., 2]
        hit = (disc > 0) & (s > 0) & (z >= TABLE_Z) & (z <= TABLE_Z + 0.10) & (s < best)
        best[hit] = s[hit]
        depth[hit] = s[hit]
        img[hit] = color
    return img, depth, R, CAM_T.copy()


def measure(scene, **kw):
    img, depth, R, t = scene
    return P.measure_cup(img, depth, K, K, (R, t), EYE, full=False, **kw)


def test_picks_the_closest_cup_not_the_biggest_blob():
    # the far cup is bigger in the image (r 5 cm at 1.3 m vs 2.5 cm at 0.8 m)
    m = measure(render([(0.8, 0.10, 0.025), (1.3, -0.10, 0.05)]))
    assert len(m["cups"]) == 2
    assert m["cup"][0] == pytest.approx(0.8, abs=0.015)
    assert m["cup"][1] == pytest.approx(0.10, abs=0.015)


def test_touching_cups_are_split_by_depth():
    # two cups side by side, rims touching, one image blob
    scene = render([(1.0, -0.04, 0.04), (1.0, 0.04, 0.04)])
    assert P.find_red_blob(scene[0])[0] is not None
    m = measure(scene)
    ys = sorted(c["cup"][1] for c in m["cups"])
    assert len(ys) == 2
    assert ys[0] == pytest.approx(-0.04, abs=0.015) and ys[1] == pytest.approx(0.04, abs=0.015)
    for c in m["cups"]:
        assert c["cup"][0] == pytest.approx(1.0, abs=0.02)


def test_touching_cups_seen_at_an_angle_are_split():
    # off to the side, one cup is ~3 cm farther than the other along the view
    m = measure(render([(0.9, 0.30, 0.04), (0.9, 0.38, 0.04)]))
    ys = sorted(c["cup"][1] for c in m["cups"])
    assert len(ys) == 2
    assert ys[0] == pytest.approx(0.30, abs=0.015) and ys[1] == pytest.approx(0.38, abs=0.015)


def test_flat_red_points_are_not_a_cup():
    # red pixels whose depth is the table behind (seen at blob edges in the sim)
    rng = np.random.default_rng(1)
    flat = np.column_stack([1.3 + 0.02 * rng.random(40), 0.2 + 0.02 * rng.random(40),
                            np.full(40, TABLE_Z)])
    assert P.split_cups(flat, CAM_T) == []


def test_one_cup_is_not_split():
    m = measure(render([(1.0, 0.0, 0.05)]))
    assert len(m["cups"]) == 1


def test_cups_inside_the_tape_are_ignored():
    tape = (0.75, 0.95, -0.15, 0.05, 0.0125)  # around the near cup
    m = measure(render([(0.85, -0.05, 0.03), (1.3, 0.25, 0.04)], tape=tape))
    assert m["tape_area"] is not None
    assert len(m["cups"]) == 2
    assert m["cup"][0] == pytest.approx(1.3, abs=0.02)  # the near one is already placed


def test_only_placed_cups_is_a_miss():
    tape = (0.75, 1.05, -0.15, 0.15, 0.0125)
    m = measure(render([(0.9, 0.0, 0.03)], tape=tape))
    assert "cup" not in m and "inside the tape" in m["why"]
    # ...unless ignore_taped is off
    m = measure(render([(0.9, 0.0, 0.03)], tape=tape), opts={"ignore_taped": False})
    assert m["cup"][0] == pytest.approx(0.9, abs=0.02)


def test_tracking_keeps_the_same_cup():
    scene = render([(0.8, 0.10, 0.03), (1.2, -0.10, 0.03)])
    m = measure(scene, target=np.array([1.21, -0.09]))
    assert m["cup"][0] == pytest.approx(1.2, abs=0.02)  # not the closer one
    m = measure(scene, target=np.array([1.0, 0.5]))
    assert "cup" not in m and "lost the target" in m["why"]


def test_red_thresholds_are_settable():
    pink = (150, 150, 230)  # HSV saturation 89: below the default sat_min 120
    scene = render([(1.0, 0.0, 0.04)], color=pink)
    assert "cup" not in measure(scene)
    m = measure(scene, opts={"red": {"hue_width": 10, "sat_min": 60, "val_min": 70}})
    assert m["cup"][0] == pytest.approx(1.0, abs=0.02)


def test_cup_height_is_measured_from_the_table():
    img, depth, R, t = render([(1.0, 0.0, 0.04)])  # 10 cm tall
    m = P.measure_cup(img, depth, K, K, (R, t), EYE, full=True)
    assert m["table_top"] == pytest.approx(TABLE_Z, abs=0.005)
    assert m["cup_height"] == pytest.approx(0.10, abs=0.01)
