"""Unit tests for perception.py (roadmap 0.5): red detection, depth geometry, the table and
reach-safety checks, on synthetic images with a known answer.

The synthetic scene: a camera 1.2 m up, looking along base_link +x and tilted down 30 deg,
sees a red cylinder (the cup) standing on a table top. Depth is rendered by ray casting, so
the true cup position and table height are known exactly.
"""

import math

import numpy as np
import pytest

from stack_cups import perception as P

W, H = 320, 240
K = [250.0, 0.0, W / 2, 0.0, 250.0, H / 2, 0.0, 0.0, 1.0]
CAM_T = np.array([0.0, 0.0, 1.2])
TABLE_Z = 0.75
CUP = {"x": 1.0, "y": 0.05, "r": 0.04, "z0": TABLE_Z, "z1": TABLE_Z + 0.10}


def camera_rotation(tilt=math.radians(30)):
    """Optical (x right, y down, z forward) -> base_link, looking along +x, tilted down."""
    look = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    c, s = math.cos(tilt), math.sin(tilt)
    pitch_down = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])  # about base y
    return pitch_down @ look


def render(cup=CUP, tilt=math.radians(30)):
    """(bgr image, depth image in meters, R, t): a red cup on a gray table."""
    R = camera_rotation(tilt)
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    vs, us = np.mgrid[0:H, 0:W]
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], -1) @ R.T
    depth = np.zeros((H, W), np.float32)
    img = np.zeros((H, W, 3), np.uint8)
    # table plane z = TABLE_Z (only where the ray points down)
    with np.errstate(divide="ignore", invalid="ignore"):
        s_table = (TABLE_Z - CAM_T[2]) / rays[..., 2]
    table = (rays[..., 2] < 0) & (s_table > 0)
    depth[table] = s_table[table]
    img[table] = (120, 120, 120)
    # vertical cylinder: |(t + s*ray)_xy - c_xy| = r, nearest hit with z in [z0, z1]
    ox, oy = CAM_T[0] - cup["x"], CAM_T[1] - cup["y"]
    a = rays[..., 0] ** 2 + rays[..., 1] ** 2
    b = 2 * (ox * rays[..., 0] + oy * rays[..., 1])
    c = ox ** 2 + oy ** 2 - cup["r"] ** 2
    disc = b ** 2 - 4 * a * c
    with np.errstate(invalid="ignore"):
        s_cyl = (-b - np.sqrt(disc)) / (2 * a)
    z_hit = CAM_T[2] + s_cyl * rays[..., 2]
    hit = (disc > 0) & (s_cyl > 0) & (z_hit >= cup["z0"]) & (z_hit <= cup["z1"])
    hit &= ~table | (s_cyl < s_table)
    depth[hit] = s_cyl[hit]
    img[hit] = (20, 20, 200)  # red in BGR
    return img, depth, R, CAM_T.copy()


# ------------------------------------------------------------------ red detection
def test_find_red_blob_picks_the_biggest_red_area():
    img = np.zeros((H, W, 3), np.uint8)
    img[10:20, 10:20] = (0, 0, 255)  # small red square
    img[100:140, 200:250] = (0, 0, 255)  # big one
    bbox, mask = P.find_red_blob(img)
    assert bbox == (200, 100, 50, 40)
    assert mask[120, 220] > 0 and mask[0, 0] == 0


def test_find_red_blob_ignores_small_and_non_red():
    img = np.zeros((H, W, 3), np.uint8)
    img[10:15, 10:15] = (0, 0, 255)  # 25 px: below MIN_AREA
    img[50:90, 50:90] = (255, 0, 0)  # blue
    assert P.find_red_blob(img)[0] is None


def test_red_mask_covers_both_ends_of_the_hue_circle():
    hues = np.array([[[0, 200, 200], [178, 200, 200], [60, 200, 200]]], np.uint8)
    import cv2
    bgr = cv2.cvtColor(hues, cv2.COLOR_HSV2BGR)
    assert list(P.red_mask(bgr)[0] > 0) == [True, True, False]


# --------------------------------------------------------------- depth geometry
def test_depth_in_meters_handles_the_robot_and_the_sim():
    mm = np.array([[0, 1250, 65535]], np.uint16)  # real D435i: 16UC1 millimeters
    assert P.depth_in_meters(mm, "16UC1") == pytest.approx(np.array([[0.0, 1.25, 65.535]]))
    m = np.array([[0.0, 1.25]], np.float32)  # sim: 32FC1 meters
    assert P.depth_in_meters(m, "32FC1") == pytest.approx(m)
    assert P.depth_in_meters(m, "32FC1").dtype == np.float32


def test_backproject_then_project_returns_the_same_pixels():
    depth = np.full((H, W), 2.0, np.float32)
    pts = P.backproject(depth, K, step=7)
    uv = P.project(pts, K)
    vs, us = np.mgrid[0:H:7, 0:W:7]
    assert np.allclose(uv[:, 0], us.ravel()) and np.allclose(uv[:, 1], vs.ravel())


def test_backproject_drops_missing_and_out_of_range_depth():
    depth = np.array([[0.0, np.nan, 0.05, 1.0, 5.0]], np.float32)
    assert len(P.backproject(depth, K)) == 1


def test_locate_cup_finds_the_cylinder_center_and_radius():
    img, depth, R, t = render()
    bbox, mask = P.find_red_blob(img)
    center, radius = P.locate_cup(depth, K, R, t, mask, K, np.eye(3), np.zeros(3), bbox)
    assert center is not None
    assert center[0] == pytest.approx(CUP["x"], abs=0.01)
    assert center[1] == pytest.approx(CUP["y"], abs=0.01)
    assert (CUP["z0"] < center[2] < CUP["z1"])
    assert radius == pytest.approx(CUP["r"], abs=0.01)


def test_estimate_table_top_from_points_around_the_cup():
    img, depth, R, t = render()
    pts = P.to_base(P.backproject(depth, K, step=2), R, t)
    top = P.estimate_table_top(pts, np.array([CUP["x"], CUP["y"], 0.8]), CUP["r"])
    assert top == pytest.approx(TABLE_Z, abs=0.005)


def test_estimate_table_top_needs_enough_points():
    assert P.estimate_table_top(np.zeros((5, 3)), np.array([1.0, 0.0, 0.8]), 0.04) is None


def test_table_edge_distance_on_the_arm_side():
    # a wall of points 0.5 m to the robot's right (-y), from the floor up
    ys = -0.5 - 0.1 * np.random.default_rng(0).random(200)
    pts = np.stack([np.linspace(-0.2, 0.2, 200), ys, np.linspace(0.1, 0.7, 200)], 1)
    d = P.table_edge_distance(pts, x_center=0.0, half_width=0.3, z_max=0.8)
    assert d == pytest.approx(0.5, abs=0.01)
    assert P.table_edge_distance(pts[:5], 0.0, 0.3, 0.8) is None


def test_measure_cup_end_to_end():
    img, depth, R, t = render()
    m = P.measure_cup(img, depth, K, K, (R, t), (np.eye(3), np.zeros(3)), full=True)
    assert "why" not in m
    assert m["cup"][0] == pytest.approx(CUP["x"], abs=0.01)
    assert m["cup"][1] == pytest.approx(CUP["y"], abs=0.01)
    assert m["table_top"] == pytest.approx(TABLE_Z, abs=0.005)


def test_measure_cup_explains_a_miss():
    img, depth, R, t = render()
    img[:] = (120, 120, 120)  # no red anywhere
    m = P.measure_cup(img, depth, K, K, (R, t), (np.eye(3), np.zeros(3)))
    assert "cup" not in m and "no red blob" in m["why"]


# -------------------------------------------------------------- reach safety
OFFS = {"h": 0.11, "c": 0.415, "x": 0.0}


def test_plan_reach_puts_the_grasp_point_at_the_cup():
    plan, why = P.plan_reach(np.array([0.0, -0.6, 0.80]), 0.04, 0.74, 0.50, OFFS)
    assert why is None
    assert plan["lift"] == pytest.approx(0.80 - 0.11)
    assert plan["extension"] == pytest.approx(0.6 - 0.415)


def test_plan_reach_keeps_the_fingers_above_the_table():
    plan, _ = P.plan_reach(np.array([0.0, -0.6, 0.76]), 0.04, 0.74, 0.50, OFFS)
    assert plan["grasp_z"] == pytest.approx(0.74 + 0.035)


@pytest.mark.parametrize("cup,top,edge,reason", [
    ((0.0, -0.6, 0.8), None, 0.5, "table top"),
    ((0.0, -1.2, 0.8), 0.74, 0.5, "out of reach"),
    ((0.0, -0.3, 0.8), 0.74, 0.5, "too close"),
    ((0.0, -0.6, 1.5), 0.74, 0.5, "lift"),
    ((0.0, -0.6, 0.8), 0.74, 0.44, "table edge"),
])
def test_plan_reach_refuses_unsafe_moves(cup, top, edge, reason):
    plan, why = P.plan_reach(np.array(cup), 0.04, top, edge, OFFS)
    assert plan is None and reason in why
