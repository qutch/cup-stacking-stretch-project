"""Unit tests for tape.py (roadmap 2.1): finding the blue-taped stacking area on synthetic
color + depth images with a known answer.

The synthetic scene: a camera 1.3 m up, looking to the robot's right (base_link -y, where
the arm reaches) and tilted down, sees a table top with a rectangle of blue tape on it.
Depth is rendered by ray casting, so the true center and direction are known exactly.
"""

import math

import numpy as np
import pytest

from stack_cups import perception as P
from stack_cups import tape as T

W, H = 424, 240
K = [300.0, 0.0, W / 2, 0.0, 300.0, H / 2, 0.0, 0.0, 1.0]
CAM_T = np.array([0.0, 0.0, 1.3])
TABLE_Z = 0.74
AREA = {"x": 0.05, "y": -0.75, "yaw": 0.0, "length": 0.48, "width": 0.30, "strip": 0.025}
ORANGE, BLUE = (60, 150, 230), (230, 89, 26)  # BGR: the sim's table and tape


def camera_rotation(pan=-math.pi / 2, tilt=math.radians(40)):
    """Optical (x right, y down, z forward) -> base_link: looking along `pan` (0 = +x,
    -pi/2 = -y), tilted down by `tilt`."""
    look = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    c, s = math.cos(tilt), math.sin(tilt)
    pitch_down = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    cp, sp = math.cos(pan), math.sin(pan)
    turn = np.array([[cp, -sp, 0.0], [sp, cp, 0.0], [0.0, 0.0, 1.0]])
    return turn @ pitch_down @ look


def render(area=AREA, pan=-math.pi / 2, tilt=math.radians(40), occluder=None):
    """(bgr image, depth in meters, R, t). occluder: (u0, u1) image columns covered by
    something red 0.3 m in front of the camera (like the cup in the gripper)."""
    R = camera_rotation(pan, tilt)
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    vs, us = np.mgrid[0:H, 0:W]
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], -1) @ R.T
    depth = np.zeros((H, W), np.float32)
    img = np.full((H, W, 3), 80, np.uint8)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (TABLE_Z - CAM_T[2]) / rays[..., 2]
    table = (rays[..., 2] < 0) & (s > 0) & (s < 2.5)
    depth[table] = s[table]
    img[table] = ORANGE
    px = CAM_T[0] + s * rays[..., 0]
    py = CAM_T[1] + s * rays[..., 1]
    c, sn = math.cos(area["yaw"]), math.sin(area["yaw"])
    lx = c * (px - area["x"]) + sn * (py - area["y"])
    ly = -sn * (px - area["x"]) + c * (py - area["y"])
    a, b, h = area["length"] / 2, area["width"] / 2, area["strip"] / 2
    outer = (np.abs(lx) <= a + h) & (np.abs(ly) <= b + h)
    inner = (np.abs(lx) < a - h) & (np.abs(ly) < b - h)
    tape = table & outer & ~inner
    img[tape] = BLUE
    if occluder is not None:
        cols = (us >= occluder[0]) & (us < occluder[1])
        depth[cols] = 0.3
        img[cols] = (20, 20, 200)
    return img, depth, R, CAM_T.copy()


def locate(**kw):
    img, depth, R, t = render(**kw)
    return T.locate_tape(img, depth, K, K, (R, t), (np.eye(3), np.zeros(3)))


def test_tape_mask_takes_the_tape_but_not_the_table_or_a_red_cup():
    import cv2
    px = np.array([[BLUE, ORANGE, (20, 20, 200), (80, 80, 80)]], np.uint8)
    assert list(P.tape_mask(px)[0] > 0) == [True, False, False, False]
    assert cv2.cvtColor(px[:, :1], cv2.COLOR_BGR2HSV)[0, 0, 0] == pytest.approx(109, abs=3)


def test_finds_the_center_direction_and_size_of_the_taped_area():
    m = locate()
    assert "why" not in m
    assert m["tape"][:2] == pytest.approx([AREA["x"], AREA["y"]], abs=0.01)
    assert m["tape"][2] == pytest.approx(TABLE_Z, abs=0.005)
    assert m["yaw"] == pytest.approx(0.0, abs=math.radians(1.0))
    # the outer edge of the tape: the area plus half a strip on each side
    assert m["length"] == pytest.approx(AREA["length"] + AREA["strip"], abs=0.02)
    assert m["width"] == pytest.approx(AREA["width"] + AREA["strip"], abs=0.02)


@pytest.mark.parametrize("deg", [-14.0, -5.0, 8.0, 20.0])
def test_yaw_is_the_turn_that_squares_the_base_to_the_tape(deg):
    m = locate(area=dict(AREA, yaw=math.radians(deg)))
    assert m["yaw"] == pytest.approx(math.radians(deg), abs=math.radians(1.0))
    assert m["tape"][:2] == pytest.approx([AREA["x"], AREA["y"]], abs=0.01)


def test_rotated_more_than_45_deg_the_other_edge_is_closest_to_the_base_x_axis():
    m = locate(area=dict(AREA, yaw=math.radians(70)))
    assert m["yaw"] == pytest.approx(math.radians(-20), abs=math.radians(1.0))
    assert m["length"] == pytest.approx(AREA["width"] + AREA["strip"], abs=0.02)


def test_works_with_part_of_the_tape_hidden_behind_the_cup():
    img, *_ = render()
    cols = np.where((P.tape_mask(img) > 0).any(axis=0))[0]
    mid = (cols.min() + cols.max()) // 2
    m = locate(occluder=(mid - 40, mid + 40))  # hides the middle of the tape
    assert "why" not in m
    assert m["tape"][:2] == pytest.approx([AREA["x"], AREA["y"]], abs=0.01)
    assert m["yaw"] == pytest.approx(0.0, abs=math.radians(1.0))


def test_follows_the_camera_turned_toward_the_area():
    m = locate(pan=-math.radians(70), area=dict(AREA, x=0.35))
    assert m["tape"][:2] == pytest.approx([0.35, AREA["y"]], abs=0.01)


def test_no_tape_in_view_says_why():
    m = locate(area=dict(AREA, x=5.0))
    assert "tape" not in m and "blue" in m["why"]


def test_fold_direction_and_rectangle_edges():
    assert T.fold_direction(math.pi) == pytest.approx(0.0, abs=1e-9)
    assert T.fold_direction(math.radians(100)) == pytest.approx(math.radians(-80))
    square = [(0, 0), (2, 0), (2, 1), (0, 1)]
    yaw, length, width = T.rectangle_edges(square)
    assert (yaw, length, width) == pytest.approx((0.0, 2.0, 1.0))
    yaw, length, width = T.rectangle_edges(square[1:] + square[:1])
    assert (yaw, length, width) == pytest.approx((0.0, 2.0, 1.0))
