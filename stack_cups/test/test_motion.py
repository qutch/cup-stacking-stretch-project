import numpy as np
import pytest

from stack_cups import motion as M


def test_base_speed_zero_inside_tol():
    assert M.base_speed(0.002, 0.15, 0.06, 0.1, tol=0.003) == 0.0
    assert M.base_speed(-0.002, 0.15, 0.06, 0.1, tol=0.003) == 0.0


def test_base_speed_sign_and_limits():
    assert M.base_speed(1.0, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(0.15)  # capped
    assert M.base_speed(0.005, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(0.06)  # floored
    assert M.base_speed(-1.0, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(-0.15)
    # sqrt(2*a*d) in between: d=0.05 -> 0.1
    assert M.base_speed(0.05, 0.15, 0.06, 0.1, tol=0.003) == pytest.approx(0.1)


def test_progress_along_heading():
    assert M.progress_along((1.0, 2.0, 0.0), (1.3, 2.4)) == pytest.approx(0.3)
    assert M.progress_along((0.0, 0.0, np.pi / 2), (0.1, -0.2)) == pytest.approx(-0.2)


def test_yaw_for_offset():
    assert M.yaw_reach(0.2, 0.25) == pytest.approx(0.2 * np.sin(0.25))
    assert M.yaw_for_offset(0.02, 0.2, 0.25, sign=1.0) == pytest.approx(np.arcsin(0.1))
    assert M.yaw_for_offset(0.02, 0.2, 0.25, sign=-1.0) == pytest.approx(-np.arcsin(0.1))
    assert M.yaw_for_offset(0.10, 0.2, 0.25, sign=1.0) is None  # beyond reach
    assert M.yaw_for_offset(0.0, 0.2, 0.25, sign=1.0) == 0.0


def test_yaw_for_offset_from_a_turned_wrist():
    L = 0.27
    for sign in (1.0, -1.0):
        for yaw0, dx in ((0.2, 0.03), (0.2, -0.03), (-0.1, 0.02)):
            yaw = M.yaw_for_offset(dx, L, 0.5, sign, current_yaw=yaw0)
            # the grasp point moves by exactly dx from where it was
            moved = sign * L * (np.sin(yaw) - np.sin(yaw0))
            assert moved == pytest.approx(dx)
    # staying put keeps the yaw
    assert M.yaw_for_offset(0.0, L, 0.5, 1.0, current_yaw=0.2) == pytest.approx(0.2)
    # the reach limit counts from straight, not from the current yaw
    assert M.yaw_for_offset(0.05, L, 0.25, 1.0, current_yaw=0.2) is None


def test_robust_center_rejects_spread():
    pts = np.array([[0.50, -0.6, 0.8]] * 4 + [[0.53, -0.6, 0.8]])
    c, spread, ok = M.robust_center(pts, max_spread=0.01)
    assert c == pytest.approx([0.50, -0.6, 0.8]) and not ok and spread == pytest.approx(0.03)
    c, spread, ok = M.robust_center(pts[:4], max_spread=0.01)
    assert ok and spread == pytest.approx(0.0)


def test_base_to_odom():
    out = M.base_to_odom(np.array([1.0, 0.0]), (2.0, 3.0, np.pi / 2))
    assert out == pytest.approx([2.0, 4.0])


def test_odom_to_base_inverts_base_to_odom():
    pose = (2.0, 3.0, np.pi / 2)
    assert M.odom_to_base(np.array([2.0, 4.0]), pose) == pytest.approx([1.0, 0.0])
    p = np.array([0.3, -0.55])
    pose = (-0.08, 0.005, 0.02)
    assert M.odom_to_base(M.base_to_odom(p, pose), pose) == pytest.approx(p)
