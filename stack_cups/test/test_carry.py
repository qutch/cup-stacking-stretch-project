"""Unit tests for carry.py (roadmap task 2): poses, the carry outline, the grip check and
pointing the head."""

import math

import numpy as np
import pytest

from stack_cups import carry as C


def test_compose_puts_odometry_into_the_map():
    start = (1.0, 2.0, math.pi / 2)  # odometry started here, facing map +y
    assert C.compose(start, (0.0, 0.0, 0.0)) == pytest.approx(start)
    # 1 m forward in odometry is 1 m along map +y; a left turn adds to the heading
    assert C.compose(start, (1.0, 0.0, 0.3)) == pytest.approx((1.0, 3.0, math.pi / 2 + 0.3))
    assert C.compose(start, (0.0, 1.0, 0.0))[:2] == pytest.approx((0.0, 2.0))


def test_wrap_keeps_angles_in_minus_pi_to_pi():
    assert C.wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert C.wrap(-0.2) == pytest.approx(-0.2)


def test_carry_outline_covers_the_base_the_arm_and_the_cup():
    cup, r, margin = (-0.02, -0.41), 0.0475, 0.05
    outline = np.array(C.carry_outline(0.22, cup, r, margin))
    xs, ys = outline[:, 0], outline[:, 1]
    assert ys.min() == pytest.approx(cup[1] - r - margin, abs=0.005)  # past the cup
    assert ys.max() == pytest.approx(0.22, abs=0.005)  # the base on the other side
    assert (xs.min(), xs.max()) == pytest.approx((-0.22, 0.22), abs=0.005)
    # convex and counterclockwise: every turn is to the left
    d = np.roll(outline, -1, axis=0) - outline
    cross = d[:, 0] * np.roll(d, -1, axis=0)[:, 1] - d[:, 1] * np.roll(d, -1, axis=0)[:, 0]
    assert (cross > -1e-9).all()


def test_carry_outline_without_a_reach_is_the_base_circle():
    outline = np.array(C.carry_outline(0.22, (0.0, 0.0), 0.05, 0.05))
    assert np.hypot(outline[:, 0], outline[:, 1]) == pytest.approx(0.22, abs=1e-4)


def test_holding_and_losing_the_cup():
    assert C.is_holding(0.18, closed=-0.1, min_open=0.05)
    assert not C.is_holding(-0.08, closed=-0.1, min_open=0.05)  # closed on nothing

    def lost(now):
        return C.grip_lost(0.167, now, closed=-0.1, min_open=0.05, open_tol=0.05)

    assert not lost(0.167) and not lost(0.144)  # the cup creeping in the grip (sim)
    assert lost(-0.08)  # closed on nothing: the cup fell out
    assert lost(0.30)  # the gripper opened


def test_look_at_points_the_head():
    eye = (0.0, 0.0, 1.3)
    pan, tilt = C.look_at(eye, (0.0, -1.0, 0.3))  # to the arm side, 45 deg down
    assert pan == pytest.approx(-math.pi / 2) and tilt == pytest.approx(-math.pi / 4)
    pan, tilt = C.look_at(eye, (1.0, 0.0, 1.3))
    assert pan == pytest.approx(0.0) and tilt == pytest.approx(0.0)


def test_reach_for_is_the_extension_to_a_point_on_the_arm_side():
    assert C.reach_for(-0.73, 0.41) == pytest.approx(0.32)
