"""CupRobot.close_gripper (roadmap 1.7) against a fake gripper: no ROS needed."""

import pytest

from stack_cups.robot import CupRobot

JOINT = "joint_gripper_finger_left"


class FakeGripper:
    """Fingers follow commands but cannot close past `obstacle` (None = nothing there).
    Effort grows with how hard the command pushes past the obstacle."""

    def __init__(self, pos=0.3, obstacle=None, effort_per_unit=None):
        self.pos, self.obstacle, self.effort_per_unit = pos, obstacle, effort_per_unit
        self.cmd, self.commands = pos, []

    # the CupRobot methods close_gripper uses
    _move_gripper = CupRobot._move_gripper

    def move(self, pose):
        self.cmd = pose[JOINT]
        self.commands.append(self.cmd)
        self.pos = self.cmd if self.obstacle is None else max(self.cmd, self.obstacle)

    def joint_pos(self, name):
        return self.pos

    def wait_joint_still(self, name):
        return self.pos

    def joint_effort(self, name):
        if self.effort_per_unit is None:
            return None  # like the sim: no effort reported
        return self.effort_per_unit * max(0.0, self.pos - self.cmd)

    def get_logger(self):
        class Log:
            def info(self, msg):
                pass
        return Log()


def close(fake, **kw):
    args = dict(closed=-0.1, squeeze=0.05)
    args.update(kw)
    return CupRobot.close_gripper(fake, **args)


def test_stops_on_the_cup_and_squeezes_a_set_amount():
    fake = FakeGripper(obstacle=-0.046)
    pos, contact = close(fake)
    assert contact
    # the last command holds the contact position minus the squeeze
    assert fake.commands[-1] == pytest.approx(-0.046 - 0.05)


def test_squeeze_never_goes_past_closed():
    fake = FakeGripper(obstacle=-0.08)
    close(fake)
    assert fake.commands[-1] == pytest.approx(-0.1)


def test_closing_on_nothing_reports_no_contact():
    fake = FakeGripper(obstacle=None)
    pos, contact = close(fake)
    assert not contact
    assert pos == pytest.approx(-0.1)
    assert min(fake.commands) == pytest.approx(-0.1)  # never commanded past closed


def test_effort_limit_stops_before_the_fingers_stall():
    # effort rises as soon as the command passes the cup; with a low limit the close
    # stops at the first step past it even though the step is below stall_tol
    fake = FakeGripper(obstacle=0.0, effort_per_unit=1000.0)
    pos, contact = close(fake, step=0.005, stall_tol=0.01, max_effort=2.0)
    assert contact
    assert fake.commands[-1] == pytest.approx(0.0 - 0.05)


def test_effort_ignored_when_limit_is_zero():
    fake = FakeGripper(obstacle=0.0, effort_per_unit=1000.0)
    close(fake, step=0.005, stall_tol=0.01, max_effort=0.0)
    # contact found by the stall instead: 0.01 past the obstacle
    assert fake.commands[-2] < -0.01 + 1e-9


class TimeoutGripper(FakeGripper):
    """Like the real driver: a goal the fingers cannot reach (stopped on the cup) aborts
    after default_goal_timeout_s with an error, and the fingers keep pushing."""

    def move(self, pose):
        super().move(pose)
        if self.obstacle is not None and self.cmd < self.obstacle:
            raise RuntimeError(
                "move failed (status 6): Time to execute the current goal point = <...> "
                f"exceeded the default_goal_timeout = 10.0; pose {pose}")


def test_real_driver_timeout_on_the_cup_counts_as_contact():
    fake = TimeoutGripper(pos=0.3, obstacle=0.2)
    pos, contact = close(fake)
    assert contact
    assert pos == pytest.approx(0.2)
    assert fake.commands[-1] == pytest.approx(0.2 - 0.05)  # held a squeeze past the cup


def test_other_move_errors_still_raise():
    class Rejecting(FakeGripper):
        def move(self, pose):
            raise RuntimeError(f"move rejected: {pose}")

    with pytest.raises(RuntimeError, match="rejected"):
        close(Rejecting(pos=0.3))


class SoftCup(FakeGripper):
    """A single Solo cup: it gives way, so the fingers follow every command, but the
    effort grows as they press into it (from `base`, the effort closing on air)."""

    def __init__(self, pos=0.3, cup=0.2, per_unit=200.0, base=-0.5):
        super().__init__(pos=pos)
        self.cup, self.per_unit, self.base = cup, per_unit, base

    def move(self, pose):
        self.cmd = pose[JOINT]
        self.commands.append(self.cmd)
        self.pos = self.cmd

    def joint_effort(self, name):
        return self.base - self.per_unit * max(0.0, self.cup - self.pos)


def test_soft_cup_crushed_without_effort_rise():
    # what the real robot did: the fingers never stop on a soft cup
    pos, contact = close(SoftCup(), step=0.025)
    assert not contact
    assert pos == pytest.approx(-0.1)


def test_soft_cup_found_by_effort_rise():
    fake = SoftCup(pos=0.3, cup=0.2)
    pos, contact = close(fake, step=0.025, squeeze=0.02, effort_rise=4.0)
    assert contact
    # free close reads -0.5; 0.175 is 0.025 into the cup: -5.5, a rise of 5.0
    assert fake.commands[-2] == pytest.approx(0.175)
    assert pos == pytest.approx(0.175 - 0.02)
    assert fake.grip_effort_base == pytest.approx(-0.5)
