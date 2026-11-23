#!/usr/bin/env python3
"""Sim stand-in for Task 1's pick-up: put a cup in the robot's gripper so Task 2 (driving
to the stacking area) can be tested on its own.

Run right after starting the task scene, with the robot still at its start pose:
    ros2 launch stack_cups sim.launch.py use_mujoco_viewer:=false
    python3 sim/handoff_cup.py              # grabs loose cup 1 of sim_scene.LOOSE_CUPS
    python3 sim/handoff_cup.py --cup 2

No perception: the cup's position comes from sim_scene.py and the robot's from /odom
(the sim's odometry starts at the scene's origin). The base lines the arm up with the
cup, the lift lowers the open gripper to the cup's middle, the arm reaches, the gripper
closes, then the cup is lifted clear of the table and the arm pulled in. That is Task 1's
carry pose (roadmap 1.9): wrist straight, cup about 0.4 m out on the arm side, its bottom
carry_clearance above the table top.

Prints "RESULT HELD" if the fingers stopped on the cup and did not close further while
lifting and pulling in (exit code 0), else "RESULT NOT HELD" (exit code 1).
"""

import argparse
import sys

import rclpy
from stack_cups import motion as M
from stack_cups import sim_scene as S
from stack_cups.robot import CupRobot

GRIPPER = "joint_gripper_finger_left"
# Sim finger units. Open wide (fingertips ~15 cm apart): at find_cup's 0.3 (~12 cm) the
# fingers pass the cup with ~1 cm to spare and sometimes knock it. CLOSED as gripper_closed.
OPEN, CLOSED = 0.5, -0.1
GRASP_AT = 0.6  # of the cup's height above its base
# Reach this far past the cup's center. link_grasp_center is ~1.4 cm beyond the fingertips,
# so with it at the cup's center the fingertips stop short of the cup's widest part and
# the closing fingers push the cup away (half the tries in the sim). Reaching further,
# they wrap around it.
REACH_PAST = 0.035
CARRY_CLEARANCE = 0.05  # m, cup bottom above the table top while carrying (as Task 1)
SQUEEZE = 0.05  # finger units past where they touched the cup (task 1's gripper_squeeze)
TRIES = 3  # grab attempts (one in ~16 misses in the sim: the cup gets pushed away)
HELD_MIN = CLOSED + 0.05  # fingers stopped at least this far open: something is held
SLIP_TOL = 0.01  # closing more than this after the grasp = the cup slipped out


def open_gripper(robot, tries=3):
    """Open the gripper and wait until it really is open. The patched sim reports a gripper
    move done as soon as the fingers are still, which can be before they started to move;
    reaching with closed fingers pushes the cup away. Returns the finger position."""
    pos = None
    for _ in range(tries):
        robot.move({GRIPPER: OPEN})
        robot.sleep(0.3)
        pos = robot.wait_joint_still(GRIPPER, still_m=0.002, window_s=0.3, timeout=5.0)
        if pos is not None and pos > OPEN - 0.05:
            return pos
        robot.get_logger().warn(f"[HANDOFF] the gripper is only at {pos}; opening it again")
    raise RuntimeError(f"the gripper did not open (fingers at {pos})")


def grab(robot, index):
    """Grab loose cup `index` and pull it into the carry pose. Returns (held, message)."""
    log = robot.get_logger().info
    robot.enter_navigation_mode()
    while robot.odom_pose() is None or robot.joint_pos(GRIPPER) is None:
        robot.sleep(0.1)
    offs = robot.arm_offsets()
    while offs is None:
        robot.sleep(0.1)
        offs = robot.arm_offsets()
    cup_xy = S.LOOSE_CUPS[index]
    cup = M.odom_to_base(cup_xy, robot.odom_pose())
    log(f"[HANDOFF] cup {index} at odom {cup_xy}, base ({cup[0]:+.3f}, {cup[1]:+.3f})")

    robot.move({"wrist_extension": 0.0, "joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0,
                "joint_wrist_roll": 0.0})
    open_gripper(robot)
    robot.drive_by(cup[0] - offs["x"], v_max=0.15, v_min=0.06, decel=0.1, tol=0.003)
    offs = robot.arm_offsets()
    cup = M.odom_to_base(cup_xy, robot.odom_pose())
    table_top = S.TABLE["height"]
    lift = table_top + GRASP_AT * S.CUP["cup_height"] - offs["h"]
    ext = -cup[1] - offs["c"] + REACH_PAST
    log(f"[HANDOFF] arm line {(cup[0] - offs['x']) * 100:+.1f} cm from the cup; "
        f"lift {lift:.3f}, extend {ext:.3f}")
    robot.move_joint_precise("joint_lift", lift, (0.0, 1.1))
    log(f"[HANDOFF] fingers at {open_gripper(robot):+.3f} before reaching")
    robot.move_joint_precise("wrist_extension", ext, (0.0, 0.52))
    try:
        robot.move({GRIPPER: CLOSED})
    except RuntimeError as e:  # the patched sim reports a stopped gripper as done
        log(f"[HANDOFF] gripper: {e}")
    grip = robot.wait_joint_still(GRIPPER)
    log(f"[HANDOFF] fingers stopped at {grip:+.3f} (closed = {CLOSED:+.3f})")
    if grip < HELD_MIN:
        return False, f"the fingers closed on nothing (at {grip:+.3f})"
    # Hold with a set squeeze past where the fingers touched (as task 1 does, roadmap 1.7),
    # not full force: squeezed hard, the tapered cup creeps up through the grip
    try:
        robot.move({GRIPPER: grip - SQUEEZE})
    except RuntimeError as e:
        log(f"[HANDOFF] gripper: {e}")
    grip = robot.wait_joint_still(GRIPPER)
    robot.move_joint_precise("joint_lift", lift + CARRY_CLEARANCE + 0.01, (0.0, 1.1))
    robot.move({"wrist_extension": 0.0})
    after = robot.wait_joint_still(GRIPPER)
    log(f"[HANDOFF] fingers {grip:+.3f} -> {after:+.3f} after lifting and pulling in")
    if after < grip - SLIP_TOL:
        return False, f"the cup slipped while lifting (fingers {grip:+.3f} -> {after:+.3f})"
    return True, f"fingers {grip:+.3f} -> {after:+.3f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cup", type=int, default=1, help="index into sim_scene.LOOSE_CUPS")
    args = ap.parse_args()
    robot = CupRobot()
    robot.start("handoff_cup")
    ok = False
    try:
        for attempt in range(1, TRIES + 1):
            ok, why = grab(robot, args.cup)
            if ok or attempt == TRIES:
                break
            robot.get_logger().warn(f"[HANDOFF] try {attempt} failed: {why}; trying again")
            open_gripper(robot)
            robot.move({"wrist_extension": 0.0})
        print(f"RESULT {'HELD' if ok else 'NOT HELD'}: {why} (try {attempt})", flush=True)
    finally:
        # Not the driver's /stop_the_robot when the cup is held: it also tells the gripper
        # to hold where it is, which drops the grip force on the cup
        if ok:
            robot.drive(0.0)
        else:
            robot.safe_stop()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
