#!/usr/bin/env python3
"""Check the CupRobot layer in the running sim, and measure the values find_cup needs.

Run with the sim up (use_cameras is not needed), after `source install/setup.bash`:
    python3 sim/check_robot_layer.py
    python3 sim/check_robot_layer.py --v-min 0.08      # if a drive stalls

Every base move is undone afterwards, so the robot ends about where it started.
Paste the RESULT lines (or the whole output) back to Claude.
"""

import _thread
import argparse
import threading
import time

import numpy as np
import rclpy

from stack_cups.robot import CupRobot


def result(name, ok, detail):
    print(f"RESULT {'PASS' if ok else 'FAIL'} {name}: {detail}", flush=True)


ROBOT = None  # set in main(); timed() and ros_sleep() use its ROS clock


def timed(fn, *args, **kwargs):
    """Run fn; return (its result, seconds of ROS time it took: sim time in the sim)."""
    t0 = ROBOT.now()
    out = fn(*args, **kwargs)
    return out, ROBOT.now() - t0


def ros_sleep(seconds):
    """Sleep `seconds` of ROS time (sim time in the sim, which can run slower)."""
    t0 = ROBOT.now()
    while ROBOT.now() - t0 < seconds:
        time.sleep(0.02)


def odom_x_along(r, start):
    from stack_cups import motion
    return motion.progress_along(start, r.odom_pose()[:2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v-max", type=float, default=0.15)
    ap.add_argument("--v-min", type=float, default=0.06)
    ap.add_argument("--decel", type=float, default=0.1)
    args, _ = ap.parse_known_args()  # leave --ros-args ... to rclpy
    drive_kw = dict(v_max=args.v_max, v_min=args.v_min, decel=args.decel)

    global ROBOT
    r = ROBOT = CupRobot()
    r.start("check_robot_layer")
    print(f"clock: {'sim time' if r.get_parameter('use_sim_time').value else 'wall time'} "
          "(all times below are in this clock)", flush=True)
    try:
        t0 = time.time()
        while r.odom_pose() is None and time.time() - t0 < 10:
            time.sleep(0.1)
        result("odom", r.odom_pose() is not None, f"{r.odom_topic} -> {r.odom_pose()}")
        r.enter_navigation_mode()
        for section in (check_joints, check_drives, check_yaw):
            try:
                section(r, args, drive_kw)
            except Exception as e:  # noqa: BLE001 - report and go on to the next section
                result(section.__name__, False, f"{type(e).__name__}: {e}")
                print(f"  joints now: lift {r.joint_pos('joint_lift')}, "
                      f"arm {r.joint_pos('wrist_extension')}, "
                      f"yaw {r.joint_pos('joint_wrist_yaw')}", flush=True)
                r.drive(0.0)
        print("done", flush=True)
    finally:
        r.drive(0.0)
        time.sleep(0.2)
        r.destroy_node()
        rclpy.try_shutdown()


def check_joints(r, args, drive_kw):
    # ---- Task 3: joint moves through move_to_pose
    _, dt = timed(r.move, {"joint_lift": 0.8, "wrist_extension": 0.0})
    result("move lift+arm", True, f"took {dt:.1f} s, lift now {r.joint_pos('joint_lift'):.3f}")
    _, dt = timed(r.move_joint_precise, "joint_lift", 0.6, (0.0, 1.1))
    lift = r.joint_pos("joint_lift")
    result("precise lift", abs(lift - 0.6) < 0.01, f"took {dt:.1f} s, lift {lift:.3f} (target 0.600)")
    T = r.lookup("base_link", "link_grasp_center")
    result("TF grasp", T is not None, f"grasp in base_link: {None if T is None else np.round(T[1], 3)}")

    # Review Focus 5: an invalid move must raise, not silently continue
    try:
        r.move({"joint_lift": "high"})
        result("invalid move raises", False, "no exception")
    except RuntimeError as e:
        result("invalid move raises", "joint_lift" in str(e), str(e))
    try:
        r.move({"joint_not_real": 0.1}, timeout=10.0)
        print("RESULT NOTE unknown joint: no exception, as expected in the sim (its server "
              "skips unknown joints; the real driver rejects them)", flush=True)
    except RuntimeError as e:
        result("unknown joint raises", True, str(e))


def check_drives(r, args, drive_kw):
    # ---- Task 4: drive_by
    for dist in (0.10, -0.10, 0.02, -0.02):
        start = r.odom_pose()
        try:
            reached, dt = timed(r.drive_by, dist, **drive_kw)
            ros_sleep(0.5)  # let it coast to a stop, then measure where it really is
            final = odom_x_along(r, start)
            err = final - dist
            result(f"drive_by {dist:+.2f}", abs(err) < 0.003,
                   f"loop ended at {reached:+.4f}, settled at {final:+.4f} "
                   f"(error {err * 1000:+.1f} mm) in {dt:.1f} s")
        except RuntimeError as e:
            result(f"drive_by {dist:+.2f}", False, f"{e}  -> rerun with a larger --v-min")

    # Review Focus 2: oscillation must end after max_reversals, not hang
    start = r.odom_pose()
    try:
        reached, dt = timed(r.drive_by, 0.05, v_max=0.25, v_min=0.25, decel=args.decel,
                            tol=0.001)
        result("oscillation ends", dt < 30, f"returned at {reached:+.4f} after {dt:.1f} s")
    except RuntimeError as e:
        result("oscillation ends", True, f"raised instead of hanging: {e}")
    r.drive_by(-odom_x_along(r, start), **drive_kw)

    # Review Focus 4: Ctrl+C during a drive must leave the command at 0
    start = r.odom_pose()
    threading.Timer(1.0, _thread.interrupt_main).start()
    try:
        r.drive_by(0.5, **drive_kw)
        result("interrupt stops base", False, "drive finished before the interrupt")
    except KeyboardInterrupt:
        x1 = odom_x_along(r, start)
        ros_sleep(0.5)
        x2 = odom_x_along(r, start)
        result("interrupt stops base", r.cmd_v == 0.0 and abs(x2 - x1) < 0.01,
               f"cmd_v {r.cmd_v}, moved {abs(x2 - x1) * 1000:.1f} mm in the 0.5 s after")
    ros_sleep(0.5)
    r.drive_by(-odom_x_along(r, start), **drive_kw)


def check_yaw(r, args, drive_kw):
    # ---- Wrist yaw geometry (for Task 6)
    r.move({"joint_lift": 0.8})
    r.move({"joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0, "joint_wrist_roll": 0.0})
    r.wait_joint_still("joint_wrist_yaw")
    x0 = r.lookup("base_link", "link_grasp_center")[1][0]
    r.move({"joint_wrist_yaw": 0.2})
    r.wait_joint_still("joint_wrist_yaw")
    yaw = r.joint_pos("joint_wrist_yaw")
    x1 = r.lookup("base_link", "link_grasp_center")[1][0]
    shift = x1 - x0
    result("wrist yaw geometry", abs(shift) > 0.005,
           f"yaw {yaw:.3f} rad moved grasp x by {shift * 1000:+.1f} mm -> "
           f"yaw_sign {np.sign(shift):+.0f}, arm_len {abs(shift) / np.sin(yaw):.3f} m")
    r.move({"joint_wrist_yaw": 0.0})


if __name__ == "__main__":
    main()
