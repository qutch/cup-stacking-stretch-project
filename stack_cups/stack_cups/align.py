"""Line the arm up with a target: drive the base by odometry, finish with the wrist yaw.

The arm reaches along base_link -y, so lining up means making the target's base_link x
equal the grasp point's x. Used for cups (find_cup) and reusable for pyramid spots (task
4): the caller passes a `measure()` that returns the target's position.

    offs = align_arm(robot, measure, offs, cfg)

robot: a CupRobot in navigation mode. offs: grasp offsets from robot.arm_offsets().
measure(): dict with `key` (xyz in base_link) and "spread", or None if the target is lost
(measure it fresh, i.e. while stopped, e.g. with HeadCamera.measure_fresh).
cfg: use_wrist_yaw, arm_len, max_yaw, yaw_sign, yaw_check_tol, align_tol, max_drives,
v_max, v_min, decel, drive_tol (see config/*.yaml).
Returns the grasp offsets to reach with (they change when the wrist turns).
"""

from stack_cups import motion as M


def odom_note(m, key="cup"):
    """'; base at odom x=..., <key> at odom (...)' for logs (empty without odometry)."""
    if "pose" not in m or "odom_point" not in m:
        return ""
    return (f"; base at odom x={m['pose'][0]:+.3f}, "
            f"{key} at odom ({m['odom_point'][0]:+.3f}, {m['odom_point'][1]:+.3f})")


def align_arm(robot, measure, offs, cfg, key="cup", why_lost=None):
    """Measure, drive the measured offset by odometry, repeat at most max_drives times.

    With use_wrist_yaw, drive only until the wrist yaw can cover the rest, then turn the
    wrist and check the grasp point (from TF) is within yaw_check_tol of the target.
    why_lost(): optional, explains a failed measurement in the error message."""
    log = robot.get_logger().info
    use_yaw = bool(cfg["use_wrist_yaw"])
    # with the wrist yaw, the base only has to get within (most of) its reach
    good_enough = (0.8 * M.yaw_reach(cfg["arm_len"], cfg["max_yaw"]) if use_yaw
                   else cfg["align_tol"])
    max_drives = int(cfg["max_drives"])
    dx = None
    for drives in range(max_drives + 1):
        m = measure()
        if m is None:
            why = f": {why_lost()}" if why_lost else ""
            raise RuntimeError(f"lost the {key} during alignment{why}")
        dx = float(m[key][0] - offs["x"])
        log(f"[ALIGN] {key} is {dx * 100:+.1f} cm from the arm line "
            f"(spread {m['spread'] * 1000:.1f} mm, {drives} drives so far){odom_note(m, key)}")
        if abs(dx) < cfg["align_tol"]:
            log(f"[ALIGN] aligned after {drives} drives")
            return offs
        if use_yaw and abs(dx) < good_enough:
            return _align_wrist(robot, measure, dx, cfg, key)
        if drives == max_drives:
            break
        moved = robot.drive_by(dx, v_max=cfg["v_max"], v_min=cfg["v_min"],
                               decel=cfg["decel"], tol=cfg["drive_tol"])
        log(f"[ALIGN] drive {drives + 1}: asked {dx * 100:+.1f} cm, "
            f"odometry says {moved * 100:+.1f} cm")
    raise RuntimeError(f"alignment: still {dx:+.3f} m off after {max_drives} drives")


def _align_wrist(robot, measure, dx, cfg, key):
    """Turn the wrist so the grasp point moves `dx` along base x, then check it against
    a fresh measurement. Returns the new grasp offsets."""
    log = robot.get_logger().info
    yaw = M.yaw_for_offset(dx, cfg["arm_len"], cfg["max_yaw"], cfg["yaw_sign"])
    if yaw is None:
        raise RuntimeError(f"wrist yaw cannot cover {dx * 100:+.1f} cm "
                           f"(max_yaw {cfg['max_yaw']:.2f} rad)")
    log(f"[ALIGN] turning the wrist to {yaw:+.3f} rad to cover {dx * 100:+.1f} cm")
    robot.move_joint_precise("joint_wrist_yaw", yaw, (-1.0, 1.0), tol=0.01)
    offs = robot.arm_offsets()
    m = measure()
    if offs is None or m is None:
        raise RuntimeError(f"lost the {key} or the grasp frame after turning the wrist")
    err = float(m[key][0] - offs["x"])
    log(f"[ALIGN] after the wrist turn the grasp point is {err * 100:+.1f} cm "
        f"from the {key} (limit {cfg['yaw_check_tol'] * 100:.1f} cm)")
    if abs(err) >= cfg["yaw_check_tol"]:
        raise RuntimeError(f"wrist yaw left {err:+.3f} m")
    return offs
