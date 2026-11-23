"""Every parameter the stack_cups nodes read, with its default (the sim value).

config/sim.yaml and config/robot.yaml set all of them for each platform, so nothing tuned
for the sim is silently used on the real robot; stack_cups/test/test_config.py checks
both files against these lists (same names, same types). Override any of them with
`--ros-args -p name:=value`.
"""

# CupRobot (robot.py): base moves, joint moves, safety
ROBOT = {
    "odom_topic": "/odom",
    # --- sim-only workarounds (see sim/patches); the real robot needs none of them
    # move_joint_precise: re-command a joint up to this many times until it is within
    # tolerance (the unpatched sim accepts a joint within 5 cm). 1 = a single move.
    "joint_tries": 5,
    # Accept an ABORTED move with no error message when every joint is within this of
    # its target (the unpatched sim aborts every goal that way). 0 = never.
    "aborted_ok_tol": 0.05,
    # --- safety (roadmap 0.6)
    "max_base_speed": 0.3,  # m/s, hard cap on every base speed command
    "max_base_turn": 1.0,  # rad/s, hard cap on every base turn rate command
    # Joint speed limits passed with every arm/lift/wrist/head move; 0 = driver default.
    # The sim ignores them; on the robot they keep the first runs slow.
    "max_joint_speed_m": 0.0,  # m/s for joint_lift and wrist_extension
    "max_joint_speed_rad": 0.0,  # rad/s for the head and wrist joints
    "step_confirm": False,  # wait for Enter or /stack_cups/continue before each step
}

# The cups (roadmap: real size still unknown; placeholder = typical 16 oz party cup)
CUP = {
    "cup_height": 0.12,  # m, base to rim
    "cup_rim_diameter": 0.095,  # m, outside of the rim (the wide end)
    "cup_base_diameter": 0.06,  # m, outside of the base (the narrow end)
    "cup_wall": 0.003,  # m, wall thickness (sim cups)
    "cup_mass": 0.02,  # kg (sim cups)
}

# find_cup (cup_finder_controller.py)
FIND_CUP = {
    "steps": "all",  # phases to run: all, or a list of aim_head,ready,align,reach
    "aim_head": True,
    "head_pan": -1.57,
    "head_tilt": -0.6,
    "grasp": False,
    "show_windows": True,
    # joint_gripper_finger_left when open, in sim units (the robot's differ, see README):
    # 0.3 puts the fingertips ~12 cm apart, enough for a 9.5 cm rim. 0.5 (~15 cm) hit the
    # box next to the red cylinder in stretch_mujoco's default scene.
    "gripper_open": 0.3,
    "gripper_closed": -0.1,
    "ready_above_cup": 0.25,  # m, grasp point height above the cup while driving
    # --- base moves (odometry-steered)
    "v_max": 0.15,  # m/s, base speed far from the target
    "v_min": 0.06,  # m/s, slowest command that still moves the base (sim: wheel friction)
    "decel": 0.1,  # m/s^2, how hard the base slows down near the target
    "drive_tol": 0.003,  # m, a drive stops this close to its target
    "align_tol": 0.008,  # m, grasps tested reliable within +/-1 cm of the arm line
    "max_drives": 3,
    # --- measuring while stopped. The sim renders a new image only every 0.1 s of sim
    # time and re-sends the last one in between, so "fresh" is decided by the ROS clock.
    "settle_s": 0.3,  # s to wait after stopping before measuring (~3 sim renders)
    "sample_dt": 0.1,  # s between samples (one sim render)
    "samples": 5,  # measurements per position (median)
    "max_spread": 0.01,  # m, if samples spread more, take more
    "grasp_height": -1.0,  # m above the table top to grasp at; < 0 = cup center
    # --- wrist yaw covers the last few cm (measured with sim/check_robot_layer.py)
    "use_wrist_yaw": True,
    "max_yaw": 0.25,  # rad (~14 deg): about +/-6.7 cm of sideways reach
    "arm_len": 0.270,  # m, wrist yaw axis to grasp center
    "yaw_sign": 1.0,  # +yaw moves the grasp point toward +x of base_link
    "yaw_check_tol": 0.010,  # m, grasp point must end this close to the cup's x
}

# carry_to_stack (roadmap task 2). It also reads CUP, and FIND_CUP's base-move (v_*, decel,
# drive_tol, align_tol, max_drives), measuring (settle_s, sample_dt, samples, max_spread)
# and gripper_closed values.
CARRY = {
    # phases to run: all, or a list of localize,navigate,find_tape,square,align
    "carry_steps": "all",
    # --- named poses in the map frame, [x, y, yaw] (roadmap 2.2). The sim map's origin
    # (stack_cups/maps/sim_task_room) is where the robot starts in the task scene.
    "start_pose": [0.0, 0.0, 0.0],  # where the robot was when its driver started (odom's origin)
    "loose_cups_pose": [0.2, 0.0, 0.0],  # next to the loose cups (for task 1.3)
    "stacking_pose": [0.75, 0.0, 0.0],  # next to the stacking area, arm side to the table
    "poses_checked": True,  # false: the poses are not measured on this map, so never navigate
    "nav_timeout": 120.0,  # s for Nav2 to get there
    # --- Nav2's outline while carrying (roadmap 2.4): the base plus the arm and the cup
    "robot_radius": 0.22,  # m, the base (the starter's Nav2 robot_radius, arm stowed)
    "carry_margin": 0.05,  # m, room around the cup
    # --- the cup in the gripper (roadmap 2.5), in gripper_closed's finger units
    "require_cup": True,  # start only with a cup in the gripper, and watch the grip
    "held_min": 0.05,  # fingers stopped at least this far open of gripper_closed = a cup
    "grip_tol": 0.05,  # fingers opening more than this from where they held it = let go
    # --- finding the tape (roadmap 2.1) and parking at it (2.6)
    "tape_head_pan": -1.2,  # rad, where to look first from stacking_pose (then the head
    "tape_head_tilt": -0.7,  # follows the tape)
    "square_tol": 0.02,  # rad, base parallel to the tape's edge within this
    "max_turns": 3,
    "w_max": 1.0,  # rad/s, base turn rate far from the target heading
    "w_min": 0.3,  # rad/s, slowest turn command that still turns the base
    "w_decel": 0.5,  # rad/s^2, how hard turns slow down near the target heading
    "max_reach": 0.5,  # m, most arm extension the tape's center may need from the parking spot
}

ALL = {**ROBOT, **CUP, **FIND_CUP, **CARRY}
