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
    "max_turn_rate": 0.5,  # rad/s, hard cap on every base turn command
    # Position mode for arm and head moves, navigation mode for base moves (switching
    # as needed, roadmap 1.3b). False = stay in navigation mode for everything.
    "switch_modes": True,
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
    "steps": "all",  # phases to run: all, or a list of aim_head,ready,align,reach,carry
    "aim_head": True,
    "head_pan": -1.57,
    "head_tilt": -0.6,
    "grasp": False,
    "show_windows": True,
    # joint_gripper_finger_left when open, in sim units (the robot's differ, see README):
    # 0.3 puts the fingertips ~12 cm apart, enough for a 9.5 cm rim. 0.5 (~15 cm) hit the
    # box next to the red cylinder in stretch_mujoco's default scene.
    "gripper_open": 0.3,
    "gripper_closed": -0.1,  # how far closing may go (the fingers stop on the cup first)
    # Open only as wide as the cup needs before reaching (roadmap 1.6): finger position
    # gripper_fit_offset + gripper_fit_per_m * (cup width + 2 * grasp_open_margin), at most
    # gripper_open. Sim fit: the fingers stop at -0.034 on a 4 cm cylinder, +0.20 on a
    # 7.75 cm cup. gripper_fit_per_m 0 = always open to gripper_open.
    "gripper_fit_per_m": 6.24,
    "gripper_fit_offset": -0.284,
    "grasp_open_margin": 0.015,  # m, finger clearance each side of the cup
    # --- grasping (roadmap 1.7, 1.8)
    # Squeeze this far past where the fingers touched the cup: sets the grip force
    # (the gripper is position-controlled), instead of how far "closed" is past the cup.
    "gripper_squeeze": 0.05,
    # Stop closing when |effort| passes this (robot only; the sim reports no effort).
    # 0 = close by position only.
    "gripper_max_effort": 0.0,
    # m, lift to check the cup comes along. Must be more than the grasp point's distance
    # below the cup's top (half a 12 cm cup at center grasps), so the fingers clear a
    # cup that stayed on the table and close on nothing.
    "grasp_check_lift": 0.08,
    "grasp_slip_tol": 0.01,  # finger units: closing more than this while lifting = slipped
    # --- carry pose and retries (roadmap 1.9, 1.10)
    "carry_clearance": 0.05,  # m, cup bottom above the table top before the arm pulls in
    # Wrist yaw while carrying (rad). 0 = straight out; the cup then sits ~0.4 m from the
    # base center, outside Nav2's 0.22 m robot_radius (roadmap 2.4).
    "carry_wrist_yaw": 0.0,
    "grasp_tries": 3,  # attempts before giving up; a failed one opens and backs off first
    # --- finding the cup (roadmap 1.1); perception.py has the method
    # HSV red thresholds (OpenCV hue 0-179): hue <= width or >= 180 - width. Tune on the
    # robot with show_windows (the "mask" window shows what counts as red).
    "red_hue_width": 10,
    "red_sat_min": 120,
    "red_val_min": 70,
    "red_min_area": 100,  # px, smaller red blobs are noise
    # Skip red cups standing inside the taped stacking area (already placed)
    "ignore_taped": True,
    "tape_hue_min": 95,  # tape colour (HSV); the sim's tape is blue
    "tape_hue_max": 130,
    "tape_sat_min": 120,
    "tape_val_min": 60,
    # m, after the first measurement keep the cup nearest its last position (in odom);
    # none within this = lost
    "track_radius": 0.08,
    # m, split a red blob where the front surface bulges back this much between two cups
    "split_min_rise": 0.015,
    # --- drive to the table with Nav2 (roadmap 1.3); Nav2 must be running
    "navigate": False,  # run the navigate step first
    "nav_goal_x": 0.0,  # map frame, near the cup table, arm side (base -y) facing it
    "nav_goal_y": 0.0,
    "nav_goal_yaw": 0.0,
    # Tell AMCL where the robot starts (sim: the map origin is the start pose). False on
    # the robot: set the initial pose in RViz.
    "nav_set_initial_pose": True,
    "nav_init_x": 0.0,
    "nav_init_y": 0.0,
    "nav_init_yaw": 0.0,
    "nav_timeout": 120.0,  # s, for Nav2 to start and to arrive
    # --- square up to the table edge (roadmap 1.3b): Nav2 stops within 0.25 rad
    "square_up": True,
    "square_tol": 0.01,  # rad, closer than this to square = no turn
    "square_tries": 3,
    # m from the base center to the table edge for picking (the sim's start pose: the
    # task cups are in reach from there); off by more than edge_tol = drive to it.
    # 0 = do not change the distance.
    "edge_distance": 0.55,
    "edge_tol": 0.03,
    "w_max": 0.3,  # rad/s, turning in place
    "w_min": 0.08,  # rad/s, slowest turn command that still turns the base
    # --- search when no cup is in view (roadmap 1.2)
    "search_pan_step": 0.6,  # rad between head pans (the camera sees ~1.2 rad across)
    "search_pan_steps": 2,  # pans each way from head_pan; 0 = do not turn the head
    "search_tilt_up": 0.3,  # rad, then the same pans with the head tilted this much up
    # m, then drive along the heading in these steps, up to search_drive_max each way
    # (forward first); 0 = do not drive. Driving is blind: keep the path clear.
    "search_drive_step": 0.4,
    "search_drive_max": 1.2,
    "search_samples": 3,  # samples per look (fewer than `samples`: only to see a cup)
    # m, grasp point height above the cup's center while driving (arm retracted, so not
    # over the table). Keep it low: at 0.25 m the raised gripper sat between the head
    # camera and a cup on a 74 cm table, so ALIGN lost the cup after the first drive.
    "ready_above_cup": 0.05,
    # --- base moves (odometry-steered)
    "v_max": 0.3,  # m/s, base speed far from the target (max_base_speed caps it)
    "v_min": 0.06,  # m/s, slowest command that still moves the base (sim: wheel friction)
    "decel": 0.2,  # m/s^2, how hard the base slows down near the target
    "drive_tol": 0.003,  # m, a drive stops this close to its target
    "align_tol": 0.008,  # m, grasps tested reliable within +/-1 cm of the arm line
    "max_drives": 3,
    # --- measuring while stopped. The sim renders a new image only every 0.1 s of sim
    # time and re-sends the last one in between, so "fresh" is decided by the ROS clock.
    "settle_s": 0.3,  # s to wait after stopping before measuring (~3 sim renders)
    "sample_dt": 0.1,  # s between samples (one sim render)
    "samples": 5,  # measurements per position (median)
    "max_spread": 0.01,  # m, if samples spread more, take more
    # Cross-check the measured cup height against cup_height (roadmap 1.5): off by more
    # than this warns, or fails with cup_height_strict
    "cup_height_tol": 0.03,
    "cup_height_strict": False,
    # m above the table top to grasp at (roadmap 1.6): half of cup_height. The measured
    # center sits ~2.5 cm too high (the camera sees the top and inside of the cup more
    # than its lower half). < 0 = grasp at the measured center.
    "grasp_height": 0.06,
    # --- wrist yaw covers the last few cm (measured with sim/check_robot_layer.py)
    "use_wrist_yaw": True,
    "max_yaw": 0.25,  # rad (~14 deg): about +/-6.7 cm of sideways reach
    "arm_len": 0.270,  # m, wrist yaw axis to grasp center
    "yaw_sign": 1.0,  # +yaw moves the grasp point toward +x of base_link
    "yaw_check_tol": 0.010,  # m, grasp point must end this close to the cup's x
}

# carry_to_stack (roadmap task 2). It also reads CUP, and FIND_CUP's base-move (v_*, decel,
# drive_tol, align_tol, max_drives), turning (w_max, w_min), measuring (settle_s,
# sample_dt, samples, max_spread), tape colour (tape_*), nav_timeout and gripper_closed
# values. Names here must not repeat FIND_CUP's: all parameters share one namespace.
CARRY = {
    # phases to run: all, or a list of localize,navigate,find_tape,square,align
    "carry_steps": "all",
    # --- named poses in the map frame, [x, y, yaw] (roadmap 2.2). The sim map's origin
    # (stack_cups/maps/sim_task_room) is where the robot starts in the task scene.
    "start_pose": [0.0, 0.0, 0.0],  # where the robot was when its driver started (odom's origin)
    "loose_cups_pose": [0.2, 0.0, 0.0],  # next to the loose cups (for task 1.3)
    "stacking_pose": [0.75, 0.0, 0.0],  # next to the stacking area, arm side to the table
    "poses_checked": True,  # false: the poses are not measured on this map, so never navigate
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
    "tape_square_tol": 0.02,  # rad, base parallel to the tape's edge within this
    "tape_square_tries": 5,  # turns to get there (sim: short turns end 1-3 deg off)
    "w_decel": 0.5,  # rad/s^2, how hard turns slow down near the target heading
    # Arm extension that should reach the tape's center from where the robot parks (m).
    # Nav2 stops up to ~25 cm off, so if the extension needed is more than reach_tol from
    # reach_target, the base sidesteps (turn 90 deg, drive, turn back) to fix it first.
    "reach_target": 0.31,
    "reach_tol": 0.12,
    "max_reach": 0.5,  # m, most arm extension the tape's center may need in the end
}

ALL = {**ROBOT, **CUP, **FIND_CUP, **CARRY}
