"""Check that the robot (real or sim) provides every topic, service, action, joint and TF
frame stack_cups uses (roadmap 0.3). It only listens; nothing moves.

    ros2 run stack_cups check_interfaces             # driver (and cameras) running
    ros2 run stack_cups check_interfaces --nav       # also the lidar topics Nav2 needs

Real robot: start the driver with broadcast_odom_tf:=True, the head camera
(stretch_realsense.launch.py) and, for --nav, rplidar.launch.py first.
Exit code 1 if anything required is missing.
"""

import argparse
import sys
import time

import rclpy
import rclpy.action
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

TOPICS = {  # topic: (type, required, why)
    "/stretch/joint_states": (JointState, True, "joint positions (HelloNode)"),
    "/odom": (Odometry, True, "odometry-steered base moves (drive_by)"),
    "/camera/color/image_raw": (Image, True, "red cup detection"),
    "/camera/depth/image_rect_raw": (Image, True, "3D cup and table position"),
    "/camera/color/camera_info": (CameraInfo, True, "color intrinsics"),
    "/camera/depth/camera_info": (CameraInfo, True, "depth intrinsics"),
    "/mode": (String, True, "driver mode (HelloNode)"),
    "/is_runstopped": (Bool, True, "stop when the runstop is pressed"),
}
NAV_TOPICS = {
    "/scan": (LaserScan, True, "AMCL localization (parcs_stretch_nav2)"),
    "/scan_filtered": (LaserScan, True, "Nav2 costmaps and slam_toolbox"),
}
SUBSCRIBED = {"/stretch/cmd_vel": "base speed commands (the driver must subscribe)"}
SERVICES = ["/switch_to_navigation_mode", "/switch_to_position_mode", "/stop_the_robot",
            "/stow_the_robot"]
ACTION = "/stretch_controller/follow_joint_trajectory"
JOINTS = ["joint_lift", "wrist_extension", "joint_wrist_yaw", "joint_wrist_pitch",
          "joint_wrist_roll", "joint_head_pan", "joint_head_tilt", "joint_gripper_finger_left"]
FRAMES = [("base_link", "link_grasp_center", True),
          ("base_link", "camera_depth_optical_frame", True),
          ("camera_color_optical_frame", "camera_depth_optical_frame", True),
          ("odom", "base_link", True), ("base_link", "laser", False),
          ("base_footprint", "base_link", False)]


class Check(Node):
    def __init__(self, nav):
        super().__init__("check_interfaces")
        self.topics = dict(TOPICS, **(NAV_TOPICS if nav else {}))
        self.msgs = {}
        for name, (kind, _, _) in self.topics.items():
            self.create_subscription(kind, name, lambda m, n=name: self.msgs.setdefault(n, m),
                                     qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nav", action="store_true", help="also check the lidar topics for Nav2")
    ap.add_argument("--wait", type=float, default=8.0, help="seconds to listen")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Check(args.nav)
    t0 = time.time()
    while time.time() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    rows, failed = [], False

    def row(status, item, detail):
        nonlocal failed
        failed |= status == "FAIL"
        rows.append((status, item, detail))

    names = dict(node.get_topic_names_and_types())
    for name, (kind, required, why) in node.topics.items():
        want = f"{kind.__module__.split('.')[0]}/msg/{kind.__name__}"
        if name not in names:
            row("FAIL" if required else "WARN", name, f"not published ({why})")
        elif want not in names[name]:
            row("FAIL", name, f"type {names[name]}, expected {want}")
        elif name not in node.msgs:
            row("WARN", name, "advertised but no message received")
        else:
            row("PASS", name, describe(name, node.msgs[name]))
    for name, why in SUBSCRIBED.items():
        n = node.count_subscribers(name)
        row("PASS" if n else "FAIL", name, f"{n} subscriber(s): {why}")
    services = {n for n, _ in node.get_service_names_and_types()}
    for name in SERVICES:
        row("PASS" if name in services else "FAIL", name, "service")
    actions = {n for n, _ in rclpy.action.get_action_names_and_types(node)}
    row("PASS" if ACTION in actions else "FAIL", ACTION, "FollowJointTrajectory action")
    js = node.msgs.get("/stretch/joint_states")
    if js is not None:
        missing = [j for j in JOINTS if j not in js.name]
        row("FAIL" if missing else "PASS", "joint names",
            f"missing {missing}" if missing
            else f"all {len(JOINTS)} joints we command are reported")
    for parent, child, required in FRAMES:
        ok = node.tf.can_transform(parent, child, Time())
        row("PASS" if ok else ("FAIL" if required else "WARN"), f"TF {parent} -> {child}",
            "available" if ok else ("missing" + ("" if required else " (needed for Nav2: launch "
                                                 "the driver with broadcast_odom_tf:=True)")))
    width = max(len(r[1]) for r in rows)
    for status, item, detail in rows:
        print(f"{status:4s}  {item:{width}s}  {detail}")
    print("RESULT", "FAIL: something stack_cups needs is missing" if failed
          else "all required interfaces present")
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


def describe(name, msg):
    """One-line summary of a received message (encodings and units matter on the robot)."""
    if isinstance(msg, Image):
        note = ""
        if "depth" in name:
            note = {"32FC1": " (meters)", "16UC1": " (millimeters)"}.get(msg.encoding,
                                                                         " (unknown units)")
        return f"{msg.width}x{msg.height} {msg.encoding}{note}, frame {msg.header.frame_id}"
    if isinstance(msg, CameraInfo):
        return f"{msg.width}x{msg.height}, fx {msg.k[0]:.1f}, frame {msg.header.frame_id}"
    if isinstance(msg, LaserScan):
        return (f"{len(msg.ranges)} beams, {msg.angle_min:.2f}..{msg.angle_max:.2f} rad, "
                f"frame {msg.header.frame_id}")
    if isinstance(msg, Odometry):
        return f"{msg.header.frame_id} -> {msg.child_frame_id}"
    if isinstance(msg, JointState):
        return f"{len(msg.name)} joints"
    if isinstance(msg, (Bool, String)):
        return f"value {msg.data!r}"
    return type(msg).__name__


if __name__ == "__main__":
    main()
