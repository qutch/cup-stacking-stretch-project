"""HeadCamera: the head camera's frames for a CupRobot node, and fresh measurements.

Any node built on CupRobot (find_cup, and the placing node of task 4) can own one:
    self.camera = HeadCamera(self)
    snap, why = self.camera.snapshot()        # latest color + depth + transforms
    m = self.camera.measure_fresh(measure_once, samples=5, settle_s=0.3, sample_dt=0.1,
                                  max_spread=0.01)

`measure_fresh` only measures while the base is stopped, on frames rendered after it
stopped, takes the median of several samples, and works in the fixed odom frame so
samples agree even if the base creeps.
"""

import threading

import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from stack_cups import motion as M
from stack_cups import perception as P

TOPICS = {
    "color": "/camera/color/image_raw",
    "depth": "/camera/depth/image_rect_raw",
    "color_info": "/camera/color/camera_info",
    "depth_info": "/camera/depth/camera_info",
}


class HeadCamera:
    def __init__(self, robot, topics=None):
        """robot: a started CupRobot (its node, TF, clock and odometry are used)."""
        self.robot = robot
        self.topics = dict(TOPICS, **(topics or {}))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color = self.depth = None
        self.color_info = self.depth_info = None
        self.depth_count = 0
        self.last_miss = ""
        cb, qos = robot.cb_group, qos_profile_sensor_data
        robot.create_subscription(Image, self.topics["color"], self._on_color, qos,
                                  callback_group=cb)
        robot.create_subscription(Image, self.topics["depth"], self._on_depth, qos,
                                  callback_group=cb)
        robot.create_subscription(CameraInfo, self.topics["color_info"], self._on_color_info,
                                  qos, callback_group=cb)
        robot.create_subscription(CameraInfo, self.topics["depth_info"], self._on_depth_info,
                                  qos, callback_group=cb)

    # Images are stored as messages and only converted when measuring: the sim re-sends
    # frames far more often than it renders them, and converting each one takes CPU
    # the sim needs.
    def _on_color(self, msg):
        with self.lock:
            self.color = msg

    def _on_depth(self, msg):
        with self.lock:
            self.depth = msg
            self.depth_count += 1

    def _on_color_info(self, msg):
        self.color_info = msg

    def _on_depth_info(self, msg):
        self.depth_info = msg

    def latest(self):
        """(color image, frame id), (depth image, frame id); each None if not received."""
        with self.lock:
            cmsg, dmsg = self.color, self.depth
        color = depth = None
        if cmsg is not None:
            color = (self.bridge.imgmsg_to_cv2(cmsg, "bgr8"), cmsg.header.frame_id)
        if dmsg is not None:
            # meters in the sim (32FC1), millimeters on the real robot (16UC1)
            raw = self.bridge.imgmsg_to_cv2(dmsg, "passthrough")
            depth = (P.depth_in_meters(raw, dmsg.encoding), dmsg.header.frame_id)
        return color, depth

    def missing(self, base_frame="base_link"):
        """Names of the camera inputs not received yet ([] when all are there)."""
        color, depth = self.latest()
        have = {
            "color image": color is not None,
            "depth image": depth is not None,
            "camera info": self.color_info is not None and self.depth_info is not None,
            "TF": depth is not None and self.robot.lookup(base_frame, depth[1]) is not None,
        }
        return [k for k, ok in have.items() if not ok]

    def wait_new_depth(self, n, timeout=3.0):
        """Wait until `n` more depth frames have arrived (ROS time). False on timeout."""
        with self.lock:
            target = self.depth_count + n
        t0 = self.robot.now()
        # _idle honours stop requests and keeps OpenCV windows alive; clock_stalled ends
        # the wait if ROS time stops (a frozen sim would never reach the timeout)
        while self.robot.now() - t0 < timeout and not self.robot.clock_stalled():
            with self.lock:
                if self.depth_count >= target:
                    return True
            self.robot._idle(0.02)
        return False

    def snapshot(self, base_frame="base_link"):
        """The latest frames and transforms: (dict, "") or (None, why).

        dict keys: img (BGR), depth (meters), K_color, K_depth, T_base (depth optical
        frame -> base_frame, as (R, t)), T_cd (depth optical -> color optical frame)."""
        color, depth = self.latest()
        if color is None or depth is None or self.color_info is None or self.depth_info is None:
            return None, "no camera data"
        (img, color_frame), (dimg, depth_frame) = color, depth
        T_base = self.robot.lookup(base_frame, depth_frame)
        T_cd = self.robot.lookup(color_frame, depth_frame)
        if T_base is None or T_cd is None:
            return None, f"TF {depth_frame} -> {base_frame} or {color_frame} unavailable"
        return {"img": img, "depth": dimg, "K_color": self.color_info.k,
                "K_depth": self.depth_info.k, "T_base": T_base, "T_cd": T_cd}, ""

    def measure_fresh(self, measure_once, samples, settle_s, sample_dt, max_spread,
                      key="cup", on_sample=None):
        """Measure a target while stopped. Returns the median measurement or None.

        measure_once() -> dict with `key` (xyz in base_link) when the target was found,
        else a dict whose "why" says what went wrong (or None). Waits `settle_s` of ROS
        time so no frame used was rendered while moving, takes `samples` measurements
        `sample_dt` apart, and returns the last one with `key` replaced by their median
        (in odom, mapped back into base_link with the base's pose now) and "spread"
        added. If they spread more than `max_spread`, takes `samples` more once, and
        returns None if they still do. On failure the reasons are in self.last_miss."""
        robot = self.robot
        n = int(samples)
        if not self.wait_new_depth(1):
            raise RuntimeError("no depth images arriving")
        robot.sleep(settle_s)
        found, misses = [], {}

        def collect(k):
            tries = 0
            while len(found) < k and tries < 3 * k:
                tries += 1
                if tries > 1:
                    robot.sleep(sample_dt)
                m = measure_once()
                if on_sample is not None and m:
                    on_sample(m)
                if m and key in m:
                    pose = robot.odom_pose()
                    if pose is not None:
                        # In the fixed odom frame, so samples taken while the base drifts
                        # still agree
                        p = m[key]
                        m.update(pose=pose, odom_point=np.array([*M.base_to_odom(p[:2], pose),
                                                                 p[2]]))
                    found.append(m)
                else:
                    why = (m or {}).get("why", "no camera data")
                    misses[why] = misses.get(why, 0) + 1

        collect(n)
        if not found:
            self.last_miss = "; ".join(f"{k} (x{v})" for k, v in misses.items())
            robot.get_logger().warn(f"{key} not found: {self.last_miss}")
            return None

        def center():
            k = "odom_point" if all("odom_point" in s for s in found) else key
            return (k, *M.robust_center([s[k] for s in found], max_spread))

        k, c, spread, ok = center()
        if not ok:
            collect(2 * n)
            k, c, spread, ok = center()
        if not ok:
            # still too noisy to drive or reach on: report it like a miss
            self.last_miss = (f"{key} measurements spread {spread * 1000:.0f} mm "
                              f"(over {max_spread * 1000:.0f} mm) in {len(found)} samples")
            robot.get_logger().warn(f"{key} not measured reliably: {self.last_miss}")
            return None
        out = dict(found[-1])
        if k == "odom_point":
            # back into base_link with the base's pose now, i.e. where the target is
            # relative to the robot right before the next move
            pose = robot.odom_pose()
            base_xy = M.odom_to_base(c[:2], pose)
            out.update(odom_point=c, pose=pose, **{key: np.array([base_xy[0], base_xy[1], c[2]])})
            drift = abs(found[-1]["pose"][0] - found[0]["pose"][0])
            if drift > 0.002:
                robot.get_logger().warn(f"the base moved {drift * 1000:.0f} mm while "
                                        f"measuring (it should be still)")
        else:
            out[key] = c
        out["spread"] = spread
        out["samples"] = found
        return out
