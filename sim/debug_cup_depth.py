#!/usr/bin/env python3
"""Run find_cup's depth pipeline on one frame and print every intermediate step.

    # from your workspace root, with the sim running (use_cameras:=true)
    source install/setup.bash
    python3 src/cup-stacking-stretch-project/sim/debug_cup_depth.py
"""
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from stack_cups import perception as P


class Grab(Node):
    def __init__(self):
        super().__init__("debug_cup_depth")
        self.b = CvBridge()
        self.data = {}
        q = qos_profile_sensor_data
        self.create_subscription(Image, "/camera/color/image_raw", lambda m: self.put("color", m), q)
        self.create_subscription(Image, "/camera/depth/image_rect_raw",
                                 lambda m: self.put("depth", m), q)
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 lambda m: self.put("cinfo", m), q)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info",
                                 lambda m: self.put("dinfo", m), q)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def put(self, k, m):
        self.data[k] = m

    def T(self, target, source):
        tf = self.tf.lookup_transform(target, source, Time())
        q, p = tf.transform.rotation, tf.transform.translation
        return P.quat_to_mat(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])


def main():
    rclpy.init()
    n = Grab()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        run(n)
    finally:
        ex.shutdown()
        th.join(timeout=2.0)
        n.destroy_node()
        rclpy.try_shutdown()


def run(n):
    t0 = time.time()
    while len(n.data) < 4 and time.time() - t0 < 15:
        time.sleep(0.2)
    print("received:", sorted(n.data))
    if len(n.data) < 4:
        return
    time.sleep(1.0)
    c, d, ci, di = (n.data[k] for k in ("color", "depth", "cinfo", "dinfo"))
    img = n.b.imgmsg_to_cv2(c, "bgr8")
    dep = np.array(n.b.imgmsg_to_cv2(d, "32FC1"), dtype=np.float32)
    print(f"color {c.width}x{c.height} {c.encoding} frame={c.header.frame_id}")
    print(f"depth {d.width}x{d.height} {d.encoding} frame={d.header.frame_id}")
    fin = dep[np.isfinite(dep)]
    print(f"depth values: min={fin.min():.3f} max={fin.max():.3f} "
          f"median={np.median(fin):.3f} zeros={np.mean(dep == 0) * 100:.0f}% "
          f"nan={np.mean(~np.isfinite(dep)) * 100:.0f}%")
    print("color K:", np.round(ci.k, 1).tolist())
    print("depth K:", np.round(di.k, 1).tolist())

    bbox, mask = P.find_red_blob(img)
    print("red bbox (x, y, w, h):", bbox, " red pixels:", int(np.count_nonzero(mask)))
    if bbox is None:
        return
    x, y, w, h = bbox
    patch = dep[y:y + h, x:x + w]
    print(f"depth inside bbox: median={np.nanmedian(patch):.3f} "
          f"min={np.nanmin(patch):.3f} max={np.nanmax(patch):.3f}")

    try:
        R, t = n.T("base_link", d.header.frame_id)
        Rcd, tcd = n.T(c.header.frame_id, d.header.frame_id)
    except Exception as e:  # noqa: BLE001
        print("TF lookup failed:", e)
        return
    print("depth cam in base_link: t =", t.round(3), "\n  R =\n", R.round(3))
    print("depth->color: t =", tcd.round(4), " R diag =", np.diag(Rcd).round(3))

    pad = 15
    pts = P.backproject(dep, di.k, region=(x - pad, y - pad, w + 2 * pad, h + 2 * pad))
    print("depth points near bbox:", len(pts))
    if len(pts):
        uv = P.project(P.to_base(pts, Rcd, tcd), ci.k)
        print(f"  they project into color image at u {uv[:, 0].min():.0f}..{uv[:, 0].max():.0f}, "
              f"v {uv[:, 1].min():.0f}..{uv[:, 1].max():.0f} (bbox u {x}..{x + w}, v {y}..{y + h})")
        uvi = np.round(uv).astype(int)
        H, W = mask.shape
        ok = (uvi[:, 0] >= 0) & (uvi[:, 0] < W) & (uvi[:, 1] >= 0) & (uvi[:, 1] < H)
        red = np.zeros(len(pts), bool)
        red[ok] = mask[uvi[ok, 1], uvi[ok, 0]] > 0
        print("  of those, landing on red pixels:", int(red.sum()))
        base = P.to_base(pts, R, t)
        print("  bbox-center point in base_link:",
              P.to_base(P.backproject(dep, di.k, region=(x + w // 2, y + h // 2, 1, 1)), R, t).round(3))
        print("  near-bbox points in base_link: x", base[:, 0].min().round(3), "..",
              base[:, 0].max().round(3), " y", base[:, 1].min().round(3), "..",
              base[:, 1].max().round(3), " z", base[:, 2].min().round(3), "..",
              base[:, 2].max().round(3))
    cup, r = P.locate_cup(dep, di.k, R, t, mask, ci.k, Rcd, tcd, bbox)
    print("locate_cup result:", None if cup is None else cup.round(3), r)


if __name__ == "__main__":
    main()
