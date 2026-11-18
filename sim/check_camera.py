#!/usr/bin/env python3
"""Grab one frame from a camera topic, print its brightness stats, and save it.

    python3 check_camera.py                          # head camera
    python3 check_camera.py /gripper_camera/image_raw
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

topic = sys.argv[1] if len(sys.argv) > 1 else "/camera/color/image_raw"
out = "/tmp/camera_check.png"


def main():
    rclpy.init()
    node = Node("camera_check")
    done = []

    def cb(msg):
        if done:
            return
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        cv2.imwrite(out, img)
        print(f"{topic}: {msg.width}x{msg.height} {msg.encoding}")
        print(f"  pixel min={img.min()} max={img.max()} mean={img.mean():.1f}")
        print(f"  red pixels (same thresholds as find_cup): {int(np.count_nonzero(red))}")
        print(f"  saved frame to {out}")
        done.append(True)

    node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    print(f"Waiting for one frame on {topic} ...")
    while rclpy.ok() and not done:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
