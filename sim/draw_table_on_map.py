#!/usr/bin/env python3
"""Draw the cup table's top into a saved sim map as occupied (roadmap 0.8).

The lidar is about 0.2 m above the floor, so it sees only the table's four legs. On the raw
SLAM map Nav2 then plans paths under the table, where the robot's mast hits the table top.
(The starter's WVH setup handles tables with a keepout map.) Run this after saving a new map
of the task scene (see sim/map_sim_scene.py), with the workspace sourced:

    python3 sim/draw_table_on_map.py stack_cups/maps/sim_task_room.yaml
"""

import os
import sys

import cv2
import yaml
from stack_cups.sim_scene import TABLE


def main():
    path = sys.argv[1]
    with open(path) as f:
        meta = yaml.safe_load(f)
    image = os.path.join(os.path.dirname(os.path.abspath(path)), meta["image"])
    img = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    rows = img.shape[0]

    def cell(x, y):  # map_server: the image's bottom-left pixel is at the origin
        return int((x - ox) / res), rows - 1 - int((y - oy) / res)

    near, far = TABLE["near_y"], TABLE["near_y"] - TABLE["depth"]
    (c0, r0), (c1, r1) = cell(TABLE["x0"], near), cell(TABLE["x1"], far)
    cv2.rectangle(img, (min(c0, c1), min(r0, r1)), (max(c0, c1), max(r0, r1)), 0, -1)
    cv2.imwrite(image, img)
    print(f"table x {TABLE['x0']:+.2f}..{TABLE['x1']:+.2f}, y {far:+.2f}..{near:+.2f} "
          f"drawn as occupied into {image}")


if __name__ == "__main__":
    main()
