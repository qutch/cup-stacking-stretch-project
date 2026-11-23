#!/usr/bin/env python3
"""Print how fast the sim runs compared with real time, from /clock over N wall seconds.
With the sim up:
    python3 sim/clock_rate.py        # 20 s
    python3 sim/clock_rate.py 60
"""

import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock


def main():
    wall_s = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = rclpy.create_node("clock_rate")
    stamps = []
    node.create_subscription(
        Clock, "/clock", lambda m: stamps.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
    print(f"measuring /clock for {wall_s:.0f} s of real time ...", flush=True)
    w0 = time.time()
    while time.time() - w0 < wall_s:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.time() - w0
    if len(stamps) < 2:
        print("fewer than 2 /clock messages: is the sim running?")
    else:
        sim = stamps[-1] - stamps[0]
        print(f"sim advanced {sim:.2f} s in {wall:.1f} s real time: "
              f"{100 * sim / wall:.1f}% of real time")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
