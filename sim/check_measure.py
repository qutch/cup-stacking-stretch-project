#!/usr/bin/env python3
"""Measure the cup the way find_cup does (same node class, same clock), without moving
anything, and print why each attempt fails. Aim the head first (or run right after a
find_cup run, which leaves it aimed). With the sim up (use_cameras:=true):
    PYTHONUNBUFFERED=1 python3 sim/check_measure.py
"""

import time

import rclpy

from stack_cups.cup_finder_controller import FindCup


def main():
    n = FindCup()
    n.start("check_measure")
    try:
        n.wait_for_data()
        print(f"clock: {'sim' if n.get_parameter('use_sim_time').value else 'wall'} time",
              flush=True)
        _, depth = n.latest()
        print("depth frame:", depth[1], flush=True)
        print("TF base<-depth:", n.lookup("base_link", depth[1]), flush=True)
        for i in range(3):
            m = n.measure(full=False)
            print(f"single measure {i + 1}:",
                  None if m is None else (m.get("cup"), m.get("bbox"), m.get("why")), flush=True)
            time.sleep(0.5)
        m = n.measure_fresh()
        print("measure_fresh:", None if m is None else (m["cup"], m["spread"]),
              "| misses:", n.last_miss, flush=True)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
