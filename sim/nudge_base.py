#!/usr/bin/env python3
"""Drive the base forward (+) or back (-) by an exact distance, to set up a test start
pose for find_cup. Run with the sim up, after `source install/setup.bash`:
    python3 sim/nudge_base.py 0.09      # 9 cm forward
"""

import sys

import rclpy

from stack_cups.robot import CupRobot


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-") or a[1:2].isdigit()]
    if not args:
        sys.exit("usage: nudge_base.py <meters, + forward / - back>")
    distance = float(args[0])
    r = CupRobot()
    r.start("nudge_base")
    try:
        r.enter_navigation_mode()
        moved = r.drive_by(distance, v_max=0.15, v_min=0.06, decel=0.1)
        print(f"asked {distance:+.3f} m, odometry says {moved:+.3f} m", flush=True)
    finally:
        r.drive(0.0)
        r.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
