#!/usr/bin/env python3
"""
Put a red cup at a random spot on the table, then drive Stretch around the
table and park in front of the cup.

The robot starts at the origin (on the table's +y side). It works out which
table edge is closest to the cup, follows a safe loop around the table's
corners (taking the shorter way round), drives in to a standoff point opposite
the cup, and turns to its final heading.

Run (needs stretch_mujoco installed, same as stretch_table_scene.py):
    python3 cup_approach.py                 # random cup, viewer on
    python3 cup_approach.py --seed 3        # repeatable cup position
    python3 cup_approach.py --face arm      # park sideways so the arm points at the cup
    python3 cup_approach.py --headless
"""

import argparse
import math
import random
import time

from stretch_table_scene import TABLE_POS, TABLE_SIDE, build_scene

# ---------------------------------------------------------------------------
# Tunables (meters, radians, seconds)
# ---------------------------------------------------------------------------
CUP_EDGE_MARGIN = 0.08      # keep the cup at least this far from the table edge
RING_CLEARANCE = 0.60       # path around the table stays this far from its edges
FRONT_STANDOFF = 0.45       # --face front: base center to table edge
ARM_STANDOFF = 0.35         # --face arm: base center to table edge
ARM_LINE_OFFSET = 0.021     # arm reach line sits this far behind base center (measured)

MAX_V = 0.20                # m/s
MAX_W = 0.8                 # rad/s
POS_TOL = 0.03
YAW_TOL = 0.03
HEADING_GATE = 0.25         # only drive forward when pointed within this of the target
LOOP_DT = 0.05
WAYPOINT_TIMEOUT = 60.0

HALF = TABLE_SIDE / 2
TX, TY = TABLE_POS

# Outward normals of each table edge, keyed by side name
SIDES = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def random_cup_xy(rng):
    lim = HALF - CUP_EDGE_MARGIN
    return (TX + rng.uniform(-lim, lim), TY + rng.uniform(-lim, lim))


def nearest_side(cx, cy):
    """Table edge closest to the cup, and the cup's distance to it."""
    dists = {
        "+x": (TX + HALF) - cx, "-x": cx - (TX - HALF),
        "+y": (TY + HALF) - cy, "-y": cy - (TY - HALF),
    }
    side = min(dists, key=dists.get)
    return side, dists[side]


def goal_pose(cx, cy, face):
    """Where the base should park (x, y, heading) for the chosen facing mode."""
    side, depth = nearest_side(cx, cy)
    nx, ny = SIDES[side]
    inward = math.atan2(-ny, -nx)               # heading that faces the table
    if face == "front":
        d = depth + FRONT_STANDOFF
        return cx + nx * d, cy + ny * d, inward, side
    # Arm extends along the base's -y axis, so heading = inward + 90 degrees.
    heading = wrap(inward + math.pi / 2)
    d = depth + ARM_STANDOFF
    gx = cx + nx * d + ARM_LINE_OFFSET * math.cos(heading)
    gy = cy + ny * d + ARM_LINE_OFFSET * math.sin(heading)
    return gx, gy, heading, side


def plan_waypoints(start_xy, goal_xy, side):
    """Corners of a clearance ring around the table, taken the shorter way round,
    then a lead-in point on the ring opposite the goal, then the goal."""
    r = HALF + RING_CLEARANCE
    nx, ny = SIDES[side]
    # Lead-in point: on the ring line of the goal's side, lined up with the goal
    if nx:
        lead = (TX + nx * r, goal_xy[1])
    else:
        lead = (goal_xy[0], TY + ny * r)

    ang = lambda p: math.atan2(p[1] - TY, p[0] - TX)
    a0, a1 = ang(start_xy), ang(lead)
    ccw = (a1 - a0) % (2 * math.pi)
    direction = 1 if ccw <= math.pi else -1
    sweep = ccw if direction == 1 else 2 * math.pi - ccw

    corners = [(TX + sx * r, TY + sy * r) for sx, sy in [(1, 1), (-1, 1), (-1, -1), (1, -1)]]
    between = []
    for c in corners:
        off = ((ang(c) - a0) * direction) % (2 * math.pi)
        if 1e-6 < off < sweep - 1e-6:
            between.append((off, c))
    return [c for _, c in sorted(between)] + [lead, goal_xy]


class BaseDriver:
    """Closed-loop point-to-point driving using the sim's ground-truth base pose."""

    def __init__(self, sim):
        self.sim = sim

    def pose(self):
        b = self.sim.pull_status().base
        return b.x, b.y, b.theta

    def stop(self):
        self.sim.set_base_velocity(0.0, 0.0)

    def turn_to(self, heading):
        t0 = time.time()
        while self.sim.is_running() and time.time() - t0 < WAYPOINT_TIMEOUT:
            err = wrap(heading - self.pose()[2])
            if abs(err) < YAW_TOL:
                break
            w = max(-MAX_W, min(MAX_W, 2.0 * err))
            if abs(w) < 0.15:
                w = math.copysign(0.15, w)      # enough to overcome wheel friction
            self.sim.set_base_velocity(0.0, w)
            time.sleep(LOOP_DT)
        self.stop()

    def go_to(self, gx, gy):
        x, y, _ = self.pose()
        self.turn_to(math.atan2(gy - y, gx - x))
        t0 = time.time()
        while self.sim.is_running() and time.time() - t0 < WAYPOINT_TIMEOUT:
            x, y, th = self.pose()
            dist = math.hypot(gx - x, gy - y)
            if dist < POS_TOL:
                break
            err = wrap(math.atan2(gy - y, gx - x) - th)
            if abs(err) > math.pi / 2:          # overshot: target is behind us
                break
            v = min(MAX_V, 1.0 * dist) if abs(err) < HEADING_GATE else 0.0
            w = max(-MAX_W, min(MAX_W, 2.5 * err))
            self.sim.set_base_velocity(v, w)
            time.sleep(LOOP_DT)
        self.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, help="random seed for the cup position")
    parser.add_argument("--face", choices=["front", "arm"], default="front",
                        help="front: robot faces the cup; arm: arm side faces the cup")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    cx, cy = random_cup_xy(rng)
    gx, gy, gth, side = goal_pose(cx, cy, args.face)
    waypoints = plan_waypoints((0.0, 0.0), (gx, gy), side)

    print(f"Cup at ({cx:.2f}, {cy:.2f}) -> nearest table edge: {side}")
    print(f"Goal pose: ({gx:.2f}, {gy:.2f}, {math.degrees(gth):.0f} deg), face={args.face}")
    print("Route:", " -> ".join(f"({x:.2f}, {y:.2f})" for x, y in waypoints))

    from stretch_mujoco import StretchMujocoSimulator

    sim = StretchMujocoSimulator(model=build_scene(cup_xy=(cx, cy)))
    sim.start(headless=args.headless)
    driver = BaseDriver(sim)
    try:
        sim.stow()
        time.sleep(2.0)
        for i, (wx, wy) in enumerate(waypoints, 1):
            print(f"  [{i}/{len(waypoints)}] driving to ({wx:.2f}, {wy:.2f})")
            driver.go_to(wx, wy)
        driver.turn_to(gth)

        x, y, th = driver.pose()
        print(f"Arrived at ({x:.2f}, {y:.2f}, {math.degrees(th):.0f} deg); "
              f"position error {math.hypot(gx - x, gy - y) * 100:.1f} cm, "
              f"heading error {math.degrees(abs(wrap(gth - th))):.1f} deg")
        print("Ctrl+C to quit.")
        while sim.is_running():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()
        sim.stop()


if __name__ == "__main__":
    main()
