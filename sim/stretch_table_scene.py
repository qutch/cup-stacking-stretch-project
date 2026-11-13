#!/usr/bin/env python3
"""
Simple MuJoCo scene: the Stretch robot next to a square table.

Uses Hello Robot's `stretch_mujoco` simulator (the same one the
`stretch_simulation` ROS 2 package drives). Requires stretch_mujoco installed from
source (it is not on PyPI); see the repo README's setup section.

Run:
    python3 stretch_table_scene.py              # with the MuJoCo viewer
    python3 stretch_table_scene.py --headless   # no window
    python3 stretch_table_scene.py --build-only # just compile the scene and exit
"""

import argparse
import time

import mujoco

# ---------------------------------------------------------------------------
# Table parameters (meters)
# ---------------------------------------------------------------------------
TABLE_SIDE = 0.80          # square top: 0.8 m x 0.8 m
TABLE_HEIGHT = 0.75        # floor to top surface
TOP_THICKNESS = 0.04
LEG_SIZE = 0.05            # square legs, 5 cm
TABLE_POS = (0.0, -1.0)    # (x, y) of table center; robot starts at the origin
TABLE_RGBA = (0.55, 0.38, 0.22, 1.0)


def _box(body, name, pos, half_size, rgba, mass):
    geom = body.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.pos = pos
    geom.size = half_size
    geom.rgba = rgba
    geom.mass = mass
    return geom


CUP_RADIUS = 0.04
CUP_HEIGHT = 0.10
CUP_RGBA = (0.85, 0.08, 0.08, 1.0)


def build_scene(cup_xy=None) -> mujoco.MjModel:
    """Load the Stretch robot and add a floor, a light, and a square table.

    If cup_xy=(x, y) is given (world coordinates), a red cup is placed upright
    on the table top at that spot.
    """
    from stretch_mujoco.utils import default_robot_xml_path

    # Loading from the robot's own file keeps its mesh asset paths valid.
    spec = mujoco.MjSpec.from_file(default_robot_xml_path)
    spec.modelname = "stretch_table_scene"
    world = spec.worldbody

    # Floor
    floor = world.add_geom()
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0, 0, 0.05]
    floor.rgba = [0.25, 0.25, 0.25, 1]

    # Overhead light
    light = world.add_light()
    light.pos = [0, 0, 2.5]
    light.dir = [0, 0, -1]
    light.directional = True

    # Square table: a static body (no joint) made of a top and four legs
    table = world.add_body()
    table.name = "table"
    table.pos = [TABLE_POS[0], TABLE_POS[1], 0]

    half_side = TABLE_SIDE / 2
    top_z = TABLE_HEIGHT - TOP_THICKNESS / 2
    _box(table, "table_top", [0, 0, top_z],
         [half_side, half_side, TOP_THICKNESS / 2], TABLE_RGBA, mass=5.0)

    leg_half_h = (TABLE_HEIGHT - TOP_THICKNESS) / 2
    inset = half_side - LEG_SIZE / 2
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        _box(table, f"table_leg_{i}", [sx * inset, sy * inset, leg_half_h],
             [LEG_SIZE / 2, LEG_SIZE / 2, leg_half_h], TABLE_RGBA, mass=1.0)

    if cup_xy is not None:
        cup = world.add_body()
        cup.name = "cup"
        cup.pos = [cup_xy[0], cup_xy[1], TABLE_HEIGHT + CUP_HEIGHT / 2 + 0.002]
        cup.add_freejoint()
        geom = cup.add_geom()
        geom.name = "cup"
        geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        geom.size = [CUP_RADIUS, CUP_HEIGHT / 2, 0]
        geom.rgba = CUP_RGBA
        geom.mass = 0.1

    return spec.compile()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true", help="run without the viewer")
    parser.add_argument("--build-only", action="store_true",
                        help="compile the scene, print a summary, and exit")
    args = parser.parse_args()

    model = build_scene()
    print(f"Scene compiled: {model.nbody} bodies, {model.ngeom} geoms, {model.nu} actuators")
    if args.build_only:
        return

    from stretch_mujoco import StretchMujocoSimulator

    sim = StretchMujocoSimulator(model=model)
    sim.start(headless=args.headless)
    try:
        sim.stow()
        while sim.is_running():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()


if __name__ == "__main__":
    main()
