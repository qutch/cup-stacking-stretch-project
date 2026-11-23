"""Sim scene for the full task (roadmap 0.4), for the Stretch MuJoCo sim.

A room with one table on the robot's arm side (-y). On the table: 6 tapered red cups
standing upright, and blue tape marking the stacking area at the table's +x end. Five
cups are within reach from the robot's start side; one is only reachable from the far
side, so going around the table can be tested.

Cup sizes use the same names as config/*.yaml (cup_height, cup_rim_diameter,
cup_base_diameter, cup_wall, cup_mass), so the sim cups match what the code expects.

    python3 -m stack_cups.sim_scene --out ~/cup_scene.xml       # write the scene file
    python3 -m stack_cups.sim_scene --check                     # simulate 3 s: cups stay up?
    python3 -m stack_cups.sim_scene --cups pyramid --check      # a finished 3-2-1 pyramid
    python3 -m stack_cups.sim_scene --seed 4 --check            # random loose-cup layout

The ROS sim loads it with: ros2 launch stack_cups sim.launch.py
World frame: the robot starts at the origin facing +x; its arm reaches toward -y.
"""

import argparse
import math
import random

import numpy as np

# --------------------------------------------------------------------- layout (meters)
TABLE = {
    "x0": -0.45, "x1": 1.45,      # table ends along x
    "near_y": -0.55,              # near edge (robot side); the robot starts 0.55 m away
    "depth": 0.76,                # near edge to far edge
    "height": 0.74,               # floor to table top
    "top_thickness": 0.03,
    "leg": 0.05,
}
TAPE = {
    "x0": 0.92, "x1": 1.40,       # stacking area on the table's +x end
    "y0": -0.58, "y1": -0.88,     # near and far side of the area
    "width": 0.025,               # blue tape strip width
}
# Loose cups (x, y) on the table, upright. The last one is only reachable from the far side.
LOOSE_CUPS = [(-0.30, -0.66), (-0.05, -0.80), (0.18, -0.67), (0.42, -0.79), (0.66, -0.68),
              (0.20, -1.12)]
# cups:=single: one loose cup on the near side (for grasp tests without other cups)
SINGLE_CUP = LOOSE_CUPS[2]
# cups:=touching: two loose cups standing rim to rim (roadmap 1.1: split them by depth)
# cups:=mixed: three loose cups, plus three already placed in the tape (the pyramid's base)
ROOM = {"x0": -2.0, "x1": 3.0, "y0": -2.6, "y1": 1.4, "height": 0.5, "thickness": 0.05}
# Asymmetric furniture so a 2D lidar map has features (an empty box room is symmetric)
FURNITURE = [  # (x, y, half sizes)
    (2.55, 1.05, (0.20, 0.30, 0.45)),
    (-1.70, -2.25, (0.15, 0.15, 0.40)),
    (2.65, -2.35, (0.30, 0.20, 0.25)),
    (-1.75, 0.90, (0.20, 0.45, 0.30)),
]
# Placeholder cup (typical 16 oz party cup); real size unknown, see config/*.yaml
CUP = {"cup_height": 0.12, "cup_rim_diameter": 0.095, "cup_base_diameter": 0.06,
       "cup_wall": 0.003, "cup_mass": 0.02}
CUP_SEGMENTS = 20       # collision boxes around a cup's wall
CUP_MESH_SEGMENTS = 32  # sides of the mesh the cameras see
# Geom group for collision-only geoms: the cameras and the viewer don't draw it by default
# (the viewer can show it). Drawing each cup's 21 collision geoms made the head camera's
# frames about 5x slower to render, and the sim's physics stalled waiting for them.
COLLISION_GROUP = 3
CUP_RGBA = (0.80, 0.05, 0.05, 1.0)
TAPE_RGBA = (0.10, 0.35, 0.90, 1.0)
TABLE_RGBA = (0.55, 0.38, 0.22, 1.0)


# ------------------------------------------------------------------- pure geometry
def quat_mul(a, b):
    """Hamilton product of (w, x, y, z) quaternions."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def cup_wall_segments(height, rim_d, base_d, wall, n=CUP_SEGMENTS):
    """The flat boxes that make a cup's tapered wall, in the cup's frame (origin at the
    center of the base, z up, rim at z = height). rim_d and base_d are outer diameters.
    Returns a list of (pos, quat (w, x, y, z), half_size); a box's local z runs up the
    wall, x points out of the cup, y along the wall."""
    r_b, r_r = base_d / 2.0, rim_d / 2.0
    lean = math.atan2(r_r - r_b, height)  # outward lean of the wall
    slant = math.hypot(height, r_r - r_b)
    r_mid = (r_b + r_r) / 2.0 - (wall / 2.0) / math.cos(lean)  # wall midline at mid-height
    half_w = 1.1 * r_mid * math.tan(math.pi / n)  # 10% overlap closes the gaps at mid-height
    segs = []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        q_z = (math.cos(th / 2), 0.0, 0.0, math.sin(th / 2))
        q_y = (math.cos(lean / 2), 0.0, math.sin(lean / 2), 0.0)
        segs.append(((r_mid * math.cos(th), r_mid * math.sin(th), height / 2.0),
                     quat_mul(q_z, q_y), (wall / 2.0, half_w, slant / 2.0)))
    return segs


def cup_mesh(height, rim_d, base_d, wall, n=CUP_MESH_SEGMENTS):
    """Triangle mesh of an open tapered cup whose wall and bottom are `wall` thick, in the
    cup's frame (origin at the center of the base, z up). It is a closed surface with
    outward-facing triangles. Returns (vertices, triangles)."""
    r_b, r_r = base_d / 2.0, rim_d / 2.0
    r_bottom_in = r_b + (r_r - r_b) * wall / height - wall  # inside radius at the inner bottom
    # rings: outer bottom, outer rim, inner rim, inner bottom
    rings = [(r_b, 0.0), (r_r, height), (r_r - wall, height), (r_bottom_in, wall)]
    verts = [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z)
             for r, z in rings for k in range(n)]
    c_out, c_in = len(verts), len(verts) + 1  # centers of the outer and inner bottom
    verts += [(0.0, 0.0, 0.0), (0.0, 0.0, wall)]

    def v(ring, k):
        return ring * n + k % n

    ob, ot, it, ib = range(4)
    tris = []
    for k in range(n):
        tris += [(v(ob, k), v(ob, k + 1), v(ot, k + 1)), (v(ob, k), v(ot, k + 1), v(ot, k)),
                 (v(ot, k), v(ot, k + 1), v(it, k + 1)), (v(ot, k), v(it, k + 1), v(it, k)),
                 (v(ib, k), v(it, k), v(it, k + 1)), (v(ib, k), v(it, k + 1), v(ib, k + 1)),
                 (c_in, v(ib, k), v(ib, k + 1)),
                 (c_out, v(ob, k + 1), v(ob, k))]
    return verts, tris


def pyramid_spots(center_x, center_y, table_top, height, rim_d, gap=0.01):
    """Cup body poses for a finished flipped 3-2-1 pyramid along x: (x, y, z) of each cup's
    origin (center of its base, which is on top when flipped). Base row first."""
    s = rim_d + gap
    spots = [(center_x + k * s, center_y, table_top + height) for k in (-1, 0, 1)]
    spots += [(center_x + k * s / 2, center_y, table_top + 2 * height) for k in (-1, 1)]
    spots.append((center_x, center_y, table_top + 3 * height))
    return spots


def random_loose_cups(seed, n=6, min_gap=0.18):
    """n cup positions on the table outside the taped area, at least min_gap apart."""
    rng = random.Random(seed)
    t, margin = TABLE, 0.08
    out = []
    while len(out) < n:
        x = rng.uniform(t["x0"] + margin, TAPE["x0"] - margin)
        y = rng.uniform(t["near_y"] - margin, t["near_y"] - t["depth"] + margin)
        if all(math.hypot(x - a, y - b) >= min_gap for a, b in out):
            out.append((x, y))
    return out


# ------------------------------------------------------------------------ MuJoCo
def _box(parent, name, pos, half, rgba, collide=True, quat=None, group=0):
    import mujoco
    g = parent.add_geom()
    g.name = name
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.pos = list(pos)
    g.size = list(half)
    g.rgba = list(rgba)
    g.group = group
    if quat is not None:
        g.quat = list(quat)
    if not collide:
        g.contype = 0
        g.conaffinity = 0
    return g


def _add_cup(world, name, pos, flipped, cup, mesh):
    """One cup: collision boxes and a base disk (not drawn), plus `mesh` for the cameras."""
    import mujoco
    h, rim, base, wall = (cup["cup_height"], cup["cup_rim_diameter"],
                          cup["cup_base_diameter"], cup["cup_wall"])
    body = world.add_body()
    body.name = name
    body.pos = list(pos)
    if flipped:
        body.quat = [0.0, 1.0, 0.0, 0.0]  # 180 deg about x: rim down, base on top
    body.add_freejoint()
    segs = cup_wall_segments(h, rim, base, wall)
    for i, (p, q, half) in enumerate(segs):
        _box(body, f"{name}_wall{i}", p, half, CUP_RGBA, quat=q, group=COLLISION_GROUP)
    disk = body.add_geom()
    disk.name = f"{name}_base"
    disk.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    r_disk = base / 2.0 - wall / 2.0
    disk.size = [r_disk, wall / 2.0, 0.0]
    disk.pos = [0.0, 0.0, wall / 2.0]
    disk.rgba = list(CUP_RGBA)
    disk.group = COLLISION_GROUP
    # spread the cup's mass by volume over its collision geoms
    vol_wall = sum(8 * a * b * c for _, _, (a, b, c) in segs)
    vol_disk = math.pi * r_disk ** 2 * wall
    density = cup["cup_mass"] / (vol_wall + vol_disk)
    for g in body.geoms:
        g.density = density
    look = body.add_geom()  # drawn only: no contacts, no mass
    look.name = f"{name}_visual"
    look.type = mujoco.mjtGeom.mjGEOM_MESH
    look.meshname = mesh
    look.rgba = list(CUP_RGBA)
    look.contype = 0
    look.conaffinity = 0
    look.density = 0.0
    return body


def build_spec(cups="upright", seed=None, cup=None):
    """MjSpec of the Stretch robot plus the task scene. cups: "upright" (loose cups on
    the table), "pyramid" (a finished flipped 3-2-1 pyramid inside the tape) or
    "single" (one loose cup, for grasp tests), "touching" (two loose cups rim to rim) or
    "mixed" (three loose cups and the pyramid's base row placed in the tape)."""
    import mujoco
    from stretch_mujoco.utils import default_robot_xml_path

    cup = dict(CUP, **(cup or {}))
    spec = mujoco.MjSpec.from_file(default_robot_xml_path)
    spec.modelname = "stack_cups_task"
    world = spec.worldbody

    floor = world.add_geom()
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0, 0, 0.05]
    floor.rgba = [0.35, 0.35, 0.37, 1]
    for pos, d, shadow in (([0, 0, 3.0], [0, 0, -1], True),
                           ([1.0, -1.0, 2.5], [-0.3, 0.3, -1], False)):  # fill light
        light = world.add_light()
        light.pos = pos
        light.dir = d
        light.directional = True
        light.castshadow = shadow  # each shadow costs a render pass per camera frame

    # Room walls and furniture (static)
    r = ROOM
    walls = world.add_body()
    walls.name = "room"
    t, hz = r["thickness"] / 2, r["height"] / 2
    cx, cy = (r["x0"] + r["x1"]) / 2, (r["y0"] + r["y1"]) / 2
    hx, hy = (r["x1"] - r["x0"]) / 2, (r["y1"] - r["y0"]) / 2
    gray = (0.75, 0.75, 0.72, 1)
    _box(walls, "wall_x0", (r["x0"] - t, cy, hz), (t, hy + 2 * t, hz), gray)
    _box(walls, "wall_x1", (r["x1"] + t, cy, hz), (t, hy + 2 * t, hz), gray)
    _box(walls, "wall_y0", (cx, r["y0"] - t, hz), (hx, t, hz), gray)
    _box(walls, "wall_y1", (cx, r["y1"] + t, hz), (hx, t, hz), gray)
    for i, (x, y, half) in enumerate(FURNITURE):
        _box(walls, f"furniture{i}", (x, y, half[2]), half, (0.45, 0.45, 0.5, 1))

    # Table: top + 4 legs (static)
    tb = TABLE
    table = world.add_body()
    table.name = "table"
    tx, ty = (tb["x0"] + tb["x1"]) / 2, tb["near_y"] - tb["depth"] / 2
    table.pos = [tx, ty, 0]
    thx, thy = (tb["x1"] - tb["x0"]) / 2, tb["depth"] / 2
    top_z = tb["height"] - tb["top_thickness"] / 2
    _box(table, "table_top", (0, 0, top_z), (thx, thy, tb["top_thickness"] / 2), TABLE_RGBA)
    leg_h = (tb["height"] - tb["top_thickness"]) / 2
    inset_x, inset_y = thx - tb["leg"] / 2, thy - tb["leg"] / 2
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        _box(table, f"table_leg{i}", (sx * inset_x, sy * inset_y, leg_h),
             (tb["leg"] / 2, tb["leg"] / 2, leg_h), TABLE_RGBA)

    # Blue tape: 4 thin visual-only strips just above the table top
    tp, w = TAPE, TAPE["width"] / 2
    z = tb["height"] + 0.0006
    lx, ly = (tp["x1"] - tp["x0"]) / 2, abs(tp["y1"] - tp["y0"]) / 2
    mx, my = (tp["x0"] + tp["x1"]) / 2, (tp["y0"] + tp["y1"]) / 2
    tape = world.add_body()
    tape.name = "tape"
    for name, pos, half in (("tape_near", (mx, tp["y0"], z), (lx + w, w, 0.0005)),
                            ("tape_far", (mx, tp["y1"], z), (lx + w, w, 0.0005)),
                            ("tape_x0", (tp["x0"], my, z), (w, ly + w, 0.0005)),
                            ("tape_x1", (tp["x1"], my, z), (w, ly + w, 0.0005))):
        _box(tape, name, pos, half, TAPE_RGBA, collide=False)

    # Cups
    verts, tris = cup_mesh(cup["cup_height"], cup["cup_rim_diameter"],
                           cup["cup_base_diameter"], cup["cup_wall"])
    mesh = spec.add_mesh()
    mesh.name = "cup_visual"
    mesh.uservert = [c for v in verts for c in v]
    mesh.userface = [i for t in tris for i in t]
    top = tb["height"]
    if cups in ("pyramid", "mixed"):
        spots = pyramid_spots(mx, my, top, cup["cup_height"], cup["cup_rim_diameter"])
        if cups == "mixed":
            spots = spots[:3]
            for i, (x, y) in enumerate(LOOSE_CUPS[:3]):
                _add_cup(world, f"cup{3 + i}", (x, y, top + 0.002), False, cup, mesh.name)
        for i, p in enumerate(spots):
            _add_cup(world, f"cup{i}", (p[0], p[1], p[2] + 0.002 * (1 + i // 3)), True, cup,
                     mesh.name)
    else:
        if cups == "single":
            xy = [SINGLE_CUP]
        elif cups == "touching":
            x, y = SINGLE_CUP
            xy = [(x, y), (x + cup["cup_rim_diameter"] + 0.001, y)]
        else:
            xy = LOOSE_CUPS if seed is None else random_loose_cups(seed)
        for i, (x, y) in enumerate(xy):
            _add_cup(world, f"cup{i}", (x, y, top + 0.002), False, cup, mesh.name)
    return spec


def write_scene(path, cups="upright", seed=None, cup=None):
    """Write the scene as one MJCF file that loads from anywhere (absolute asset paths)."""
    import os

    from stretch_mujoco.utils import default_robot_xml_path

    spec = build_spec(cups, seed, cup)
    assets = os.path.join(os.path.dirname(default_robot_xml_path), "assets") + os.sep
    spec.meshdir = assets
    spec.texturedir = assets
    spec.compile()
    with open(os.path.expanduser(path), "w") as f:
        f.write(spec.to_xml())
    return os.path.expanduser(path)


def check(model, seconds=3.0):
    """Simulate the scene with the robot held in its home pose. Returns one row per cup:
    (name, start xyz, end xyz, tilt in degrees from its start orientation)."""
    import mujoco

    d = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key >= 0:
        d.ctrl[:] = model.key_ctrl[key]
    mujoco.mj_forward(model, d)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(model.nbody)]
    cups = [b for b, nm in enumerate(names) if nm and nm.startswith("cup")]
    start = {b: (d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()) for b in cups}
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, d)
    rows = []
    for b in cups:
        p0, R0 = start[b]
        z0, z1 = R0[:, 2], d.xmat[b].reshape(3, 3)[:, 2]
        tilt = math.degrees(math.acos(float(np.clip(z0 @ z1, -1.0, 1.0))))
        rows.append((names[b], p0, d.xpos[b].copy(), tilt))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", help="write the scene MJCF here")
    ap.add_argument("--cups", default="upright",
                    choices=["upright", "pyramid", "single", "touching", "mixed"])
    ap.add_argument("--seed", type=int, help="random loose-cup layout (default: fixed layout)")
    ap.add_argument("--check", action="store_true", help="simulate 3 s and report each cup")
    args = ap.parse_args()
    spec = build_spec(args.cups, args.seed)
    model = spec.compile()
    print(f"scene: {model.nbody} bodies, {model.ngeom} geoms, {model.nu} actuators, "
          f"cups={args.cups}")
    if args.out:
        print("wrote", write_scene(args.out, args.cups, args.seed))
    if args.check:
        ok = True
        for name, p0, p1, tilt in check(model):
            moved = float(np.linalg.norm(p1[:2] - p0[:2]))
            good = tilt < 10.0 and moved < 0.01
            ok &= good
            print(f"{'OK  ' if good else 'FAIL'} {name}: moved {moved * 1000:.1f} mm, "
                  f"tilt {tilt:.1f} deg, z {p0[2]:.3f} -> {p1[2]:.3f}")
        print("RESULT", "all cups stable" if ok else "some cups fell or slid")


if __name__ == "__main__":
    main()
