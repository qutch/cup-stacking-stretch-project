"""The full-task sim scene (roadmap 0.4): cup geometry, layouts, and (where MuJoCo and
stretch_mujoco are installed) that the scene compiles and its cups stay put."""

import math

import numpy as np
import pytest

from stack_cups import sim_scene as S


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_cup_wall_spans_base_to_rim_with_the_right_diameters():
    h, rim, base, wall = 0.12, 0.095, 0.06, 0.003
    for pos, quat, half in S.cup_wall_segments(h, rim, base, wall, n=16):
        R = rot(quat)
        up = R[:, 2] * half[2]  # along the wall
        out = R[:, 0] * half[0]  # outward through the wall
        bottom, top = np.array(pos) - up + out, np.array(pos) + up + out  # outer surface
        assert bottom[2] == pytest.approx(0.0, abs=1e-3) and top[2] == pytest.approx(h, abs=1e-3)
        assert math.hypot(*bottom[:2]) == pytest.approx(base / 2, abs=1e-3)
        assert math.hypot(*top[:2]) == pytest.approx(rim / 2, abs=1e-3)


def test_cup_mesh_is_a_closed_outward_shell_of_the_cup_size():
    h, rim, base, wall, n = 0.12, 0.095, 0.06, 0.003, 24
    verts, tris = S.cup_mesh(h, rim, base, wall, n=n)
    v = np.array(verts)
    edges = [(t[i], t[(i + 1) % 3]) for t in tris for i in range(3)]
    assert len(set(edges)) == len(edges)  # no edge used twice the same way
    assert {(b, a) for a, b in edges} == set(edges)  # closed: every edge has a partner
    # outward triangles: positive volume, equal to the n-sided outer minus inner frustum
    vol = sum(np.dot(v[a], np.cross(v[b], v[c])) for a, b, c in tris) / 6.0
    k = n / 2.0 * math.sin(2 * math.pi / n)  # n-gon area / r^2

    def frustum(r1, r2, height):
        return k * height / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2)

    r_in = base / 2 + (rim - base) / 2 * wall / h - wall
    assert vol == pytest.approx(frustum(base / 2, rim / 2, h)
                                - frustum(r_in, rim / 2 - wall, h - wall), rel=1e-9)
    r = np.hypot(v[:, 0], v[:, 1])
    assert r[v[:, 2] == 0.0].max() == pytest.approx(base / 2)
    assert r.max() == pytest.approx(rim / 2) and v[:, 2].max() == pytest.approx(h)


def test_quat_mul_matches_rotation_composition():
    a, b = (math.cos(0.3), 0, 0, math.sin(0.3)), (math.cos(0.2), 0, math.sin(0.2), 0)
    assert np.allclose(rot(S.quat_mul(a, b)), rot(a) @ rot(b))


def test_pyramid_spots_form_a_3_2_1_on_the_cups_below():
    spots = S.pyramid_spots(1.0, -0.7, 0.74, 0.12, 0.095, gap=0.01)
    xs = [round(s[0], 4) for s in spots]
    assert len(spots) == 6
    assert xs[:3] == [round(1.0 - 0.105, 4), 1.0, round(1.0 + 0.105, 4)]
    assert xs[3:5] == [round(1.0 - 0.0525, 4), round(1.0 + 0.0525, 4)] and xs[5] == 1.0
    assert [s[2] for s in spots] == pytest.approx([0.86, 0.86, 0.86, 0.98, 0.98, 1.10])


def test_random_layout_is_spread_out_and_off_the_tape():
    for seed in range(5):
        cups = S.random_loose_cups(seed)
        assert len(cups) == 6
        for i, (x, y) in enumerate(cups):
            assert x < S.TAPE["x0"]
            assert all(math.hypot(x - a, y - b) >= 0.18 for a, b in cups[i + 1:])


def test_fixed_layout_cups_are_on_the_table_and_off_the_tape():
    t = S.TABLE
    for x, y in S.LOOSE_CUPS:
        assert t["x0"] < x < S.TAPE["x0"]
        assert t["near_y"] - t["depth"] < y < t["near_y"]


mujoco = pytest.importorskip("mujoco")
pytest.importorskip("stretch_mujoco")


@pytest.fixture(scope="module")
def upright():
    return S.build_spec("upright").compile()


def test_scene_compiles_with_robot_table_tape_and_six_cups(upright):
    names = {mujoco.mj_id2name(upright, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(upright.nbody)}
    assert {"table", "tape", "room"} <= names
    assert {f"cup{i}" for i in range(6)} <= names
    assert mujoco.mj_name2id(upright, mujoco.mjtObj.mjOBJ_CAMERA, "d435i_camera_rgb") >= 0


def test_cameras_draw_each_cup_as_one_mesh(upright):
    # Drawing the 21 collision geoms per cup made camera frames slow enough to stall the sim
    m = upright
    shown = np.ones(6, bool)
    shown[S.COLLISION_GROUP:] = False  # the default geom groups a camera draws: 0, 1, 2
    for i in range(6):
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cup{i}")
        geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body]
        drawn = [g for g in geoms if shown[m.geom_group[g]]]
        assert len(drawn) == 1 and m.geom_type[drawn[0]] == mujoco.mjtGeom.mjGEOM_MESH
        assert m.geom_contype[drawn[0]] == 0 and m.geom_conaffinity[drawn[0]] == 0
        assert len(geoms) == S.CUP_SEGMENTS + 2
    assert m.body_mass[body] == pytest.approx(S.CUP["cup_mass"])


def test_written_scene_loads_from_anywhere(tmp_path):
    path = S.write_scene(str(tmp_path / "scene.xml"))
    m = mujoco.MjModel.from_xml_path(path)
    assert m.nbody == S.build_spec().compile().nbody


def test_single_scene_has_one_loose_cup():
    m = S.build_spec("single").compile()
    names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)}
    assert "cup0" in names and "cup1" not in names


@pytest.mark.parametrize("cups,n", [("upright", 6), ("pyramid", 6), ("single", 1)])
def test_cups_stay_put(cups, n):
    rows = S.check(S.build_spec(cups).compile(), seconds=2.0)
    assert len(rows) == n
    for name, p0, p1, tilt in rows:
        assert tilt < 10.0, f"{name} tipped {tilt:.1f} deg"
        assert np.linalg.norm(p1[:2] - p0[:2]) < 0.01, f"{name} slid"
