#!/usr/bin/env python3
"""
tests/test_arjuna_terrain_objects.py
------------------------------------
Object sizes and ids from the OAK-D terrain detector (robot/arjuna_quantum_nav/scripts:
terrain_core.py measures, terrain_objects.py tracks, crater_detector_node.py publishes), on
synthetic depth images from tests/arena_world.py whose true sizes are known. No camera, no ROS.

What the camera can and cannot see decides what may be asserted. The test camera is 25 cm up,
tilted 30 deg down, and its sampled image ends ~10 deg below the horizon, so it sees the back of
a rock only when the whole rock (top included) projects inside the image. For those views the
size must be right (+/-3 cm footprint, +/-2 cm height); for views where the back or the top is
out of the image the detector must SAY so (footprint_complete False / height_is_min True)
instead of reporting a wrong size as if it were measured.
"""

import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(__file__)
SCRIPTS = os.path.abspath(os.path.join(HERE, '..', 'robot', 'arjuna_quantum_nav', 'scripts'))
for p in (SCRIPTS, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import arena_world as aw  # noqa: E402
import crater_core as cc  # noqa: E402
import terrain_core as tc  # noqa: E402
import terrain_objects as to  # noqa: E402

INTR = cc.Intrinsics(fx=450.0, fy=450.0, cx=320.0, cy=200.0, width=640, height=400)
CAM = aw.Camera(INTR, height=0.25, tilt_deg=30.0, forward=0.10)
PARAMS = tc.TerrainParams()
U, V = cc.pixel_grid(INTR, step=PARAMS.pixel_step, step_y=PARAMS.pixel_step_y,
                     top_fraction=PARAMS.top_fraction)
US, VS = np.unique(U), np.unique(V)
ROBOT = (1.0, 1.0, 0.0)

FOOTPRINT_TOL = 0.03      # m, length / width
HEIGHT_TOL = 0.02         # m


@pytest.fixture(scope='module')
def floor():
    img = aw.render(aw.Arena(8.0), CAM, (3, 3, 0), U, V, seed=99)
    return cc.fit_floor(cc.depth_to_points(img, INTR, U, V))


def measure(floor, arena, pose=ROBOT, frames=3, **render_kw):
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    dets, info = [], {}
    for i in range(frames):
        dets, info = det.process(aw.render(arena, CAM, pose, U, V, seed=i, **render_kw), pose)
    return dets, info["objects"], det


def of_class(objs, cls):
    return [o for o in objs if o['class'] == cls]


# ── ground truth: what the camera can see ────────────────────────────────────
def pixel(pose, pt):
    """(u, v) of world point pt = (x, y, z) in the arena_world camera."""
    o, _ = aw.camera_rays(CAM, pose, np.array([INTR.cx]), np.array([INTR.cy]))
    x, y, yaw = pose
    h = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    L = math.cos(CAM.tilt) * h + np.array([0.0, 0.0, -math.sin(CAM.tilt)])
    D = -math.sin(CAM.tilt) * h + np.array([0.0, 0.0, -math.cos(CAM.tilt)])
    R = np.cross(D, L)
    w = np.asarray(pt, dtype=np.float64) - o[0]
    z = w.dot(L)
    return INTR.cx + INTR.fx * w.dot(R) / z, INTR.cy + INTR.fy * w.dot(D) / z


def box_corners(rock):
    c, s = math.cos(rock.rot), math.sin(rock.rot)
    out = []
    for dx in (-rock.hw, rock.hw):
        for dy in (-rock.hd, rock.hd):
            for z in (0.0, rock.h):
                out.append((rock.x + c * dx - s * dy, rock.y + s * dx + c * dy, z))
    return out


def to_robot(pose, x, y):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = x - pose[0], y - pose[1]
    return dx * c + dy * s, -dx * s + dy * c


def rock_view(rock, pose, margin=6.0):
    """'full' when every corner of the box is inside the sampled image (by `margin` px), the
    top is below the camera and the footprint is inside the watched area; 'cut' when some
    corner is clearly outside; None in between (no assertion about the flag there)."""
    worst = float('inf')            # smallest distance (px) of a corner inside the image border
    for pt in box_corners(rock):
        u, v = pixel(pose, pt)
        worst = min(worst, u - US[0], US[-1] - u, v - VS[0], VS[-1] - v)
        fx, fy = to_robot(pose, pt[0], pt[1])
        inside = min(fx - PARAMS.x_min, PARAMS.x_max - fx, PARAMS.half_width - abs(fy))
        worst = min(worst, inside / 0.002)       # 1 px ~ 2 mm here: same scale for the margin
    low = rock.h + 0.02 < CAM.height
    if worst >= margin and low:
        return 'full'
    if worst < -2.0 or rock.h >= CAM.height:
        return 'cut'
    return None


def octagon_mean_diameter(apothem):
    """Mean diameter of a regular octagon with this apothem (average over all bearings)."""
    phi = np.linspace(-math.pi / 8, math.pi / 8, 4001)
    return 2.0 * apothem * float(np.mean(1.0 / np.cos(phi)))


# ── rocks: size of known boxes at 0.4-1.0 m ──────────────────────────────────
ROCKS = [(0.31, 0.27, 0.12, 0), (0.20, 0.15, 0.10, 30), (0.31, 0.27, 0.18, 15),
         (0.25, 0.25, 0.15, 45), (0.15, 0.12, 0.08, 10), (0.35, 0.30, 0.24, 15)]
AHEAD = (0.40, 0.55, 0.70, 0.85, 1.00)
CASES = [(r, a) for r in ROCKS for a in AHEAD
         # a 35 x 30 x 24 cm rock 0.4 m ahead fills the bottom of the image and is classified as
         # the table edge (no floor in front of it): detector behaviour that predates sizes
         if not (r[2] >= 0.24 and a <= 0.40)]


@pytest.fixture(scope='module')
def rock_results(floor):
    out = {}
    for (w, d, h, rot), ahead in CASES:
        rock = aw.Rock(ROBOT[0] + ahead, ROBOT[1] + 0.02, w, d, h, rot_deg=rot)
        _, objs, _ = measure(floor, aw.Arena(8.0, rocks=[rock]))
        out[((w, d, h, rot), ahead)] = (rock, objs)
    return out


@pytest.mark.parametrize('case', CASES, ids=lambda c: '%gx%gx%g_rot%d@%.2f' % (c[0] + (c[1],)))
def test_rock_size(rock_results, case):
    (w, d, h, rot), ahead = case
    rock, objs = rock_results[case]
    rocks = of_class(objs, 'rock')
    assert len(rocks) == 1 and len(objs) == 1, objs
    o = rocks[0]
    true_l, true_w = max(w, d), min(w, d)
    view = rock_view(rock, ROBOT)
    if view == 'full':
        assert o['footprint_complete'], 'whole rock in view but reported as cut'
    elif view == 'cut':
        assert not o['footprint_complete'], 'rock is cut by the image but reported as complete'
    if o['footprint_complete']:
        assert o['length'] == pytest.approx(true_l, abs=FOOTPRINT_TOL)
        assert o['width'] == pytest.approx(true_w, abs=FOOTPRINT_TOL)
        # the rectangle is centred on the rock
        assert math.hypot(o['x'] - ahead, o['y'] - 0.02) < 0.03
    if o['height_is_min']:
        assert o['height'] <= h + HEIGHT_TOL           # a lower bound is never too high
    else:
        assert o['height'] == pytest.approx(h, abs=HEIGHT_TOL)


def test_enough_rock_views_are_complete_for_the_size_test_to_mean_something(rock_results):
    full = [k for k, (rock, objs) in rock_results.items()
            if objs and objs[0]['footprint_complete']]
    exact_h = [k for k, (rock, objs) in rock_results.items()
               if objs and not objs[0]['height_is_min']]
    assert len(full) >= 6, full
    assert len(exact_h) >= 15
    assert {a for _, a in full} >= {0.40, 0.55, 0.70, 0.85}  # several distances, not one


def test_tall_rock_reports_its_width_and_a_minimum_height(floor):
    """The user's example rock (31 x 27 x 24 cm) is about as tall as the camera: its top and
    back are never seen. The face it shows is measured; the rest is flagged."""
    rock = aw.Rock(1.6, 1.0, 0.27, 0.31, 0.24)          # 31 cm across the view, 27 deep
    _, objs, _ = measure(floor, aw.Arena(8.0, rocks=[rock]))
    o = of_class(objs, 'rock')[0]
    assert not o['footprint_complete']
    assert o['length'] == pytest.approx(0.31, abs=FOOTPRINT_TOL)   # the face across the view
    assert o['height_is_min'] and o['height'] <= 0.24 + HEIGHT_TOL


# ── craters ──────────────────────────────────────────────────────────────────
CRATERS = [(dia, ahead) for dia in (0.25, 0.35, 0.45) for ahead in (0.45, 0.60, 0.75, 0.85)]


@pytest.fixture(scope='module')
def crater_results(floor):
    out = {}
    for dia, ahead in CRATERS:
        cr = aw.Crater(ROBOT[0] + ahead, ROBOT[1], dia)
        _, objs, _ = measure(floor, aw.Arena(8.0, craters=[cr]))
        out[(dia, ahead)] = (cr, objs)
    return out


@pytest.mark.parametrize('case', CRATERS, ids=lambda c: 'D%g@%.2f' % c)
def test_crater_size(crater_results, case):
    cr, objs = crater_results[case]
    craters = of_class(objs, 'crater')
    assert len(craters) == 1 and len(objs) == 1, objs
    o = craters[0]
    a = cr.apothem
    assert o['outer_diameter'] == pytest.approx(octagon_mean_diameter(a), abs=FOOTPRINT_TOL)
    assert o['inner_diameter'] == pytest.approx(octagon_mean_diameter(a - cr.rim_w),
                                                abs=FOOTPRINT_TOL)
    assert o['inner_diameter'] < o['outer_diameter']
    assert o['height'] == pytest.approx(cr.rim_h, abs=0.005)       # rim height above the floor
    if o['footprint_complete']:          # whole rim seen: footprint across the octagon's flats
        assert o['length'] == pytest.approx(2 * a, abs=FOOTPRINT_TOL)
        assert o['width'] == pytest.approx(2 * a, abs=FOOTPRINT_TOL)
    else:                                # partly seen: assumed round, outer diameter both ways
        assert o['length'] == pytest.approx(o['outer_diameter'], abs=1e-9)


def test_crater_footprint_is_measured_when_the_whole_rim_is_seen(crater_results):
    full = [k for k, (cr, objs) in crater_results.items() if objs and objs[0]['footprint_complete']]
    assert len(full) >= 4, full


@pytest.mark.parametrize('rim_h', [0.015, 0.03])
def test_crater_rim_height_follows_the_real_rim(floor, rim_h):
    cr = aw.Crater(1.6, 1.0, 0.35, rim_h=rim_h)
    _, objs, _ = measure(floor, aw.Arena(8.0, craters=[cr]))
    assert of_class(objs, 'crater')[0]['height'] == pytest.approx(rim_h, abs=0.005)


def test_crater_seen_as_a_short_arc_is_reported_as_an_estimate(floor):
    """Only the near edge of a big crater is inside the rim search range: the fitted radius is
    a guess, and the report says so."""
    cr = aw.Crater(2.0, 1.0, 0.45)
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    tr = to.ObjectTracker()
    rep = []
    for i in range(3):
        _, info = det.process(aw.render(aw.Arena(8.0, craters=[cr]), CAM, ROBOT, U, V, seed=i),
                              ROBOT)
        rep = tr.update(info['objects'], ROBOT, sees=det.sees, stamp=0.1 * i)
    craters = of_class(rep, 'crater')
    assert craters
    for o in craters:
        assert o['size_is_estimate'] or o['footprint_complete'] is False
        assert '~' in o['label'] and 'part of rim' in o['text']


# ── several objects: each its own ────────────────────────────────────────────
@pytest.mark.parametrize('gap', [0.05, 0.08, 0.15])
def test_two_rocks_side_by_side_are_two_objects(floor, gap):
    w = 0.15
    rocks = [aw.Rock(1.65, 1.0 + (gap + w) / 2, 0.15, w, 0.10),
             aw.Rock(1.65, 1.0 - (gap + w) / 2, 0.15, w, 0.10)]
    _, objs, _ = measure(floor, aw.Arena(8.0, rocks=rocks))
    found = of_class(objs, 'rock')
    assert len(found) == 2
    for o in found:
        assert o['length'] == pytest.approx(0.15, abs=FOOTPRINT_TOL)
        assert o['width'] == pytest.approx(0.15, abs=FOOTPRINT_TOL)
    ys = sorted(o['y'] for o in found)
    assert ys[1] - ys[0] == pytest.approx(gap + w, abs=0.03)


@pytest.mark.parametrize('gap', [0.10, 0.15])
def test_two_rocks_one_behind_the_other_are_two_objects(floor, gap):
    rocks = [aw.Rock(1.55, 1.0, 0.12, 0.15, 0.08), aw.Rock(1.55 + 0.12 + gap, 1.0, 0.12, 0.15, 0.08)]
    _, objs, _ = measure(floor, aw.Arena(8.0, rocks=rocks))
    assert len(of_class(objs, 'rock')) == 2


def test_rock_and_crater_together(floor):
    arena = aw.Arena(8.0, rocks=[aw.Rock(1.65, 1.3, 0.15, 0.12, 0.08)],
                     craters=[aw.Crater(1.6, 0.85, 0.30)])
    _, objs, _ = measure(floor, arena)
    assert len(of_class(objs, 'rock')) == 1 and len(of_class(objs, 'crater')) == 1


@pytest.mark.parametrize('seed', range(3))
def test_shiny_rock_with_dropouts_stays_one_object(floor, seed):
    rock = aw.Rock(1.6, 1.0, 0.35, 0.30, 0.25, rot_deg=7 * seed)
    _, objs, _ = measure(floor, aw.Arena(8.0, rocks=[rock]), shiny_dropout=0.6, shiny_spike=0.10)
    assert len(of_class(objs, 'rock')) == 1


def test_wall_and_table_edge_are_objects_too(floor):
    _, objs, _ = measure(floor, aw.Arena(8.0, wall_h=0.08), pose=(7.4, 4.0, 0.0))
    walls = of_class(objs, 'wall')
    assert walls and max(o['length'] for o in walls) > 0.5
    _, objs, _ = measure(floor, aw.Arena(2.21, wall_h=0.0), pose=(1.7, 1.0, 0.0))
    assert of_class(objs, 'table_edge')


def test_planner_detections_are_unchanged_by_measuring(floor):
    """measure_objects on or off: /terrain/detections (what the navigator plans with) is the same."""
    arena = aw.Arena(8.0, rocks=[aw.Rock(1.6, 1.25, 0.2, 0.15, 0.10)],
                     craters=[aw.Crater(1.6, 0.8, 0.30)])
    out = []
    for on in (True, False):
        det = tc.TerrainDetector(floor, INTR, tc.TerrainParams(measure_objects=on))
        for i in range(3):
            dets, info = det.process(aw.render(arena, CAM, ROBOT, U, V, seed=i), ROBOT)
        out.append((tc.detections_to_json_obj(dets), len(info['objects'])))
    assert out[0][0] == out[1][0]
    assert out[0][1] == 2 and out[1][1] == 0


# ── ids across frames ────────────────────────────────────────────────────────
DRIVE_ARENA = aw.Arena(8.0, rocks=[aw.Rock(2.0, 1.15, 0.20, 0.15, 0.10),
                                   aw.Rock(2.1, 0.75, 0.15, 0.12, 0.08)],
                       craters=[aw.Crater(1.9, 0.95, 0.30)])


def drive(floor, poses, arena=DRIVE_ARENA, tracker=None):
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    tr = tracker or to.ObjectTracker()
    reports = []
    for i, pose in enumerate(poses):
        _, info = det.process(aw.render(arena, CAM, pose, U, V, seed=i), pose)
        reports.append(tr.update(info['objects'], pose, sees=det.sees, stamp=0.125 * i))
    return reports, tr


@pytest.fixture(scope='module')
def approach(floor):
    poses = [(0.8 + 0.025 * i, 1.0 + 0.003 * i, math.radians(8 * math.sin(i / 4.0)))
             for i in range(30)]
    return drive(floor, poses)


def test_ids_stay_the_same_while_the_rover_drives_and_wiggles(approach):
    reports, tr = approach
    assert tr.next_id - 1 == 3                       # three objects, never a duplicate
    where = {}
    for rep in reports:
        for o in rep:
            where.setdefault(o['id'], []).append((o['class'], o['x'], o['y']))
    assert sorted(c for c in {v[0][0] for v in where.values()}) == ['crater', 'rock']
    for oid, seen in where.items():
        assert len({c for c, _, _ in seen}) == 1     # an id never changes class
        # ... nor jumps to another object: the centre only moves as more of the object is
        # seen (the centre of a partly seen rock is the centre of its visible part)
        fx, fy = seen[-1][1], seen[-1][2]
        assert max(math.hypot(x - fx, y - fy) for _, x, y in seen) < 0.12
    # the objects keep distinct places: no two ids on the same object
    final = [(o['x'], o['y']) for o in reports[-1]]
    assert min(math.hypot(a[0] - b[0], a[1] - b[1])
               for i, a in enumerate(final) for b in final[i + 1:]) > 0.15


def test_sizes_settle_on_the_truth_as_the_rover_gets_close(approach):
    reports, _ = approach
    final = {o['class'] + str(round(o['y'], 1)): o for o in reports[-1]}
    big, small = final['rock1.1'], final['rock0.8']
    assert big['footprint_complete'] and small['footprint_complete']
    assert (big['length_m'], big['width_m']) == (pytest.approx(0.20, abs=FOOTPRINT_TOL),
                                                 pytest.approx(0.15, abs=FOOTPRINT_TOL))
    assert big['height_m'] == pytest.approx(0.10, abs=HEIGHT_TOL) and not big['height_is_min']
    assert (small['length_m'], small['width_m']) == (pytest.approx(0.15, abs=FOOTPRINT_TOL),
                                                     pytest.approx(0.12, abs=FOOTPRINT_TOL))
    assert small['height_m'] == pytest.approx(0.08, abs=HEIGHT_TOL)
    crater = [o for rep in reports for o in rep if o['class'] == 'crater'][-1]
    cr = DRIVE_ARENA.craters[0]
    assert crater['outer_diameter_m'] == pytest.approx(octagon_mean_diameter(cr.apothem),
                                                       abs=FOOTPRINT_TOL)
    assert crater['height_m'] == pytest.approx(0.02, abs=0.005)


def test_ids_survive_turning_away_and_back(floor):
    yaw = [0, 10, 20, 35, 50, 65, 50, 35, 20, 10, 0, -10, -20, 0]
    poses = [(1.25, 1.0, math.radians(a)) for a in yaw]
    reports, tr = drive(floor, poses)
    first, last = reports[2], reports[-1]
    assert {o['id'] for o in first if o['visible']} <= {o['id'] for o in last}
    assert tr.next_id - 1 == 3


def test_object_that_is_no_longer_there_is_forgotten_but_not_one_out_of_view(floor):
    here = (4.0, 4.0, 0.0)                   # middle of the table: no border wall in view
    rock = aw.Rock(4.7, 4.0, 0.15, 0.12, 0.08)
    with_rock, empty = aw.Arena(8.0, rocks=[rock]), aw.Arena(8.0)
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    tr = to.ObjectTracker(miss_limit=6)
    t = 0.0
    for i in range(4):
        _, info = det.process(aw.render(with_rock, CAM, here, U, V, seed=i), here)
        tr.update(info['objects'], here, sees=det.sees, stamp=t)
        t += 0.125
    assert len(tr.report(here)) == 1
    # turned away: the rock is out of view, so not seeing it says nothing; it is kept
    away = (4.0, 4.0, math.radians(120))
    for i in range(10):
        _, info = det.process(aw.render(with_rock, CAM, away, U, V, seed=10 + i), away)
        rep = tr.update(info['objects'], away, sees=det.sees, stamp=t)
        t += 0.125
    assert len(rep) == 1 and not rep[0]['visible']
    # looking at the spot again and the rock is gone: forgotten after miss_limit frames
    det.reset()
    for i in range(8):
        _, info = det.process(aw.render(empty, CAM, here, U, V, seed=30 + i), here)
        rep = tr.update(info['objects'], here, sees=det.sees, stamp=t)
        t += 0.125
    assert rep == []


def test_single_frame_noise_is_not_reported():
    tr = to.ObjectTracker(min_frames=2)
    m = {"class": "rock", "type": "rock", "x": 0.5, "y": 0.0, "yaw": 0.0, "length": 0.1,
         "width": 0.1, "height": 0.08, "height_is_min": False, "footprint_complete": True,
         "quality": 1.0, "outline": [[0.55, 0.05], [0.45, 0.05], [0.45, -0.05], [0.55, -0.05]]}
    assert tr.update([m], ROBOT, stamp=0.0) == []
    assert len(tr.update([m], ROBOT, stamp=0.1)) == 1


# ── what the user reads ──────────────────────────────────────────────────────
def obj(**kw):
    o = {"id": 2, "class": "rock", "length_m": 0.31, "width_m": 0.27, "height_m": 0.24,
         "height_is_min": False, "footprint_complete": True, "size_is_estimate": False,
         "inner_diameter_m": None, "outer_diameter_m": None, "distance_m": 0.82,
         "bearing_deg": 4.0, "source": "depth", "visible": True}
    o.update(kw)
    return o


def test_status_line_format():
    assert to.describe(obj()) == "rock #2: 31 x 27 cm, 24 cm tall, 0.82 m ahead"
    assert to.describe(obj(footprint_complete=False, height_is_min=True, height_m=0.16,
                           bearing_deg=45)) == \
        "rock #2: 31 x 27 cm (back not seen), >=16 cm tall, 0.82 m ahead-left"
    assert to.describe(obj(id=1, **{"class": "crater"}, outer_diameter_m=0.34,
                           inner_diameter_m=0.29, height_m=0.02, bearing_deg=-30)) == \
        "crater #1: 34 cm across (inner 29 cm), rim 2.0 cm high, 0.82 m ahead-right"
    assert to.describe(obj(id=5, **{"class": "table_edge"}, length_m=0.6, height_m=None,
                           bearing_deg=95)) == "table edge #5: 60 cm wide, 0.82 m to the left"
    assert to.short_label(obj()) == "rock #2 31x27x24 cm"


def test_objects_json_and_status_from_a_real_scene(approach):
    reports, _ = approach
    msg = to.objects_message(reports[-1], "map", 12.5, (1.5, 1.09, 0.1))
    text = json.dumps(msg)                     # plain JSON types only (no numpy bool / float)
    back = json.loads(text)
    assert back["frame_id"] == "map" and len(back["objects"]) == len(reports[-1])
    keys = {"id", "class", "x", "y", "length_m", "width_m", "height_m", "inner_diameter_m",
            "outer_diameter_m", "distance_m", "frames_seen", "confidence", "outline", "text"}
    for o in back["objects"]:
        assert keys <= set(o)
        assert 0.0 <= o["confidence"] <= 1.0
        o["text"].encode("ascii")              # the Jetson terminal may only print ASCII
    lines = to.status_lines(reports[-1])
    assert lines and all(' m ' in line for line in lines)


def test_distance_is_to_the_objects_edge():
    sq = [[1.0, -0.1], [1.2, -0.1], [1.2, 0.1], [1.0, 0.1]]
    assert to.distance_to_polygon(0.0, 0.0, sq) == pytest.approx(1.0)
    assert to.distance_to_polygon(1.1, 0.0, sq) == 0.0


# ── geometry helpers ─────────────────────────────────────────────────────────
@pytest.mark.parametrize('yaw_deg', [0, 17, 45, 80, -30])
def test_min_area_rect_recovers_a_rotated_rectangle(yaw_deg):
    rs = np.random.RandomState(1)
    loc = np.stack([rs.uniform(-0.15, 0.15, 3000), rs.uniform(-0.05, 0.05, 3000)], axis=1)
    a = math.radians(yaw_deg)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    pts = loc.dot(rot.T) + [0.6, -0.2]
    r = tc.min_area_rect(pts)
    assert (r['length'], r['width']) == (pytest.approx(0.30, abs=0.01), pytest.approx(0.10, abs=0.01))
    assert (r['x'], r['y']) == (pytest.approx(0.6, abs=0.01), pytest.approx(-0.2, abs=0.01))
    d = (r['yaw'] - a + math.pi / 2) % math.pi - math.pi / 2
    assert abs(d) < math.radians(2)


def test_split_clusters_joins_across_missing_depth_but_not_across_floor():
    m = np.zeros((20, 20), dtype=bool)
    m[2:8, 2:8] = True               # rock A
    m[2:8, 9] = True                 # a sliver of A's side face ...
    hidden = np.zeros_like(m)
    hidden[2:8, 8] = True            # ... beyond a column of no-depth cells (not floor)
    m[12:18, 12:18] = True           # rock B, visible floor all round it
    m[0, 19] = True                  # a speck of noise
    parts = tc.split_clusters(m, 4, connect=hidden)
    assert sorted(int(p.sum()) for p in parts) == [36, 43]    # speck joins its nearest rock
    # floor visible between the sliver and A: the sliver is a small rock of its own
    parts = tc.split_clusters(m, 4, connect=np.zeros_like(m))
    assert sorted(int(p.sum()) for p in parts) == [6, 36, 37]
    # two big groups are two rocks even when hidden floor joins them (one behind the other)
    assert len(tc.split_clusters(m, 4, connect=np.ones_like(m))) == 2


def test_overlay_projection_matches_the_camera(floor):
    """Identity colour-to-depth transform with the depth intrinsics: a rock's top corner must
    land on the pixel where the renderer's camera sees it."""
    rock = aw.Rock(1.6, 1.05, 0.2, 0.15, 0.10, rot_deg=20)
    pose = (1.0, 1.0, 0.3)
    corner = box_corners(rock)[1]                     # (x, y, top)
    uv, ok = to.project_world_points([corner], pose, floor, CAM.forward, np.eye(3), np.zeros(3),
                                     INTR)
    assert ok.all()
    assert tuple(uv[0]) == (pytest.approx(pixel(pose, corner)[0], abs=2.0),
                            pytest.approx(pixel(pose, corner)[1], abs=2.0))


def test_image_overlays_draw_only_visible_objects(approach, floor):
    reports, _ = approach
    rep = reports[-1]
    pose = (0.8 + 0.025 * 29, 1.0 + 0.003 * 29, math.radians(8 * math.sin(29 / 4.0)))
    ovs = to.image_overlays(rep, pose, floor, CAM.forward, np.eye(3), np.zeros(3), INTR)
    assert {o['id'] for o in ovs} == {o['id'] for o in rep if o['visible']}
    for ov in ovs:
        assert ov['label'] and ov['lines']
        pts = [p for line in ov['lines'] for p in line]       # outlines may run off the image,
        assert any(0 <= u < INTR.width and 0 <= v < INTR.height for u, v in pts)  # not all of it


# ── the ROS node, with stub rospy (tests/ros_stubs.py) ───────────────────────
@pytest.fixture(scope='module')
def node_module():
    import ros_stubs
    rospy = ros_stubs.install()
    import crater_detector_node as node_mod
    return rospy, node_mod


def test_node_publishes_objects_status_lines_and_rviz_labels(node_module, approach, floor,
                                                             monkeypatch):
    rospy, node_mod = node_module
    # all frames in the same instant, as on a fast machine: the RViz labels must not depend
    # on the 4 Hz marker throttle letting a frame through (this failed on CI)
    monkeypatch.setattr(node_mod.time, "time", lambda: 1000.0)
    node = node_mod.TerrainNode()
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    pose = (1.2, 1.0, 0.0)
    for i in range(3):
        dets, info = det.process(aw.render(DRIVE_ARENA, CAM, pose, U, V, seed=i), pose)
        node._update_objects(info, pose, det)
        node._publish(dets)
    msg = json.loads(rospy.PUBLISHED["terrain/objects"][-1].data)
    assert msg["frame_id"] == "map" and msg["rover"]["x"] == 1.2
    classes = sorted(o["class"] for o in msg["objects"])
    assert classes == ["crater", "rock", "rock"]
    status = rospy.PUBLISHED["terrain/status"][-1].data.split("\n")
    assert status[0].startswith("sees ")                     # the old first line is unchanged
    assert len(status) == 1 + len([o for o in msg["objects"] if o["visible"]])
    assert any(line.startswith("rock #") and " cm tall, " in line for line in status[1:])
    markers = rospy.PUBLISHED["terrain/object_markers"][-1].markers
    texts = [m.text for m in markers if m.type == 9]           # TEXT_VIEW_FACING
    assert sorted(texts) == sorted(o["label"] for o in msg["objects"])
    assert all(m.header.frame_id == "map" for m in markers[1:])
    # the planner's topic is still published as before
    assert "detections" in json.loads(rospy.PUBLISHED["terrain/detections"][-1].data)


def test_node_without_a_pose_reports_in_the_robot_frame(node_module, floor):
    rospy, node_mod = node_module
    node = node_mod.TerrainNode()
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    arena = aw.Arena(8.0, rocks=[aw.Rock(1.65, 1.0, 0.15, 0.12, 0.08)])
    for i in range(3):
        _, info = det.process(aw.render(arena, CAM, ROBOT, U, V, seed=i), None)
        node._update_objects(info, None, det)
    msg = json.loads(rospy.PUBLISHED["terrain/objects"][-1].data)
    assert msg["frame_id"] == "base_footprint" and msg["rover"] is None
    assert msg["objects"][0]["x"] == pytest.approx(0.65, abs=0.03)


def test_preview_worker_draws_and_encodes(node_module, approach, floor):
    cv2 = pytest.importorskip('cv2')
    rospy, node_mod = node_module
    reports, _ = approach
    pose = (0.8 + 0.025 * 29, 1.0 + 0.003 * 29, math.radians(8 * math.sin(29 / 4.0)))
    pub = rospy.Publisher("/terrain/preview/compressed", None)
    w = node_mod.PreviewWorker(cv2, pub, 70, (INTR, np.eye(3), np.zeros(3)), floor, CAM.forward)
    img = np.zeros((INTR.height, INTR.width, 3), dtype=np.uint8)
    w._draw(img, reports[-1], pose)
    assert img.any()                                  # outlines and labels were drawn
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    assert ok and buf.tobytes()[:2] == b"\xff\xd8"
    assert w.offer(img, reports[-1], pose) and not w.offer(img, reports[-1], pose)  # 1 pending max


def test_python_36_syntax_of_new_files():
    import ast
    for name in ('terrain_objects.py', 'terrain_core.py', 'crater_detector_node.py'):
        src = open(os.path.join(SCRIPTS, name), encoding='utf-8').read()
        ast.parse(src, feature_version=(3, 6))
