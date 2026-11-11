#!/usr/bin/env python3
"""
tests/test_arjuna_terrain_core.py
---------------------------------
Unit tests for robot/arjuna_quantum_nav/scripts/terrain_core.py: crater rims, rocks, border
wall, table edge and invalid-depth handling, on synthetic OAK-D-like depth images rendered
from tests/arena_world.py. No camera, no ROS.
"""

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

INTR = cc.Intrinsics(fx=450.0, fy=450.0, cx=320.0, cy=200.0, width=640, height=400)
CAM = aw.Camera(INTR, height=0.25, tilt_deg=30.0, forward=0.10)
PARAMS = tc.TerrainParams()
U, V = cc.pixel_grid(INTR, step=PARAMS.pixel_step, step_y=PARAMS.pixel_step_y,
                     top_fraction=PARAMS.top_fraction)
BIG = aw.Arena(8.0)     # a table big enough that its edge is never in view
ROBOT = (1.0, 1.0, 0.0)


@pytest.fixture(scope='module')
def floor():
    img = aw.render(BIG, CAM, (3, 3, 0), U, V, seed=99)
    return cc.fit_floor(cc.depth_to_points(img, INTR, U, V))


def detect(floor, arena, pose=ROBOT, frames=3, params=None, **render_kw):
    det = tc.TerrainDetector(floor, INTR, params or PARAMS)
    dets = []
    for i in range(frames):
        dets, _ = det.process(aw.render(arena, CAM, pose, U, V, seed=i, **render_kw), pose)
    return dets, det


def world_to_robot(pose, wx, wy):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = wx - pose[0], wy - pose[1]
    return dx * c + dy * s, -dx * s + dy * c


def of_type(dets, kind):
    return [d for d in dets if d['type'] == kind]


def coverage(dets, pts, tol=0.03):
    """Fraction of robot-frame points `pts` within `tol` of some published cell."""
    cells = np.concatenate([np.asarray(d['cells']) for d in dets]) if dets else np.zeros((0, 2))
    if len(cells) == 0:
        return 0.0
    hit = 0
    for x, y in pts:
        if np.min(np.hypot(cells[:, 0] - x, cells[:, 1] - y)) <= tol:
            hit += 1
    return hit / float(len(pts))


def octagon_points(cr, pose, step=0.03):
    """Robot-frame points covering the whole crater octagon (rim and inside)."""
    pts = []
    for x in np.arange(cr.x - cr.R, cr.x + cr.R + 1e-9, step):
        for y in np.arange(cr.y - cr.R, cr.y + cr.R + 1e-9, step):
            if cr.contains(x, y):
                pts.append(world_to_robot(pose, x, y))
    return pts


def in_watched_view(det, pts):
    g = det.geom
    out = []
    for x, y in pts:
        ix = int((x - PARAMS.x_min) / PARAMS.cell)
        iy = int((y + PARAMS.half_width) / PARAMS.cell)
        if 0 <= ix < g.nx and 0 <= iy < g.ny and g.in_view[ix, iy]:
            out.append((x, y))
    return out


# ── calibration through the renderer ─────────────────────────────────────────
def test_floor_calibration_from_rendered_depth(floor):
    assert floor.camera_height == pytest.approx(0.25, abs=0.01)
    assert floor.tilt_deg == pytest.approx(30.0, abs=1.0)


def test_from_robot_inverts_to_robot(floor):
    pts = floor.from_robot([0.5, 0.8], [0.1, -0.2], camera_forward=0.1, height=0.03)
    xy = floor.to_robot(pts, 0.1)
    assert xy[:, 0] == pytest.approx([0.5, 0.8], abs=1e-9)
    assert xy[:, 1] == pytest.approx([0.1, -0.2], abs=1e-9)
    assert floor.heights(pts) == pytest.approx([0.03, 0.03], abs=1e-9)


# ── nothing there: no obstacles invented from noise ─────────────────────────
@pytest.mark.parametrize('seed', range(6))
def test_flat_floor_gives_no_detections(floor, seed):
    rs = np.random.RandomState(seed)
    pose = (rs.uniform(3, 5), rs.uniform(3, 5), rs.uniform(-math.pi, math.pi))
    dets, _ = detect(floor, BIG, pose)
    assert dets == []


def test_noisy_floor_with_dropouts_and_blocks_gives_no_detections(floor):
    """3x the stereo noise, 8% dropped pixels, plus patches of missing depth ~2 cm wide."""
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    rs = np.random.RandomState(5)
    for i in range(3):
        img = aw.render(BIG, CAM, (3, 3, 0), U, V, seed=i, noise_scale=3.0, dropout=0.08)
        for _ in range(30):                       # small patches of missing depth
            r, c = rs.randint(120, 390), rs.randint(30, 600)
            img[r:r + 4, c:c + 6] = 0
        dets, _ = det.process(img, (3, 3, 0))
    assert dets == []


# ── crater rims ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize('ahead', [0.45, 0.60, 0.85])
def test_crater_ring_fully_in_view_is_found_and_filled(floor, ahead):
    cr = aw.Crater(ROBOT[0] + ahead, ROBOT[1], 0.35)
    dets, det = detect(floor, aw.Arena(8.0, craters=[cr]))
    craters = of_type(dets, 'crater')
    assert len(craters) == 1 and len(dets) == 1
    c = craters[0]
    assert c['x'] == pytest.approx(ahead, abs=0.03) and c['y'] == pytest.approx(0.0, abs=0.03)
    assert c['r'] == pytest.approx(cr.apothem - cr.rim_w / 2 + 0.01, abs=0.03)
    assert 0.020 * 0.5 < c['height'] < 0.020 * 1.5          # the ~2 cm rim height
    # the area INSIDE the ring and the rim itself are blocked, including the octagon corners
    assert coverage(dets, octagon_points(cr, ROBOT)) >= 0.97


@pytest.mark.parametrize('diameter,rot', [(0.25, 0.0), (0.45, 0.0), (0.30, 22.0)])
def test_crater_sizes_and_rotations(floor, diameter, rot):
    cr = aw.Crater(1.55, 1.0, diameter, rot_deg=rot)
    dets, _ = detect(floor, aw.Arena(8.0, craters=[cr]))
    assert len(of_type(dets, 'crater')) == 1
    assert coverage(dets, octagon_points(cr, ROBOT)) >= 0.97
    # ... but it does not block a big area outside the crater
    cells = np.concatenate([np.asarray(d['cells']) for d in dets])
    far = np.hypot(cells[:, 0] - 0.55, cells[:, 1])
    assert far.max() <= cr.R + 0.07


def test_crater_only_partly_in_view_is_still_filled_whole(floor):
    """Crater half out of the camera's field of view: the visible arc still gives the circle,
    and the part nobody has seen is blocked too."""
    cr = aw.Crater(1.5, 1.32, 0.35)               # 0.5 m ahead, 0.32 m left: cut by the view
    dets, det = detect(floor, aw.Arena(8.0, craters=[cr]))
    craters = of_type(dets, 'crater')
    assert len(craters) == 1
    assert not craters[0].get('partial')
    full = octagon_points(cr, ROBOT)
    seen = in_watched_view(det, full)
    assert len(seen) < 0.85 * len(full)           # the setup really hides part of it
    assert coverage(dets, full) >= 0.95            # yet nearly all of the crater is blocked
    c = craters[0]
    assert math.hypot(c["x"] - 0.5, c["y"] - 0.32) < 0.07


def test_short_rim_arc_is_blocked_as_seen_not_guessed(floor):
    """A sliver of rim (< ~60 degrees) cannot give a radius: block the arc, not a guess."""
    cr = aw.Crater(1.8, 1.0, 0.45)
    dets, det = detect(floor, aw.Arena(8.0, craters=[cr]), pose=(1.0, 1.0, 0.0))
    # robot looks at the crater's near edge only (its far side is beyond the 1.2 m view)
    near_edge = [(x, y) for x, y in octagon_points(cr, (1.0, 1.0, 0.0)) if x < 0.62]
    assert coverage(dets, in_watched_view(det, near_edge)) >= 0.9


def test_rim_below_the_threshold_is_ignored(floor):
    cr = aw.Crater(1.55, 1.0, 0.35, rim_h=0.004)
    dets, _ = detect(floor, aw.Arena(8.0, craters=[cr]))
    assert dets == []


def test_crater_fusion_across_moving_frames(floor):
    """Frames taken while the robot drives 4 cm per frame are fused with the odometry pose."""
    cr = aw.Crater(1.8, 1.0, 0.35)
    arena = aw.Arena(8.0, craters=[cr])
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    pose = None
    for i in range(3):
        pose = (1.0 + 0.04 * i, 1.0, 0.0)
        dets, _ = det.process(aw.render(arena, CAM, pose, U, V, seed=i), pose)
    c = of_type(dets, 'crater')[0]
    assert (c['x'], c['y']) == pytest.approx(world_to_robot(pose, 1.8, 1.0), abs=0.04)


# ── rocks, border, table edge ────────────────────────────────────────────────
def rock_points(rock, pose, step=0.03):
    pts = []
    for x in np.arange(rock.x - rock.hw, rock.x + rock.hw + 1e-9, step):
        for y in np.arange(rock.y - rock.hd, rock.y + rock.hd + 1e-9, step):
            pts.append(world_to_robot(pose, x, y))
    return pts


def front_of_rock(rock, pose, depth=0.15):
    """The part of the rock a camera in front of it can vouch for: its front face and the
    first `depth` metres behind it (the detector blocks 15 cm of hidden floor behind a face)."""
    pts = rock_points(rock, pose)
    x0 = min(x for x, _ in pts)
    return [(x, y) for x, y in pts if x <= x0 + depth]


@pytest.mark.parametrize('height', [0.20, 0.25, 0.30])
def test_rock_is_found_and_its_footprint_blocked(floor, height):
    rock = aw.Rock(1.6, 1.05, 0.35, 0.30, height, rot_deg=15)
    dets, det = detect(floor, aw.Arena(8.0, rocks=[rock]))
    rocks = of_type(dets, 'rock')
    assert len(rocks) == 1 and len(dets) == 1
    assert coverage(dets, front_of_rock(rock, ROBOT)) >= 0.9
    assert coverage(dets, rock_points(rock, ROBOT)) >= 0.5      # and about half of it overall
    cells = np.asarray(rocks[0]['cells'])
    area = len(cells) * PARAMS.out_spacing ** 2
    assert area < 4 * 0.35 * 0.30                  # blocked area stays near the rock's size
    assert rocks[0]['height'] > 0.08


def test_shiny_rock_with_dropouts_and_spikes_is_still_a_rock(floor):
    rock = aw.Rock(1.6, 1.0, 0.35, 0.30, 0.25)
    dets, _ = detect(floor, aw.Arena(8.0, rocks=[rock]), shiny_dropout=0.6, shiny_spike=0.10)
    assert len(of_type(dets, 'rock')) == 1
    assert coverage(dets, front_of_rock(rock, ROBOT)) >= 0.9


def test_rock_that_returns_no_depth_at_all_is_not_ignored(floor):
    """Worst case for stereo: the plastic gives NO depth. The empty patch in front of the floor
    is blocked as an object (and only its first stretch, not the whole shadow)."""
    rock = aw.Rock(1.6, 1.0, 0.35, 0.30, 0.25)
    dets, _ = detect(floor, aw.Arena(8.0, rocks=[rock]), shiny_dropout=1.0)
    assert len(dets) >= 1 and all(d['type'] in ('rock',) for d in dets)
    assert coverage(dets, rock_points(rock, ROBOT)) >= 0.9
    cells = np.concatenate([np.asarray(d['cells']) for d in dets])
    assert cells[:, 0].max() < 0.6 + 0.35 + 0.2    # not the whole 1.2 m view


def test_two_rocks_stay_separate(floor):
    rocks = [aw.Rock(1.6, 0.8, 0.35, 0.3, 0.25), aw.Rock(1.6, 1.35, 0.35, 0.3, 0.25)]
    dets, _ = detect(floor, aw.Arena(8.0, rocks=rocks))
    assert len(of_type(dets, 'rock')) == 2


@pytest.mark.parametrize('wall_h', [0.08, 0.15])
def test_border_wall_is_a_long_thin_obstacle(floor, wall_h):
    arena = aw.Arena(8.0, wall_h=wall_h)
    pose = (7.4, 4.0, 0.0)                         # facing the +x wall, 0.55 m away
    dets, _ = detect(floor, arena, pose)
    borders = of_type(dets, 'border')
    assert borders
    wall_x = 8.0 - arena.wall_t - pose[0]
    cx = np.concatenate([np.asarray(d['cells']) for d in borders])
    assert cx[:, 0].min() == pytest.approx(wall_x - 0.04, abs=0.05)
    assert not of_type(dets, 'rock')


@pytest.mark.parametrize('ground', [False, True])
def test_table_edge_is_no_floor(floor, ground):
    """No wall: the floor simply stops 0.5 m ahead (nothing there, or ground far below)."""
    arena = aw.Arena(2.21, wall_h=0.0)
    pose = (1.7, 1.0, 0.0)
    edge = 2.21 - pose[0]
    dets, _ = detect(floor, arena, pose, ground_below_edge=ground)
    nf = of_type(dets, 'nofloor')
    assert len(nf) == 1
    cells = np.asarray(nf[0]['cells'])
    assert cells[:, 0].min() >= edge - 0.06       # never claims floor that is really there
    assert cells[:, 0].min() <= edge + 0.06       # ... and starts at the edge
    beyond = [(x, y) for x in np.arange(edge + 0.05, 1.1, 0.05) for y in (-0.3, 0.0, 0.3)]
    assert coverage(nf, beyond, tol=0.04) >= 0.9


def test_black_wall_with_no_depth_also_reads_as_no_floor(floor):
    """The black border returns no depth (not even a wall): same as the table edge."""
    arena = aw.Arena(2.21, wall_h=0.0)
    dets, _ = detect(floor, arena, (1.7, 1.0, 0.0))
    assert of_type(dets, 'nofloor')


def test_table_far_beyond_the_reliable_range_is_not_called_an_edge(floor):
    """Stereo is unreliable far away: floor ending 1.1 m ahead is not reported."""
    arena = aw.Arena(2.21, wall_h=0.0)
    dets, _ = detect(floor, arena, (1.0, 1.0, 0.0))
    assert dets == []


# ── measure mode ─────────────────────────────────────────────────────────────
def test_measure_reports_noise_and_rim_height(floor):
    cr = aw.Crater(1.55, 1.0, 0.35)
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    frames = [aw.render(aw.Arena(8.0, craters=[cr]), CAM, ROBOT, U, V, seed=i) for i in range(5)]
    m = tc.measure(det, frames)
    assert m['floor_noise_mm_single_frame'] == pytest.approx(1.5, abs=1.0)
    assert m['rim_height_mm_median'] == pytest.approx(20.0, abs=6.0)
    assert m['rim_snr'] > 4
    assert 'reliable' in m['message']
    assert ('crater', 0.55, 0.0, pytest.approx(0.16, abs=0.03)) == \
        (m['detections'][0][0], m['detections'][0][1], m['detections'][0][2], m['detections'][0][3])


def test_measure_warns_when_the_rim_is_lost_in_noise(floor):
    cr = aw.Crater(1.55, 1.0, 0.35)
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    frames = [aw.render(aw.Arena(8.0, craters=[cr]), CAM, ROBOT, U, V, seed=i, noise_scale=8.0)
              for i in range(5)]
    m = tc.measure(det, frames)
    assert 'reliably' in m['message'] or 'marginal' in m['message'] or 'No rim' in m['message']


def test_measure_without_a_crater_says_so(floor):
    det = tc.TerrainDetector(floor, INTR, PARAMS)
    frames = [aw.render(BIG, CAM, (3, 3, 0), U, V, seed=i) for i in range(4)]
    m = tc.measure(det, frames)
    assert m['rim_height_mm_median'] is None and 'No rim' in m['message']


# ── geometry helpers ─────────────────────────────────────────────────────────
def test_fit_rim_recovers_a_full_circle_and_pins_a_short_arc():
    ang = np.linspace(0, 2 * math.pi, 60, endpoint=False)
    pts = np.stack([0.5 + 0.15 * np.cos(ang), 0.1 + 0.15 * np.sin(ang)], axis=1)
    c, r, span, rms = tc.fit_rim(pts, PARAMS)
    assert (c[0], c[1], r) == pytest.approx((0.5, 0.1, 0.15), abs=0.01) and span > 5.5
    arc = pts[:12]                                  # 70 degrees
    c2, r2, span2, rms2 = tc.fit_rim(arc, PARAMS)
    assert PARAMS.crater_r_min <= r2 <= PARAMS.crater_r_max and span2 < 2.0
    # a straight line is not a crater
    line = np.stack([np.linspace(0.4, 0.9, 30), np.full(30, 0.2)], axis=1)
    fit = tc.fit_rim(line, PARAMS)
    assert fit[3] > PARAMS.fit_tol


def test_speckle_filter_removes_isolated_spikes_but_keeps_a_ramp():
    ramp = np.tile(np.linspace(0.5, 1.0, 40)[:, None], (1, 40))
    spiked = ramp.copy()
    spiked[10, 10] += 0.3
    spiked[20, 20] = np.nan
    out = tc.speckle_filter(spiked, 0.03, 0.03)
    assert np.isnan(out[10, 10])
    assert np.isfinite(out[15, 15]) and np.isfinite(out[30, 30])
    assert np.isfinite(out[5:9, 5:9]).all()


def test_all_python_36_syntax():
    import ast
    for name in ('terrain_core.py', 'crater_core.py'):
        src = open(os.path.join(SCRIPTS, name), encoding='utf-8').read()
        ast.parse(src, feature_version=(3, 6))
