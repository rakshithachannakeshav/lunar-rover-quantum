#!/usr/bin/env python3
"""
tests/test_arjuna_crater_core.py
--------------------------------
Unit tests for robot/arjuna_quantum_nav/scripts/crater_core.py (floor calibration and depth
geometry) and the nav_core crater helpers, using synthetic OAK-D-Lite-like depth frames.
Terrain detection itself is tested in test_arjuna_terrain_core.py. No camera, no ROS.
"""

import math
import os
import sys

import numpy as np
import pytest

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'robot',
                                       'arjuna_quantum_nav', 'scripts'))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import crater_core as cc  # noqa: E402

INTR = cc.Intrinsics(fx=450.0, fy=450.0, cx=320.0, cy=200.0, width=640, height=400)
CAM_H, CAM_TILT, CAM_FWD = 0.25, math.radians(30.0), 0.10


def render(craters=(), boxes=(), noise_mm=0.0, seed=0):
    """Depth image (uint16 mm) of flat floor seen by a camera CAM_H up, tilted CAM_TILT down,
    CAM_FWD ahead of the robot centre. craters: (x, y, radius, depth) discs sunk into the
    floor; boxes: (x, y, half_size, height) blocks standing on it (robot frame, metres)."""
    v, u = np.mgrid[0:INTR.height, 0:INTR.width].astype(np.float64)
    xr, yr = (u - INTR.cx) / INTR.fx, (v - INTR.cy) / INTR.fy
    s, c = math.sin(CAM_TILT), math.cos(CAM_TILT)
    # camera right = -left, camera down, lens axis -> robot frame (fwd, left, up)
    dx = yr * -s + c
    dy = -xr
    dz = yr * -c - s
    t = np.full(xr.shape, np.inf)
    down = dz < 0
    t[down] = CAM_H / -dz[down]
    px, py = CAM_FWD + t * dx, t * dy
    for (bx, by, half, bh) in boxes:          # top face of each box
        tb = np.where(down, (CAM_H - bh) / np.where(down, -dz, 1), np.inf)
        bxp, byp = CAM_FWD + tb * dx, tb * dy
        hit = down & (np.abs(bxp - bx) <= half) & (np.abs(byp - by) <= half) & (tb < t)
        t = np.where(hit, tb, t)
    for (cx, cy, rad, depth) in craters:      # bottom of each crater
        inside = (px - cx) ** 2 + (py - cy) ** 2 <= rad ** 2
        tc = np.where(down, (CAM_H + depth) / np.where(down, -dz, 1), np.inf)
        bx, by = CAM_FWD + tc * dx, tc * dy
        sees_bottom = inside & ((bx - cx) ** 2 + (by - cy) ** 2 <= rad ** 2)
        t = np.where(sees_bottom, tc, t)
    z = t  # ray has unit lens component, so t is the depth along the lens axis
    if noise_mm:
        z = z + np.random.RandomState(seed).normal(0, noise_mm / 1000.0, z.shape)
    z = np.where(np.isfinite(z), z * 1000.0, 0.0)
    return np.clip(z, 0, 65535).astype(np.uint16)


def calibrated_floor(noise_mm=0.0):
    u, v = cc.pixel_grid(INTR)
    pts = cc.depth_to_points(render(noise_mm=noise_mm), INTR, u, v)
    return cc.fit_floor(pts), (u, v)


def test_calibration_recovers_camera_height_and_tilt():
    floor, _ = calibrated_floor(noise_mm=3.0)
    assert floor.camera_height == pytest.approx(CAM_H, abs=0.01)
    assert floor.tilt_deg == pytest.approx(30.0, abs=1.0)


def test_floor_round_trips_through_json():
    floor, _ = calibrated_floor()
    again = cc.FloorModel.from_json(floor.to_json())
    assert np.allclose(again.n, floor.n) and np.allclose(again.c, floor.c)


def test_lifted_points_read_positive_height():
    floor, _ = calibrated_floor()
    xy = np.array([[0.5, 0.0], [0.8, 0.2]])
    lifted = floor.from_robot(xy[:, 0], xy[:, 1], CAM_FWD, height=0.10)
    assert floor.heights(lifted) == pytest.approx([0.10, 0.10], abs=1e-6)
    assert floor.heights(floor.from_robot(xy[:, 0], xy[:, 1], CAM_FWD, height=-0.05)) ==         pytest.approx([-0.05, -0.05], abs=1e-6)


def test_floor_points_project_back_to_the_pixels_they_came_from():
    floor, _ = calibrated_floor()
    pts = floor.from_robot([0.6], [0.1], CAM_FWD)
    u = pts[0, 0] / pts[0, 2] * INTR.fx + INTR.cx
    v = pts[0, 1] / pts[0, 2] * INTR.fy + INTR.cy
    img = render()
    assert 0 <= u < INTR.width and 0 <= v < INTR.height
    z = img[int(round(v)), int(round(u))] / 1000.0
    assert z == pytest.approx(pts[0, 2], abs=0.03)


# ── nav_core helpers used by the navigator for craters ───────────────────────
import nav_core  # noqa: E402


def test_robot_to_world_rotates_and_shifts():
    pts = nav_core.robot_to_world((1.0, 2.0, math.pi / 2), [(0.5, 0.0), (0.0, 0.5)])
    assert pts[0] == pytest.approx((1.0, 2.5))
    assert pts[1] == pytest.approx((0.5, 2.0))


def test_points_near_path_only_checks_the_path_still_ahead():
    wps = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    crater_on_leg_2 = [(1.05, 0.5)]
    crater_behind = [(0.3, 0.05)]
    assert nav_core.points_near_path(crater_on_leg_2, (0.9, 0.0), wps, 2, 0.2)
    assert not nav_core.points_near_path(crater_behind, (0.9, 0.0), wps, 2, 0.2)
    assert not nav_core.points_near_path([(1.6, 0.5)], (0.9, 0.0), wps, 2, 0.2)


def test_crater_map_deduplicates():
    m = nav_core.CraterMap(cell=0.05)
    assert m.add([(0.71, 0.11), (0.72, 0.12)]) == [(0.71, 0.11)]
    assert m.add([(0.72, 0.12), (0.90, 0.10)]) == [(0.90, 0.10)]
    assert len(m.points) == 2


def test_planner_routes_around_a_marked_crater():
    grid = nav_core.PlanningGrid.empty(1.0, 0.0, size=4.0, resolution=0.05)
    start, goal = (0.0, 0.0), (2.0, 0.0)
    straight = nav_core.plan(grid, "astar", start, goal)
    assert all(abs(p[1]) < 0.06 for p in straight.points)
    crater = [(1.0 + dx * 0.05, dy * 0.05) for dx in range(-2, 3) for dy in range(-2, 3)]
    grid.add_obstacle_points(crater, 0.15)
    for algo in nav_core.ALGORITHMS:
        r = nav_core.plan(grid, algo, start, goal)
        assert r is not None
        assert min(math.hypot(p[0] - 1.0, p[1]) for p in r.points) > 0.2
