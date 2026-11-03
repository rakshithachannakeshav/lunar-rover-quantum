#!/usr/bin/env python3
"""
tests/test_arjuna_nav_core.py
-----------------------------
Unit tests for robot/arjuna_quantum_nav/scripts/nav_core.py — the planning and
path-following core that runs on the Arjuna Edu Kit (ROS Melodic, Python 3.6).
No ROS dependency.
"""

import ast
import math
import os
import sys

import pytest

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'robot',
                                       'arjuna_quantum_nav', 'scripts'))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import nav_core  # noqa: E402


def hill(cx, cy, amp, sigma):
    def h(x, y):
        return amp * math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * sigma * sigma))
    return h


def open_grid(elevation=None, max_slope_deg=25.0):
    return nav_core.PlanningGrid.empty(0.0, 0.0, size=8.0, resolution=0.1,
                                       elevation=elevation, max_slope_deg=max_slope_deg)


# ── energy model ──────────────────────────────────────────────────────────────
def test_grade_factor_uphill_costs_more_than_downhill():
    assert nav_core.grade_factor(0.0) == 1.0
    assert nav_core.grade_factor(0.1) > 1.0 > nav_core.grade_factor(-0.1)
    # steep descents need braking, so they cost more than gentle ones
    assert nav_core.grade_factor(-0.4) > nav_core.grade_factor(-0.15)
    assert min(nav_core.grade_factor(-g / 100.0) for g in range(0, 60)) >= nav_core.MIN_GRADE_FACTOR


def test_too_steep_step_is_impassable():
    grid = open_grid(elevation=lambda x, y: 2.0 * x, max_slope_deg=25.0)  # 63° ramp
    a = grid.world_to_cell(0.0, 0.0)
    assert grid.step(a, (a[0] + 1, a[1]))[2] is None


# ── planners ──────────────────────────────────────────────────────────────────
@pytest.mark.parametrize("algo", nav_core.ALGORITHMS)
def test_every_planner_reaches_goal_on_flat_ground(algo):
    r = nav_core.plan(open_grid(), algo, (-3.0, 0.0), (3.0, 0.0))
    assert r is not None
    assert math.hypot(r.points[-1][0] - 3.0, r.points[-1][1]) < 0.15
    assert r.distance == pytest.approx(6.0, abs=0.2)


def test_planners_avoid_obstacles():
    grid = open_grid()
    wall = [(0.0, y / 10.0) for y in range(-30, 31)]  # wall at x=0 with a gap above y=3
    grid.add_obstacle_points(wall, radius=0.2)
    r = nav_core.plan(grid, "astar", (-2.0, 0.0), (2.0, 0.0))
    assert r is not None
    for x, y in r.points:
        assert not grid.is_blocked(*grid.world_to_cell(x, y))
    assert max(abs(y) for _, y in r.points) > 3.0  # had to go round the end of the wall


def test_energy_aware_astar_goes_round_hill_that_dijkstra_climbs():
    grid = open_grid(elevation=hill(0.0, 0.0, 0.8, 0.9))
    res = nav_core.plan_all(grid, (-3.0, 0.0), (3.0, 0.0))
    d, a = res["dijkstra"], res["astar"]
    assert d.distance < a.distance          # Dijkstra: shortest distance...
    assert a.energy < d.energy              # ...but A* uses less energy
    assert a.climb < d.climb
    assert nav_core.rank(res.values())[0].algorithm in ("astar", "qaoa")


def test_astar_matches_dijkstra_energy_when_both_minimise_energy():
    # admissible heuristic => A* is optimal: compare against an energy Dijkstra
    grid = open_grid(elevation=hill(0.5, 0.5, 0.6, 1.0))
    start = grid.world_to_cell(-3.0, -1.0)
    goal = grid.world_to_cell(3.0, 1.5)
    cells_a, n_a = nav_core.astar_cells(grid, start, goal)
    cells_d, n_d = nav_core._search(grid, start, goal, nav_core._energy_cost(grid), nav_core._zero)
    e_a = nav_core.evaluate_cells(grid, cells_a)[0]
    e_d = nav_core.evaluate_cells(grid, cells_d)[0]
    assert e_a == pytest.approx(e_d, abs=1e-6)
    assert n_a < n_d


def test_qaoa_never_beats_optimal_astar_and_reports_circuit_info():
    grid = open_grid(elevation=hill(0.0, 0.5, 0.7, 1.0))
    a = nav_core.plan(grid, "astar", (-3.0, 0.0), (3.0, 0.0))
    q = nav_core.plan(grid, "qaoa", (-3.0, 0.0), (3.0, 0.0))
    assert q.energy >= a.energy - 1e-6
    assert q.extra["qubits"] > 0 and q.extra["shots"] == 2048


def test_start_inside_inflated_obstacle_snaps_to_free_cell():
    grid = open_grid()
    grid.add_obstacle_points([(-3.0, 0.0)], radius=0.2)
    assert nav_core.plan(grid, "astar", (-3.0, 0.0), (2.0, 0.0)) is not None


def test_from_occupancy_downsamples_and_inflates():
    w = h = 40  # 2 m x 2 m at 0.05 m
    data = [0] * (w * h)
    data[20 * w + 20] = 100  # one occupied cell at (1.0, 1.0)
    grid = nav_core.PlanningGrid.from_occupancy(w, h, 0.05, 0.0, 0.0, data,
                                                plan_resolution=0.1, robot_radius=0.2)
    assert grid.width == 20 and grid.resolution == pytest.approx(0.1)
    assert grid.is_blocked(*grid.world_to_cell(1.0, 1.0))
    assert grid.is_blocked(*grid.world_to_cell(1.15, 1.0))   # inflated
    assert not grid.is_blocked(*grid.world_to_cell(1.6, 1.0))


def test_thin_path_keeps_endpoints_and_drops_dense_points():
    pts = [(i * 0.1, 0.0) for i in range(31)]
    thin = nav_core.thin_path(pts, spacing=0.3)
    assert thin[0] == pts[0] and thin[-1] == pts[-1]
    assert len(thin) < len(pts) / 2


# ── following ─────────────────────────────────────────────────────────────────
def test_follower_drives_simulated_robot_to_goal():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    f = nav_core.WaypointFollower(path, goal_tolerance=0.1, waypoint_tolerance=0.15,
                                  max_linear=0.15, max_angular=0.8)
    x = y = yaw = 0.0
    dt = 0.1
    for _ in range(2000):
        v, w, done = f.step(x, y, yaw)
        if done:
            break
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
    assert done
    assert math.hypot(x - 1.0, y - 1.0) <= 0.1


def test_uphill_cap_slows_climbing_only():
    assert nav_core.uphill_speed_cap(0.12, math.radians(10)) < 0.12
    assert nav_core.uphill_speed_cap(0.12, math.radians(-10)) == 0.12


def test_energy_meter_counts_climb_and_gravity_work():
    flat, up = nav_core.EnergyMeter(), nav_core.EnergyMeter()
    for _ in range(100):
        flat.update(0.1, 0.0, 0.0, 0.1)
        up.update(0.1, 0.0, math.radians(10), 0.1)
    assert up.joules > flat.joules
    assert up.climb == pytest.approx(1.0 * math.sin(math.radians(10)), rel=1e-6)


# ── Python 3.6 compatibility (the Jetson runs 3.6.9) ──────────────────────────
@pytest.mark.parametrize("name", ["nav_core.py", "quantum_nav_node.py", "drive_test.py"])
def test_robot_scripts_avoid_python37_only_features(name):
    src = open(os.path.join(SCRIPTS, name), encoding="utf-8").read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            assert node.module != "dataclasses", name
            if node.module == "__future__":
                assert "annotations" not in [a.name for a in node.names], name
        if isinstance(node, ast.Import):
            assert "dataclasses" not in [a.name for a in node.names], name
        assert type(node).__name__ != "NamedExpr", name  # walrus is 3.8+
    assert "breakpoint(" not in src


# ── obstacle avoidance ────────────────────────────────────────────────────────
N_BEAMS = 360
INC = 2 * math.pi / N_BEAMS


def _cast(x, y, yaw, rocks):
    """360-beam scan (angle_min=-pi) of circular rocks [(cx, cy, r)] from pose (x, y, yaw)."""
    ranges = []
    for i in range(N_BEAMS):
        a = yaw - math.pi + i * INC
        dx, dy = math.cos(a), math.sin(a)
        best = float('inf')
        for cx, cy, rad in rocks:
            fx, fy = x - cx, y - cy
            b = fx * dx + fy * dy
            disc = b * b - (fx * fx + fy * fy - rad * rad)
            if disc >= 0 and -b - math.sqrt(disc) > 0:
                best = min(best, -b - math.sqrt(disc))
        ranges.append(best)
    return ranges


def test_scan_sectors_front_sides_rear_and_yaw_offset():
    ranges = [float('inf')] * N_BEAMS
    ranges[180] = 0.5           # straight ahead (angle 0)
    ranges[0] = 0.3             # straight behind
    ranges[180 + 60] = 0.4      # 60 deg left
    s = nav_core.scan_sectors(ranges, -math.pi, INC, 12.0)
    assert s.front == pytest.approx(0.5) and s.rear == pytest.approx(0.3)
    assert s.left < s.right
    # lidar mounted backwards: what it calls 0 deg is really behind the robot
    flipped = nav_core.scan_sectors(ranges, -math.pi, INC, 12.0, yaw_offset=math.pi)
    assert flipped.front == pytest.approx(0.3) and flipped.rear == pytest.approx(0.5)


def test_scan_sectors_drops_self_hits_and_nan():
    ranges = [float('nan')] * N_BEAMS
    ranges[180] = 0.05
    s = nav_core.scan_sectors(ranges, -math.pi, INC, 12.0)
    assert s.front == pytest.approx(12.0)


def test_avoider_turns_to_the_open_side():
    av = nav_core.ObstacleAvoider()
    cmd, _ = av.update(nav_core.ScanSectors(0.3, 3.0, 0.5), 0, 0)
    assert cmd[0] == 0.0 and cmd[1] > 0          # left is open
    av = nav_core.ObstacleAvoider()
    cmd, _ = av.update(nav_core.ScanSectors(0.3, 0.5, 3.0), 0, 0)
    assert cmd[0] == 0.0 and cmd[1] < 0          # right is open


def test_avoider_only_reverses_when_the_rear_is_clear():
    av = nav_core.ObstacleAvoider()
    cmd, _ = av.update(nav_core.ScanSectors(0.15, 2.0, 2.0, rear=2.0), 0, 0)
    assert cmd[0] < 0.0
    av = nav_core.ObstacleAvoider()
    cmd, _ = av.update(nav_core.ScanSectors(0.15, 2.0, 2.0, rear=0.1), 0, 0)
    assert cmd[0] == 0.0 and cmd[1] != 0.0        # wall behind: turn in place instead


def test_flank_guard_veers_away_from_a_close_side():
    av = nav_core.ObstacleAvoider()
    cmd, _ = av.update(nav_core.ScanSectors(5.0, 2.0, 2.0, left_near=0.25), 0, 0)
    assert cmd[0] > 0.0 and cmd[1] < 0.0          # keep going, veer right
    assert not av.active                           # not a full detour
    cmd, _ = av.update(nav_core.ScanSectors(5.0, 2.0, 2.0), 0, 0)
    assert cmd is None


def test_retarget_nearest_never_goes_backwards():
    f = nav_core.WaypointFollower([(0, 0), (1, 0), (2, 0), (3, 0)])
    f.index = 2
    f.retarget_nearest(0.0, 0.0)
    assert f.index == 2
    f.retarget_nearest(3.1, 0.0)
    assert f.index == 3


def test_closed_loop_goes_round_a_rock_and_reaches_the_goal():
    rock = (1.0, 0.03, 0.15)
    goal = (2.0, 0.0)
    f = nav_core.WaypointFollower([(0.5, 0.0), (1.0, 0.0), (1.5, 0.0), goal],
                                  max_linear=0.12, max_angular=0.6)
    av = nav_core.ObstacleAvoider(max_linear=0.12, max_angular=0.6)
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.1
    closest, avoided, last = float('inf'), False, 0
    for _ in range(6000):
        s = nav_core.scan_sectors(_cast(x, y, yaw, [rock]), -math.pi, INC, 12.0)
        cmd, done = av.update(s, x, y)
        if cmd is None:
            if done:
                f.retarget_nearest(x, y)
            lin, ang, finished = f.step(x, y, yaw)
            if f.index != last:
                last = f.index
                av.reset_side()
            if finished:
                break
        else:
            avoided = True
            lin, ang = cmd
        x += lin * math.cos(yaw) * dt
        y += lin * math.sin(yaw) * dt
        yaw = nav_core.wrap_angle(yaw + ang * dt)
        closest = min(closest, math.hypot(x - rock[0], y - rock[1]) - rock[2])
    assert f.done and avoided
    assert closest > 0.22                       # robot radius is 0.2 m: never touched the rock
    assert math.hypot(x - goal[0], y - goal[1]) <= 0.15
