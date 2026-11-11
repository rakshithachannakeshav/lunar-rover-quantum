#!/usr/bin/env python3
"""
tests/test_arjuna_arena_tools.py
--------------------------------
Arena helpers in robot/arjuna_quantum_nav/scripts (no ROS, no camera):
  nav_core.TerrainMap / BumpDetector / PlanningGrid.arena / add_disc / add_soft_margin
  make_arena_map.py (layout -> PGM + YAML), compare_plans.py, compare_runs.py
"""

import ast
import json
import math
import os
import sys

import pytest

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'robot',
                                       'arjuna_quantum_nav', 'scripts'))
CONFIG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'robot',
                                      'arjuna_quantum_nav', 'config'))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import compare_plans  # noqa: E402
import compare_runs  # noqa: E402
import make_arena_map as mam  # noqa: E402
import nav_core  # noqa: E402


# ── TerrainMap ───────────────────────────────────────────────────────────────
def crater_det(x, y, r, span=5.0, fill_r=None):
    return {"type": "crater", "x": x, "y": y, "r": r, "fill_r": fill_r or r + 0.03, "span": span,
            "cells": []}


def rock_det(cells):
    return {"type": "rock", "cells": cells}


def test_terrain_map_moves_detections_to_the_world_frame():
    t = nav_core.TerrainMap()
    pose = (1.0, 2.0, math.pi / 2)                     # facing +y
    t.add(pose, [crater_det(0.5, 0.0, 0.15)])
    ch = t.add(pose, [crater_det(0.5, 0.0, 0.15)])
    assert ch and ch[0][0] == "crater"
    (x, y, r), = t.discs()
    assert (x, y) == pytest.approx((1.0, 2.5), abs=1e-9) and r == pytest.approx(0.18)


def test_crater_needs_to_be_seen_twice_and_is_stored_once():
    t = nav_core.TerrainMap()
    assert t.add((0, 0, 0), [crater_det(0.6, 0.0, 0.15)]) == []
    assert t.discs() == []
    assert t.add((0, 0, 0), [crater_det(0.61, 0.01, 0.15)])
    for _ in range(5):
        t.add((0.1, 0.0, 0.0), [crater_det(0.51, 0.0, 0.15)])
    assert len(t.discs()) == 1


def test_a_better_view_of_a_crater_replaces_a_poor_one():
    t = nav_core.TerrainMap()
    t.add((0, 0, 0), [crater_det(0.9, 0.0, 0.25, span=1.2)])      # glimpse: wrong radius
    t.add((0, 0, 0), [crater_det(0.9, 0.0, 0.25, span=1.2)])
    t.add((0.4, 0, 0), [crater_det(0.5, 0.0, 0.13, span=5.0)])    # full ring
    (x, y, r), = t.discs()
    assert r == pytest.approx(0.16, abs=1e-6) and x == pytest.approx(0.9, abs=1e-6)


def test_rock_cells_are_confirmed_and_found_by_blocking():
    t = nav_core.TerrainMap()
    cells = [[0.6, 0.0], [0.64, 0.0], [0.6, 0.04]]
    assert t.add((0, 0, 0), [rock_det(cells)]) == []                # first sighting: not yet
    assert t.blocking(0.6, 0.0) is None
    ch = t.add((0, 0, 0), [rock_det(cells)])
    assert ch and ch[0][0] == "cells" and len(ch[0][2]) == 3
    assert t.blocking(0.61, 0.01) == "rock" and t.blocking(1.5, 1.5) is None
    assert t.blocking(0.9, 0.0, pad=0.35) == "rock"                 # within the pad


def test_snapshot_lists_craters_and_cells_for_the_web_page():
    t = nav_core.TerrainMap()
    for _ in range(2):
        t.add((0, 0, 0), [crater_det(0.6, 0.1, 0.15), rock_det([[1.0, 1.0]]),
                          {"type": "border", "cells": [[1.5, 0.0]]}])
    s = t.snapshot(clearance=0.14)
    assert s["craters"][0]["r"] == pytest.approx(0.18) and s["clearance"] == 0.14
    assert s["rock"] == [[1.0, 1.0]] and s["border"] == [[1.5, 0.0]]
    json.dumps(s)                                                   # must be serialisable


def test_bump_mark_is_a_confirmed_disc_and_not_duplicated():
    t = nav_core.TerrainMap()
    assert t.add_mark(1.0, 1.0, 0.2) == ("crater", 1.0, 1.0, 0.2)
    assert t.add_mark(1.02, 1.0, 0.2) is None
    assert t.blocking(1.1, 1.0) == "crater"


def test_detections_off_the_table_are_dropped_when_the_arena_is_known():
    t = nav_core.TerrainMap(bounds=(0.0, 0.0, 2.21, 2.21))
    for _ in range(3):
        t.add((2.0, 1.0, 0.0), [rock_det([[0.1, 0.0], [0.5, 0.0]]), crater_det(0.6, 0.0, 0.15)])
    pts = t.points(['rock'])
    assert pts == [(2.1, 1.0)] or len(pts) == 1 and pts[0][0] < 2.21     # the cell at x=2.5 is gone
    assert t.discs() == []                                              # crater centre off the table


def test_discs_near_path_checks_only_the_path_still_ahead():
    wps = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert nav_core.discs_near_path([(1.1, 0.5, 0.1)], (0.9, 0.0), wps, 2, 0.1)
    assert not nav_core.discs_near_path([(0.3, 0.0, 0.1)], (0.9, 0.0), wps, 2, 0.1)


# ── BumpDetector ─────────────────────────────────────────────────────────────
def feed(b, series, dt=0.02, driving=True):
    t, out = 0.0, []
    for pitch, roll in series:
        t += dt
        ev = b.update(t, math.radians(pitch), math.radians(roll), driving)
        if ev:
            out.append((round(t, 2), ev))
    return out


def test_bump_detector_fires_on_a_short_nose_up_jolt():
    b = nav_core.BumpDetector(pitch_deg=2.5)
    series = [(0.3, 0.1)] * 100 + [(4.0, 0.1)] * 15 + [(0.3, 0.1)] * 50
    ev = feed(b, series)
    assert len(ev) == 1 and ev[0][1]["kind"] == "nose_up"
    assert ev[0][0] < 2.0 + 0.1                                       # within a couple of samples


def test_bump_detector_ignores_noise_and_slow_ramps():
    b = nav_core.BumpDetector(pitch_deg=2.5)
    noise = [(0.3 + 0.4 * math.sin(i), 0.1) for i in range(300)]
    ramp = [(0.3 + 12.0 * i / 1000.0, 0.1) for i in range(1000)]      # up to 12 deg over 20 s
    assert feed(b, noise) == []
    assert feed(nav_core.BumpDetector(pitch_deg=2.5), ramp) == []


def test_bump_detector_needs_driving_and_has_a_holdoff():
    b = nav_core.BumpDetector(pitch_deg=2.5)
    assert feed(b, [(0.0, 0.0)] * 50 + [(5.0, 0.0)] * 10, driving=False) == []
    b2 = nav_core.BumpDetector(pitch_deg=2.5, holdoff=3.0)
    jolts = [(0.0, 0.0)] * 50 + [(5.0, 0.0)] * 10 + [(0.0, 0.0)] * 10 + [(5.0, 0.0)] * 10
    assert len(feed(b2, jolts)) == 1                                  # the second is in the holdoff


def test_bump_detector_reports_roll_and_nose_down_and_can_be_switched_off():
    ev = feed(nav_core.BumpDetector(pitch_deg=2.5, roll_deg=3.0), [(0, 0)] * 50 + [(0, 6.0)] * 6)
    assert ev and ev[0][1]["kind"] == "roll"
    ev = feed(nav_core.BumpDetector(pitch_deg=2.5), [(0, 0)] * 50 + [(-5.0, 0)] * 6)
    assert ev and ev[0][1]["kind"] == "nose_down"
    assert feed(nav_core.BumpDetector(pitch_deg=0.0), [(0, 0)] * 50 + [(9.0, 0)] * 6) == []


# ── planning grid additions ──────────────────────────────────────────────────
def test_arena_grid_blocks_outside_and_the_border_strip():
    g = nav_core.PlanningGrid.arena(0.0, 0.0, 2.21, 2.21, resolution=0.05, wall_margin=0.04,
                                    robot_radius=0.12)
    assert g.is_blocked(*g.world_to_cell(-0.2, 1.0))                  # off the table
    assert g.is_blocked(*g.world_to_cell(0.10, 1.0))                  # robot radius from the edge
    assert not g.is_blocked(*g.world_to_cell(0.25, 1.0))
    assert not g.is_blocked(*g.world_to_cell(1.1, 1.1))
    r = nav_core.plan(g, "astar", (0.25, 0.25), (1.95, 1.95))
    assert r is not None
    assert all(0.1 < x < 2.1 and 0.1 < y < 2.1 for x, y in r.points)


def test_add_disc_blocks_exactly_the_disc():
    g = nav_core.PlanningGrid.empty(1.0, 1.0, size=2.0, resolution=0.05)
    g.add_disc(1.0, 1.0, 0.2)
    assert g.is_blocked(*g.world_to_cell(1.15, 1.0)) and not g.is_blocked(*g.world_to_cell(1.3, 1.0))


def test_soft_margin_keeps_the_path_off_the_wall_when_there_is_room():
    def build(soft):
        g = nav_core.PlanningGrid.empty(1.0, 1.0, size=2.0, resolution=0.05)
        g.add_disc(1.0, 1.0, 0.25)
        if soft:
            g.add_soft_margin(0.25, 1.5)
        return g
    hard = nav_core.plan(build(False), "astar", (0.2, 1.0), (1.8, 1.0))
    soft = nav_core.plan(build(True), "astar", (0.2, 1.0), (1.8, 1.0))
    near = lambda r: min(math.hypot(x - 1.0, y - 1.0) for x, y in r.points)   # noqa: E731
    assert near(soft) > near(hard)
    # ... but a corridor that is only just wide enough is still usable
    g = nav_core.PlanningGrid.empty(1.0, 1.0, size=2.0, resolution=0.05)
    g.add_disc(1.0, 0.0, 0.8)
    g.add_disc(1.0, 2.0, 0.8)
    g.add_soft_margin(0.25, 1.5)
    assert nav_core.plan(g, "astar", (0.2, 1.0), (1.8, 1.0)) is not None


def test_soft_margin_does_not_change_the_reported_numbers_of_an_open_field():
    g = nav_core.PlanningGrid.empty(1.0, 1.0, size=2.0, resolution=0.05)
    base = nav_core.plan(g, "astar", (0.2, 1.0), (1.8, 1.0))
    g.add_soft_margin(0.2, 1.5)
    soft = nav_core.plan(g, "astar", (0.2, 1.0), (1.8, 1.0))
    assert soft.energy == pytest.approx(base.energy)


def test_planners_agree_on_a_grid_with_soft_margin():
    g = nav_core.PlanningGrid.empty(1.0, 1.0, size=2.0, resolution=0.05)
    g.add_disc(1.0, 1.0, 0.3)
    g.add_soft_margin(0.2, 1.5)
    for algo in nav_core.ALGORITHMS:
        assert nav_core.plan(g, algo, (0.2, 1.0), (1.8, 1.0)) is not None


# ── make_arena_map.py ────────────────────────────────────────────────────────
LAYOUT = {"units": "in", "size": 87, "border_width": 1.2,
          "craters": [{"x": 23, "y": 27, "diameter": 11}],
          "rocks": [{"x": 49, "y": 51, "width": 12, "depth": 12},
                    {"x": 68, "y": 62, "width": 13, "depth": 11, "rotation_deg": 45}]}


def test_map_has_border_craters_rocks_and_blocked_outside(tmp_path):
    pgm, yml = mam.write_map(LAYOUT, str(tmp_path / "arena"), resolution=0.01, margin=0.2)
    w, h, res, ox, oy, data = mam.read_map(yml)
    assert res == 0.01 and (ox, oy) == pytest.approx((-0.2, -0.2))
    side = 87 * 0.0254

    def occ(x, y):
        return data[int((y - oy) / res) * w + int((x - ox) / res)] == 100
    assert occ(-0.1, 1.0) and occ(side + 0.1, 1.0)                    # outside the table
    assert occ(0.01, 1.0) and occ(1.0, side - 0.01)                   # border band
    assert not occ(0.10, 1.0) and not occ(1.1, 1.1)                   # open floor
    assert occ(23 * 0.0254, 27 * 0.0254)                              # crater centre: inside counts
    assert occ((23 + 5) * 0.0254, 27 * 0.0254) and not occ((23 + 8) * 0.0254, 27 * 0.0254)
    assert occ(49 * 0.0254, 51 * 0.0254) and not occ(49 * 0.0254, 60 * 0.0254)
    # the 45 degree rock is a diamond, not an axis-aligned square
    cx, cy = 68 * 0.0254, 62 * 0.0254
    assert occ(cx, cy) and not occ(cx + 0.14, cy + 0.14) and occ(cx + 0.14, cy)


def test_map_yaml_is_what_map_server_expects(tmp_path):
    _, yml = mam.write_map(LAYOUT, str(tmp_path / "a"))
    text = open(yml).read()
    for key in ("image: a.pgm", "resolution: 0.01", "origin: [-0.2000, -0.2000, 0.0]", "negate: 0",
                "occupied_thresh: 0.65", "free_thresh: 0.196"):
        assert key in text
    raw = open(str(tmp_path / "a.pgm"), "rb").read()
    assert raw.startswith(b"P5")


def test_layout_outside_the_arena_is_rejected():
    with pytest.raises(ValueError):
        mam.normalise({"size": 2.0, "craters": [{"x": 3.0, "y": 1.0, "diameter": 0.3}]})


def test_example_layout_loads_with_comments_and_is_passable(tmp_path):
    layout = mam.load_layout(os.path.join(CONFIG, "arena_layout_example.json"))
    lay = mam.normalise(layout)
    assert lay["width"] == pytest.approx(2.2098) and len(lay["craters"]) == 3 and len(lay["rocks"]) == 4
    grid = compare_plans.grid_from_layout(layout, 0.10, 0.015, 0.05)
    assert nav_core.plan(grid, "astar", (0.3, 0.3), (1.9, 1.9)) is not None


def test_map_round_trips_into_the_planner(tmp_path):
    """The generated map, read back like map_server serves it, plans without crossing objects."""
    _, yml = mam.write_map(LAYOUT, str(tmp_path / "m"), resolution=0.01)
    w, h, res, ox, oy, data = mam.read_map(yml)
    grid = nav_core.PlanningGrid.from_occupancy(w, h, res, ox, oy, data, plan_resolution=0.05,
                                                robot_radius=0.12)
    r = nav_core.plan(grid, "astar", (0.3, 0.3), (1.9, 1.9))
    assert r is not None
    lay = mam.normalise(LAYOUT)
    for x, y in r.points:
        assert not any(mam.in_octagon(x, y, c) for c in lay["craters"])
        assert not any(mam.in_rect(x, y, k) for k in lay["rocks"])
        assert 0.1 < x < lay["width"] - 0.1 and 0.1 < y < lay["height"] - 0.1


def test_make_arena_map_cli(tmp_path, capsys):
    lay = tmp_path / "l.json"
    lay.write_text(json.dumps(LAYOUT))
    assert mam.main([str(lay), "--out", str(tmp_path / "o"), "--resolution", "0.02", "--preview"]) == 0
    out = capsys.readouterr().out
    assert "3 craters" not in out and "1 craters, 2 rocks" in out and "#" in out
    assert os.path.exists(str(tmp_path / "o.pgm")) and os.path.exists(str(tmp_path / "o.yaml"))


# ── compare_plans.py / compare_runs.py ───────────────────────────────────────
def test_compare_plans_prints_all_three_planners(capsys):
    layout = mam.load_layout(os.path.join(CONFIG, "arena_layout_example.json"))
    lay_path = os.path.join(CONFIG, "arena_layout_example.json")
    assert compare_plans.main([lay_path, "--start", "0.3,0.3", "--goal", "1.9,1.9",
                               "--robot-radius", "0.10", "--margin", "0.015"]) == 0
    out = capsys.readouterr().out
    for a in nav_core.ALGORITHMS:
        assert a in out
    assert layout["size"] == 87


def test_compare_plans_says_when_the_gap_is_too_narrow(capsys):
    lay_path = os.path.join(CONFIG, "arena_layout_example.json")
    compare_plans.main([lay_path, "--start", "0.3,0.3", "--goal", "1.9,1.9", "--robot-radius", "0.5"])
    assert "no path" in capsys.readouterr().out


def test_compare_runs_averages_per_algorithm(tmp_path, capsys):
    csv_path = tmp_path / "runs.csv"
    csv_path.write_text(
        "time,algorithm,outcome,planned_energy,planned_distance_m,planned_climb_m,plan_runtime_ms,"
        "nodes,measured_energy_J,driven_distance_m,measured_climb_m,measured_descent_m,drive_time_s\n"
        "2026-11-12 10:00:00,astar,reached,3.0,3.0,0,5,100,50,3.1,0,0,40\n"
        "2026-11-12 10:05:00,astar,reached,3.0,3.0,0,5,100,70,3.3,0,0,44\n"
        "2026-11-12 10:10:00,qaoa,reached,3.2,3.2,0,9,300,80,3.4,0,0,50\n"
        "2026-11-12 10:15:00,dijkstra,aborted_edge,3.0,3.0,0,2,90,10,0.5,0,0,8\n")
    assert compare_runs.main(["--csv", str(csv_path)]) == 0
    out = capsys.readouterr().out
    assert "astar" in out and "60.0" in out and "Lowest measured energy" in out
    lines = compare_runs.summarise([
        {"algorithm": "a", "outcome": "reached", "measured_energy_J": "10", "planned_energy": "x"}])
    assert lines[0]["measured_energy_J"] == 10.0 and math.isnan(lines[0]["planned_energy"])


def test_hill_file_makes_the_energy_aware_planners_choose_another_route(capsys):
    """Ramp / elevation demo: over the hill Dijkstra goes straight, A* goes round it."""
    lay = {"units": "m", "size": 2.21, "border_width": 0.03}
    elev = nav_core.load_elevation_csv(os.path.join(CONFIG, "hill_example.csv"), 0.1, 0.0, 0.0)
    grid = compare_plans.grid_from_layout(lay, 0.1, 0.015, 0.05, elevation=elev)
    d = nav_core.plan(grid, "dijkstra", (0.3, 0.3), (1.9, 1.9))
    a = nav_core.plan(grid, "astar", (0.3, 0.3), (1.9, 1.9))
    assert a.climb < d.climb and a.energy < d.energy and a.distance > d.distance
    assert compare_plans.main([os.path.join(CONFIG, "arena_layout_example.json"), "--start", "0.3,0.3",
                               "--goal", "1.9,1.9", "--robot-radius", "0.10", "--margin", "0.015",
                               "--elevation", os.path.join(CONFIG, "hill_example.csv")]) == 0


# ── Python 3.6 compatibility of every robot script ───────────────────────────
@pytest.mark.parametrize("name", ["nav_core.py", "quantum_nav_node.py", "crater_core.py",
                                  "terrain_core.py", "crater_detector_node.py", "make_arena_map.py",
                                  "compare_plans.py", "compare_runs.py", "drive_test.py"])
def test_robot_scripts_parse_as_python36(name):
    src = open(os.path.join(SCRIPTS, name), encoding="utf-8").read()
    tree = ast.parse(src, feature_version=(3, 6))
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            assert node.module != "dataclasses", name
        assert type(node).__name__ != "NamedExpr", name
    assert "breakpoint(" not in src
