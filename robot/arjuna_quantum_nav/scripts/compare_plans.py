#!/usr/bin/env python3
"""Compare Dijkstra / A* / QAOA on the full arena layout, with no robot (offline).

    python3 compare_plans.py layout.json --start 0.25,0.25 --goal 1.95,1.95
    python3 compare_plans.py layout.json --start 0.25,0.25 --goal 1.95,1.95 --goal 0.3,1.9
    python3 compare_plans.py --map ~/arena_map.yaml --start 0.25,0.25 --goal 1.95,1.95

Plans on the same grid the navigator builds with use_map:=true (map from make_arena_map.py,
robot_radius + crater_margin round every blocked cell) and prints planned energy, length,
search nodes and planning time per algorithm. Use it to check a layout is passable for your
robot_radius before driving, and to see how the planners differ without wearing the batteries.
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import make_arena_map  # noqa: E402
import nav_core  # noqa: E402


def point(text):
    x, y = text.split(",")
    return float(x), float(y)


def load_elevation(args):
    """Optional height model (CSV of heights in metres, as for quantum_nav's elevation_csv)."""
    if not args.elevation:
        return None
    return nav_core.load_elevation_csv(os.path.expanduser(args.elevation), args.elevation_resolution,
                                       args.elevation_origin[0], args.elevation_origin[1])


def grid_from_layout(layout, robot_radius, margin, resolution, map_res=0.01, elevation=None):
    w, h, rows, (ox, oy) = make_arena_map.build_map(layout, map_res, 0.2)
    data = [0] * (w * h)
    for j, row in enumerate(rows):
        for i, v in enumerate(row):
            data[(h - 1 - j) * w + i] = 100 if v == make_arena_map.OCC else 0
    return nav_core.PlanningGrid.from_occupancy(
        w, h, map_res, ox, oy, data, plan_resolution=resolution,
        robot_radius=robot_radius + margin, unknown_is_blocked=True, elevation=elevation)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("layout", nargs="?", help="layout JSON (see make_arena_map.py)")
    ap.add_argument("--map", help="map_server YAML instead of a layout")
    ap.add_argument("--start", type=point, required=True, help="x,y in metres")
    ap.add_argument("--goal", type=point, action="append", required=True,
                    help="x,y in metres; repeat for a route")
    ap.add_argument("--robot-radius", type=float, default=0.12)
    ap.add_argument("--margin", type=float, default=0.02)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--elevation", help="height CSV (a hill or ramp), so the energy-aware planners "
                    "can choose a different route from Dijkstra")
    ap.add_argument("--elevation-resolution", type=float, default=0.1)
    ap.add_argument("--elevation-origin", type=point, default=(0.0, 0.0))
    args = ap.parse_args(argv)

    if args.map:
        w, h, res, ox, oy, data = make_arena_map.read_map(args.map)
        grid = nav_core.PlanningGrid.from_occupancy(
            w, h, res, ox, oy, data, plan_resolution=args.resolution,
            robot_radius=args.robot_radius + args.margin, unknown_is_blocked=True,
            elevation=load_elevation(args))
    elif args.layout:
        grid = grid_from_layout(make_arena_map.load_layout(args.layout), args.robot_radius,
                                args.margin, args.resolution,
                                elevation=load_elevation(args))
    else:
        ap.error("give a layout JSON or --map")

    legs = []
    here = args.start
    for g in args.goal:
        legs.append((here, g))
        here = g
    totals = dict((a, [0.0, 0.0, 0, 0.0, 0]) for a in nav_core.ALGORITHMS)   # E, m, nodes, ms, fails
    for s, g in legs:
        print("leg (%.2f, %.2f) -> (%.2f, %.2f)   straight line %.2f m" % (
            s[0], s[1], g[0], g[1], math.hypot(g[0] - s[0], g[1] - s[1])))
        print("  %-9s %9s %8s %8s %9s" % ("algorithm", "energy", "length m", "nodes", "time ms"))
        for a in nav_core.ALGORITHMS:
            r = nav_core.plan(grid, a, s, g)
            if r is None:
                print("  %-9s no path (a gap is narrower than the robot + margin: lower "
                      "--robot-radius / --margin)" % a)
                totals[a][4] += 1
                continue
            print("  %-9s %9.2f %8.2f %8d %9.1f" % (a, r.energy, r.distance, r.expanded,
                                                   r.runtime_s * 1000))
            t = totals[a]
            t[0] += r.energy
            t[1] += r.distance
            t[2] += r.expanded
            t[3] += r.runtime_s * 1000
    if len(legs) > 1:
        print("\nroute total")
        for a in nav_core.ALGORITHMS:
            t = totals[a]
            print("  %-9s %9.2f %8.2f %8d %9.1f%s" % (a, t[0], t[1], t[2], t[3],
                                                    "   (%d leg(s) failed)" % t[4] if t[4] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
