#!/usr/bin/env python3
"""Summarise ~/.ros/quantum_nav_runs.csv: one line per algorithm (and outcome).

    python3 compare_runs.py                       # all runs
    python3 compare_runs.py --since "2026-11-12"  # only runs on/after a date (string compare)
    python3 compare_runs.py --csv other.csv

Averages are over the rows of each algorithm / outcome. Only "reached" rows are comparable:
a run that stopped early drove less distance and used less energy. The planned columns come
from the planner (before driving), the measured ones from the drive itself.
"""

import argparse
import csv
import os
import sys

NUM = ["planned_energy", "planned_distance_m", "plan_runtime_ms", "nodes", "measured_energy_J",
       "driven_distance_m", "measured_climb_m", "drive_time_s"]


def summarise(rows):
    groups = {}
    for r in rows:
        groups.setdefault((r["algorithm"], r["outcome"]), []).append(r)
    out = []
    for (algo, outcome), rs in sorted(groups.items()):
        line = {"algorithm": algo, "outcome": outcome, "runs": len(rs)}
        for k in NUM:
            vals = []
            for r in rs:
                try:
                    vals.append(float(r[k]))
                except (KeyError, ValueError, TypeError):
                    pass
            line[k] = sum(vals) / len(vals) if vals else float("nan")
        out.append(line)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", default="~/.ros/quantum_nav_runs.csv")
    ap.add_argument("--since", default="", help='keep rows with time >= this text, e.g. "2026-11-12"')
    args = ap.parse_args(argv)
    path = os.path.expanduser(args.csv)
    if not os.path.exists(path):
        print("No run log at %s yet. Drive a goal first." % path)
        return 1
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r.get("time", "") >= args.since]
    if not rows:
        print("No rows%s." % (" since " + args.since if args.since else ""))
        return 1
    lines = summarise(rows)
    head = "%-9s %-20s %4s %9s %9s %9s %10s %9s %8s" % (
        "algorithm", "outcome", "runs", "plan E", "plan m", "plan ms", "meas J", "drove m", "time s")
    print(head)
    print("-" * len(head))
    for L in lines:
        print("%-9s %-20s %4d %9.2f %9.2f %9.1f %10.1f %9.2f %8.1f" % (
            L["algorithm"], L["outcome"], L["runs"], L["planned_energy"], L["planned_distance_m"],
            L["plan_runtime_ms"], L["measured_energy_J"], L["driven_distance_m"], L["drive_time_s"]))
    reached = [L for L in lines if L["outcome"] == "reached"]
    if len(reached) > 1:
        best = min(reached, key=lambda L: L["measured_energy_J"])
        print("\nLowest measured energy among reached runs: %s (%.1f J)" % (
            best["algorithm"], best["measured_energy_J"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
