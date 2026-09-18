# Phase 10 — Evaluation (energy comparison + battery monitoring) — Design

**Date:** 2026-09-18
**Status:** Approved in chat (scope: full — pure core + plots + ROS nodes).
**Scope:** `evaluation_pkg`: compare Dijkstra / A* / QAOA / a distance-only baseline on
the same graph, plot the comparison, and monitor the rover's actual energy use while it
follows a path.

## Problem

Phases 7 and 8 each write a planning result (`results/paths/latest_classical.json`,
`latest_quantum.json`). Nothing yet compares them, measures how much energy the rover
actually spends executing a path, or produces figures for the report.

## Decisions

1. **Pure-Python core, thin ROS nodes** (same split as every earlier phase). All logic
   lives in `evaluation_pkg/metrics.py` and `evaluation_pkg/energy_model.py`, unit-tested
   on the Windows dev machine. The two ROS nodes only wire topics to that core and can
   only be verified on the Linux/Gazebo box.
2. **Honest framing.** QAOA matched the classical optimum in the Phase 8 run (it does not
   find a cheaper path than A*). The comparison therefore reports equivalence and the
   runtime cost of the quantum solver; any "energy saving" claim is made only against a
   **distance-only baseline** — the shortest path by Euclidean length on the same graph,
   re-priced with the real energy weights. On flat terrain that baseline can coincide with
   the energy-optimal path (saving 0 %); the report must say so rather than imply otherwise.
3. **No custom ROS messages.** `/battery/status` is `sensor_msgs/BatteryState`;
   `/metrics` is a JSON string in `std_msgs/String`.
4. **Battery model is an explicit assumption**, not a hardware calibration:
   `P = P_idle + k_lin·|v| + k_ang·|ω|` (W), `E += P·dt` (J). Defaults are configurable
   parameters (40 Wh, 24 V, 3 W idle, 15 W per m/s, 4 W per rad/s).

## Modules

### `evaluation_pkg/metrics.py` (no rclpy)

- `PlannerMetrics` — name, energy, distance_m, waypoints, runtime_ms, straight_line_m,
  path_efficiency (= straight-line distance / path length), energy_per_m, path_nodes,
  path_coords, extras.
- `load_planner_metrics(classical_json, quantum_json)` → `{"dijkstra", "astar", "qaoa"}`.
- `distance_only_baseline(graph, start, goal)` — Dijkstra on Euclidean edge length, then
  energy = sum of the real `weight` along that path.
- `build_comparison(...)` / `save_comparison(...)` → `results/energy_comparison/latest_comparison.json`
  (+ timestamped history). Summary block: reference planner, best-energy planner,
  `qaoa_matches_classical`, `qaoa_energy_ratio`, `qaoa_fallback`, `qaoa_runtime_ratio_vs_astar`,
  `energy_savings_vs_distance_only_pct`.
- `execution_report(planned, battery)` — planned vs actual: actual distance, joules,
  execution efficiency (= planned distance / actual distance), joules per metre.
- CLI: `python -m evaluation_pkg.metrics --classical … --quantum … [--graph …] [--out …]`.

### `evaluation_pkg/energy_model.py` (no rclpy)

`BatteryParams`, `BatteryModel` (`step`, `soc`, `status`), `DistanceTracker`.

### ROS 2

- `battery_monitor_node` — `/cmd_vel` + `/odom` → `/battery/status`.
- `evaluator_node` — `/battery/status` + `/odom` + latest comparison → `/metrics` (JSON)
  and `results/energy_comparison/latest_evaluation.json`.
- `launch/evaluation.launch.py` starts both.

### `scripts/plot_energy_comparison.py`

Reads `latest_comparison.json` and writes `energy_comparison.png` (energy, distance,
runtime per planner) and `path_overlay.png` (the planned paths over the graph).

## Out of scope

- Calibrating the battery model to the Arjuna hardware (Phase 11).
- Fixing Phase 9 issues (QoS race, tautological executor tests) — reported separately.

## Verification

- `python -m pytest` here (all pure Python).
- CLI + plots run against the real `results/graphs/latest.graphml` and the Phase 7/8 JSON
  already produced on this machine.
- ROS nodes: `py_compile` here; run on the Linux box (steps in `docs/PROGRESS.md`).
