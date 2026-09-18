# Progress

What has been built and verified, what is assumed, and where the next agent
picks up. Guidelines, setup, architecture and the forward plan live in
[`../README.md`](../README.md).

Last verified in simulation: **2026-09-10** (Phases 1-5), Ubuntu 24.04 + ROS 2 Jazzy +
Gazebo Harmonic 8.15 (WSL2). Phase 10 was added on **2026-09-18** on a Windows box with no
ROS 2: its pure-Python core and plots are verified, its two ROS nodes are **not yet run**.

---

## 1. Status by phase

| # | Phase | State | Evidence |
|---|---|---|---|
| 1 | Planning | **done** | repo/workspace layout, `requirements.txt`, architecture (README §1) |
| 2 | Environment | **done** | ROS 2 Jazzy + Gazebo Harmonic 8.15 build & run on the target box |
| 3 | Simulation | **done, verified** | `simulation_launch.py` brings up gz server + bridge + RSP; rover drives on `/cmd_vel` (measured −2 m → +11 m); `/odom`, `/tf`, `/joint_states` bridged |
| 4 | Sensors | **done, verified** | `/scan` 10 Hz (`frame_id: lidar_link`, finite ranges), `/imu/data` 50 Hz; `lidar_processor` / `imu_processor` / `encoder_processor` run; odometry uses real diff-drive kinematics (unit-tested) |
| 5 | Mapping | **done, verified** | `occupancy_grid_node` → `/map` 600×600 @ 0.05 m; `terrain_classifier_node` → `/terrain_map` + markers; `results/terrain_maps/latest.npz` written; renders in RViz; 13 pure-Python unit tests pass |
| 6 | Energy modeling | **done, verified** | `planning_pkg.graph_model` (load/coarsen/sample_elevation/build_graph/save_graph/load_graph + CLI); 13 pure-Python unit tests; run against a real Gazebo-produced `latest.npz` → 94 nodes, 266 edges, weight range 0.50–1.19 |
| 7 | Classical planning | **done** | `planning_pkg.classical_planning` + `classical_planner_node` (Dijkstra, A*); ran on the Ubuntu box against a real graph; 14 tests |
| 8 | Quantum optimization | **done, verified** | `planning_pkg.quantum_optimizer` (corridor reduction, QUBO, Ising conversion, QAOA via Qiskit Aer, unit tests, ROS 2 node publishing `/path/quantum`) |
| 9 | Integration | **done; controller unit-tested, live drive not re-run since the executor refactor** | `navigation_pkg.path_following` (control law, 13 tests incl. a closed-loop drive) + `path_executor_node` (`path_executor` & `quantum_path_executor`), `navigation_pipeline.launch.py` connecting planner to rover `/cmd_vel` |
| 10 | Evaluation | **core done & verified; ROS nodes written, not yet run in sim** | `evaluation_pkg.metrics` / `energy_model` + `scripts/plot_energy_comparison.py` run on the real Phase 7/8 results and graph (33 unit tests); `battery_monitor_node` / `evaluator_node` / `evaluation.launch.py` only smoke-tested against stubbed `rclpy` |
| 11–12 | Real robot → Docs | **not started** | - |

Pure-Python check (runs anywhere, no ROS):

```bash
python3 -m pytest -q            # 91 passed (5 ROS/Gazebo tests deselected)
python3 -m compileall -q src scripts tests
```

---

## 2. What was built / changed (this line of work)

### Simulation — the rover gained sensors and a drivable ground

Before this work the simulation had **no LiDAR and no IMU** (neither in the world
model nor the URDF, nor bridged), and the terrain had **no working collision**, so
the mapping pipeline had no input and the rover could not move.

- `src/rover_simulation/worlds/lunar_terrain.world`
  - Added `gz-sim-imu-system` to the world plugins.
  - Added a `gpu_lidar` sensor to the inlined rover's `base_footprint` link:
    360 samples over ±π, 0.12–12 m, 10 Hz, `<gz_frame_id>lidar_link</gz_frame_id>`,
    `<topic>scan</topic>`. (`gpu_lidar` is the only ray sensor in Harmonic; it
    renders offscreen via `gz-sim-sensors-system`.)
  - Added an `imu` sensor (50 Hz, `<topic>imu/data</topic>`,
    `<gz_frame_id>imu_link</gz_frame_id>`).
  - **Terrain collision:** was `<heightmap>`, which DART silently ignores → wheels
    spun with no traction. A triangle-mesh collision (from
    `scripts/heightmap_to_collision_obj.py`) **segfaults DART+ODE**
    (`OdeMesh::fillArrays`). Final fix: a flat `<box>` collision (300×300×0.5,
    top at z 0.375 m — the visual terrain height along the flat spawn corridor),
    μ 0.9. The **visual** was flattened to a matching `<plane>` at z 0.375 so the
    rover does not appear to fly where the old relief dipped. Rock models keep
    their own collisions.
  - Rover spawn nudged `z 0.38 → 0.42`.
- `src/rover_simulation/urdf/rover.urdf.xacro` — added `imu_link` + `imu_joint`
  and mirrored `<gazebo>` sensor blocks (kept in sync for a future `gz sdf`
  regeneration; **not used at runtime** — the world file is authoritative).
- `src/rover_simulation/config/bridge.yaml` — added `GZ_TO_ROS` bridges for
  `/scan` (`LaserScan`) and `/imu/data` (`Imu`).

### `sensor_pkg` — encoder odometry rewrite

- New `src/sensor_pkg/sensor_pkg/diff_drive_odometry.py` — pure kinematics:
  `integrate_pose()` (midpoint differential-drive), `wrap_angle()`,
  `yaw_to_quaternion()`.
- `encoder_processor.py` rewritten: wheel-joint-angle deltas × `wheel_radius`
  → arc length; heading `theta` updated from `(d_right − d_left) / wheel_separation`;
  real yaw quaternion; `twist` filled; joint names are parameters (averages the
  three left / three right `*_joint`s). Previously `theta` never updated and the
  deltas were treated as metres, so `/odom` was wrong on every turn and ~5.5×
  short.
- `tests/test_diff_drive_odometry.py` — 7 tests (straight line, in-place spin,
  midpoint step, θ wrap, quarter-circle accumulation, quaternion values).

### `mapping_pkg` — no code changes

The Phase 5 nodes (`occupancy_grid_node`, `terrain_classifier_node`,
`grid_utils.py`, `terrain_storage.py`) were already correct; they were only
starved of `/scan`. They now run end to end.

### planning_pkg — Phase 6 energy-weighted graph

New `src/planning_pkg/planning_pkg/graph_model.py` — a pure-Python module (no
`rclpy`) that turns a Phase-5 terrain snapshot into an energy-weighted
NetworkX graph for Phases 7/8 to consume:

- `load_terrain_map(npz_path)` — reads `results/terrain_maps/latest.npz`
  (same fields `mapping_pkg.terrain_storage.TerrainStorage` writes).
- `coarsen(grid, cell_stride)` — aggregates the 600×600 @ 0.05 m occupancy
  grid into `cell_stride × cell_stride` blocks (default stride 10 → 0.5 m
  spacing), one value per block: any obstacle cell in the block → block is
  obstacle (safety-first, never averaged away); else majority `-1` → block is
  unknown; else majority vote among `{10, 50, 90}`, ties broken toward the
  higher-cost (safer) class.
- `sample_elevation(heightmap_png_path, world_x, world_y)` — nearest-pixel
  sample of `worlds/heightmap.png` (world x, y ∈ [-75, 75] over the image),
  `z = (pixel16 / 65535) * 10.0 - 5.0`, matching
  `scripts/heightmap_to_collision_obj.py`. Used because the live sim terrain
  is currently flat (DART heightmap-collision limitation, see §3 issue 2) —
  `heightmap.png` remains the elevation source of record independent of what
  physics collision is active.
- `build_graph(...)` — one node per traversable block (`pos_x`, `pos_y`,
  `terrain_class`, `terrain_factor`), 8-connected edges between traversable
  neighbours, weight `E = d * S(theta) * T_avg` where `d` is block-centre
  distance, `S(theta) = 1.0 + 2.0 * |sin(theta)|` with
  `theta = atan2(|z_j - z_i|, d)`, and `T_avg` is the mean of the two
  endpoint terrain factors (`{10: 1.0, 50: 1.5, 90: 3.0}`). No separate
  roughness multiplier — the terrain class already encodes Phase 5's
  roughness/crater signal, so a second multiplier would double-count it.
- `save_graph(G, out_dir, meta)` / `load_graph(graphml_path)` — file-based
  output, `results/graphs/latest.graphml` (NetworkX native) +
  `results/graphs/latest_meta.json` (node/edge counts, weight min/max/mean,
  resolution, cell stride, source timestamp, generation timestamp), mirroring
  `terrain_storage.py`'s `latest.npz`/`latest_meta.json` pattern. No custom
  ROS message — Phase 7/8 just load the file.
- CLI: `python -m planning_pkg.graph_model --npz <path> --heightmap <path>
  --out <dir> [--cell-stride N]`. Runs standalone, no ROS2 required.

`requirements.txt` gained `Pillow>=10.0.0` (already an undeclared transitive
dependency via `scripts/heightmap_to_collision_obj.py`); `planning_pkg/package.xml`
gained `exec_depend`s on `python3-networkx`, `python3-numpy`, `python3-pil`.
13 new unit tests in `tests/test_graph_model.py` (coarsen majority/obstacle/
unknown/tie rules, elevation sampling + edge clamping, graph node exclusion +
weight formula + attrs, save/load round-trip including the empty-graph case,
CLI end-to-end). Verified with a synthetic `.npz` smoke test (60×60 grid,
stride 5, `worlds/heightmap.png`) since this dev machine has no real
Gazebo-produced `results/terrain_maps/latest.npz`.

### planning_pkg — Phase 7 classical path planning (Dijkstra + A*)

New `src/planning_pkg/planning_pkg/classical_planning.py` and `classical_planner_node.py`:
- `classical_planning.py` (pure Python, zero `rclpy` imports):
  - `load_weighted_graph(path)`: Reuses Phase 6 `graph_model.load_graph`.
  - `resolve_node(graph, x, y)`: Snaps continuous world coordinates to nearest graph node via Euclidean distance with a 5× bounding box threshold sanity check.
  - `run_dijkstra(graph, start, goal)`: Uniform-cost exact shortest path via `networkx.dijkstra_path`.
  - `run_astar(graph, start, goal)`: Heuristic search via `networkx.astar_path` using straight-line Euclidean distance heuristic $h(u) = \|u - \text{goal}\|_2$ (admissible and consistent because all edge energy multipliers $S(\theta) \ge 1.0$, $T_{avg} \ge 1.0$).
  - Catches `networkx.NetworkXNoPath` and raises descriptive `ValueError` naming start/goal.
  - `PlanResult` dataclass storing algorithm, path nodes, world coordinates, energy, distance, waypoints, runtime, timestamp.
  - `compare()`: Generates JSON-serializable comparison dictionary (energy delta, distance delta, runtime delta, `paths_identical`).
  - `save_plan_results()`: Writes `results/paths/latest_classical.json` + timestamped history.
  - Standalone CLI: `python -m planning_pkg.classical_planning --start-x <x> --start-y <y> --goal-x <gx> --goal-y <gy> [--graph <path>] [--out <dir>]`.
- `classical_planner_node.py` (ROS 2 `rclpy` node wrapper):
  - Publishes canonical A* path on `/path/classical` (`nav_msgs/msg/Path`) with `TransientLocal` QoS for late subscribers.
  - Publishes debug Dijkstra path on `/path/classical/dijkstra` (`nav_msgs/msg/Path`) for RViz comparison.
  - Declares parameters: `graph_path`, `start_x`, `start_y`, `goal_x`, `goal_y`, `frame_id`, `out_dir`, `save_results`.
  - Logs one-line human-readable summary at INFO level.
- `launch/classical_planner.launch.py`: Launch description for node and arguments.
- 14 new pure-Python unit tests in `tests/test_classical_planning.py` (40 total passing).

### evaluation_pkg - Phase 10 evaluation

Pure-Python core (no `rclpy`), thin ROS wrappers, file-based I/O like Phases 6-8.

- `evaluation_pkg/metrics.py`
  - `load_planner_metrics(classical_json, quantum_json)` -> `PlannerMetrics` for
    `dijkstra`, `astar`, `qaoa` (node ids normalised, `[29, 33]` -> `"(29, 33)"`).
  - `distance_only_baseline(graph, start, goal)` - shortest path by Euclidean edge
    length, then **re-priced with the real energy weights**: the energy-unaware planner
    the energy-aware ones are compared against.
  - `baseline_sweep(graph, n_pairs, seed, min_separation_m)` - seeded random start/goal
    pairs in the largest connected component; energy-optimal vs distance-only per pair,
    with a summary (mean / median / max saving, routes that differ).
  - `build_comparison` / `save_comparison` -> `results/energy_comparison/latest_comparison.json`
    (+ timestamped copy). Summary flags `qaoa_matches_classical` (tolerance 1e-6*max(1,E)),
    `qaoa_energy_ratio`, `qaoa_fallback`, `qaoa_runtime_ratio_vs_astar`.
  - `execution_report` + `ExecutionMonitor` - planned vs actual distance/energy for one
    drive; goal detection; the tally freezes on arrival.
  - CLI: `PYTHONPATH=src/evaluation_pkg python3 -m evaluation_pkg.metrics [--sweep N] ...`
- `evaluation_pkg/energy_model.py` - `BatteryModel` (`P = P_idle + k_lin*|v| + k_ang*|w|`,
  integrated over `dt`; SoC clamped to [0, 1]) and `DistanceTracker`.
- `evaluation_pkg/battery_monitor_node.py` - `/cmd_vel` + `/odom` -> `/battery/status`
  (`sensor_msgs/BatteryState`). A `/cmd_vel` older than `cmd_timeout_s` counts as idle.
- `evaluation_pkg/evaluator_node.py` - loads one planner's path from the comparison file,
  follows `/odom` + `/battery/status`, publishes JSON on `/metrics` at 1 Hz and writes
  `results/energy_comparison/latest_execution.json` on arrival. Fails at start-up if the
  comparison file or the planner is missing.
- `launch/evaluation.launch.py` (`planner`, `comparison_path`, `goal_tolerance`,
  `capacity_wh`, `use_sim_time`), console scripts registered in `setup.py`, deps added to
  `package.xml`.
- `scripts/plot_energy_comparison.py` - `energy_comparison.png` (energy / distance /
  runtime per planner), `route_sweep.png` (savings histogram), `path_overlay.png`
  (paths on the graph + the biggest-saving sweep route), `comparison_table.csv`;
  `--theme light|dark`. Palette checked for colour-vision deficiency; the CSV is the
  non-colour view.
- Tests: `tests/test_battery_model.py` (12), `tests/test_evaluation_metrics.py` (21).

**Result on the real project data (be honest about this):**

| | energy | distance | runtime |
|---|---|---|---|
| Dijkstra | 2.828 | 2.83 m | 1.0 ms |
| A* | 2.828 | 2.83 m | 0.25 ms |
| QAOA (simulated) | 2.828 | 2.83 m | 5 554 ms |
| distance-only | 2.828 | 2.83 m | 1.1 ms |

QAOA **matches** the classical optimum (ratio 1.00, no fallback) and is ~2x10^4 times
slower; it does not beat A*. Energy-aware planning saves 0 % on this route; over a 200-route
sweep it saves mean **0.24 %**, max **8.5 %** (109 of 200 routes differ from the
shortest-distance path). The savings are small because the current graph is almost entirely
flat class-10 cells, with slope from the heightmap as the only weight variation (issues 1-2).

### Codebase cleanup

Removed dead/superseded: `scratch/`, `src/rover_simulation/models/` (world inlines
its own model), `rover_simulation/setup.py` + `pybullet_sim.py` + `odom_to_tf.py`
+ `rover_teleop.py`, unused terrain/rock textures, `generate_terrain_mesh.py` /
`generate_textures.py` / `check_scipy.py` / `drive_test.sh` / `test_physics.sh`,
the pre-migration roadmap HTML, Humble-era teammate scratch files, per-directory
READMEs (folded into README.md + this file), `gazebo_params.yaml`, doubled
shebangs, and commented cruft in `rover_simulation/CMakeLists.txt` /
`package.xml`.

---

## 3. Known issues

1. **Terrain classifier over-triggers craters.** A representative run gave
   `/terrain_map` counts `flat 9 325 / rocky 0 / crater_interior 161 440 /
   obstacle 5 792` — ~87 % of known cells labelled crater, "rocky" never fires.
   The pipeline is correct; the parameters in
   `src/mapping_pkg/config/mapping_params.yaml` are wrong for this terrain.
   Suggested direction: `hough_param2` 20 → ~35–40, `crater_max_radius_px` 80 →
   ~25, `hough_min_dist` 15 → ~40, require rim support before flooding an
   interior; `roughness_threshold` 0.02 → lower, and check the variance window.
   Needs edit → relaunch `mapping.launch.py` → re-check `cell_counts` iterations.
2. **Flat terrain.** The heightmap relief is gone (DART limitation, above). The
   2-D LiDAR scans horizontally at rover height, so on flat ground it only
   returns off the rock models — `/map` is free space + rock obstacles, not the
   terrain-shaped returns you would get with relief. Restoring relief = switch
   physics to Bullet and revert the terrain collision/visual to `<heightmap>`
   (untested; Bullet handles the 6-wheel drivetrain differently).
3. **Rock / goal-marker heights.** Rocks and the goal marker were placed for the
   old terrain elevation; on the flat z 0.375 ground a few sit slightly high or
   low. Cosmetic.
4. **Phase 10 ROS nodes are unverified in simulation.** The battery model is an assumption
   (see section 4); `evaluator` counts energy from its first battery sample, so idle time
   before the rover starts moving is included; `execution_efficiency` (= planned / actual
   distance) is only meaningful after arrival.
5. **Phase 9 review findings - fixed 2026-09-18, not yet re-run in simulation.**
   (a) `path_executor` now subscribes to the path with `RELIABLE` + `TRANSIENT_LOCAL`, depth 1,
   matching both planners, so a path published before the executor is up is not missed.
   (b) The control law moved to `navigation_pkg/path_following.py`; `tests/test_path_executor.py`
   (which re-implemented the formulas inside the test) was replaced by
   `tests/test_path_following.py`, which imports the real module. (c) `ROADMAP.md`
   (an AI-assistant prompt), `Implementation.md` (duplicated this file and the README) and
   `PHASE8_README_ADDENDUM.md` were removed; anything unique was folded into the README.
   Verify with one live drive (`navigation_pipeline.launch.py`) that the executor still
   receives the path and reaches the goal.
6. **`<gz_frame_id>` schema warning.** `gz sim` prints
   `XML Element[gz_frame_id] … not defined in SDF` for both sensors. It is
   cosmetic — `/scan` still comes through with `frame_id: lidar_link`.

---

## 4. Assumptions & constraints that shaped the work

See README §7 for the full list. The ones that mattered here:

- The dev machine used for editing is **Windows with no ROS 2 / Gazebo**;
  simulation changes are written + XML-validated there and verified on a separate
  Ubuntu box. Only pure-Python (`pytest`, `compileall`) runs on Windows.
- There are **two working copies**: the maintained tree, and an older
  `~/lunar-rover-quantum` clone on the Linux box that predates this branch (it
  has a `tests/test_mapping.py` importing a non-existent `mapping_pkg.occupancy_mapper`).
  Do not test against the old clone.
- DART ignores `<heightmap>` collision; DART+ODE trimesh collision segfaults.
- `sensor_pkg` ships flat scripts, not an importable package.
- The rover is inlined in the world; the Xacro is TF-only.
- **Phase 7 limitations:** Waypoint orientations in `PoseStamped` are set to identity
  quaternions; heading calculation between waypoints is deferred to Phase 9 `path_executor`.
  Replanning is startup-only; on-demand replanning via action/service will be added in Phase 9.

---

## 5. Next steps

1. **Verify Phase 10 in simulation** (needs the Ubuntu box): run Gazebo + the Phase 9
   pipeline + `ros2 launch evaluation_pkg evaluation.launch.py planner:=<astar|qaoa>`;
   confirm `/battery/status` falls while driving, `/metrics` updates, and
   `latest_execution.json` appears on arrival. Then flip the Phase 10 row to "verified".
2. **Tune the terrain classifier** (issue 1). Small, self-contained, needs the
   running sim. Deliverable: `mapping_params.yaml` values that give a sane
   flat/rocky/crater/obstacle split, plus a note in this file. This is also what would make
   the Phase 10 savings numbers meaningful (a graph that is not almost all flat).
3. **Phase 11 - real robot** (Arjuna kit / Jetson): replace the battery model constants
   with measured values.
4. Re-run the Phase 9 pipeline once after the executor refactor (known issue 5).
5. Optional polish: Bullet physics for contour terrain (issue 2); re-seat rocks
   (issue 3).

---

## 6. Picking this up — quick orientation for the next agent

- **Run the pure-Python tests first** (`python3 -m pytest -q`, 91 pass) — fastest
  confidence check, no ROS needed.
- **To bring the sim up:** README §4–§5. On WSL, always `export
  LIBGL_ALWAYS_SOFTWARE=1` first.
- **Phase 7 Planner entry point:**
  - CLI: `python -m planning_pkg.classical_planning --start-x 0 --start-y 0 --goal-x 10 --goal-y 10`
  - ROS 2: `ros2 launch planning_pkg classical_planner.launch.py goal_x:=10.0 goal_y:=10.0`
- **Phase 10 entry points:**
  - CLI: `PYTHONPATH=src/evaluation_pkg python3 -m evaluation_pkg.metrics --sweep 200`, then
    `python3 scripts/plot_energy_comparison.py`
  - ROS 2: `ros2 launch evaluation_pkg evaluation.launch.py planner:=qaoa`
  - Outputs: `results/energy_comparison/latest_comparison.json`, `latest_execution.json`,
    PNGs + `comparison_table.csv`; topics `/battery/status`, `/metrics`
- **Output contracts:**
  - `results/paths/latest_classical.json` (contains both Dijkstra & A* results + comparison)
  - `/path/classical` (`nav_msgs/msg/Path`, published by A*)
  - `/path/classical/dijkstra` (`nav_msgs/msg/Path`, debug)
- **Design specs:**
  - Phase 6: `docs/superpowers/specs/2026-09-11-phase6-energy-graph-design.md`
  - Phase 7: `docs/superpowers/specs/2026-09-13-phase7-classical-planner-design.md`
  - Phase 10: `docs/superpowers/specs/2026-09-18-phase10-evaluation-design.md`
