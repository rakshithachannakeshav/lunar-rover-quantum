# Progress

What has been built and verified, what is assumed, and where the next agent
picks up. Guidelines, setup, architecture and the forward plan live in
[`../README.md`](../README.md).

Last verified: **2026-09-10**, Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 8.15
(WSL2).

---

## 1. Status by phase

| # | Phase | State | Evidence |
|---|---|---|---|
| 1 | Planning | **done** | repo/workspace layout, `requirements.txt`, architecture (README §1) |
| 2 | Environment | **done** | ROS 2 Jazzy + Gazebo Harmonic 8.15 build & run on the target box |
| 3 | Simulation | **done, verified** | `simulation_launch.py` brings up gz server + bridge + RSP; rover drives on `/cmd_vel` (measured −2 m → +11 m); `/odom`, `/tf`, `/joint_states` bridged |
| 4 | Sensors | **done, verified** | `/scan` 10 Hz (`frame_id: lidar_link`, finite ranges), `/imu/data` 50 Hz; `lidar_processor` / `imu_processor` / `encoder_processor` run; odometry uses real diff-drive kinematics (unit-tested) |
| 5 | Mapping | **done, verified** | `occupancy_grid_node` → `/map` 600×600 @ 0.05 m; `terrain_classifier_node` → `/terrain_map` + markers; `results/terrain_maps/latest.npz` written; renders in RViz; 13 pure-Python unit tests pass |
| 6 | Energy modeling | **done, verified** | `planning_pkg.graph_model` (load/coarsen/sample_elevation/build_graph/save_graph/load_graph + CLI); 13 new pure-Python unit tests; run against a real Gazebo-produced `latest.npz` → 94 nodes, 266 edges, weight range 0.50–1.19 (min matches the flat/zero-slope closed form exactly; max reflects real heightmap relief) |
| 7–12 | Classical planning → Docs | **not started** | `navigation_pkg`, `evaluation_pkg`, `quantum/` are scaffolding only |

Pure-Python check (runs anywhere, no ROS):

```bash
python3 -m pytest -q            # 26 passed
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
4. **`<gz_frame_id>` schema warning.** `gz sim` prints
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

---

## 5. Next steps

1. **Phase 7 — `classical_planner`** (`planning_pkg`): load
   `results/graphs/latest.graphml` via `graph_model.load_graph`, implement
   Dijkstra + A*, publish/save `/path/classical`.
2. **Tune the terrain classifier** (issue 1). Small, self-contained, needs the
   running sim. Deliverable: `mapping_params.yaml` values that give a sane
   flat/rocky/crater/obstacle split, plus a note in this file.
3. Optional polish: Bullet physics for contour terrain (issue 2); re-seat rocks
   (issue 3).

---

## 6. Picking this up — quick orientation for the next agent

- **Run the pure-Python tests first** (`python3 -m pytest -q`, 26 pass) — fastest
  confidence check, no ROS needed.
- **To bring the sim up:** README §4–§5. On WSL, always `export
  LIBGL_ALWAYS_SOFTWARE=1` first.
- **The authoritative sim definition is `src/rover_simulation/worlds/lunar_terrain.world`**
  (the rover is inlined in it). `rover.urdf.xacro` only feeds
  `robot_state_publisher`; keep sensor blocks mirrored in both but expect the
  world file to be what actually loads.
- **Mapping data contract:** README §6. `planning_pkg` should consume
  `latest.npz` offline, not require a live `/terrain_map`.
- **Design spec** for this line of work:
  `docs/superpowers/specs/2026-09-09-complete-phases-1-5-design.md`.
- **Gotchas:** DART/heightmap (README §7); `sensor_pkg` flat-script imports;
  BEST_EFFORT QoS on `/scan`; the stale `~/lunar-rover-quantum` clone.
