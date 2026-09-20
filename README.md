# Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation

Final-year major project. An autonomous lunar-rover navigation system that plans
energy-efficient paths across simulated lunar terrain using **both** a classical
graph-search planner and a **quantum optimizer (QAOA)**, so the two can be
compared directly on the same terrain graph.

This file is the **single source of guidelines**: what the system is, how it is
put together, how to set it up and run it, and the plan for the work that is not
done yet. For *what has actually been built and verified so far*, see
[`docs/PROGRESS.md`](docs/PROGRESS.md).

---

## 1. Architecture

Four layers, wired through ROS 2 topics:

```
LAYER 1 — PERCEPTION            (src/sensor_pkg)
  LiDAR   /scan        -> lidar_processor    -> /terrain/pointcloud
  IMU     /imu/data    -> imu_processor      -> /imu/slope
  Wheels  /joint_states-> encoder_processor  -> /odom

LAYER 2 — MAPPING & PLANNING    (src/mapping_pkg, src/planning_pkg)
  /scan + TF           -> occupancy_grid_node    -> /map
  /map + /scan         -> terrain_classifier_node-> /terrain_map (+ markers, .npz)
  terrain_maps/latest.npz -> graph_model (CLI)   -> graphs/latest.graphml (done)
  graphs/latest.graphml   -> classical_planner    -> /path/classical   (done)
  graphs/latest.graphml   -> quantum_optimizer    -> /path/quantum     (Phase 8)

LAYER 3 — EXECUTION             (src/navigation_pkg)
  /path/*              -> path_executor          -> /cmd_vel          (Phase 9)

LAYER 4 — EVALUATION            (src/evaluation_pkg)
  /cmd_vel + /odom     -> battery_monitor        -> /battery/status   (Phase 10)
  /odom + /battery/status + comparison.json -> evaluator -> /metrics  (Phase 10)
  latest_classical/quantum.json + graph -> evaluation_pkg.metrics (CLI) -> comparison.json
  comparison.json      -> scripts/plot_energy_comparison.py -> PNG + CSV
```

### Full node / topic map

| Node | Package | Publishes | Subscribes | Status |
|---|---|---|---|---|
| `lidar_processor` | sensor_pkg | `/terrain/pointcloud`, `/terrain/roughness` | `/scan` | done |
| `imu_processor` | sensor_pkg | `/imu/slope`, `/imu/terrain_status` | `/imu/data` | done |
| `encoder_processor` | sensor_pkg | `/odom` | `/joint_states` | done |
| `occupancy_grid_node` | mapping_pkg | `/map`, static `map->odom` TF | `/scan`, TF | done |
| `terrain_classifier_node` | mapping_pkg | `/terrain_map`, `/terrain_map_markers` | `/map`, `/scan` | done |
| `graph_builder` | planning_pkg | `results/graphs/latest.graphml` (file, offline) | `results/terrain_maps/latest.npz` | done |
| `classical_planner` | planning_pkg | `/path/classical`, `/path/classical/dijkstra` | `results/graphs/latest.graphml` | done |
| `quantum_optimizer` | planning_pkg / quantum | `/path/quantum` | `/odom`, `results/graphs/latest.graphml` | done |
| `path_executor` | navigation_pkg | `/cmd_vel` | `/path/quantum`, `/path/classical`, `/odom` | done |
| `battery_monitor` | evaluation_pkg | `/battery/status` (`sensor_msgs/BatteryState`) | `/cmd_vel`, `/odom` | written, not yet run in sim |
| `evaluator` | evaluation_pkg | `/metrics` (JSON in `std_msgs/String`), `results/energy_comparison/execution_*.json` | `/odom`, `/battery/status`, `results/energy_comparison/latest_comparison.json` | written, not yet run in sim |

---

## 2. Stack

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 LTS ("Noble") |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic (`gz sim`) 8.x, DART physics |
| Python | 3.12 (sim) / any 3.10+ (pure-Python tests) |
| Quantum | Qiskit + Qiskit-Aer (`AerSimulator`, CPU — no real quantum hardware) |
| Graph | NetworkX |
| CV | OpenCV (`python3-opencv`) — used by the terrain classifier |

An earlier iteration targeted ROS 2 Humble / Gazebo Classic 11 / Ubuntu 22.04.
Everything now is Jazzy / Harmonic / 24.04. Wherever an old note says
`ros-humble-*`, the equivalent is `ros-jazzy-*`.

The real-hardware target (a later phase) is the Arjuna AMR kit with a Jetson Nano.

---

## 3. Repository layout

The repo root **is** the ROS 2 workspace — `src/` is at the top, there is no
nested `*_ws/` folder.

```
lunar-rover-quantum/
├── README.md                     # this file — guidelines
├── docs/
│   ├── PROGRESS.md               # what is done + handoff notes for the next agent
│   └── superpowers/              # design specs kept for history
├── src/
│   ├── sensor_pkg/               # LiDAR / IMU / encoder processing (ament_cmake, flat scripts)
│   ├── mapping_pkg/              # occupancy grid + terrain classifier (ament_python)
│   ├── planning_pkg/             # graph builder, energy model, classical planner  (scaffold)
│   ├── navigation_pkg/           # path executor / motor integration              (scaffold)
│   ├── evaluation_pkg/           # battery monitor, comparison metrics            (scaffold)
│   └── rover_simulation/         # Gazebo world, URDF/Xacro, bridge, launch (ament_cmake)
├── quantum/notebooks/            # QAOA / Qiskit experiments (empty)
├── scripts/                      # install + setup + asset-generation scripts
├── tests/                        # pure-Python unit tests (run in CI, no ROS/Gazebo)
├── results/                      # planner outputs, terrain snapshots (gitignored contents)
└── web/                          # standalone Three.js rover viewer (no ROS, runs on Windows)
```

---

## 4. Step 0 — one-time setup

ROS 2 Jazzy + Gazebo Harmonic need **Ubuntu 24.04**. It does not run natively on
Windows — use WSL2, a VM, or dual-boot. Confirm with `cat /etc/os-release`
(22.04 "Jammy" **cannot** install Jazzy).

Rules that avoid the most common failures:

- Work inside the Linux home folder (`~/lunar-rover-quantum`), **not** under
  `/mnt/d/...` (slow, CRLF problems) and **not** in a path containing a space.
- Run every `ros2` command **from the repo root** — nodes read and write `results/...`
  relative to the current directory.
- Never test from an older copy of the repo. If `git log` does not show a recent
  commit, or `ros2 pkg prefix <pkg>` points somewhere else, you are on a stale copy
  (see §9).

### 0.1 Clone

```bash
sudo apt update && sudo apt install -y git build-essential cmake
git clone https://github.com/rakshithachannakeshav/lunar-rover-quantum.git ~/lunar-rover-quantum
cd ~/lunar-rover-quantum
git switch develop
git log --oneline -1          # should be a recent "Merge pull request ..." line
```

`build-essential` provides the C compiler the two CMake packages
(`rover_simulation`, `sensor_pkg`) need; without it the build fails with
`CMAKE_C_COMPILER not set`.

### 0.2 ROS 2, Gazebo and simulation tools (once)

```bash
chmod +x scripts/*.sh
bash scripts/setup_ros2_repo.sh          # ROS 2 apt repo + locale
bash scripts/install_ros2_jazzy.sh       # ROS 2 Jazzy desktop + colcon + rosdep
bash scripts/install_gazebo_harmonic.sh  # Gazebo Harmonic + ros_gz
bash scripts/install_sim_dependencies.sh # xacro, rviz2, nav2, teleop, tf tools
sudo apt install -y ros-jazzy-tf2-ros ros-jazzy-visualization-msgs ros-jazzy-tf2-tools
```

Notes:
- `install_ros2_jazzy.sh` / `install_python_quantum.sh` hard-code `/home/monis/.bashrc`
  for the auto-source line. If your username differs that line silently no-ops — just
  source ROS in each terminal (step 0.5).
- Do not add `source .../install/setup.bash` to `~/.bashrc` for a specific copy of the
  repo; it makes every new terminal use that copy.

### 0.3 Python packages (for the system `python3` that ROS nodes use)

ROS nodes run under `/usr/bin/python3`, so install the packages there (the virtualenv
created by `install_python_quantum.sh` is not visible to `ros2 launch`).

```bash
sudo apt install -y python3-pip python3-pytest python3-numpy python3-scipy \
  python3-networkx python3-matplotlib python3-pil python3-yaml python3-opencv
python3 -m pip install --user --break-system-packages qiskit qiskit-aer
python3 -c "import qiskit, qiskit_aer, networkx, scipy, numpy, cv2, PIL, matplotlib; print('python deps ok')"
```

(Ubuntu 24.04 blocks plain `pip install` — PEP 668 — hence `--break-system-packages`
with `--user`.)

### 0.4 Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

All **6 packages must report `Finished`**, none `Failed`/`Aborted`.

### 0.5 Every new terminal starts with this

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1     # WSL2 / no GPU; needed in the terminal that starts Gazebo
```

### 0.6 Sanity check (no ROS needed)

```bash
python3 -m pytest -q               # all pass; 5 ROS/Gazebo tests are deselected
ros2 pkg prefix mapping_pkg        # must print /home/<you>/lunar-rover-quantum/install/mapping_pkg
```

---

## 5. Running it — step by step

The flow is: **map the terrain → build the energy graph → plan offline → drive live →
evaluate.** Steps 1–4 build the inputs; steps 5–7 are the live checks; step 8 collects
results.

Terminals used (each starts with the 0.5 block; **run everything from the repo root**):

| Terminal | Role |
|---|---|
| **T1** | Gazebo simulation |
| **T2** | first ROS launch (sensors, then the navigation pipeline) |
| **T3** | second ROS launch (mapping, then evaluation) |
| **T4** | checks, `ros2 topic` commands and the offline Python commands |

**Restart the simulation (Ctrl+C in T1, launch again) before every live run.** `/odom`
restarts at (0, 0) on the spawn point, which is where the planners start, and the rover
must be back there.

### Step 1 — Start the simulation (T1)

```bash
ros2 launch rover_simulation simulation_launch.py
```

Check in T4:

```bash
ros2 topic list        # expect /clock /cmd_vel /odom /joint_states /tf /scan /imu/data
ros2 topic hz /scan    # ~10 Hz
```

### Step 2 — Map the terrain (T1, T2, T3, T4)

The graph is built from what the LiDAR has seen, so the rover has to drive first.

1. **T2** — sensor processing: `ros2 launch sensor_pkg sensors_launch.py`
2. **T3** — mapping: `ros2 launch mapping_pkg mapping.launch.py`
   (publishes `/map`, `/terrain_map`; writes `results/terrain_maps/latest.npz` every ~10 s)
3. **Drive the rover for 60–90 s** — pick one:
   - *Autonomous:* stop T1 and instead run
     `ros2 launch rover_simulation demo.launch.py mode:=patrol` in T1. It starts the
     same simulation and, after 12 s, drives the rover along the trail toward x = 8 m.
   - *Manual (T4):*
     `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"`
     (drives a circle; Ctrl+C to stop), or
     `ros2 run rover_simulation rover_keyboard.py` (needs `sudo apt install python3-pynput`
     and a display).
4. **Check (T4):** `cat results/terrain_maps/latest_meta.json` — `cell_counts.flat_count`
   must be **> 0**.
5. Stop everything: Ctrl+C in T1, T2, T3.

### Step 3 — Build the energy graph (T4, no ROS)

```bash
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.graph_model \
  --npz results/terrain_maps/latest.npz \
  --heightmap src/rover_simulation/worlds/heightmap.png \
  --out results/graphs --cell-stride 5
cat results/graphs/latest_meta.json     # node_count and edge_count must be > 0
```

`node_count: 0` means the map was empty: go back to step 2 and drive longer.

### Step 4 — Plan offline and compare (T4, no ROS)

Same start (0, 0) and goal (1.5, 1.5) for every command; the live steps reuse them.

```bash
# Dijkstra + A*   -> results/paths/latest_classical.json
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.classical_planning \
  --start-x 0.0 --start-y 0.0 --goal-x 1.5 --goal-y 1.5

# QAOA (simulated) -> results/paths/latest_quantum.json   (takes a few seconds)
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.quantum_optimizer \
  --start-x 0.0 --start-y 0.0 --goal-x 1.5 --goal-y 1.5 \
  --graph results/graphs/latest.graphml --max-edges 10 --shots 2048

# Compare planners + distance-only baseline over 200 random routes
#   -> results/energy_comparison/latest_comparison.json
PYTHONPATH=src/evaluation_pkg python3 -m evaluation_pkg.metrics --sweep 200

# Plots + table (add --theme dark for dark plots)
python3 scripts/plot_energy_comparison.py
python3 scripts/visualize_plan.py        # 2-D route overlay -> results/paths/latest_classical_plot.png
```

Expect: QAOA energy equals A*/Dijkstra energy with `fallback=False`, and three PNGs
(`energy_comparison`, `route_sweep`, `path_overlay`) plus `comparison_table.csv` in
`results/energy_comparison/`. QAOA matches the classical optimum; it does not beat it.

### Step 5 — Live drive, classical planner (Phase 9)

1. **T1** — fresh simulation: `ros2 launch rover_simulation simulation_launch.py`
2. **T2** — planner + executor:
   ```bash
   ros2 launch navigation_pkg navigation_pipeline.launch.py \
     planner_mode:=classical goal_x:=1.5 goal_y:=1.5
   ```
3. **Pass:** T2 logs `Received new path on /path/classical with N waypoints`, then
   `Waypoint i/N reached ...`, then `Final waypoint reached. Rover stopped.`
   In T4: `ros2 topic echo /odom --once` shows a position within 0.5 m of the goal
   (the planner snaps the goal to the nearest graph node), and
   `ros2 topic echo /cmd_vel --once` is all zeros.
4. *Optional QoS check (late-joining executor):* in T2 run only the planner
   `ros2 launch planning_pkg classical_planner.launch.py goal_x:=1.5 goal_y:=1.5`, wait
   10 s, then in T3
   `ros2 run navigation_pkg path_executor --ros-args -p path_topic:=/path/classical`.
   The rover must still receive the path and drive.
5. Ctrl+C T2 (and T3); restart T1.

### Step 6 — Live drive, quantum planner (Phase 8 + 9)

Fresh simulation in T1, then in T2:

```bash
ros2 launch navigation_pkg navigation_pipeline.launch.py \
  planner_mode:=quantum goal_x:=1.5 goal_y:=1.5
```

The quantum node waits for `/odom`, plans (a few seconds on the CPU simulator), publishes
`/path/quantum`, and the executor drives it. Pass criteria are the same as step 5. Restart
T1 afterwards.

### Step 7 — Live evaluation (Phase 10)

Needs `results/energy_comparison/latest_comparison.json` from step 4 **for the same goal**.

1. **T1** — fresh simulation.
2. **T2** — start the evaluation **first**, so the battery baseline starts before the
   rover moves:
   ```bash
   ros2 launch evaluation_pkg evaluation.launch.py planner:=qaoa goal_tolerance:=0.5
   # planner:=astar or dijkstra to evaluate the classical path instead
   ```
3. **T3** — the pipeline (match the planner):
   ```bash
   ros2 launch navigation_pkg navigation_pipeline.launch.py \
     planner_mode:=quantum goal_x:=1.5 goal_y:=1.5
   ```
4. **T4** while it drives:
   ```bash
   ros2 topic echo /battery/status --once   # percentage ~0.99x and falling, current negative
   ros2 topic echo /metrics --once          # JSON: planned vs actual distance, consumed_j, goal_reached
   ```
5. **Pass:** the evaluator (T2) logs `Goal reached: drove X m (planned P m), used Y J` (P is the distance in `latest_comparison.json`, about 2.1 m for this route on the reference graph),
   and `results/energy_comparison/latest_execution.json` exists. Expect the actual distance
   within roughly 20 % of the planned distance and a few hundred joules at most; the battery
   percentage only drops a little (40 Wh capacity), which is normal.
6. If `/battery/status` never appears, relaunch the evaluation with `use_sim_time:=false`.
   `goal_tolerance` must be at least the executor's goal tolerance (0.5 m).

### Step 8 — Look at the results (T4)

```bash
ls results/energy_comparison/           # comparison + execution JSON, PNGs, comparison_table.csv
explorer.exe results/energy_comparison  # WSL2: opens the folder in Windows Explorer
```

Outputs by step: `results/terrain_maps/` (2), `results/graphs/` (3), `results/paths/` (4),
`results/energy_comparison/` (4, 7). `results/` is gitignored.

### Step 9 — Shut down

Ctrl+C in every terminal. If Gazebo does not exit cleanly, `pkill -f "gz sim"` before the
next launch (a leftover server blocks the next start).

### Optional — RViz

```bash
ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true
# In RViz: Fixed Frame = map; add Path /path/quantum and /path/classical, Map /map and
# /terrain_map, MarkerArray /terrain_map_markers, LaserScan /scan, RobotModel.
```

### Web viewer (no Linux/ROS needed)

```bash
cd web && python -m http.server 8080     # open http://localhost:8080/index.html
```

Three.js preview: rover model, rocks, terrain, drive with WASD, Patrol AI tab,
Orbit/Top camera. Standalone — not connected to ROS or Gazebo.

---

## 6. Interfaces / data contracts

### `/terrain_map` cell encoding (Phase 5 → Phase 6 contract)

`nav_msgs/OccupancyGrid`, `int8` cell values:

| Value | Class | How it is decided | Suggested Phase-6 cost multiplier |
|---|---|---|---|
| `-1` | unknown / unobserved | never hit by a LiDAR beam | unreachable |
| `10` | flat | observed free space in `/map` | 1.0× (baseline) |
| `50` | rocky | high local variance across consecutive LiDAR beams | 1.5–2.0× |
| `90` | crater interior | Hough-circle rim detection on the obstacle mask | 3.0–5.0× |
| `100` | obstacle | occupied cell in `/map` | ∞ (impassable) |

### Persisted map

- `results/terrain_maps/latest.npz` — keys `grid` (int8 2-D), `resolution`,
  `origin_x`, `origin_y`, `width`, `height`, `timestamp` (ISO-8601).
- `results/terrain_maps/latest_meta.json` — dimensions, origin, resolution, and
  per-class cell counts.

`planning_pkg` can load the `.npz` and build its graph offline, without a running
sim.

### Key TF frames

`map` → `odom` (static identity, from `occupancy_grid_node`) → `base_footprint`
(from Gazebo DiffDrive, bridged) → `base_link` → … → `lidar_link` / `imu_link`
(static, from `robot_state_publisher` via the Xacro).

### Energy cost model (Phase 6 target)

```
E(edge) = d · S(θ) · T(type) · R(r)
  d      = Euclidean distance between graph nodes (m)
  S(θ)   = 1.0 + 2.0·|sin θ|            slope factor (θ from IMU slope / grid gradient)
  T(type)= {flat 1.0, rocky 1.5, crater 3.0, obstacle ∞}   from /terrain_map
  R(r)   = 1.0 + r                      roughness factor, r ∈ [0,1] from LiDAR variance
```

### Evaluation outputs (Phase 10 contract)

- `results/energy_comparison/latest_comparison.json` (+ timestamped copy): `planners`
  (`dijkstra`, `astar`, `qaoa`, `distance_only` — each `energy`, `distance_m`,
  `waypoints`, `runtime_ms`, `path_efficiency`, `energy_per_m`, `path_nodes`,
  `path_coords`, `extras`), `summary` (`best_energy_planner`, `qaoa_matches_classical`,
  `qaoa_energy_ratio`, `qaoa_fallback`, `qaoa_runtime_ratio_vs_astar`,
  `energy_savings_vs_distance_only_pct`) and, with `--sweep`, `sweep` (per-route
  `savings_pct` + summary).
- `/battery/status` — `sensor_msgs/BatteryState`; `percentage` is 0–1, `current` is
  negative while discharging, `capacity`/`design_capacity` in Ah.
- `/metrics` — `std_msgs/String` holding JSON: planned vs actual distance,
  `execution_efficiency`, `consumed_j`, `joules_per_m`, `goal_reached`.
- `results/energy_comparison/latest_execution.json` — the same report frozen at arrival.
- No custom ROS messages: the two topics above use standard types by design.

### Classical path planning outputs (Phase 7 contract)

- `/path/classical` (`nav_msgs/msg/Path`, `TransientLocal` QoS): Canonical optimal energy path computed via A* for Layer 3 execution (`path_executor` in Phase 9).
- `/path/classical/dijkstra` (`nav_msgs/msg/Path`, `TransientLocal` QoS): Debug path computed via Dijkstra for side-by-side RViz inspection.
- `results/paths/latest_classical.json`: Canonical latest planner run metrics:
  - `"dijkstra"`: `path_nodes`, `path_coords`, `total_energy`, `total_distance_m`, `num_waypoints`, `runtime_ms`, `timestamp`.
  - `"astar"`: `path_nodes`, `path_coords`, `total_energy`, `total_distance_m`, `num_waypoints`, `runtime_ms`, `timestamp`.
  - `"comparison"`: `energy_delta`, `abs_energy_delta`, `distance_delta_m`, `runtime_delta_ms`, `speedup_factor`, `paths_identical`.
- `results/paths/classical_<timestamp>.json`: Timestamped history copy.

---

## 7. Assumptions & constraints

- **Simulation only.** No real quantum hardware (Qiskit `AerSimulator` on CPU) and
  no real rover yet. The Arjuna/Jetson deployment is Phase 11.
- **DART cannot collide a `<heightmap>`.** The Gazebo terrain is therefore a
  **flat box** collision with a matching flat visual (z = 0.375 m). The rock
  models keep their own box collisions and are the only obstacles the 2-D LiDAR
  sees. A contour terrain requires switching the physics engine to **Bullet**
  (`gz-physics-bullet-featherstone-plugin`) and reverting the terrain
  collision/visual to `<heightmap>` — untested. `scripts/heightmap_to_collision_obj.py`
  + `terrain_collision.obj` are kept for that.
- **`sensor_pkg` is `ament_cmake`** and installs its nodes as flat scripts into
  `lib/sensor_pkg/`, not as an importable package. Shared modules (e.g.
  `diff_drive_odometry.py`) are imported by bare name and listed in
  `install(PROGRAMS ...)`.
- **Odometry is dead-reckoned.** `encoder_processor` integrates wheel-joint angles
  with differential-drive kinematics; there is no correction/filter, so error
  accumulates over long runs.
- **The rover is inlined into `lunar_terrain.world`** (not spawned from the Xacro)
  so `JointStatePublisher` sees the joints at world-init. The Xacro is only used
  by `robot_state_publisher` for TF; sensor/plugin edits must be made in **both**
  the world file (authoritative) and the Xacro (kept in sync).
- **CI** (`.github/workflows/tests.yml`) runs only the pure-Python `tests/` +
  `compileall`. Anything importing `rclpy` cannot run there.
- Pure-Python tests import the module under test via a `sys.path` insert of its
  `src/<pkg>` dir (see `tests/test_grid_utils.py`, `tests/test_diff_drive_odometry.py`).

---

## 8. Forward plan (Phases 6–12)

Phases 1–5 are done (see `docs/PROGRESS.md`). Remaining:

| # | Phase | Deliverable | Key approach |
|---|---|---|---|
| 6 | Energy modeling — **done** | `results/graphs/latest.graphml` — a NetworkX graph of the terrain snapshot with `E(edge)` weights (formula in §6) | `planning_pkg.graph_model` (offline module + CLI); 8-connected coarsened grid graph; obstacle/unknown cells → no node |
| 7 | Classical planning — **done** | `/path/classical` + energy total | `planning_pkg.classical_planning` (offline CLI + `classical_planner_node`); Dijkstra and A* over weighted graph; publishes `nav_msgs/Path`; writes `results/paths/latest_classical.json` |
| 8 | Quantum optimization — **done** | `/path/quantum` + energy total | Formulate path choice as **QUBO** (edge-selection binaries, penalty terms for start/goal/continuity/no-branching); solve with **QAOA** on `AerSimulator`; keep the graph small (≈8–12 edges) for a tractable demo; decode best bitstring → path |
| 9 | Integration — **done** | sensor → map → graph → planner → `/cmd_vel` running end to end | `path_executor` in `navigation_pkg`: follow `nav_msgs/Path` waypoints with a proportional controller; `navigation_pipeline.launch.py` runs end to end |
| 10 | Evaluation — **core done; ROS nodes not yet run** | `results/energy_comparison/latest_comparison.json`, plots, `/battery/status`, `/metrics` | `evaluation_pkg.metrics` + `energy_model` (pure Python, unit-tested), `battery_monitor_node`, `evaluator_node`, `scripts/plot_energy_comparison.py`; see the Phase 10 note below |
| 11 | Real robot | rover driving lunar-like terrain on the Arjuna kit | ROS 2 on Jetson Nano; Arduino motor bridge; replace Gazebo topics with hardware drivers |
| 12 | Docs & report | final report, slides, paper draft | — |

Phase 6 is done: `planning_pkg.graph_model` (`python -m planning_pkg.graph_model`)
builds `results/graphs/latest.graphml` from a terrain snapshot + `worlds/heightmap.png`.
See `docs/PROGRESS.md` for details and `docs/superpowers/specs/2026-09-11-phase6-energy-graph-design.md`
for the design.

Phase 7 is done: `planning_pkg.classical_planning` (`PYTHONPATH=src/planning_pkg python -m planning_pkg.classical_planning`)
and `classical_planner_node` compute energy-optimal routes via Dijkstra and A*,
publishing canonical `/path/classical` (`nav_msgs/msg/Path`) and persisting
`results/paths/latest_classical.json`. Visualize via `python scripts/visualize_plan.py`
or in RViz. See `docs/PROGRESS.md` and
`docs/superpowers/specs/2026-09-13-phase7-classical-planner-design.md`.

Phase 8 is done: `planning_pkg.quantum_optimizer` and `quantum_optimizer_node` formulate path selection as a QUBO and solve via QAOA with Qiskit AerSimulator, using `/odom` for the start and publishing `/path/quantum` (`nav_msgs/msg/Path`) with results saved to `results/paths/latest_quantum.json`. The quantum instance is deliberately reduced to roughly 8-12 edges (a corridor around the A* backbone) so it is tractable on the CPU `AerSimulator`; the resulting macro-edge path is expanded back into the original terrain graph before metrics are reported.

**Phase 9 is done:** `navigation_pkg.path_executor_node` (`path_executor`) subscribes to `/path/quantum` or `/path/classical` and `/odom`, translating waypoints into velocity commands on `/cmd_vel`. `navigation_pipeline.launch.py` connects the full pipeline end-to-end. The control law lives in `navigation_pkg/path_following.py` (pure Python, unit-tested); the node subscribes to the path with `RELIABLE` + `TRANSIENT_LOCAL` QoS to match the planners, so a path published before the executor starts is still received.

**Phase 10 (evaluation) — what it does and does not show.** `evaluation_pkg.metrics`
compares Dijkstra, A*, the simulated QAOA path and a **distance-only baseline** (shortest
path by Euclidean length, re-priced with the real energy weights). On real project data
QAOA returns the **same** energy as A*/Dijkstra (ratio 1.00, no classical fallback) but is
~2×10⁴× slower (5.5 s vs 0.25 ms) — it matches the classical optimum, it does not beat it. The only energy
saving is against the distance-only baseline: 0 % on the single evaluated route, mean
0.24 % / max 8.5 % over a 200-route sweep of the current (near-flat) terrain graph. The
battery model (`P = P_idle + k_lin·|v| + k_ang·|ω|`, 40 Wh, 24 V) is an **assumption**, not
a measurement — replace the constants when the real rover exists (Phase 11). The pure-Python
core and plots are verified (84 tests pass); `battery_monitor` / `evaluator` have only been
smoke-tested against stubbed `rclpy`, so run the live steps in §5 on the Ubuntu box before
calling Phase 10 fully verified. See `docs/PROGRESS.md` and
`docs/superpowers/specs/2026-09-18-phase10-evaluation-design.md`.

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` |
| `ros2 run` / `ros2 launch` says an executable is not found, but the build passed | an `ament_python` package is missing `setup.cfg` (`install_scripts=$base/lib/<pkg>`), so its scripts landed in `install/<pkg>/bin/` instead of `lib/<pkg>/`. Add the file (copy `src/planning_pkg/setup.cfg`, change the package name), then rebuild. `tests/test_package_layout.py` checks every package |
| Build fails: `CMAKE_C_COMPILER not set` (`rover_simulation` / `sensor_pkg`) | no C compiler: `sudo apt install -y build-essential cmake`, rebuild |
| `file 'X.launch.py' was not found in the share directory` or `No module named planning_pkg.graph_model` | you are on a stale copy of the repo or a stale overlay: `ros2 pkg prefix <pkg>` must point into your current clone; remove any `source .../install/setup.bash` for another copy from `~/.bashrc`; re-clone if `git log` is old |
| `pip install` says `externally-managed-environment` | Ubuntu 24.04 PEP 668: `python3 -m pip install --user --break-system-packages <pkg>` |
| `results/graphs/latest_meta.json` has `node_count: 0` | the map was empty: drive longer in step 2, check `cell_counts.flat_count > 0`, rebuild the graph |
| `ros2 run` says the executable is not found / `bad interpreter` after a Windows checkout | CRLF line endings in the node scripts: `sed -i 's/\r$//' src/sensor_pkg/sensor_pkg/*.py`, then rebuild (`rm -rf build install log && colcon build --symlink-install`) |
| Package not found after build | `source install/setup.bash` (separate from the ROS 2 source) |
| Wall of "Depends: … not installable" during Jazzy install | Wrong Ubuntu — need 24.04, not 22.04 |
| `gz sim` dies with `exit code -11` / render-engine error | WSL/VM GL: `export LIBGL_ALWAYS_SOFTWARE=1`; or switch render engine `ogre2`→`ogre` in `lunar_terrain.world` |
| `/scan` missing from `ros2 topic list` | check `gz topic -l`; if the gz topic is scoped, set it as `gz_topic_name` in `src/rover_simulation/config/bridge.yaml` and rebuild |
| `/map` stays all `-1` while driving | `/scan` not reaching the node (BEST_EFFORT QoS) — echo `/scan` in the same shell; check `/clock` is bridged and `use_sim_time` is true |
| Rover spins but doesn't move | terrain collision — see §7 (DART + heightmap) |
| `occupancy_grid_node` TF errors | `robot_state_publisher` down, or `lidar_link` missing from the Xacro |
| `/battery/status` never appears | `battery_monitor` timers follow the sim clock — check `/clock` is bridged, or launch with `use_sim_time:=false` |
| `evaluator` exits: "Comparison file … not found" | run `python3 -m evaluation_pkg.metrics` first (needs the Phase 7/8 result JSONs) |
| `/terrain_map` is ~all "crater" | Hough params in `mapping_params.yaml` too loose for the rock field - use the tightened values already in the file (see `docs/PROGRESS.md`, issue 1) |

---

## Team / Institution

_(fill in)_
