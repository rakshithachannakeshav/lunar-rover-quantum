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
  graphs/latest.graphml   -> classical_planner    -> /path/classical   (Phase 7)
  graphs/latest.graphml   -> quantum_optimizer    -> /path/quantum     (Phase 8)

LAYER 3 — EXECUTION             (src/navigation_pkg)
  /path/*              -> path_executor          -> /cmd_vel          (Phase 9)

LAYER 4 — EVALUATION            (src/evaluation_pkg)
  /path/* + /odom      -> evaluator              -> /metrics          (Phase 10)
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
| `classical_planner` | planning_pkg | `/path/classical` | `/graph/weighted` | **not built** |
| `quantum_optimizer` | planning_pkg / quantum | `/path/quantum` | `/graph/weighted` | **not built** |
| `path_executor` | navigation_pkg | `/cmd_vel` | `/path/quantum`, `/path/classical` | **not built** |
| `battery_monitor` | evaluation_pkg | `/battery/status` | `/cmd_vel`, `/odom` | **not built** |
| `evaluator` | evaluation_pkg | `/metrics` | `/path/*`, `/battery/status` | **not built** |

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

## 4. Setup from a fresh clone

ROS 2 Jazzy + Gazebo Harmonic need **Ubuntu 24.04**. It does not run natively on
Windows — use a VM, WSL2, or dual-boot. Confirm the guest OS with
`cat /etc/os-release`; 22.04 ("Jammy") **cannot** install Jazzy.

```bash
git clone https://github.com/rakshithachannakeshav/lunar-rover-quantum.git ~/lunar-rover-quantum
cd ~/lunar-rover-quantum
```

### 4a. Dependencies

Run once, in order (each does real `apt-get` work):

```bash
chmod +x scripts/*.sh
bash scripts/setup_ros2_repo.sh          # ROS 2 apt repo + locale
bash scripts/install_ros2_jazzy.sh       # ROS 2 Jazzy desktop + colcon + rosdep
bash scripts/install_gazebo_harmonic.sh  # Gazebo Harmonic + ros_gz
bash scripts/install_sim_dependencies.sh # xacro, rviz2, nav2, teleop, tf tools
bash scripts/install_python_quantum.sh   # Python + Qiskit stack (creates a venv)

# extras the mapping + sensor phase need explicitly:
sudo apt install -y ros-jazzy-tf2-ros ros-jazzy-visualization-msgs \
  ros-jazzy-tf2-tools python3-opencv
```

Notes:
- `install_ros2_jazzy.sh` / `install_python_quantum.sh` hard-code
  `/home/monis/.bashrc` for the auto-source line. If your username differs, that
  line silently no-ops — just `source /opt/ros/jazzy/setup.bash` in each new
  terminal.
- On WSL2, Gazebo's `gpu_lidar` needs a GL context. If the sim errors on the
  render engine, `export LIBGL_ALWAYS_SOFTWARE=1` before launching.

### 4b. Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

All six packages should report **Finished**, none **Failed**.
`planning_pkg`, `navigation_pkg`, `evaluation_pkg` are still scaffolding and build
instantly.

---

## 5. Running it

Each step is its own terminal; source `/opt/ros/jazzy/setup.bash` **and**
`install/setup.bash` in each.

```bash
# 1. Simulation (headless Gazebo server + ROS<->GZ bridge + robot_state_publisher)
export LIBGL_ALWAYS_SOFTWARE=1            # WSL2 / no-GPU only
ros2 launch rover_simulation simulation_launch.py

# 2. Sanity check
ros2 topic list          # expect /clock /cmd_vel /odom /joint_states /tf /scan /imu/data
ros2 topic hz /scan      # ~10 Hz
ros2 topic hz /imu/data  # ~50 Hz

# 3. Drive the rover
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
# or:  ros2 run rover_simulation rover_keyboard.py     (needs: sudo apt install python3-pynput)

# 4. Sensor processing nodes
ros2 launch sensor_pkg sensors_launch.py

# 5. Mapping pipeline (Phase 5)
ros2 launch mapping_pkg mapping.launch.py
#   -> /map (600x600 @ 0.05 m), /terrain_map, /terrain_map_markers
#   -> writes results/terrain_maps/latest.npz every ~10 s

# 6. Visualise
ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true
#   RViz: Fixed Frame = map; add Map /map, Map /terrain_map, MarkerArray
#         /terrain_map_markers, LaserScan /scan, RobotModel (/robot_description)
# or the Gazebo GUI:  bash scripts/open_gazebo_gui.sh   (then right-click rover -> Follow)
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
| 7 | Classical planning | `/path/classical` + energy total | Dijkstra and A* over the weighted graph; publish `nav_msgs/Path`; record energy + length |
| 8 | Quantum optimization | `/path/quantum` + energy total | Formulate path choice as **QUBO** (edge-selection binaries, penalty terms for start/goal/continuity/no-branching); solve with **QAOA** on `AerSimulator`; keep the graph small (≈8–12 edges) for a tractable demo; decode best bitstring → path |
| 9 | Integration | sensor → map → graph → planner → `/cmd_vel` running end to end | `path_executor` in `navigation_pkg`: follow `nav_msgs/Path` waypoints with a simple pure-pursuit / go-to-goal controller |
| 10 | Evaluation | plots: energy classical vs quantum, path efficiency, runtime | `battery_monitor` (integrate power ∝ `|v|` + turn cost), `evaluator` node, matplotlib comparison scripts under `scripts/` |
| 11 | Real robot | rover driving lunar-like terrain on the Arjuna kit | ROS 2 on Jetson Nano; Arduino motor bridge; replace Gazebo topics with hardware drivers |
| 12 | Docs & report | final report, slides, paper draft | — |

Phase 6 is done: `planning_pkg.graph_model` (`python -m planning_pkg.graph_model`)
builds `results/graphs/latest.graphml` from a terrain snapshot + `worlds/heightmap.png`.
See `docs/PROGRESS.md` for details and `docs/superpowers/specs/2026-09-11-phase6-energy-graph-design.md`
for the design.

**Recommended next task:** Phase 7 `classical_planner`. Load
`results/graphs/latest.graphml` via `graph_model.load_graph`, implement
Dijkstra and A* over it, and publish/save the result as `/path/classical`.

---

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` |
| Package not found after build | `source install/setup.bash` (separate from the ROS 2 source) |
| Wall of "Depends: … not installable" during Jazzy install | Wrong Ubuntu — need 24.04, not 22.04 |
| `gz sim` dies with `exit code -11` / render-engine error | WSL/VM GL: `export LIBGL_ALWAYS_SOFTWARE=1`; or switch render engine `ogre2`→`ogre` in `lunar_terrain.world` |
| `/scan` missing from `ros2 topic list` | check `gz topic -l`; if the gz topic is scoped, set it as `gz_topic_name` in `src/rover_simulation/config/bridge.yaml` and rebuild |
| `/map` stays all `-1` while driving | `/scan` not reaching the node (BEST_EFFORT QoS) — echo `/scan` in the same shell; check `/clock` is bridged and `use_sim_time` is true |
| Rover spins but doesn't move | terrain collision — see §7 (DART + heightmap) |
| `occupancy_grid_node` TF errors | `robot_state_publisher` down, or `lidar_link` missing from the Xacro |
| `/terrain_map` is ~all "crater" | known issue — Hough params in `mapping_params.yaml` need tuning (see `docs/PROGRESS.md`) |

---

## Team / Institution

_(fill in)_
