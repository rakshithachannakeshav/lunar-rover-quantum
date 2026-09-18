# Autonomous Lunar Rover Navigation — Implementation & Execution Guide

This document details everything that has been implemented across **Phases 1 through 9** of the **Quantum-Assisted Energy-Aware Lunar Rover Navigation System**, along with exact step-by-step commands to build, run, verify, and visualize the entire pipeline.

---

## 1. System Architecture & Layers

The system is organized into four decoupled layers communicating via ROS 2 Jazzy topics and persisted data artifacts:

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
  /path/* + /odom      -> path_executor          -> /cmd_vel          (Phase 9)

LAYER 4 — EVALUATION            (src/evaluation_pkg)
  /path/* + /odom      -> evaluator              -> /metrics          (Phase 10)
```

---

## 2. Summary of Implemented Components (Phases 1 – 9)

### Phase 1 & 2: Architecture & Environment Setup
- Workspace structure established with ROS 2 Jazzy, Gazebo Harmonic, Qiskit Aer, NetworkX, and OpenCV.
- Dependency installation shell scripts in `scripts/` (`setup_ros2_repo.sh`, `install_ros2_jazzy.sh`, `install_gazebo_harmonic.sh`, `install_python_quantum.sh`).

### Phase 3: Gazebo Lunar Terrain Simulation (`src/rover_simulation`)
- World definition `src/rover_simulation/worlds/lunar_terrain.world` with flat collision box, heightmap visual, lunar ground properties ($\mu=0.9$), and rock obstacles.
- 6-wheel rover URDF/Xacro (`rover.urdf.xacro`) with `gpu_lidar` (360° ray scan @ 10 Hz) and `imu` sensor (50 Hz).
- Gazebo-ROS bridge configuration (`config/bridge.yaml`) bridging `/clock`, `/scan`, `/imu/data`, `/odom`, `/tf`, and `/cmd_vel`.

### Phase 4: Sensor Processing (`src/sensor_pkg`)
- `lidar_processor.py`: Subscribes to `/scan` and publishes 3D `sensor_msgs/PointCloud2` on `/terrain/pointcloud`.
- `imu_processor.py`: Subscribes to `/imu/data` and publishes terrain slope angle on `/imu/slope`.
- `encoder_processor.py` & `diff_drive_odometry.py`: Integrates wheel joint angles from `/joint_states` using midpoint differential-drive kinematics to publish dead-reckoned odometry on `/odom`.

### Phase 5: Mapping & Terrain Classification (`src/mapping_pkg`)
- `occupancy_grid_node.py`: Subscribes to `/scan` and TF to construct a 600×600 occupancy grid (`/map`) at 0.05 m resolution and broadcasts static `map -> odom` transform.
- `terrain_classifier_node.py`: Classifies grid cells into `flat` (10), `rocky` (50), `crater_interior` (90), and `obstacle` (100). Periodically saves snapshots to `results/terrain_maps/latest.npz` and `latest_meta.json`.

### Phase 6: Energy Cost Model & Graph Generation (`src/planning_pkg`)
- `graph_model.py`: Coarsens the grid map using majority/obstacle aggregation, samples elevation from `heightmap.png`, and builds an 8-connected NetworkX graph (`results/graphs/latest.graphml`).
- **Energy Cost Function**:
  $$E(e) = d \cdot S(\theta) \cdot T(type)$$
  where $d$ is Euclidean distance, $S(\theta) = 1.0 + 2.0 \cdot |\sin\theta|$ is slope penalty, and $T(type) \in \{1.0, 1.5, 3.0\}$ is terrain cost multiplier.

### Phase 7: Classical Path Planning (`src/planning_pkg`)
- `classical_planning.py`: Pure-Python module implementing Dijkstra and A* shortest-path algorithms over weighted GraphML models with continuous coordinate snapping (`resolve_node`).
- `classical_planner_node.py`: ROS 2 node subscribing to `/odom` (or parameters), executing A* planning, publishing `/path/classical` and `/path/classical/dijkstra`, and outputting `results/paths/latest_classical.json`.

### Phase 8: Quantum Path Optimization (`src/planning_pkg`)
- `quantum_optimizer.py`:
  - **Corridor Reduction**: Splits A* backbone into anchors and extracts candidate macro-edges to build a tractable $\le 10$-edge reduced graph.
  - **QUBO Formulation**: Edge selection binaries $x_e$, endpoint degree constraints $P(d_{start}-1)^2$, and internal anchor activation binaries $y_v$ with $P(d_v - 2y_v)^2$.
  - **Ising & QAOA**: Converts QUBO to Ising Hamiltonian ($x = \frac{1-Z}{2}$), builds QAOA circuit ($p=1$), optimizes angles against statevector expectation with SciPy `L-BFGS-B`, and samples bitstrings on Qiskit `AerSimulator`.
  - **Path Decoding**: Expands macro-edges back into original terrain waypoints and validates connectivity.
- `quantum_optimizer_node.py`: ROS 2 node publishing `/path/quantum` and outputting `results/paths/latest_quantum.json`.

### Phase 9: Integration & Path Execution (`src/navigation_pkg`)
- `path_executor_node.py`: Unified path follower subscribing to `/path/quantum` (or `/path/classical`) and `/odom`. Implements proportional linear/angular velocity scaling, heading alignment error normalization, and goal tolerance checks to publish velocity commands on `/cmd_vel`.
- `quantum_path_executor_node.py`: Backward-compatible alias node.
- Launch files: `navigation_pipeline.launch.py` and `quantum_navigation.launch.py` for end-to-end multi-node orchestration.

---

## 3. Step-by-Step Instructions to Build, Run & Check

### Step 0: Clean Build (Run Once First in WSL)

```bash
cd ~/lunar-rover-quantum

# 1. Strip Windows carriage returns (\r) from script shebangs
sed -i 's/\r$//' src/sensor_pkg/sensor_pkg/*.py
sed -i 's/\r$//' src/rover_simulation/rover_simulation/*.py

# 2. Clean previous build folders and rebuild workspace
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

---

### Step 1: Launch Gazebo Simulation (Terminal 1)

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch rover_simulation simulation_launch.py
```
*Spawns Gazebo lunar world, rover model, LiDAR sensor (`/scan`), IMU (`/imu/data`), and ROS-GZ bridges.*

---

### Step 2: Launch Sensors & Mapping Pipeline (Terminal 2)

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run sensor processing nodes:
ros2 launch sensor_pkg sensors_launch.py
```

In a new tab / pane of Terminal 2:

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run mapping node:
ros2 launch mapping_pkg mapping.launch.py
```
*Publishes `/map`, `/terrain_map`, and saves map snapshots to `results/terrain_maps/latest.npz`.*

---

### Step 3: Generate Energy Graph Snapshot (Terminal 3)

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1. Generate map snapshot file (if running offline or snapshot needed):
python3 scripts/generate_phase5_snapshot.py

# 2. Build the NetworkX energy-weighted terrain graph:
python3 -m planning_pkg.graph_model \
  --npz results/terrain_maps/latest.npz \
  --heightmap src/rover_simulation/worlds/heightmap.png \
  --out results/graphs --cell-stride 5
```
*Generates `results/graphs/latest.graphml` (40,578 nodes, 160,702 weighted edges).*

---

### Step 4: Launch Phase 9 Quantum Navigation Pipeline (Terminal 4)

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch planning_pkg quantum_navigation.launch.py goal_x:=8.0 goal_y:=0.0
```

---

### Step 5: Standalone Offline Python CLI Execution (No ROS Needed)

#### A. Run Classical Planner (Dijkstra + A*)
```bash
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.classical_planning \
  --start-x 0.0 --start-y 0.0 --goal-x 8.0 --goal-y 0.0 \
  --graph results/graphs/latest.graphml
```
*Outputs `results/paths/latest_classical.json`.*

#### B. Run QAOA Quantum Optimizer
```bash
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.quantum_optimizer \
  --start-x 0.0 --start-y 0.0 --goal-x 8.0 --goal-y 0.0 \
  --graph results/graphs/latest.graphml --max-edges 10 --shots 2048
```
*Outputs `results/paths/latest_quantum.json`.*

#### C. Generate 2D Visual Comparison Plot
```bash
python3 scripts/visualize_plan.py
```
*Generates `results/paths/latest_classical_plot.png` comparing paths, waypoints, and energy metrics.*

---

### Step 6: Run Pure-Python Unit Test Suite

```bash
python3 -m pytest -q
```
*Runs unit tests in `tests/` covering odometry kinematics, grid processing, graph construction, Dijkstra/A* search, QUBO formulation, Ising conversion, and path execution controller logic.*

---

## 4. Key Output Data Contracts

- `results/graphs/latest.graphml`: GraphML file containing NetworkX graph with node positions and edge energy weights.
- `results/paths/latest_classical.json`: JSON output containing Dijkstra and A* path nodes, waypoints, energy totals, and computation runtime.
- `results/paths/latest_quantum.json`: JSON output containing QAOA quantum path nodes, waypoints, best bitstring, expectation value, qubit count, and classical-vs-quantum comparison metrics.
- `results/paths/latest_classical_plot.png`: 2D visualization plot of planned routes over terrain map.

