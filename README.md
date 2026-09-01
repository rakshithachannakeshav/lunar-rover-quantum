# Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation

Final-year major project: an autonomous lunar rover navigation system that plans
energy-efficient paths across simulated lunar terrain using both classical
graph-search algorithms and a quantum optimization algorithm (QAOA), so the two
approaches can be compared directly.

## Architecture

The system is organized into four layers:

1. **Perception** (`src/sensor_pkg`) — LiDAR, IMU, and encoder nodes build a
   terrain map (ROS2).
2. **Planning** (`src/mapping_pkg`, `src/planning_pkg`) — the terrain map is
   converted into a weighted graph (NetworkX), where edge weights represent
   energy cost (slope, terrain type, roughness). A classical planner
   (A*/Dijkstra) and a quantum planner (QAOA/QUBO, via Qiskit AerSimulator,
   CPU-simulated) both compute paths on this graph.
3. **Execution** (`src/navigation_pkg`) — the chosen path is sent as waypoints
   to the rover (ROS2 Nav2 / Arduino on real hardware).
4. **Evaluation** (`src/evaluation_pkg`) — energy usage, path efficiency, and
   classical-vs-quantum comparison metrics.

## Stack

Ubuntu, ROS2 (Jazzy), Gazebo Harmonic for simulation, Python, Qiskit +
Qiskit-Aer, NetworkX, Matplotlib. Real hardware target is the Arjuna AMR kit
with a Jetson Nano, for a later phase.

## Current status

- **Phase 1 (planning) and Phase 2 (environment setup): done.**
- The Gazebo rover simulation (terrain + rover physics + movement) is
  **working** — see `src/rover_simulation`. The rover is defined as an inline
  SDF model in the world file (generated from the URDF/Xacro), which ensures
  all plugins (DiffDrive, JointStatePublisher) initialize correctly at world
  load. Odometry (`/odom`), joint states (`/joint_states`), and TF are all
  bridged to ROS2 and verified publishing.
- `sensor_pkg`: implemented (LiDAR/IMU/encoder processing nodes), all three
  nodes launch and run cleanly.
- `mapping_pkg`, `planning_pkg`, `navigation_pkg`, `evaluation_pkg`: scaffolded
  (standard ROS2 package structure in place), implementation not yet started.
- `quantum/notebooks`: QAOA/Qiskit experimentation, not yet started.

### Resolved issues (QA phase)

- **`scripts/diagnose.py` hardcoded path** — replaced machine-specific absolute
  path with a portable `os.path.join(os.path.dirname(__file__), ...)` lookup.
- **`sensor_pkg` nodes not executable** — added missing `#!/usr/bin/env python3`
  shebangs and `chmod +x` to all three processing scripts.
- **Rover odometry not publishing** — root cause: the world file used a static
  `model://lunar_rover` include that loaded a stale, pre-built SDF from
  Gazebo's model cache, disconnected from the corrected URDF/Xacro. Fixed by
  generating SDF from Xacro and inlining the `<model>` block directly into
  `lunar_terrain.world`.
- **`/joint_states` not publishing** — same root cause: `JointStatePublisher`
  requires joint entities at world-init time. Resolved by the same inline-SDF
  fix, plus a new `/joint_states` entry in `bridge.yaml`.

## Folder structure

```
lunar-rover-quantum/
├── src/
│   ├── sensor_pkg/          # LiDAR, IMU, encoder processing nodes
│   ├── mapping_pkg/         # occupancy grid, terrain classification
│   ├── planning_pkg/        # graph builder, energy model, classical planner
│   ├── navigation_pkg/      # path executor, ROS2/motor integration
│   ├── evaluation_pkg/      # battery monitor, comparison metrics
│   └── rover_simulation/    # URDF/Xacro, Gazebo world (inline SDF), launch files
├── quantum/notebooks/       # QAOA/Qiskit experiments
├── docs/
│   ├── report/
│   ├── presentation/
│   └── reference/           # reference material not wired into the build
├── scripts/                  # setup/install scripts, asset generators, diagnostics
├── tests/
├── results/
│   ├── classical_paths/
│   ├── quantum_paths/
│   └── energy_comparison/
└── web/                       # browser-based (Three.js) rover visualization
```

## Team

[Your team names here]

## Institution

[Your institution name here]
