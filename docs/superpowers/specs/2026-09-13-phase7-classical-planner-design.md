# Phase 7 — Classical Path Planning (Dijkstra and A*) Design

**Date:** 2026-09-13
**Status:** Implemented and verified (pure-Python unit tests, CLI, ROS 2 Jazzy node).
**Scope:** Core planning logic, CLI entry point, ROS 2 `classical_planner` node wrapper, `/path/classical` and `/path/classical/dijkstra` topic publication, JSON persistence, and unit test suite.

---

## 1. Problem Statement

Phase 6 produces an energy-weighted 8-connected grid graph (`results/graphs/latest.graphml`) from the terrain occupancy grid and heightmap. However, before Phase 7, there was no planning node to compute optimal routes through this graph.

The navigation system requires:
1. Finding the minimum-energy route between arbitrary start and goal world coordinates.
2. Implementing two canonical classical search algorithms:
   - **Dijkstra's Algorithm**: Exhaustive, exact uniform-cost shortest path.
   - **A\* Search**: Heuristic-guided optimal path search.
3. Publishing the canonical path on `/path/classical` (`nav_msgs/msg/Path`) for Layer 3 execution (`path_executor` in Phase 9).
4. Publishing a secondary debug topic `/path/classical/dijkstra` for side-by-side RViz inspection.
5. Persisting energy, distance, waypoint, and runtime metrics to `results/paths/latest_classical.json` for Layer 4 benchmarking in Phase 10 (`evaluator`).
6. Operating in both **offline CLI mode** (zero ROS dependencies) and **live ROS 2 node mode**.

---

## 2. Constraints

1. **CI Independence**: GitHub Actions CI runs only pure-Python `tests/` and `compileall`. No ROS 2 dependencies (`rclpy`, `nav_msgs`) may be imported in the core algorithm file. Core planning must reside in `classical_planning.py`, isolated from the node wrapper in `classical_planner_node.py`.
2. **Graph Data Contract**: Must consume the GraphML format exported by Phase 6 (`planning_pkg.graph_model`), respecting:
   - Node attributes: `pos_x`, `pos_y`, `terrain_class`, `terrain_factor`.
   - Edge attribute: `weight` ($E = d \cdot S(\theta) \cdot T_{avg}$).
3. **Terrain Feasibility**: Obstacle (`100`) and unknown (`-1`) cells are pruned during Phase 6 graph construction; disconnected components due to crater rims or rock clusters must be handled gracefully without unhandled exceptions.
4. **ROS 2 Jazzy**: The node wrapper must target ROS 2 Jazzy on Ubuntu 24.04 LTS.

---

## 3. Design Decisions

### 3.1 Algorithm Implementations & Separation
- **Dijkstra**: Uses `networkx.dijkstra_path(graph, start, goal, weight='weight')`. Serves as the ground-truth optimal energy baseline.
- **A\* Search**: Uses `networkx.astar_path(graph, start, goal, heuristic=h_fn, weight='weight')`. Selected as the canonical path to execute because heuristic pruning yields significant runtime speedups on large graphs while guaranteeing optimal energy.

### 3.2 Heuristic Admissibility & Consistency Justification
For A\* to guarantee finding the optimal energy path, the heuristic $h(u, \text{goal})$ must be **admissible** (never overestimate the true cost to reach the goal) and **consistent** ($h(u) \le c(u, v) + h(v)$).

The edge cost formula from README Section 6 is:
$$E(\text{edge}) = d \cdot S(\theta) \cdot T(\text{type})$$
where:
- $d = \| \text{pos}_j - \text{pos}_i \|_2$ (Euclidean distance between adjacent block centers in meters)
- $S(\theta) = 1.0 + 2.0 \cdot |\sin\theta| \ge 1.0$ (slope factor, where $\theta = \arctan(|\Delta z| / d)$)
- $T(\text{type}) \in \{1.0, 1.5, 3.0\} \ge 1.0$ (terrain factor: flat ground $= 1.0$, rocky $= 1.5$, crater $= 3.0$)

Since every cost multiplier is $\ge 1.0$:
$$E(\text{edge}) \ge d \cdot 1.0 \cdot 1.0 = d$$

Therefore, the straight-line Euclidean distance between node $u$ and the goal:
$$h(u, \text{goal}) = \sqrt{(x_{\text{goal}} - x_u)^2 + (y_{\text{goal}} - y_u)^2}$$
is strictly $\le$ the true minimum energy to reach the goal from $u$. Straight-line Euclidean distance in a metric space also satisfies the triangle inequality:
$$h(u) \le d(u, v) \le E(u, v) + h(v)$$
which guarantees both **admissibility** and **consistency**.

### 3.3 Coordinate Resolution
Arbitrary world coordinates $(x, y)$ are snapped to the nearest traversable node using Euclidean distance over `pos_x` and `pos_y`. If the coordinates exceed $5 \times$ the graph's bounding-box diagonal, a descriptive `ValueError` is raised to prevent planning from nonsense locations.

---

## 4. Interfaces & Data Contracts

### 4.1 Persisted JSON Artifacts (`results/paths/`)
Saved by both CLI and node mode:
- `results/paths/latest_classical.json` (canonical latest)
- `results/paths/classical_<YYYYMMDD_HHMMSS>.json` (timestamped history)

#### Schema:
```json
{
  "dijkstra": {
    "algorithm": "Dijkstra",
    "path_nodes": [ "(0, 0)", "(1, 1)", ... ],
    "path_coords": [ [0.0, 0.0], [0.5, 0.5], ... ],
    "total_energy": 14.82,
    "total_distance_m": 12.45,
    "num_waypoints": 25,
    "runtime_ms": 3.42,
    "timestamp": "2026-09-13T23:15:00"
  },
  "astar": {
    "algorithm": "A*",
    "path_nodes": [ "(0, 0)", "(1, 1)", ... ],
    "path_coords": [ [0.0, 0.0], [0.5, 0.5], ... ],
    "total_energy": 14.82,
    "total_distance_m": 12.45,
    "num_waypoints": 25,
    "runtime_ms": 1.15,
    "timestamp": "2026-09-13T23:15:00"
  },
  "comparison": {
    "energy_delta": 0.0,
    "abs_energy_delta": 0.0,
    "distance_delta_m": 0.0,
    "runtime_delta_ms": -2.27,
    "speedup_factor": 2.97,
    "paths_identical": true,
    "dijkstra": { "energy": 14.82, "distance_m": 12.45, "waypoints": 25, "runtime_ms": 3.42 },
    "astar": { "energy": 14.82, "distance_m": 12.45, "waypoints": 25, "runtime_ms": 1.15 },
    "compared_at": "2026-09-13T23:15:00"
  },
  "saved_at": "2026-09-13T23:15:00"
}
```

### 4.2 ROS 2 Topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/path/classical` | `nav_msgs/msg/Path` | Transient Local, Reliable | Canonical path computed by A\* for Phase 9 `path_executor` |
| `/path/classical/dijkstra` | `nav_msgs/msg/Path` | Transient Local, Reliable | Dijkstra path for side-by-side RViz visualization |

---

## 5. What Phase 8 & Phase 9 Can Assume Exists

- **Phase 8 (Quantum Optimizer)**:
  - Can load `results/paths/latest_classical.json` to get the baseline classical energy and path length to benchmark against.
  - Can use `load_weighted_graph` and `resolve_node` from `planning_pkg.classical_planning`.
- **Phase 9 (Path Executor)**:
  - Can subscribe directly to `/path/classical` (`nav_msgs/msg/Path`).
  - Waypoints have valid `pose.position.x` and `pose.position.y` in the `map` frame ($z = 0.0$).
