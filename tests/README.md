# tests/

Pure-Python unit tests only. Run in GitHub Actions CI on every push/PR (see
`.github/workflows/tests.yml`) using a plain `pip install`, no ROS2 or Gazebo.

## What goes here
- Logic with no `rclpy`/ROS2/Gazebo import: energy model math, graph building
  (NetworkX), classical planners (Dijkstra/A*), QAOA/Qiskit optimizer code.

## What doesn't
- Anything in `sensor_pkg` or `rover_simulation` that imports `rclpy` — mark
  it `@pytest.mark.ros2` (or `@pytest.mark.gazebo`) if it ever gets a test
  file, so CI skips it automatically instead of failing on a missing import.

## Current state
Empty. `mapping_pkg`, `planning_pkg`, `navigation_pkg`, `evaluation_pkg`, and
`quantum/` are all scaffolding with no logic yet (see `CLAUDE.md`), so there's
nothing to test yet. CI is wired up and will start actually verifying things
the moment real modules land.
