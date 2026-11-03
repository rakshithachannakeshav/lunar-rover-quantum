# Running the rover in Gazebo + ROS 2 with QAOA only

Target: WSL distro `Ubuntu-24.04`, user `ubuntu`, repo at `~/lunar-rover-quantum`.
Dependencies are installed by `sudo bash scripts/install_all_jazzy.sh`.

QAOA path planning (`quantum_optimizer`, `quantum_optimizer_node`) never imports or
runs A*/Dijkstra and has no fallback: if QAOA produces no valid path it errors out
and nothing is published on `/path/quantum`. `planner_mode:=quantum` launches only the
quantum planner plus an executor listening on `/path/quantum`.

`selection` decides which sampled path is reported:

| selection | what it measures |
|---|---|
| `best` (default) | lowest-energy valid path among all shots; on ~19 qubits, sampling alone often finds it |
| `most_likely` | the valid path QAOA sampled most often, i.e. QAOA's own answer |
| `top` | the single most frequent bitstring; fails if QAOA's top answer is not a valid path |

Every run also logs QAOA's valid-path rate and a uniform-random baseline that uses
the same shot count, so you can see whether QAOA does better than random guessing.

## Every terminal

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
```

## Build (once, and after code changes)

```bash
cd ~/lunar-rover-quantum
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q
```

## 1. Map the terrain (T1, T2, T3)

```bash
# T1
ros2 launch rover_simulation demo.launch.py mode:=patrol
# T2
ros2 launch sensor_pkg sensors_launch.py
# T3
ros2 launch mapping_pkg mapping.launch.py
# T4, after 60-90 s of driving: flat_count must be > 0
cat results/terrain_maps/latest_meta.json
```

Ctrl+C in T1, T2 and T3.

## 2. Build the energy graph (T4)

```bash
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.graph_model \
  --npz results/terrain_maps/latest.npz \
  --heightmap src/rover_simulation/worlds/heightmap.png \
  --out results/graphs --cell-stride 5
cat results/graphs/latest_meta.json
```

## 3. QAOA alone, offline (T4)

```bash
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.quantum_optimizer \
  --start-x 0 --start-y 0 --goal-x 1.5 --goal-y 1.5 --selection most_likely
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.quantum_optimizer \
  --start-x 0 --start-y 0 --goal-x 1.5 --goal-y 1.5 --selection top
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.quantum_optimizer \
  --start-x 0 --start-y 0 --goal-x 1.5 --goal-y 1.5 --selection most_likely --reps 3 --restarts 8
```

Optional: an A* reference, run after QAOA finishes. A* output never feeds into QAOA.

```bash
PYTHONPATH=src/planning_pkg python3 -m planning_pkg.compare_planners \
  --start-x 0 --start-y 0 --goal-x 1.5 --goal-y 1.5 --selection most_likely
```

## 4. Drive the QAOA path live in Gazebo

```bash
# T1: fresh sim (headless:=false opens the Gazebo window)
ros2 launch rover_simulation simulation_launch.py headless:=false
# T2
ros2 launch navigation_pkg navigation_pipeline.launch.py \
  planner_mode:=quantum goal_x:=1.5 goal_y:=1.5 selection:=most_likely
# T3: confirm no classical planner is running
ros2 node list
ros2 topic info /path/classical
ros2 topic echo /odom --once
```

## 5. Gazebo window (optional, while T1 runs)

```bash
bash scripts/open_gazebo_gui.sh
```

## Shut down

Ctrl+C in all terminals, then `pkill -f "gz sim"`.
