I am working on a project called:

LUNAR ROVER — QUANTUM NAVIGATION

The goal is to build an energy-aware, terrain-adaptive lunar rover navigation system using ROS2, Gazebo, classical path planning, and quantum optimization.

IMPORTANT:

* Work with the existing project/codebase.
* Do NOT unnecessarily rewrite or delete working code.
* Before making changes, inspect the existing repository structure and understand what is already implemented.
* Implement things incrementally and test each phase before moving to the next.
* Keep the code modular and ROS2-compatible.
* Reuse existing nodes, topics, launch files, URDF/Xacro, worlds, and configurations whenever possible.
* If something is already implemented, verify it rather than recreating it.
* Clearly tell me which files you modify and why.
* Do not make unrelated changes.

PROJECT ROADMAP

PHASE 1 — PLANNING
Timeline: Week 1
Tasks:

1. Define system architecture.
2. Map ROS2 nodes and topics.
3. Create/verify project folder structure.
4. Set up GitHub repository.
5. Define the energy cost model.

Expected output:

* Architecture diagram
* GitHub repository
* Folder skeleton

PHASE 2 — ENVIRONMENT SETUP
Timeline: Week 2
Tasks:

1. Ubuntu 22.04 setup.
2. ROS2 Humble setup.
3. Gazebo setup.
4. Qiskit + Aer setup.
5. VS Code + required extensions.

Expected output:

* All required tools installed.
* Basic ROS2 test passes.

PHASE 3 — SIMULATION
Timeline: Week 3
Tasks:

1. Create lunar terrain in Gazebo.
2. Create/verify rover URDF/Xacro.
3. Attach virtual sensors.
4. Test keyboard/manual rover movement.
5. Visualize the rover in RViz2.

Expected output:

* Rover moves correctly in lunar terrain.
* Sensors publish data.

PHASE 4 — SENSORS
Timeline: Week 4
Tasks:

1. Implement LiDAR processor node.
2. Implement IMU processor node.
3. Implement encoder odometry node.
4. Test ROS2 topics.
5. Validate sensor data.

Expected output:
These topics should be live and producing valid data:

* /scan
* /imu/data
* /odom

PHASE 5 — MAPPING
Timeline: Week 5
Tasks:

1. Build occupancy grid.
2. Create terrain classification node.
3. Classify terrain into:

   * flat
   * rocky
   * craters
4. Visualize the map in RViz2.
5. Store terrain data.

Expected output:

* Classified terrain map updating in real time.

PHASE 6 — ENERGY MODEL
Timeline: Week 6
Tasks:

1. Define an energy cost function.
2. Assign slope-based weights.
3. Add terrain-type multipliers.
4. Calculate a roughness factor from LiDAR.
5. Publish a weighted graph.

Expected output:

* Energy-weighted terrain graph represented as a NetworkX object.

PHASE 7 — CLASSICAL PATH PLANNING
Timeline: Week 7
Tasks:

1. Implement Dijkstra's algorithm.
2. Implement A* algorithm.
3. Compare Dijkstra and A* results.
4. Publish the selected path as a ROS2 topic.
5. Measure energy usage.

Expected output:

* Classical A* path published on:
  /path/classical

PHASE 8 — QUANTUM OPTIMIZATION
Timeline: Weeks 8–9
Tasks:

1. Formulate the terrain/path-planning problem as a QUBO.
2. Build a QAOA circuit using Qiskit.
3. Run the circuit using AerSimulator.
4. Extract optimal waypoints from the quantum result.
5. Compare the quantum result against the classical solution.

Required technology:

* Python
* Qiskit
* Qiskit-Aer
* QAOA

Expected output:

* Optimized quantum path published on:
  /path/quantum

PHASE 9 — INTEGRATION
Timeline: Week 10
Tasks:

1. Connect the quantum optimizer to ROS2.
2. Implement a path executor node.
3. Send waypoints to the rover motors.
4. Perform a full pipeline test.
5. Perform an end-to-end navigation run.

Required technology:

* ROS2
* Nav2
* Python

Expected output:
Complete pipeline:

Sensors
↓
Terrain / Mapping
↓
Energy Model
↓
Quantum Optimizer
↓
Path
↓
Path Executor
↓
Motors

PHASE 10 — EVALUATION
Timeline: Week 11
Tasks:

1. Record energy usage metrics.
2. Compare classical vs quantum navigation.
3. Plot energy comparison graphs.
4. Calculate path efficiency.
5. Document the results.

Required technology:

* Python
* Pandas
* Matplotlib

Expected output:

* Graphs and metrics comparing classical and quantum approaches.
* Documented experimental results.

IMPORTANT:
Do not claim that quantum optimization saves energy unless the experimental results actually demonstrate this. Measure and report the results objectively.

PHASE 11 — REAL ROBOT
Timeline: Week 12, Part 1
Tasks:

1. Connect the Arjuna rover hardware.
2. Flash Arduino firmware.
3. Map ROS2 commands/topics to the physical motors.
4. Run live navigation.
5. Debug real-world hardware issues.

Hardware:

* Arjuna Kit
* Arduino
* Jetson Nano

Expected output:

* Physical rover navigating lunar-like terrain.

PHASE 12 — DOCUMENTATION & REPORT
Timeline: Week 12, Part 2
Tasks:

1. Write the final project report.
2. Create presentation slides.
3. Create/update GitHub README.
4. Prepare viva answers.
5. Prepare/research the research-paper component.

Required tools:

* LaTeX
* PowerPoint
* GitHub

Expected output:

* Full project report
* Presentation slides
* GitHub README
* Viva preparation material
* Research paper draft/guidance

CURRENT PRIORITY

Phases 1 through 9 are fully implemented and verified.

The current priority is **PHASE 10 — EVALUATION**, which involves building the `battery_monitor` and `evaluator` nodes in `src/evaluation_pkg` to record energy metrics, generate comparative plots, and analyze classical vs quantum performance.

PHASE 8 IMPLEMENTATION REQUIREMENTS

The quantum optimization component should:

1. Receive or access the energy-weighted terrain/path graph.

2. Convert the relevant path-planning problem into a QUBO formulation.

3. Build a QAOA circuit using Qiskit.

4. Execute it with Qiskit AerSimulator.

5. Decode the resulting bitstring/solution.

6. Convert the solution into rover waypoints/path coordinates.

7. Publish the resulting path through:

   /path/quantum

8. Keep the classical A* implementation intact so that both solutions can be compared.

9. Provide a clear interface between ROS2 and the quantum optimizer.

10. Log useful metrics such as:

* path length
* estimated energy cost
* computation time
* number of waypoints
* objective/QUBO cost

IMPORTANT ENGINEERING RULES

* Do not hard-code fake quantum results.
* Do not use mocked results when an actual Qiskit/Aer simulation can be performed.
* If the full terrain graph is too large for practical QAOA simulation, implement a well-defined reduced/subgraph formulation and clearly document the reduction.
* Preserve reproducibility with configurable random seeds where appropriate.
* Keep the QUBO formulation mathematically documented in the code.
* Make parameters configurable instead of hard-coded wherever practical.
* Add error handling for missing graph/path data.
* Make ROS2 nodes properly discoverable and launchable.
* Update package dependencies and requirements where necessary.
* Add or update launch files where necessary.
* Add tests for important components.
* Do not break the existing Gazebo simulation.

TESTING REQUIREMENTS

After implementation:

1. Build the ROS2 workspace.
2. Source the workspace.
3. Launch the required simulation/nodes.
4. Verify ROS2 topics.
5. Verify the quantum optimizer receives valid input.
6. Run the QAOA/Aer simulation.
7. Verify /path/quantum publishes a valid path.
8. Verify the classical /path/classical path still works.
9. Compare both outputs.
10. Report any remaining errors clearly.

Useful commands include:

ros2 topic list
ros2 topic echo /scan
ros2 topic echo /imu/data
ros2 topic echo /odom
ros2 topic echo /path/classical
ros2 topic echo /path/quantum





