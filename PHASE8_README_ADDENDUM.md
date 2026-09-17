## Phase 8 — Quantum optimization

Phase 8 is implemented in `planning_pkg.quantum_optimizer`. It loads the same
energy-weighted GraphML used by Phase 7, builds a small quantum corridor around
the A* backbone, formulates edge selection as a QUBO, and solves it with a
hand-built QAOA circuit on Qiskit Aer.

Pure-Python CLI:

```bash
python -m planning_pkg.quantum_optimizer \
  --start-x 0 --start-y 0 --goal-x 10 --goal-y 10 \
  --graph results/graphs/latest.graphml \
  --max-edges 10 --reps 1 --shots 2048
```

ROS 2:

```bash
ros2 launch planning_pkg quantum_optimizer.launch.py \
  goal_x:=10.0 goal_y:=10.0
```

Output:
- `/path/quantum`
- `results/paths/latest_quantum.json`

The quantum instance is intentionally reduced to roughly 8–12 edges so it is
tractable on the CPU `AerSimulator`. The resulting macro-edge path is expanded
back into the original terrain graph before metrics are reported.
