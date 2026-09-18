# Phase 8 — Quantum optimizer design

## Goal

Add a CPU-simulated QAOA planner that consumes the Phase-6 weighted graph and
publishes `/path/quantum`, while remaining comparable with the Phase-7 A* path.

## Why a reduced graph

The verified Phase-6 graph can contain ~94 nodes / 266 edges. Encoding every
edge as a QAOA variable would require an impractical number of qubits for a
demonstration on `AerSimulator`. Phase 8 therefore constructs a small quantum
corridor around the Phase-7 A* route.

The corridor:
1. splits the A* route into a small number of anchor-to-anchor segments;
2. represents each segment by a macro-edge containing its original node sequence;
3. keeps the A* segment plus at most one low-cost alternative segment;
4. limits the reduced graph to `--max-edges` (default 10).

This preserves a physically valid path expansion while keeping the quantum
instance small enough for a local simulator.

## QUBO

For every reduced-graph edge `e`, binary `x_e` indicates whether it is selected.

Objective:
`sum_e weight(e) * x_e`

Endpoint constraints:
`P * (degree(start)-1)^2`
`P * (degree(goal)-1)^2`

For every internal anchor `v`, an auxiliary binary `y_v` represents whether the
anchor is active:
`P * (degree(v)-2*y_v)^2`

This permits degree 0 or 2 for internal anchors and degree 1 at start/goal.
The decoded sample is additionally required to be a single simple start-to-goal
path. Invalid samples are discarded.

## QAOA

The QUBO is mapped to an Ising diagonal Hamiltonian using `x=(1-Z)/2`.
A hand-built QAOA circuit applies:
- Hadamards for the initial |+> state;
- cost unitaries for Z and ZZ terms;
- X mixer rotations.

Angles are optimized with SciPy against an exact statevector expectation.
Final samples are taken from Qiskit Aer `AerSimulator`.

Default settings:
- `p=1`
- 2048 shots
- 4 optimization restarts
- 10 reduced edges
- deterministic seed 7

## Outputs

- `/path/quantum` — `nav_msgs/msg/Path`
- `results/paths/latest_quantum.json`
- timestamped quantum result history
- classical-vs-quantum comparison metrics

A shallow QAOA run is allowed to miss a feasible sample. In that case the node
falls back to the known-valid A* path instead of publishing an invalid route;
the result records `fallback_to_classical=true`.
