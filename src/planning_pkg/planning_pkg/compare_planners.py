#!/usr/bin/env python3
"""
QAOA vs. A* comparison harness (reporting only).

planning_pkg.quantum_optimizer is deliberately free of any classical planner:
QAOA builds its own corridor, QUBO and path. This module is the only place where
the two are put side by side. It runs QAOA first, then A* on the same graph as an
independent reference, and writes the delta to results/paths/latest_quantum.json
under "comparison". Nothing here feeds back into QAOA.

    python -m planning_pkg.compare_planners --goal-x 1.5 --goal-y 1.5
"""

from __future__ import annotations

import argparse
import os
from datetime import datetime
from typing import Any, Dict, List, Optional

from planning_pkg.classical_planning import PlanResult, run_astar
from planning_pkg.graph_model import load_graph, resolve_node
from planning_pkg.quantum_optimizer import (
    QuantumPlanResult,
    run_qaoa,
    save_quantum_results,
)


def compare_quantum_classical(
    classical: PlanResult,
    quantum: QuantumPlanResult,
) -> Dict[str, Any]:
    return {
        "energy_delta": quantum.total_energy - classical.total_energy,
        "abs_energy_delta": abs(quantum.total_energy - classical.total_energy),
        "distance_delta_m": quantum.total_distance_m - classical.total_distance_m,
        "runtime_delta_ms": quantum.runtime_ms - classical.runtime_ms,
        "energy_ratio": (
            quantum.total_energy / classical.total_energy
            if classical.total_energy > 0 else None
        ),
        "paths_identical": quantum.path_nodes == classical.path_nodes,
        "quantum": {
            "energy": quantum.total_energy,
            "distance_m": quantum.total_distance_m,
            "waypoints": quantum.num_waypoints,
            "runtime_ms": quantum.runtime_ms,
            "qubits": quantum.qubits,
            "reduced_edges": quantum.reduced_edges,
            "shots": quantum.shots,
            "reps": quantum.reps,
            "valid_samples": quantum.valid_samples,
            "device": quantum.device,
        },
        "classical": {
            "algorithm": classical.algorithm,
            "energy": classical.total_energy,
            "distance_m": classical.total_distance_m,
            "waypoints": classical.num_waypoints,
            "runtime_ms": classical.runtime_ms,
        },
        "compared_at": datetime.now().isoformat(),
    }


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run QAOA, then A* as a reference, and save the comparison."
    )
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--graph", default="results/graphs/latest.graphml")
    parser.add_argument("--out", default="results/paths")
    parser.add_argument("--max-edges", type=int, default=12)
    parser.add_argument("--corridor-width", type=float, default=0.5)
    parser.add_argument("--reps", type=int, default=1)
    parser.add_argument("--shots", type=int, default=2048)
    parser.add_argument("--restarts", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--penalty", type=float, default=None)
    parser.add_argument("--device", default="auto", choices=["auto", "CPU", "GPU"])
    args = parser.parse_args(argv)

    print(f"[compare_planners] Loading graph: {args.graph}")
    try:
        if not os.path.exists(args.graph):
            raise FileNotFoundError(f"Weighted graph file not found at: {args.graph}")
        graph = load_graph(args.graph)
        start = resolve_node(graph, args.start_x, args.start_y)
        goal = resolve_node(graph, args.goal_x, args.goal_y)
        quantum = run_qaoa(
            graph, start, goal,
            max_edges=args.max_edges,
            width_ratio=args.corridor_width,
            reps=args.reps,
            shots=args.shots,
            restarts=args.restarts,
            seed=args.seed,
            penalty=args.penalty,
            device=args.device,
        )
        classical = run_astar(graph, start, goal)
    except Exception as exc:
        print(f"[compare_planners] ERROR: {exc}")
        return 1

    comparison = compare_quantum_classical(classical, quantum)
    saved = save_quantum_results(quantum, args.out, comparison)

    print(
        f"[compare_planners] A*:   {classical.total_energy:.4f} energy / "
        f"{classical.num_waypoints} waypoints"
    )
    print(
        f"[compare_planners] QAOA: {quantum.total_energy:.4f} energy / "
        f"{quantum.num_waypoints} waypoints / {quantum.qubits} qubits / "
        f"{quantum.runtime_ms:.2f} ms on {quantum.device}"
    )
    print(
        f"[compare_planners] paths identical: {comparison['paths_identical']}; "
        f"valid samples={quantum.valid_samples}/{quantum.sampled_candidates}"
    )
    print(f"[compare_planners] Saved: {saved['latest']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
