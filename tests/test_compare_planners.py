#!/usr/bin/env python3
"""Tests for the QAOA-vs-A* reporting harness (kept out of the QAOA module)."""

import os
import sys

import networkx as nx
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.classical_planning import run_astar
from planning_pkg.compare_planners import compare_quantum_classical
from planning_pkg.quantum_optimizer import QuantumPlanResult, save_quantum_results


@pytest.fixture
def graph():
    g = nx.Graph()
    for name, x, y in [("A", 0, 0), ("B", 1, 0), ("C", 2, 0), ("D", 3, 0)]:
        g.add_node(name, pos_x=float(x), pos_y=float(y))
    g.add_edge("A", "B", weight=1.0)
    g.add_edge("B", "C", weight=1.0)
    g.add_edge("C", "D", weight=1.0)
    return g


def _quantum_like(baseline):
    return QuantumPlanResult(
        algorithm="QAOA",
        path_nodes=baseline.path_nodes,
        path_coords=baseline.path_coords,
        total_energy=baseline.total_energy,
        total_distance_m=baseline.total_distance_m,
        num_waypoints=baseline.num_waypoints,
        runtime_ms=12.0,
        timestamp="2026-09-15T00:00:00",
        best_bitstring="0000",
        qaoa_expectation=3.0,
        reduced_nodes=4,
        reduced_edges=3,
        qubits=5,
        shots=128,
        reps=1,
        penalty=10.0,
        valid_samples=64,
        sampled_candidates=128,
    )


def test_classical_comparison_schema(graph):
    baseline = run_astar(graph, "A", "D")
    result = compare_quantum_classical(baseline, _quantum_like(baseline))
    assert result["paths_identical"] is True
    assert result["energy_delta"] == 0.0
    assert "quantum" in result
    assert "classical" in result


def test_save_quantum_results_comparison_is_optional(graph, tmp_path):
    import json

    baseline = run_astar(graph, "A", "D")
    quantum = _quantum_like(baseline)

    saved = save_quantum_results(quantum, tmp_path)
    plain = json.load(open(saved["latest"], encoding="utf-8"))
    assert "quantum" in plain and "comparison" not in plain

    comparison = compare_quantum_classical(baseline, quantum)
    saved = save_quantum_results(quantum, tmp_path, comparison)
    with_cmp = json.load(open(saved["latest"], encoding="utf-8"))
    assert "comparison" in with_cmp
