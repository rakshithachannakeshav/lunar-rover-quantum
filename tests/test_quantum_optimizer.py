#!/usr/bin/env python3
"""Pure-Python Phase-8 tests.

The QUBO construction and reduced-corridor logic are tested without requiring
Qiskit. The actual Aer/QAOA execution is intentionally a separate smoke test
because CI environments may not have a quantum simulator installed.
"""

import math
import os
import sys

import networkx as nx
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.classical_planning import run_astar
from planning_pkg.quantum_optimizer import (
    build_quantum_corridor,
    build_qubo,
    compare_quantum_classical,
    qubo_to_ising,
)


@pytest.fixture
def graph():
    g = nx.Graph()
    # Main route A-B-C-D plus a higher-cost detour through X/Y.
    for name, x, y in [
        ("A", 0, 0), ("B", 1, 0), ("C", 2, 0), ("D", 3, 0),
        ("X", 1, 1), ("Y", 2, 1),
    ]:
        g.add_node(name, pos_x=float(x), pos_y=float(y))
    g.add_edge("A", "B", weight=1.0)
    g.add_edge("B", "C", weight=1.0)
    g.add_edge("C", "D", weight=1.0)
    g.add_edge("A", "X", weight=1.4)
    g.add_edge("X", "Y", weight=1.4)
    g.add_edge("Y", "D", weight=1.4)
    g.add_edge("B", "X", weight=0.8)
    g.add_edge("C", "Y", weight=0.8)
    return g


def test_corridor_is_connected_and_small(graph):
    baseline = run_astar(graph, "A", "D")
    reduced = build_quantum_corridor(
        graph, "A", "D", baseline.path_nodes, max_edges=6
    )
    assert nx.has_path(reduced, "A", "D")
    assert reduced.number_of_edges() <= 6


def test_qubo_has_edge_and_activation_variables(graph):
    baseline = run_astar(graph, "A", "D")
    reduced = build_quantum_corridor(
        graph, "A", "D", baseline.path_nodes, max_edges=6
    )
    qubo = build_qubo(reduced, "A", "D")
    assert qubo.num_variables >= reduced.number_of_edges()
    assert len(qubo.edge_variables) == reduced.number_of_edges()
    assert qubo.penalty > 0
    assert qubo.variable_names


def test_qubo_penalty_prefers_valid_selection(graph):
    baseline = run_astar(graph, "A", "D")
    reduced = build_quantum_corridor(
        graph, "A", "D", baseline.path_nodes, max_edges=6
    )
    qubo = build_qubo(reduced, "A", "D")

    zeros = [0] * qubo.num_variables
    zeros_value = qubo.value(zeros)

    # Select all edge variables: this should incur degree-constraint penalties.
    all_edges = [0] * qubo.num_variables
    for i in range(len(qubo.edge_variables)):
        all_edges[i] = 1
    assert qubo.value(all_edges) > zeros_value


def test_ising_conversion_is_finite(graph):
    baseline = run_astar(graph, "A", "D")
    reduced = build_quantum_corridor(
        graph, "A", "D", baseline.path_nodes, max_edges=6
    )
    qubo = build_qubo(reduced, "A", "D")
    h, zz = qubo_to_ising(qubo)
    assert len(h) == qubo.num_variables or len(h) > 0
    assert all(math.isfinite(v) for v in h.values())
    assert all(math.isfinite(v) for v in zz.values())


def test_classical_comparison_schema(graph):
    baseline = run_astar(graph, "A", "D")
    # Minimal stand-in object for the comparison helper.
    from planning_pkg.quantum_optimizer import QuantumPlanResult
    quantum = QuantumPlanResult(
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
    result = compare_quantum_classical(baseline, quantum)
    assert result["paths_identical"] is True
    assert result["energy_delta"] == 0.0
    assert "quantum" in result
    assert "classical" in result
