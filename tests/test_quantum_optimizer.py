#!/usr/bin/env python3
"""Pure-Python Phase-8 tests.

The QUBO construction and reduced-corridor logic are tested without requiring
Qiskit. The actual Aer/QAOA execution is intentionally a separate smoke test
because CI environments may not have a quantum simulator installed.
"""

import math
import os
import subprocess
import sys

import networkx as nx
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.quantum_optimizer import (
    _lattice_shape,
    build_quantum_corridor,
    build_qubo,
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
    reduced = build_quantum_corridor(graph, "A", "D", max_edges=8)
    assert nx.has_path(reduced, "A", "D")
    assert reduced.number_of_edges() <= 8


def _grid(n, blocked=()):
    g = nx.Graph()
    for r in range(n):
        for c in range(n):
            if (r, c) not in blocked:
                g.add_node((r, c), pos_x=float(c), pos_y=float(r))
    for r, c in list(g.nodes):
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            nb = (r + dr, c + dc)
            if nb in g:
                g.add_edge((r, c), nb, weight=math.hypot(dr, dc))
    return g


def test_lattice_shape_fits_edge_budget():
    assert _lattice_shape(12) == (3, 3)
    assert _lattice_shape(7) == (3, 2)
    assert _lattice_shape(4) == (1, 5)
    rows, cols = _lattice_shape(40)
    assert rows % 2 == 1 and rows * (cols - 1) + cols * (rows - 1) <= 40


def test_lattice_corridor_on_large_grid_uses_no_classical_planner(monkeypatch):
    import planning_pkg.quantum_optimizer as qo

    def forbidden(*args, **kwargs):
        raise AssertionError("corridor must not call a classical planner")

    monkeypatch.setattr(nx, "shortest_simple_paths", forbidden)
    monkeypatch.setattr(nx, "shortest_path", forbidden)
    monkeypatch.setattr(nx, "dijkstra_path", forbidden)

    g = _grid(15)
    start, goal = (7, 1), (7, 13)
    reduced = qo.build_quantum_corridor(g, start, goal, max_edges=12)
    assert reduced.number_of_edges() == 12
    assert goal in nx.node_connected_component(reduced, start)
    for u, v, data in reduced.edges(data=True):
        segment = data["path_nodes"]
        assert {segment[0], segment[-1]} == {u, v}
        assert nx.is_path(g, segment)


def test_lattice_corridor_routes_around_a_wall():
    # Wall across the straight start-goal line, open above and below.
    wall = {(r, 7) for r in range(4, 11)}
    g = _grid(15, blocked=wall)
    start, goal = (7, 1), (7, 13)
    reduced = build_quantum_corridor(g, start, goal, max_edges=12)
    assert nx.has_path(reduced, start, goal)
    for _, _, data in reduced.edges(data=True):
        assert not wall.intersection(data["path_nodes"])


def test_qubo_has_edge_and_activation_variables(graph):
    reduced = build_quantum_corridor(graph, "A", "D", max_edges=8)
    qubo = build_qubo(reduced, "A", "D")
    assert qubo.num_variables >= reduced.number_of_edges()
    assert len(qubo.edge_variables) == reduced.number_of_edges()
    assert qubo.penalty > 0
    assert qubo.variable_names


def test_qubo_penalty_prefers_valid_selection(graph):
    reduced = build_quantum_corridor(graph, "A", "D", max_edges=8)
    qubo = build_qubo(reduced, "A", "D")

    zeros = [0] * qubo.num_variables
    zeros_value = qubo.value(zeros)

    # Select all edge variables: this should incur degree-constraint penalties.
    all_edges = [0] * qubo.num_variables
    for i in range(len(qubo.edge_variables)):
        all_edges[i] = 1
    assert qubo.value(all_edges) > zeros_value


def test_ising_conversion_is_finite(graph):
    reduced = build_quantum_corridor(graph, "A", "D", max_edges=8)
    qubo = build_qubo(reduced, "A", "D")
    h, zz = qubo_to_ising(qubo)
    assert len(h) == qubo.num_variables or len(h) > 0
    assert all(math.isfinite(v) for v in h.values())
    assert all(math.isfinite(v) for v in zz.values())


def test_quantum_optimizer_does_not_import_classical_planner():
    """QAOA must be independent of A*/Dijkstra: importing it never loads them."""
    code = (
        "import sys; sys.path.insert(0, sys.argv[1]);"
        "import planning_pkg.quantum_optimizer as qo;"
        "assert 'planning_pkg.classical_planning' not in sys.modules;"
        "assert not hasattr(qo, 'run_astar') and not hasattr(qo, 'run_dijkstra')"
    )
    subprocess.run([sys.executable, "-c", code, PKG_DIR], check=True)


def test_aer_expectation_matches_exact_qubo_cost(graph):
    """The Aer-side <C> must equal sum_x p(x) * QUBO(x) over all basis states."""
    pytest.importorskip("qiskit_aer")
    import numpy as np
    from qiskit import transpile
    from qiskit_aer import AerSimulator

    from planning_pkg.quantum_optimizer import _cost_lookup, _ising_operator, _qaoa_circuit

    reduced = build_quantum_corridor(graph, "A", "D", max_edges=8)
    qubo = build_qubo(reduced, "A", "D")
    n = qubo.num_variables
    h, zz = qubo_to_ising(qubo)

    qc = _qaoa_circuit(n, h, zz, [0.4, 1.1], [0.3, 0.8])
    qc.save_statevector()
    qc.save_expectation_value(_ising_operator(qubo), list(range(n)))
    backend = AerSimulator(method="statevector")
    data = backend.run(transpile(qc, backend)).result().data(0)

    probs = np.abs(np.asarray(data["statevector"])) ** 2
    exact = float(np.dot(probs, _cost_lookup(qubo)))
    assert float(np.real(data["expectation_value"])) == pytest.approx(exact, rel=1e-9)


def test_resolve_device_auto_and_rejects_missing():
    pytest.importorskip("qiskit_aer")
    from qiskit_aer import AerSimulator

    from planning_pkg.quantum_optimizer import resolve_device

    available = AerSimulator().available_devices()
    assert resolve_device("auto") in available
    assert resolve_device("cpu") == "CPU"
    if "GPU" not in available:
        with pytest.raises(RuntimeError, match="qiskit-aer-gpu"):
            resolve_device("GPU")
