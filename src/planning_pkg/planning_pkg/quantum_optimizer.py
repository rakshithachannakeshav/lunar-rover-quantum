#!/usr/bin/env python3
"""
Phase 8: Quantum path optimization with a small, tractable QAOA instance.

The Phase-6 graph can contain hundreds of edges. Running QAOA directly on that
graph is not a useful CPU demonstration because the qubit count would explode.
This module therefore builds a small "quantum corridor" without any classical
planner: a lattice of anchor nodes laid out geometrically between start and goal,
with neighbouring anchors joined by straight-line walks through the terrain graph.
Each reduced-graph edge is an edge-selection binary in the QUBO; its metadata
contains the original path segment so the quantum solution can be expanded back
to the real terrain graph. Choosing which edges form the route is left to QAOA.

The QUBO enforces:
  * exactly one selected edge at the start,
  * exactly one selected edge at the goal,
  * every internal anchor has degree 0 or 2 (no branching),
and the decoded solution is additionally required to form one valid start->goal
path. Subtour/corruption protection is deliberately handled during decoding;
the final path is always validated against the original graph.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import networkx as nx
import numpy as np
from scipy.optimize import minimize

from planning_pkg.graph_model import load_graph, resolve_node


EdgeId = Tuple[Any, Any, int]


@dataclass
class QuboModel:
    """Sparse QUBO: E(x) = constant + sum Qii*x_i + sum Qij*x_i*x_j."""

    num_variables: int
    linear: Dict[int, float]
    quadratic: Dict[Tuple[int, int], float]
    constant: float
    variable_names: List[str]
    edge_variables: List[EdgeId]
    activation_variables: Dict[Any, int]
    penalty: float

    def value(self, bits: Sequence[int]) -> float:
        value = self.constant
        for i, coeff in self.linear.items():
            value += coeff * int(bits[i])
        for (i, j), coeff in self.quadratic.items():
            value += coeff * int(bits[i]) * int(bits[j])
        return float(value)


@dataclass
class QuantumPlanResult:
    """Phase-8 output and diagnostics."""

    algorithm: str
    path_nodes: List[Any]
    path_coords: List[Tuple[float, float]]
    total_energy: float
    total_distance_m: float
    num_waypoints: int
    runtime_ms: float
    timestamp: str
    best_bitstring: str
    qaoa_expectation: float
    reduced_nodes: int
    reduced_edges: int
    qubits: int
    shots: int
    reps: int
    penalty: float
    valid_samples: int
    sampled_candidates: int
    device: str = "CPU"

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["path_nodes"] = [
            list(node) if isinstance(node, tuple) else node for node in self.path_nodes
        ]
        data["path_coords"] = [list(coord) for coord in self.path_coords]
        return data


def _edge_key(u: Any, v: Any, key: int) -> EdgeId:
    return (u, v, key)


def _path_weight(graph: nx.Graph, path: Sequence[Any]) -> float:
    total = 0.0
    for u, v in zip(path[:-1], path[1:]):
        data = graph.get_edge_data(u, v)
        if data is None:
            raise ValueError(f"Path uses missing edge {u!r}->{v!r}.")
        if graph.is_multigraph():
            # For an original graph we do not expect MultiGraph; choose minimum.
            weights = [float(d.get("weight", 0.0)) for d in data.values()]
            total += min(weights)
        else:
            total += float(data.get("weight", 0.0))
    return total


def _path_distance(graph: nx.Graph, path: Sequence[Any]) -> float:
    total = 0.0
    for u, v in zip(path[:-1], path[1:]):
        ux = float(graph.nodes[u].get("pos_x", 0.0))
        uy = float(graph.nodes[u].get("pos_y", 0.0))
        vx = float(graph.nodes[v].get("pos_x", 0.0))
        vy = float(graph.nodes[v].get("pos_y", 0.0))
        total += math.hypot(vx - ux, vy - uy)
    return total


def _connected(graph: nx.Graph, a: Any, b: Any) -> bool:
    """Connectivity via connected components (no shortest-path search)."""
    return b in nx.node_connected_component(graph, a)


def _lattice_shape(max_edges: int) -> Tuple[int, int]:
    """Largest rows x cols anchor lattice (rows odd) with at most max_edges edges.

    A 4-connected rows x cols lattice has rows*(cols-1) + cols*(rows-1) edges.
    Rows are odd so start and goal sit on the centre row; a single row is only
    used when max_edges is too small for three.
    """
    best: Optional[Tuple[int, int, int, int]] = None
    for rows in range(1, max_edges + 2, 2):
        for cols in range(2, max_edges + 2):
            edges = rows * (cols - 1) + cols * (rows - 1)
            if edges > max_edges:
                break
            candidate = (int(rows >= 3), edges, rows, cols)
            if best is None or candidate > best:
                best = candidate
    if best is None:
        raise ValueError("max_edges must be at least 1.")
    return best[2], best[3]


def _straight_walk(graph: nx.Graph, a: Any, b: Any) -> Optional[List[Any]]:
    """Walk a->b by always stepping to the neighbour geometrically closest to b.

    Pure geometry, no cost-based search: the walk ignores edge weights, so the
    energy trade-off between macro-edges is left entirely to QAOA. Returns None
    if an obstacle stops the walk from getting closer to b.
    """
    bx = float(graph.nodes[b].get("pos_x", 0.0))
    by = float(graph.nodes[b].get("pos_y", 0.0))

    def dist(n: Any) -> float:
        return math.hypot(float(graph.nodes[n].get("pos_x", 0.0)) - bx,
                          float(graph.nodes[n].get("pos_y", 0.0)) - by)

    path = [a]
    current = a
    while current != b:
        options = [(dist(n), str(n), n) for n in graph.neighbors(current)]
        if not options:
            return None
        d, _, nxt = min(options)
        if d >= dist(current):
            return None
        path.append(nxt)
        current = nxt
    return path


def build_quantum_corridor(
    graph: nx.Graph,
    start: Any,
    goal: Any,
    max_edges: int = 12,
    width_ratio: float = 0.5,
) -> nx.MultiGraph:
    """
    Build a <=max_edges reduced graph from a geometric lattice, with no classical planner.

    Anchors form a rows x cols lattice aligned with the start->goal line, spanning
    +/- width_ratio * |goal - start| either side of it, each snapped to the nearest
    terrain node. Lattice neighbours are joined by a straight-line walk through the
    terrain graph; that walk becomes one macro-edge whose weight is its real energy.
    """
    if max_edges < 1:
        raise ValueError("max_edges must be at least 1.")

    # If the original graph is already tiny, retain it exactly.
    if graph.number_of_edges() <= max_edges:
        reduced = nx.MultiGraph()
        for node, attrs in graph.nodes(data=True):
            reduced.add_node(node, **dict(attrs))
        for u, v, data in graph.edges(data=True):
            reduced.add_edge(
                u, v,
                weight=float(data.get("weight", 0.0)),
                path_nodes=[u, v],
                macro=False,
            )
        return reduced

    def pos(n: Any) -> np.ndarray:
        return np.array([float(graph.nodes[n].get("pos_x", 0.0)),
                         float(graph.nodes[n].get("pos_y", 0.0))])

    nodes = list(graph.nodes)
    xy = np.array([pos(n) for n in nodes])
    s_xy, g_xy = pos(start), pos(goal)
    axis = g_xy - s_xy
    length = float(np.hypot(*axis))
    if length == 0.0:
        raise ValueError("Start and goal resolve to the same position.")
    perp = np.array([-axis[1], axis[0]]) / length

    rows, cols = _lattice_shape(max_edges)
    mid = rows // 2
    half_width = width_ratio * length

    # Snap lattice points to terrain nodes; start/goal are fixed, duplicates dropped.
    anchors: Dict[Tuple[int, int], Any] = {}
    used = {start, goal}
    for i in range(rows):
        for j in range(cols):
            if (i, j) == (mid, 0):
                anchors[(i, j)] = start
                continue
            if (i, j) == (mid, cols - 1):
                anchors[(i, j)] = goal
                continue
            offset = (i - mid) / mid * half_width if mid else 0.0
            point = s_xy + axis * (j / (cols - 1)) + perp * offset
            node = nodes[int(np.argmin(np.hypot(*(xy - point).T)))]
            if node in used:
                continue
            anchors[(i, j)] = node
            used.add(node)

    reduced = nx.MultiGraph()
    for node in anchors.values():
        reduced.add_node(node, **dict(graph.nodes[node]))

    anchor_set = set(anchors.values())
    for (i, j), a in anchors.items():
        for ni, nj in ((i + 1, j), (i, j + 1)):
            b = anchors.get((ni, nj))
            if b is None:
                continue
            segment = _straight_walk(graph, a, b)
            # Drop blocked walks and walks that cut through another anchor.
            if segment is None or anchor_set.intersection(segment[1:-1]):
                continue
            reduced.add_edge(
                a, b,
                weight=_path_weight(graph, segment),
                path_nodes=segment,
                macro=True,
            )

    if not _connected(reduced, start, goal):
        raise ValueError(
            "Quantum corridor has no start-to-goal connection "
            "(obstacles block the lattice). Increase --max-edges or --corridor-width."
        )
    return reduced


def _add_linear(linear: Dict[int, float], i: int, value: float) -> None:
    linear[i] = linear.get(i, 0.0) + value


def _add_quadratic(
    quadratic: Dict[Tuple[int, int], float], i: int, j: int, value: float
) -> None:
    if i == j:
        raise ValueError("QUBO quadratic terms require distinct variables.")
    key = (i, j) if i < j else (j, i)
    quadratic[key] = quadratic.get(key, 0.0) + value


def _add_squared_sum_minus_target(
    linear: Dict[int, float],
    quadratic: Dict[Tuple[int, int], float],
    constant: float,
    variables: Sequence[int],
    target: int,
    penalty: float,
) -> float:
    """Add P*(sum(x)-target)^2 to a sparse QUBO."""
    constant += penalty * (target ** 2)
    for i in variables:
        # x_i^2 = x_i
        _add_linear(linear, i, penalty * (1.0 - 2.0 * target))
    for idx, i in enumerate(variables):
        for j in variables[idx + 1:]:
            _add_quadratic(quadratic, i, j, 2.0 * penalty)
    return constant


def build_qubo(
    reduced_graph: nx.MultiGraph,
    start: Any,
    goal: Any,
    penalty: Optional[float] = None,
) -> QuboModel:
    """
    Build an edge-selection QUBO.

    Start/goal: degree exactly one.
    Internal nodes: degree either zero or two, represented with one auxiliary
    activation binary y_v:
        P * (degree(v) - 2*y_v)^2
    """
    if start not in reduced_graph or goal not in reduced_graph:
        raise ValueError("Start and goal must be present in the reduced graph.")
    if not _connected(reduced_graph, start, goal):
        raise ValueError("Reduced graph must connect start to goal.")

    edges = list(reduced_graph.edges(keys=True, data=True))
    if not edges:
        raise ValueError("Cannot build a QUBO from an empty edge set.")

    weights = [float(data.get("weight", 0.0)) for _, _, _, data in edges]
    if any(w < 0 for w in weights):
        raise ValueError("QAOA path QUBO requires non-negative edge weights.")

    if penalty is None:
        penalty = max(1.0, 2.0 * sum(weights))

    linear: Dict[int, float] = {}
    quadratic: Dict[Tuple[int, int], float] = {}
    constant = 0.0
    names: List[str] = []
    edge_variables: List[EdgeId] = []

    incident: Dict[Any, List[int]] = {}
    for idx, (u, v, key, data) in enumerate(edges):
        edge_variables.append(_edge_key(u, v, key))
        names.append(f"edge:{u}->{v}:{key}")
        _add_linear(linear, idx, float(data.get("weight", 0.0)))
        incident.setdefault(u, []).append(idx)
        incident.setdefault(v, []).append(idx)

    activation_variables: Dict[Any, int] = {}
    next_var = len(edges)

    # Endpoint degree = 1.
    for endpoint in (start, goal):
        constant = _add_squared_sum_minus_target(
            linear, quadratic, constant,
            incident.get(endpoint, []), target=1, penalty=penalty,
        )

    # Internal anchor degree = 0 or 2, using one auxiliary binary.
    for node in reduced_graph.nodes:
        if node in (start, goal):
            continue
        vars_for_node = incident.get(node, [])
        if not vars_for_node:
            continue
        y = next_var
        next_var += 1
        activation_variables[node] = y
        names.append(f"active:{node}")

        # P*(d - 2y)^2 = P*d^2 - 4P*d*y + 4P*y
        # d^2 contributes P*x_i + 2P*x_i*x_j.
        for i in vars_for_node:
            _add_linear(linear, i, penalty)
            _add_quadratic(quadratic, i, y, -4.0 * penalty)
        for idx, i in enumerate(vars_for_node):
            for j in vars_for_node[idx + 1:]:
                _add_quadratic(quadratic, i, j, 2.0 * penalty)
        _add_linear(linear, y, 4.0 * penalty)

    return QuboModel(
        num_variables=next_var,
        linear=linear,
        quadratic=quadratic,
        constant=constant,
        variable_names=names,
        edge_variables=edge_variables,
        activation_variables=activation_variables,
        penalty=float(penalty),
    )


def qubo_to_ising(qubo: QuboModel) -> Tuple[Dict[int, float], Dict[Tuple[int, int], float]]:
    """Convert x=(1-Z)/2 into diagonal Ising coefficients."""
    h: Dict[int, float] = {}
    zz: Dict[Tuple[int, int], float] = {}
    for i, coeff in qubo.linear.items():
        h[i] = h.get(i, 0.0) - 0.5 * coeff
    for (i, j), coeff in qubo.quadratic.items():
        zz[(i, j)] = zz.get((i, j), 0.0) + 0.25 * coeff
        h[i] = h.get(i, 0.0) - 0.25 * coeff
        h[j] = h.get(j, 0.0) - 0.25 * coeff
    return h, zz


def _require_qiskit():
    try:
        from qiskit import QuantumCircuit, transpile
        from qiskit.circuit import ParameterVector
        from qiskit.quantum_info import SparsePauliOp
        from qiskit_aer import AerSimulator
    except ImportError as exc:
        raise RuntimeError(
            "Phase 8 requires qiskit and qiskit-aer. "
            "Install with: pip install -r requirements.txt "
            "(qiskit-aer-gpu instead of qiskit-aer for GPU runs)"
        ) from exc
    return QuantumCircuit, transpile, AerSimulator, ParameterVector, SparsePauliOp


def resolve_device(device: str = "auto") -> str:
    """Map 'auto' / 'CPU' / 'GPU' to an Aer device that is actually available."""
    _, _, AerSimulator, _, _ = _require_qiskit()
    available = tuple(AerSimulator().available_devices())
    wanted = device.upper()
    if wanted == "AUTO":
        return "GPU" if "GPU" in available else "CPU"
    if wanted not in available:
        raise RuntimeError(
            f"Aer device {wanted!r} is not available (available: {available}). "
            "For GPU runs install qiskit-aer-gpu and check the CUDA driver."
        )
    return wanted


def _aer_backend(device: str):
    _, _, AerSimulator, _, _ = _require_qiskit()
    return AerSimulator(method="statevector", device=device)


def _qaoa_circuit(
    num_qubits: int,
    h: Mapping[int, float],
    zz: Mapping[Tuple[int, int], float],
    gammas: Sequence[Any],
    betas: Sequence[Any],
    measure: bool = False,
):
    """QAOA ansatz; gammas/betas may be floats or circuit Parameters."""
    QuantumCircuit, _, _, _, _ = _require_qiskit()
    qc = QuantumCircuit(num_qubits)
    qc.h(range(num_qubits))

    for gamma, beta in zip(gammas, betas):
        # exp(-i*gamma*h_i*Z_i)
        for i, coeff in h.items():
            qc.rz(2.0 * coeff * gamma, i)

        # exp(-i*gamma*J_ij*Z_i Z_j)
        for (i, j), coeff in zz.items():
            qc.rzz(2.0 * coeff * gamma, i, j)

        # exp(-i*beta*X_i)
        for i in range(num_qubits):
            qc.rx(2.0 * beta, i)

    if measure:
        qc.measure_all()
    return qc


def _ising_operator(qubo: QuboModel):
    """QUBO cost as a diagonal Pauli operator: offset + sum h Z + sum J ZZ."""
    _, _, _, _, SparsePauliOp = _require_qiskit()
    h, zz = qubo_to_ising(qubo)
    offset = (
        qubo.constant
        + 0.5 * sum(qubo.linear.values())
        + 0.25 * sum(qubo.quadratic.values())
    )
    terms = [("", [], offset)]
    terms += [("Z", [i], c) for i, c in h.items()]
    terms += [("ZZ", [i, j], c) for (i, j), c in zz.items()]
    return SparsePauliOp.from_sparse_list(terms, num_qubits=qubo.num_variables).simplify()


def _cost_lookup(qubo: QuboModel) -> np.ndarray:
    """Build exact diagonal QUBO costs for all computational basis states."""
    n = qubo.num_variables
    states = np.arange(2 ** n, dtype=np.uint64)
    costs = np.full(states.shape, qubo.constant, dtype=np.float64)
    for i, coeff in qubo.linear.items():
        bits = ((states >> np.uint64(i)) & 1).astype(np.float64)
        costs += coeff * bits
    for (i, j), coeff in qubo.quadratic.items():
        bi = ((states >> np.uint64(i)) & 1).astype(np.float64)
        bj = ((states >> np.uint64(j)) & 1).astype(np.float64)
        costs += coeff * bi * bj
    return costs


def optimize_qaoa(
    qubo: QuboModel,
    reps: int = 1,
    restarts: int = 4,
    seed: int = 7,
    device: str = "auto",
) -> Tuple[np.ndarray, float]:
    """Optimize QAOA angles against the exact Aer statevector expectation.

    The expectation <C> is evaluated inside Aer (on the GPU when device is GPU),
    so only one number per evaluation leaves the simulator. Memory is the only
    limit on qubits: a double-precision statevector needs 16 * 2^n bytes.
    """
    if reps < 1:
        raise ValueError("reps must be >= 1.")

    _, transpile, _, ParameterVector, _ = _require_qiskit()
    backend = _aer_backend(resolve_device(device))
    rng = np.random.default_rng(seed)

    gamma_p = ParameterVector("gamma", reps)
    beta_p = ParameterVector("beta", reps)
    h, zz = qubo_to_ising(qubo)
    qc = _qaoa_circuit(qubo.num_variables, h, zz, list(gamma_p), list(beta_p))
    qc.save_expectation_value(_ising_operator(qubo), list(range(qubo.num_variables)))
    compiled = transpile(qc, backend)
    params = list(gamma_p) + list(beta_p)

    def objective(values: np.ndarray) -> float:
        bound = compiled.assign_parameters(dict(zip(params, values)))
        result = backend.run(bound).result()
        return float(np.real(result.data(0)["expectation_value"]))

    best_x: Optional[np.ndarray] = None
    best_value = math.inf

    # Include a deterministic small-angle start plus seeded random starts.
    starts = [np.concatenate([
        np.full(reps, 0.7),
        np.full(reps, 0.25),
    ])]
    for _ in range(max(0, restarts - 1)):
        starts.append(np.concatenate([
            rng.uniform(0.0, 2.0 * math.pi, size=reps),
            rng.uniform(0.0, math.pi, size=reps),
        ]))

    bounds = [(0.0, 2.0 * math.pi)] * reps + [(0.0, math.pi)] * reps
    for x0 in starts:
        result = minimize(
            objective,
            x0,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 80, "ftol": 1e-8},
        )
        if result.fun < best_value:
            best_value = float(result.fun)
            best_x = np.asarray(result.x, dtype=float)

    assert best_x is not None
    return best_x, best_value


def _decode_bitstring(bitstring: str, num_variables: int) -> List[int]:
    clean = bitstring.replace(" ", "")
    if len(clean) != num_variables:
        raise ValueError(
            f"Bitstring has {len(clean)} bits; expected {num_variables}."
        )
    # Qiskit counts strings are printed q[n-1]...q[0].
    return [int(clean[num_variables - 1 - q]) for q in range(num_variables)]


def _selected_path_from_bits(
    reduced_graph: nx.MultiGraph,
    qubo: QuboModel,
    bits: Sequence[int],
    start: Any,
    goal: Any,
) -> Optional[List[Any]]:
    selected: List[EdgeId] = []
    for idx, edge_id in enumerate(qubo.edge_variables):
        if int(bits[idx]) == 1:
            selected.append(edge_id)

    if not selected:
        return None

    H = nx.MultiGraph()
    H.add_nodes_from(reduced_graph.nodes(data=True))
    for u, v, key in selected:
        if not reduced_graph.has_edge(u, v, key):
            return None
        H.add_edge(u, v, key=key)

    if H.degree(start) != 1 or H.degree(goal) != 1:
        return None

    for node in H.nodes:
        if node in (start, goal):
            continue
        degree = H.degree(node)
        if degree not in (0, 2):
            return None

    if not _connected(H, start, goal):
        return None

    # Follow the unique selected edge at each internal node.
    path = [start]
    previous = None
    current = start
    visited = {start}
    while current != goal:
        neighbours = []
        for nxt, keydict in H.adj[current].items():
            for key in keydict:
                if nxt != previous:
                    neighbours.append(nxt)
        if len(neighbours) != 1:
            return None
        nxt = neighbours[0]
        if nxt in visited:
            return None
        visited.add(nxt)
        path.append(nxt)
        previous, current = current, nxt

    return path


def _expand_macro_path(
    reduced_graph: nx.MultiGraph,
    reduced_path: Sequence[Any],
) -> List[Any]:
    """Expand anchor path into original graph nodes using cheapest selected edge."""
    expanded: List[Any] = [reduced_path[0]]
    for u, v in zip(reduced_path[:-1], reduced_path[1:]):
        candidates = []
        for key, data in reduced_graph.get_edge_data(u, v).items():
            candidates.append(
                (float(data.get("weight", math.inf)), key, data.get("path_nodes", [u, v]))
            )
        if not candidates:
            raise ValueError(f"Reduced path uses missing edge {u!r}->{v!r}.")
        _, _, segment = min(candidates, key=lambda item: item[0])
        segment = list(segment)
        if segment[0] != u:
            segment.reverse()
        if segment[-1] != v:
            raise ValueError("Malformed macro-edge path metadata.")
        expanded.extend(segment[1:])
    return expanded


def _selected_original_path(
    reduced_graph: nx.MultiGraph,
    reduced_path: Sequence[Any],
    bits: Sequence[int],
    qubo: QuboModel,
) -> List[Any]:
    """Expand using the actual selected edge keys, not the cheapest alternative."""
    expanded = [reduced_path[0]]
    for u, v in zip(reduced_path[:-1], reduced_path[1:]):
        selected = []
        for idx, (eu, ev, key) in enumerate(qubo.edge_variables):
            if not bits[idx]:
                continue
            if {eu, ev} == {u, v}:
                data = reduced_graph.get_edge_data(eu, ev, key)
                selected.append((key, data))
        if len(selected) != 1:
            raise ValueError("Could not identify unique selected macro-edge.")
        key, data = selected[0]
        segment = list(data.get("path_nodes", [u, v]))
        if segment[0] != u:
            segment.reverse()
        if segment[-1] != v:
            raise ValueError("Malformed macro-edge path metadata.")
        expanded.extend(segment[1:])
    return expanded


def _counts_to_candidates(counts: Mapping[str, int]) -> Iterable[Tuple[str, int]]:
    return sorted(counts.items(), key=lambda item: item[1], reverse=True)


def run_qaoa(
    graph: nx.Graph,
    start: Any,
    goal: Any,
    max_edges: int = 12,
    width_ratio: float = 0.5,
    reps: int = 1,
    shots: int = 2048,
    restarts: int = 4,
    seed: int = 7,
    penalty: Optional[float] = None,
    device: str = "auto",
) -> QuantumPlanResult:
    """Build the reduced QUBO, run QAOA on AerSimulator, and decode the best path."""
    _, transpile, _, _, _ = _require_qiskit()
    device = resolve_device(device)
    if shots < 1:
        raise ValueError("shots must be >= 1.")

    started = time.perf_counter()
    reduced = build_quantum_corridor(
        graph, start, goal, max_edges=max_edges, width_ratio=width_ratio
    )
    qubo = build_qubo(reduced, start, goal, penalty=penalty)
    params, expectation = optimize_qaoa(
        qubo, reps=reps, restarts=restarts, seed=seed, device=device
    )
    gammas = params[:reps]
    betas = params[reps:]

    qc = _qaoa_circuit(
        qubo.num_variables,
        *qubo_to_ising(qubo),
        gammas,
        betas,
        measure=True,
    )
    backend = _aer_backend(device)
    compiled = transpile(qc, backend)
    result = backend.run(compiled, shots=shots, seed_simulator=seed).result()
    counts = result.get_counts(compiled)

    valid_samples = 0
    sampled_candidates = sum(counts.values())
    best_valid: Optional[Tuple[float, str, List[Any], List[int]]] = None

    for bitstring, count in _counts_to_candidates(counts):
        bits = _decode_bitstring(bitstring, qubo.num_variables)
        reduced_path = _selected_path_from_bits(reduced, qubo, bits, start, goal)
        if reduced_path is None:
            continue
        try:
            original_path = _selected_original_path(
                reduced, reduced_path, bits, qubo
            )
        except ValueError:
            continue
        if original_path[0] != start or original_path[-1] != goal:
            continue
        # Validate against the real graph and ensure no repeated node.
        if len(set(original_path)) != len(original_path):
            continue
        if not nx.is_simple_path(graph, original_path):
            continue
        valid_samples += count
        energy = _path_weight(graph, original_path)
        candidate = (energy, bitstring, original_path, bits)
        if best_valid is None or candidate[0] < best_valid[0]:
            best_valid = candidate

    if best_valid is None:
        # No classical fallback: if QAOA sampled no feasible path, report the failure.
        raise RuntimeError(
            f"QAOA found no valid path in {sampled_candidates} shots "
            f"({qubo.num_variables} qubits, reps={reps}); try more reps/shots/restarts."
        )
    _, best_bitstring, best_path, _ = best_valid

    coords = [
        (float(graph.nodes[n].get("pos_x", 0.0)),
         float(graph.nodes[n].get("pos_y", 0.0)))
        for n in best_path
    ]
    total_energy = _path_weight(graph, best_path)
    total_distance = _path_distance(graph, best_path)
    runtime_ms = (time.perf_counter() - started) * 1000.0

    return QuantumPlanResult(
        algorithm="QAOA",
        path_nodes=list(best_path),
        path_coords=coords,
        total_energy=total_energy,
        total_distance_m=total_distance,
        num_waypoints=len(best_path),
        runtime_ms=runtime_ms,
        timestamp=datetime.now().isoformat(),
        best_bitstring=best_bitstring,
        qaoa_expectation=float(expectation),
        reduced_nodes=reduced.number_of_nodes(),
        reduced_edges=reduced.number_of_edges(),
        qubits=qubo.num_variables,
        shots=shots,
        reps=reps,
        penalty=qubo.penalty,
        valid_samples=valid_samples,
        sampled_candidates=sampled_candidates,
        device=device,
    )


def save_quantum_results(
    quantum: QuantumPlanResult,
    out_dir: str | Path = "results/paths",
    comparison: Optional[Dict[str, Any]] = None,
) -> Dict[str, str]:
    """Write latest_quantum.json plus a timestamped copy.

    `comparison` is optional and only supplied by planning_pkg.compare_planners;
    the QAOA run itself never produces or needs a classical baseline.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    data: Dict[str, Any] = {
        "quantum": quantum.to_dict(),
        "saved_at": datetime.now().isoformat(),
    }
    if comparison is not None:
        data["comparison"] = comparison
    latest = out / "latest_quantum.json"
    with latest.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    history = out / f"quantum_{timestamp}.json"
    with history.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
    return {"latest": str(latest), "history": str(history)}


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run Phase-8 QAOA quantum path optimization."
    )
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument(
        "--graph", default="results/graphs/latest.graphml",
        help="Phase-6 energy-weighted GraphML.",
    )
    parser.add_argument("--out", default="results/paths")
    parser.add_argument("--max-edges", type=int, default=12)
    parser.add_argument(
        "--corridor-width", type=float, default=0.5,
        help="Lattice half-width as a fraction of the start-goal distance.",
    )
    parser.add_argument("--reps", type=int, default=1)
    parser.add_argument("--shots", type=int, default=2048)
    parser.add_argument("--restarts", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--penalty", type=float, default=None)
    parser.add_argument(
        "--device", default="auto", choices=["auto", "CPU", "GPU"],
        help="Aer simulator device; auto picks GPU when qiskit-aer-gpu sees one.",
    )
    args = parser.parse_args(argv)

    print(f"[quantum_optimizer] Loading graph: {args.graph}")
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
    except Exception as exc:
        print(f"[quantum_optimizer] ERROR: {exc}")
        return 1

    saved = save_quantum_results(quantum, args.out)

    print(
        f"[quantum_optimizer] QAOA: {quantum.total_energy:.4f} energy / "
        f"{quantum.num_waypoints} waypoints / {quantum.qubits} qubits / "
        f"{quantum.runtime_ms:.2f} ms on {quantum.device}"
    )
    print(
        f"[quantum_optimizer] reduced graph: {quantum.reduced_nodes} nodes, "
        f"{quantum.reduced_edges} edges; valid samples={quantum.valid_samples}"
    )
    print(f"[quantum_optimizer] Saved: {saved['latest']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
