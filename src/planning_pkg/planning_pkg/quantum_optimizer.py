#!/usr/bin/env python3
"""
Phase 8: Quantum path optimization with a small, tractable QAOA instance.

The Phase-6 graph can contain hundreds of edges. Running QAOA directly on that
graph is not a useful CPU demonstration because the qubit count would explode.
This module therefore builds a small "quantum corridor" from the Phase-7 A*
backbone. Consecutive backbone anchors are connected by the baseline segment and,
when available, a low-cost alternative segment. Each reduced-graph edge is an
edge-selection binary in the QUBO; its metadata contains the original path
segment so the quantum solution can be expanded back to the real terrain graph.

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

from planning_pkg.classical_planning import (
    PlanResult,
    load_weighted_graph,
    resolve_node,
    run_astar,
)
from planning_pkg.graph_model import load_graph


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
    fallback_to_classical: bool = False

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


def _segment_candidates(
    graph: nx.Graph,
    start: Any,
    goal: Any,
    baseline: Sequence[Any],
    max_candidates: int = 6,
) -> List[List[Any]]:
    """Return distinct low-cost simple paths between two anchor nodes."""
    try:
        generator = nx.shortest_simple_paths(graph, start, goal, weight="weight")
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return [list(baseline)]

    candidates: List[List[Any]] = []
    seen = set()
    try:
        for path in generator:
            key = tuple(path)
            if key in seen:
                continue
            seen.add(key)
            candidates.append(list(path))
            if len(candidates) >= max_candidates:
                break
    except nx.NetworkXNoPath:
        pass

    baseline_key = tuple(baseline)
    ordered: List[List[Any]] = []
    for path in candidates:
        if tuple(path) == baseline_key:
            ordered.insert(0, path)
        else:
            ordered.append(path)
    if not ordered:
        ordered = [list(baseline)]
    return ordered


def build_quantum_corridor(
    graph: nx.Graph,
    start: Any,
    goal: Any,
    baseline_path: Sequence[Any],
    max_edges: int = 10,
    alternatives_per_segment: int = 2,
) -> nx.MultiGraph:
    """
    Build a <=max_edges reduced graph while preserving a valid A* backbone.

    The backbone is split into a small number of anchor-to-anchor segments.
    Each segment becomes one baseline macro-edge plus at most one alternative
    macro-edge. Macro-edge metadata stores the original node sequence.
    """
    baseline = list(baseline_path)
    if len(baseline) < 2:
        raise ValueError("Baseline path must contain at least start and goal.")

    if max_edges < 2:
        raise ValueError("max_edges must be at least 2.")

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

    # Each segment gets baseline + one alternative, so this stays tractable.
    max_segments = max(1, max_edges // 2)
    n_segments = min(max_segments, len(baseline) - 1)

    # Evenly distribute anchors over the A* path.
    anchor_indices = np.linspace(
        0, len(baseline) - 1, n_segments + 1, dtype=int
    ).tolist()
    anchor_indices = sorted(set(anchor_indices))
    if anchor_indices[-1] != len(baseline) - 1:
        anchor_indices.append(len(baseline) - 1)

    # If duplicate indices reduced the number of segments, that's fine.
    reduced = nx.MultiGraph()
    for anchor in [baseline[i] for i in anchor_indices]:
        reduced.add_node(anchor, **dict(graph.nodes[anchor]))

    for left, right in zip(anchor_indices[:-1], anchor_indices[1:]):
        a = baseline[left]
        b = baseline[right]
        baseline_segment = baseline[left:right + 1]
        candidates = _segment_candidates(
            graph, a, b, baseline_segment,
            max_candidates=max(3, alternatives_per_segment + 2),
        )

        # Always add the A* segment first.
        chosen = [baseline_segment]
        for candidate in candidates:
            if tuple(candidate) != tuple(baseline_segment):
                chosen.append(candidate)
                break
        if len(chosen) > alternatives_per_segment:
            chosen = chosen[:alternatives_per_segment]

        for candidate in chosen:
            reduced.add_edge(
                a,
                b,
                weight=_path_weight(graph, candidate),
                path_nodes=list(candidate),
                macro=True,
            )

    if reduced.number_of_edges() > max_edges:
        # Deterministic safety valve: retain the lowest-cost edge per segment
        # and the cheapest alternative edges until the requested limit.
        edges = list(reduced.edges(keys=True, data=True))
        edges.sort(key=lambda item: float(item[3].get("weight", math.inf)))
        keep = edges[:max_edges]
        trimmed = nx.MultiGraph()
        for node, attrs in reduced.nodes(data=True):
            trimmed.add_node(node, **dict(attrs))
        for u, v, key, data in keep:
            trimmed.add_edge(u, v, **dict(data))
        reduced = trimmed

    if not nx.has_path(reduced, start, goal):
        raise ValueError(
            "Quantum corridor lost start-to-goal connectivity. "
            "Increase --max-edges or use a shorter classical path."
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
    if not nx.has_path(reduced_graph, start, goal):
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
        from qiskit_aer import AerSimulator
        from qiskit.quantum_info import Statevector
    except ImportError as exc:
        raise RuntimeError(
            "Phase 8 requires qiskit and qiskit-aer. "
            "Install with: pip install -r requirements.txt"
        ) from exc
    return QuantumCircuit, transpile, AerSimulator, Statevector


def _qaoa_circuit(
    num_qubits: int,
    h: Mapping[int, float],
    zz: Mapping[Tuple[int, int], float],
    gammas: Sequence[float],
    betas: Sequence[float],
    measure: bool = False,
):
    QuantumCircuit, _, _, _ = _require_qiskit()
    qc = QuantumCircuit(num_qubits)
    qc.h(range(num_qubits))

    for gamma, beta in zip(gammas, betas):
        # exp(-i*gamma*h_i*Z_i)
        for i, coeff in h.items():
            qc.rz(2.0 * gamma * coeff, i)

        # exp(-i*gamma*J_ij*Z_i Z_j)
        for (i, j), coeff in zz.items():
            qc.cx(i, j)
            qc.rz(2.0 * gamma * coeff, j)
            qc.cx(i, j)

        # exp(-i*beta*X_i)
        for i in range(num_qubits):
            qc.rx(2.0 * beta, i)

    if measure:
        qc.measure_all()
    return qc


def _statevector_expectation(
    qubo: QuboModel,
    gammas: Sequence[float],
    betas: Sequence[float],
    cost_values: np.ndarray,
) -> float:
    _, _, _, Statevector = _require_qiskit()
    qc = _qaoa_circuit(
        qubo.num_variables,
        *qubo_to_ising(qubo),
        gammas,
        betas,
        measure=False,
    )
    state = np.asarray(Statevector.from_instruction(qc).data)
    probs = np.abs(state) ** 2
    return float(np.dot(probs, cost_values))


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
) -> Tuple[np.ndarray, float]:
    """Optimize QAOA angles against the exact statevector expectation."""
    if reps < 1:
        raise ValueError("reps must be >= 1.")
    if qubo.num_variables > 22:
        raise ValueError(
            f"QAOA demo is limited to 22 qubits; got {qubo.num_variables}. "
            "Reduce --max-edges."
        )

    rng = np.random.default_rng(seed)
    cost_values = _cost_lookup(qubo)
    h, zz = qubo_to_ising(qubo)

    def objective(params: np.ndarray) -> float:
        gammas = params[:reps]
        betas = params[reps:]
        return _statevector_expectation(qubo, gammas, betas, cost_values)

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

    if not nx.has_path(H, start, goal):
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
    baseline_path: Sequence[Any],
    max_edges: int = 10,
    reps: int = 1,
    shots: int = 2048,
    restarts: int = 4,
    seed: int = 7,
    penalty: Optional[float] = None,
) -> QuantumPlanResult:
    """Build the reduced QUBO, run QAOA on AerSimulator, and decode the best path."""
    QuantumCircuit, transpile, AerSimulator, _ = _require_qiskit()
    if shots < 1:
        raise ValueError("shots must be >= 1.")

    started = time.perf_counter()
    reduced = build_quantum_corridor(
        graph, start, goal, baseline_path, max_edges=max_edges
    )
    qubo = build_qubo(reduced, start, goal, penalty=penalty)
    params, expectation = optimize_qaoa(
        qubo, reps=reps, restarts=restarts, seed=seed
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
    backend = AerSimulator()
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

    fallback = False
    if best_valid is None:
        # The QAOA sample distribution can miss the feasible state at shallow p.
        # Return the known-valid classical baseline rather than emitting a bad path.
        fallback = True
        best_path = list(baseline_path)
        best_bitstring = "0" * qubo.num_variables
    else:
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
        fallback_to_classical=fallback,
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
            "fallback_to_classical": quantum.fallback_to_classical,
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


def save_quantum_results(
    quantum: QuantumPlanResult,
    comparison: Dict[str, Any],
    out_dir: str | Path = "results/paths",
) -> Dict[str, str]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    data = {
        "quantum": quantum.to_dict(),
        "comparison": comparison,
        "saved_at": datetime.now().isoformat(),
    }
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
    parser.add_argument("--max-edges", type=int, default=10)
    parser.add_argument("--reps", type=int, default=1)
    parser.add_argument("--shots", type=int, default=2048)
    parser.add_argument("--restarts", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--penalty", type=float, default=None)
    args = parser.parse_args(argv)

    print(f"[quantum_optimizer] Loading graph: {args.graph}")
    try:
        graph = load_weighted_graph(args.graph)
        start = resolve_node(graph, args.start_x, args.start_y)
        goal = resolve_node(graph, args.goal_x, args.goal_y)
        classical = run_astar(graph, start, goal)
        quantum = run_qaoa(
            graph, start, goal, classical.path_nodes,
            max_edges=args.max_edges,
            reps=args.reps,
            shots=args.shots,
            restarts=args.restarts,
            seed=args.seed,
            penalty=args.penalty,
        )
    except Exception as exc:
        print(f"[quantum_optimizer] ERROR: {exc}")
        return 1

    comparison = compare_quantum_classical(classical, quantum)
    saved = save_quantum_results(quantum, comparison, args.out)

    print(
        f"[quantum_optimizer] A*: {classical.total_energy:.4f} energy / "
        f"{classical.num_waypoints} waypoints"
    )
    print(
        f"[quantum_optimizer] QAOA: {quantum.total_energy:.4f} energy / "
        f"{quantum.num_waypoints} waypoints / {quantum.qubits} qubits / "
        f"{quantum.runtime_ms:.2f} ms"
    )
    print(
        f"[quantum_optimizer] reduced graph: {quantum.reduced_nodes} nodes, "
        f"{quantum.reduced_edges} edges; valid samples={quantum.valid_samples}; "
        f"fallback={quantum.fallback_to_classical}"
    )
    print(f"[quantum_optimizer] Saved: {saved['latest']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
