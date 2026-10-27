#!/usr/bin/env python3
"""
tsp_qubo.py
-----------
QUBO formulation of the visiting-order problem (position / permutation encoding).

For k free nodes v_0..v_{k-1} placed at positions t = 0..k-1, binary
x[a, t] = 1 means "node v_a is visited at position t" (k^2 variables,
index a*k + t). With an optional fixed predecessor `start` and successor `end`:

  H = A * sum_a (1 - sum_t x[a,t])^2           each node used once
    + A * sum_t (1 - sum_a x[a,t])^2           each position filled once
    + sum_a c(start, v_a) x[a, 0]              leave the start
    + sum_{t} sum_{a != b} c(v_a, v_b) x[a,t] x[b,t+1]
    + sum_a c(v_a, end) x[a, k-1]              arrive at the end

Costs are divided by `scale` (the largest leg cost involved) so they lie in
[0, 1]; this keeps QAOA angles in a comparable range across instances. Any
assignment satisfying the constraints has H = path_cost / scale exactly.
With A > max scaled leg cost (Lucas 2014, sec. 7.1) the ground state is a
valid permutation; the default A = 1.5 satisfies this.

Same QUBO covers: closed tour (start = end = depot), open mission
(start = depot, end = None), and a window inside a tour (start/end = the
fixed neighbours of the window).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from planning_pkg.quantum_optimizer import QuboModel


@dataclass
class TspQubo:
    qubo: QuboModel
    free_nodes: List[int]
    start: Optional[int]
    end: Optional[int]
    scale: float
    penalty: float

    @property
    def k(self) -> int:
        return len(self.free_nodes)

    def var(self, a: int, t: int) -> int:
        return a * self.k + t


def _add(d: Dict, key, value: float) -> None:
    d[key] = d.get(key, 0.0) + value


def build_tsp_qubo(
    cost: np.ndarray,
    free_nodes: Sequence[int],
    start: Optional[int] = 0,
    end: Optional[int] = 0,
    penalty: float = 1.5,
) -> TspQubo:
    """Build the position-encoded QUBO; `penalty` is relative to scaled costs."""
    cost = np.asarray(cost, dtype=np.float64)
    nodes = [int(v) for v in free_nodes]
    k = len(nodes)
    if k < 1:
        raise ValueError("Need at least one free node.")
    if len(set(nodes)) != k:
        raise ValueError("free_nodes must be distinct.")
    for v in nodes + [s for s in (start, end) if s is not None]:
        if not 0 <= v < cost.shape[0]:
            raise ValueError(f"Node index {v} outside cost matrix.")
    if np.any(cost < 0):
        raise ValueError("Leg costs must be non-negative.")
    if penalty <= 0:
        raise ValueError("penalty must be positive.")

    involved = nodes + [s for s in (start, end) if s is not None]
    sub = cost[np.ix_(involved, involved)]
    scale = float(sub.max()) if sub.max() > 0 else 1.0
    c = cost / scale

    linear: Dict[int, float] = {}
    quadratic: Dict[Tuple[int, int], float] = {}
    constant = 0.0

    def idx(a: int, t: int) -> int:
        return a * k + t

    # A*(1 - sum x)^2 = A - A*sum x + 2A*sum_{i<j} x_i x_j   (x^2 = x)
    groups = [[idx(a, t) for t in range(k)] for a in range(k)]
    groups += [[idx(a, t) for a in range(k)] for t in range(k)]
    for group in groups:
        constant += penalty
        for i in group:
            _add(linear, i, -penalty)
        for p, i in enumerate(group):
            for j in group[p + 1:]:
                _add(quadratic, (min(i, j), max(i, j)), 2.0 * penalty)

    for a, v in enumerate(nodes):
        if start is not None:
            _add(linear, idx(a, 0), c[start, v])
        if end is not None:
            _add(linear, idx(a, k - 1), c[v, end])
    for t in range(k - 1):
        for a, va in enumerate(nodes):
            for b, vb in enumerate(nodes):
                if a == b:
                    continue
                i, j = idx(a, t), idx(b, t + 1)
                _add(quadratic, (min(i, j), max(i, j)), c[va, vb])

    names = [f"x[{v},{t}]" for v in nodes for t in range(k)]
    qubo = QuboModel(
        num_variables=k * k,
        linear=linear,
        quadratic=quadratic,
        constant=constant,
        variable_names=names,
        edge_variables=[],
        activation_variables={},
        penalty=float(penalty),
    )
    return TspQubo(qubo, nodes, start, end, scale, float(penalty))


def encode_order(tq: TspQubo, order: Sequence[int]) -> List[int]:
    """Bit vector (index = variable) for visiting `order` (node indices)."""
    if sorted(order) != sorted(tq.free_nodes):
        raise ValueError("order must be a permutation of the free nodes.")
    local = {v: a for a, v in enumerate(tq.free_nodes)}
    bits = [0] * tq.qubo.num_variables
    for t, v in enumerate(order):
        bits[tq.var(local[v], t)] = 1
    return bits


def decode_bits(tq: TspQubo, bits: Sequence[int]) -> Optional[List[int]]:
    """Order of free nodes if `bits` is a valid permutation matrix, else None."""
    k = tq.k
    x = np.asarray(bits, dtype=int).reshape(k, k)  # [node, position]
    if not (np.all(x.sum(axis=0) == 1) and np.all(x.sum(axis=1) == 1)):
        return None
    return [tq.free_nodes[int(a)] for a in np.argmax(x, axis=0)]


def repair_from_marginals(tq: TspQubo, marginals: Sequence[float]) -> List[int]:
    """Most-likely permutation under per-variable marginals P(x_i = 1)
    (maximum-weight assignment). Used only when no sample is feasible."""
    k = tq.k
    m = np.asarray(marginals, dtype=np.float64).reshape(k, k)
    rows, cols = linear_sum_assignment(-m)
    order = [0] * k
    for a, t in zip(rows, cols):
        order[t] = tq.free_nodes[int(a)]
    return order


def count_feasible_states(k: int) -> int:
    """Number of valid bitstrings (= k!) among 2^(k^2)."""
    out = 1
    for i in range(2, k + 1):
        out *= i
    return out
