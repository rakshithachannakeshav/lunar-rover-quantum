#!/usr/bin/env python3
"""
quantum_local_search.py
-----------------------
Threshold-QAOA local search (LS-ThQAOA) for the multi-site rover mission (ATSP).

Plain QAOA samples whole tours from all N! orders, so its edge over random
sampling stays constant while N! explodes (RESULTS_TABLES.md, finding 5). Here
QAOA is used where a quantum search can pay off: *inside* 2-opt local search,
to find an improving move in the current tour's neighbourhood.

One step, from tour T with cost c(T):
    register  |s>  = uniform superposition over the K = n(n-1)/2 2-opt moves
    phase     U_P  = threshold phase separator: -1 on moves with c(move(T)) < c(T)
    mixer     U_M  = Grover mixer exp(-i beta |s><s|)
With gamma = beta = pi, p layers of this Threshold-QAOA with Grover mixer are p
Grover iterations (Baertschi & Eidenbenz 2020; Golden et al. 2021, "Threshold-
based quantum optimization"). The number m of improving moves is unknown, so p
is drawn as in Boyer-Brassard-Hoyer-Tapp (BBHT): about sqrt(K/m) oracle calls
per improving move, versus about K/m for classical random-order first improvement.

Fairness by construction: a measured improving move is uniform over the m
improving moves, and so is the first improving move a classical random-order
scan finds. The two descents therefore follow the SAME distribution of tours;
only the number of oracle queries differs.

Query model (same as scripts/qaoa_vs_classical_queries.py): one query = one
evaluation of a complete tour's energy. Quantum accounting, conservative:
    each Grover layer       = `oracle_calls_per_layer` queries (default 2:
                              compute + uncompute the tour energy coherently)
    each measured candidate = 1 classical query to verify it
    each start tour         = 1 query

Simulation: the costs of all K neighbours are computed classically (vectorised)
only to obtain the exact measurement probabilities sin^2((2p+1) theta),
sin(theta) = sqrt(m/K). Those evaluations are simulator bookkeeping and are not
counted; on a fault-tolerant device the oracle is a reversible circuit.

Honest limits: this is an advantage in oracle-query count on ideal (fault-
tolerant) hardware, not wall-clock time on today's devices or in this simulator.
The classical baseline only sees tour energies (black box); white-box tricks
such as neighbour lists or don't-look bits exploit edge structure and are a
different comparison.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

BBHT_GROWTH = 6.0 / 5.0       # lambda in BBHT (any 1 < lambda < 4/3 works)
DEFAULT_STOP_FACTOR = 4.0     # declare "local optimum" after this * sqrt(K) layers (false stop <= 0.03 % at m = 1)
REL_TOL = 1e-12


def two_opt_pairs(n: int) -> np.ndarray:
    """All reversal moves (i, j), 0 <= i < j < n, on the n visited sites."""
    i, j = np.triu_indices(n, k=1)
    return np.stack([i, j], axis=1)


def tour_cost(cost: np.ndarray, order: Sequence[int]) -> float:
    full = [0] + list(order) + [0]
    return float(sum(cost[a, b] for a, b in zip(full[:-1], full[1:])))


def neighbour_costs(cost: np.ndarray, order: Sequence[int], pairs: np.ndarray) -> np.ndarray:
    """Cost of every 2-opt reversal of `order` (depot 0 fixed at both ends), exactly.

    Reversing sites i..j of an asymmetric tour also reverses the direction of every
    leg inside the segment, so both forward and backward prefix sums are needed.
    """
    full = np.concatenate([[0], np.asarray(order, dtype=np.intp), [0]])
    fwd = cost[full[:-1], full[1:]]
    bwd = cost[full[1:], full[:-1]]
    F = np.concatenate([[0.0], np.cumsum(fwd)])
    B = np.concatenate([[0.0], np.cumsum(bwd)])
    total = F[-1]
    a = pairs[:, 0] + 1          # first reversed position in `full`
    b = pairs[:, 1] + 1          # last reversed position in `full`
    old = cost[full[a - 1], full[a]] + (F[b] - F[a]) + cost[full[b], full[b + 1]]
    new = cost[full[a - 1], full[b]] + (B[b] - B[a]) + cost[full[a], full[b + 1]]
    return total - old + new


def apply_move(order: Sequence[int], i: int, j: int) -> List[int]:
    order = list(order)
    return order[:i] + order[i:j + 1][::-1] + order[j + 1:]


def grover_success_probability(m: int, K: int, layers: int) -> float:
    """P(measure an improving move) after `layers` Grover / Th-QAOA(pi, pi) layers."""
    if m <= 0:
        return 0.0
    if m >= K:
        return 1.0
    theta = math.asin(math.sqrt(m / K))
    return math.sin((2 * layers + 1) * theta) ** 2


@dataclass
class DescentResult:
    order: List[int]
    cost: float
    queries: float
    steps: int
    true_local_optimum: bool          # False if the quantum search stopped early
    trace: List[float] = field(default_factory=list)


def _improving(nc: np.ndarray, best: float) -> np.ndarray:
    return np.nonzero(nc < best - REL_TOL * max(1.0, abs(best)))[0]


def classical_descent(cost: np.ndarray, rng: np.random.Generator,
                      start: Optional[Sequence[int]] = None,
                      target: Optional[float] = None) -> DescentResult:
    """Random-order first-improvement 2-opt (the black-box classical baseline).

    Scanning moves in a random order until the first improving one is the same as
    drawing the position of the first of m marked items in a random permutation of
    K, and that item is uniform over the m; sampled exactly here, query for query.
    Stops early (counting the query that found it) when a tour <= target is seen.
    """
    n = cost.shape[0] - 1
    pairs = two_opt_pairs(n)
    K = len(pairs)
    order = list(rng.permutation(np.arange(1, n + 1))) if start is None else list(start)
    best = tour_cost(cost, order)
    queries = 1.0
    steps = 0
    if target is not None and best <= target:
        return DescentResult(order, best, queries, steps, False)
    while True:
        nc = neighbour_costs(cost, order, pairs)
        marked = _improving(nc, best)
        m = len(marked)
        if m == 0:
            return DescentResult(order, best, queries + K, steps, True)
        if target is not None:
            hits = np.nonzero(nc <= target)[0]
            if len(hits):
                # a scan stops at whichever comes first: a target hit or the first improver
                perm = rng.permutation(K)
                pos = np.empty(K, dtype=np.intp)
                pos[perm] = np.arange(K)
                first_hit, first_imp = pos[hits].min(), pos[marked].min()
                if first_hit <= first_imp:
                    k = int(perm[first_hit])
                    order = apply_move(order, *pairs[k])
                    return DescentResult(order, float(nc[k]), queries + first_hit + 1, steps + 1, False)
                k = int(perm[first_imp])
                queries += first_imp + 1
                order, best, steps = apply_move(order, *pairs[k]), float(nc[k]), steps + 1
                continue
        # position of the first marked item in a uniform random permutation of K
        first = _first_marked_position(K, m, rng)
        queries += first + 1
        k = int(marked[rng.integers(m)])
        order, best, steps = apply_move(order, *pairs[k]), float(nc[k]), steps + 1


def _first_marked_position(K: int, m: int, rng: np.random.Generator) -> int:
    """0-based index of the first of m marked items in a random order of K items."""
    # surv[t] = P(none of the first t + 1 items is marked); inverse transform.
    t = np.arange(K - m + 1)
    surv = np.cumprod(1.0 - m / (K - t))
    return int(np.argmax(surv <= rng.random()))


def quantum_search_move(m: int, K: int, rng: np.random.Generator,
                        oracle_calls_per_layer: float = 2.0,
                        stop_factor: float = DEFAULT_STOP_FACTOR):
    """BBHT search over the K moves with Threshold-QAOA(pi, pi) layers.

    Returns (found, queries). `found` is True when an improving move was measured
    (which one is uniform over the m improvers: the caller draws it).
    """
    budget = stop_factor * math.sqrt(K)
    lam_m = 1.0
    layers_used = 0
    queries = 0.0
    while layers_used <= budget:
        layers = int(rng.integers(0, max(1, int(math.ceil(lam_m)))))
        layers_used += layers
        queries += oracle_calls_per_layer * layers + 1.0    # + 1 classical verification
        if rng.random() < grover_success_probability(m, K, layers):
            return True, queries
        lam_m = min(BBHT_GROWTH * lam_m, math.sqrt(K))
    return False, queries


def quantum_descent(cost: np.ndarray, rng: np.random.Generator,
                    start: Optional[Sequence[int]] = None,
                    target: Optional[float] = None,
                    oracle_calls_per_layer: float = 2.0,
                    stop_factor: float = DEFAULT_STOP_FACTOR) -> DescentResult:
    """2-opt descent whose improving moves are found by Threshold-QAOA search.

    With `target`, the descent stops as soon as the tour it moved to reaches it.
    (The classical scan may also stop on a target neighbour it merely queried; the
    quantum search only sees the move it measures. That favours classical.)
    """
    n = cost.shape[0] - 1
    pairs = two_opt_pairs(n)
    K = len(pairs)
    order = list(rng.permutation(np.arange(1, n + 1))) if start is None else list(start)
    best = tour_cost(cost, order)
    queries = 1.0
    steps = 0
    if target is not None and best <= target:
        return DescentResult(order, best, queries, steps, False)
    while True:
        nc = neighbour_costs(cost, order, pairs)
        marked = _improving(nc, best)
        m = len(marked)
        found, q = quantum_search_move(m, K, rng, oracle_calls_per_layer, stop_factor)
        queries += q
        if not found:
            return DescentResult(order, best, queries, steps, m == 0)
        k = int(marked[rng.integers(m)])
        order, best, steps = apply_move(order, *pairs[k]), float(nc[k]), steps + 1
        if target is not None and best <= target:
            return DescentResult(order, best, queries, steps, False)


def queries_to_target(descent, cost: np.ndarray, target: float, rng: np.random.Generator,
                      cap: float = 1e9, **kw) -> float:
    """Random-restart descents until a tour with cost <= target is seen."""
    total = 0.0
    while total < cap:
        r = descent(cost, rng, target=target, **kw)
        total += r.queries
        if r.cost <= target:
            return total
    return math.inf
