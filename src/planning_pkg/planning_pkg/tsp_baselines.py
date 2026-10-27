#!/usr/bin/env python3
"""
tsp_baselines.py
----------------
Tour utilities and classical solvers for the multi-site visiting-order problem.

Conventions shared by every solver (and by tsp_qubo / tsp_qaoa):
  * cost[a, b] is the leg FROM a TO b; matrices may be asymmetric.
  * `cost` is a (N+1)x(N+1) leg-cost matrix; index 0 is the depot (rover start),
    indices 1..N are science sites.
  * A tour is an `order`: a permutation of 1..N. The rover drives
    0 -> order[0] -> ... -> order[-1] and, when `closed`, back to 0.

Solvers:
  brute_force        - exhaustive, vectorised over permutation chunks (exact)
  count_optimal_orders - how many orders attain the optimum
  held_karp          - O(2^N N^2) dynamic programme (exact, used as reference)
  nearest_neighbour  - greedy from the depot
  two_opt            - best-improvement 2-opt run until no improving move
  window_reorder_exact - exact re-ordering of sliding windows (QAOA-window twin)
  random_tour_stats / best_of_random - uniformly random tours (controls only)
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np


@dataclass
class TourResult:
    method: str
    order: List[int]
    cost: float
    runtime_s: float
    info: Dict[str, Any] = field(default_factory=dict)


def n_sites(cost: np.ndarray) -> int:
    cost = np.asarray(cost)
    if cost.ndim != 2 or cost.shape[0] != cost.shape[1] or cost.shape[0] < 2:
        raise ValueError("cost must be a square matrix with a depot and >= 1 site.")
    return cost.shape[0] - 1


def validate_tour(order: Sequence[int], num_sites: int) -> None:
    """Raise ValueError unless `order` visits every site 1..num_sites exactly once."""
    if sorted(int(s) for s in order) != list(range(1, num_sites + 1)):
        raise ValueError(
            f"Invalid tour {list(order)}: must visit each of sites 1..{num_sites} exactly once."
        )


def is_valid_tour(order: Sequence[int], num_sites: int) -> bool:
    try:
        validate_tour(order, num_sites)
    except ValueError:
        return False
    return True


def path_cost(cost: np.ndarray, start: Optional[int], nodes: Sequence[int],
              end: Optional[int]) -> float:
    """Cost of start -> nodes... -> end; start/end may be None (free endpoint)."""
    seq = ([start] if start is not None else []) + list(nodes) + (
        [end] if end is not None else [])
    return float(sum(cost[a, b] for a, b in zip(seq[:-1], seq[1:])))


def tour_cost(cost: np.ndarray, order: Sequence[int], closed: bool = True) -> float:
    return path_cost(cost, 0, order, 0 if closed else None)


def _permutation_costs(cost: np.ndarray, closed: bool, chunk: int):
    """Yield (orders, tour_costs) blocks covering all N! visiting orders."""
    n = n_sites(cost)
    perms = itertools.permutations(range(1, n + 1))
    while True:
        block = np.array(list(itertools.islice(perms, chunk)), dtype=np.intp)
        if block.size == 0:
            return
        totals = cost[0, block[:, 0]] + cost[block[:, :-1], block[:, 1:]].sum(axis=1)
        if closed:
            totals = totals + cost[block[:, -1], 0]
        yield block, totals


def brute_force(cost: np.ndarray, closed: bool = True, chunk: int = 200_000) -> TourResult:
    """Exact optimum by enumerating all N! orders (numpy-vectorised in chunks).
    Legs are read as cost[from, to], so asymmetric matrices are handled."""
    cost = np.asarray(cost, dtype=np.float64)
    started = time.perf_counter()
    best_cost, best_order = math.inf, None
    evaluated = 0
    for block, totals in _permutation_costs(cost, closed, chunk):
        evaluated += len(block)
        i = int(np.argmin(totals))
        if totals[i] < best_cost:
            best_cost, best_order = float(totals[i]), block[i].tolist()
    return TourResult("brute_force", best_order, best_cost,
                      time.perf_counter() - started, {"evaluated": evaluated})


def count_optimal_orders(cost: np.ndarray, closed: bool = True, rel_tol: float = 1e-9,
                         chunk: int = 200_000) -> int:
    """Number of visiting orders whose cost equals the optimum. For symmetric
    closed tours this is usually 2 (a tour and its reverse); with asymmetric
    legs it is usually 1."""
    cost = np.asarray(cost, dtype=np.float64)
    opt = held_karp(cost, closed).cost
    tol = rel_tol * max(1.0, abs(opt))
    return int(sum(int(np.sum(totals <= opt + tol))
                   for _, totals in _permutation_costs(cost, closed, chunk)))


def held_karp(cost: np.ndarray, closed: bool = True) -> TourResult:
    """Exact optimum via the Held-Karp subset DP, vectorised over all subsets
    of the same size (practical up to N ~ 20)."""
    cost = np.asarray(cost, dtype=np.float64)
    n = n_sites(cost)
    started = time.perf_counter()
    full = (1 << n) - 1
    sites = cost[1:, 1:]
    # dp[mask][j]: cheapest path from depot visiting `mask`, ending at site j+1.
    dp = np.full((1 << n, n), np.inf)
    parent = np.full((1 << n, n), -1, dtype=np.int8)
    masks = np.arange(1 << n)
    popcount = np.zeros(1 << n, dtype=np.int8)
    for j in range(n):
        popcount += ((masks >> j) & 1).astype(np.int8)
        dp[1 << j, j] = cost[0, j + 1]
    for size in range(2, n + 1):
        layer = masks[popcount == size]
        for k in range(n):
            sub = layer[(layer >> k) & 1 == 1] ^ (1 << k)  # masks with k removed
            cand = dp[sub] + sites[:, k]                    # (subsets, j)
            j = np.argmin(cand, axis=1)
            dp[sub | (1 << k), k] = cand[np.arange(len(sub)), j]
            parent[sub | (1 << k), k] = j
    finals = dp[full] + (cost[1:, 0] if closed else 0.0)
    last = int(np.argmin(finals))
    order: List[int] = []
    mask, j = full, last
    while j != -1:
        order.append(j + 1)
        prev = int(parent[mask, j])
        mask ^= 1 << j
        j = prev
    order.reverse()
    return TourResult("held_karp", order, float(finals[last]),
                      time.perf_counter() - started)


def nearest_neighbour(cost: np.ndarray, closed: bool = True) -> TourResult:
    cost = np.asarray(cost, dtype=np.float64)
    n = n_sites(cost)
    started = time.perf_counter()
    remaining = set(range(1, n + 1))
    order: List[int] = []
    current = 0
    while remaining:
        # Ties broken by index for determinism.
        nxt = min(remaining, key=lambda s: (cost[current, s], s))
        order.append(nxt)
        remaining.remove(nxt)
        current = nxt
    return TourResult("nearest_neighbour", order, tour_cost(cost, order, closed),
                      time.perf_counter() - started)


def two_opt(cost: np.ndarray, order: Sequence[int], closed: bool = True) -> TourResult:
    """Best-improvement 2-opt (segment reversal) until a local optimum.

    With asymmetric legs, reversing seq[i..j] also flips the direction of every
    leg inside the segment, so the move is priced as

        delta = c(prev, s_j) + sum_{k=i}^{j-1} c(s_{k+1}, s_k) + c(s_i, next)
              - c(prev, s_i) - sum_{k=i}^{j-1} c(s_k, s_{k+1}) - c(s_j, next)

    using forward / reverse prefix sums. For open tours the segment may end at
    the last site, in which case the `next` terms vanish.
    """
    cost = np.asarray(cost, dtype=np.float64)
    n = n_sites(cost)
    validate_tour(order, n)
    started = time.perf_counter()
    best = list(order)
    moves = 0
    while True:
        seq = [0] + best + ([0] if closed else [])
        m = len(seq)
        fwd = np.zeros(m)
        rev = np.zeros(m)
        for k in range(m - 1):
            fwd[k + 1] = fwd[k] + cost[seq[k], seq[k + 1]]
            rev[k + 1] = rev[k] + cost[seq[k + 1], seq[k]]
        best_delta, move = -1e-12, None
        for i in range(1, n):            # seq[1..n] are the sites
            for j in range(i + 1, n + 1):
                old = cost[seq[i - 1], seq[i]] + (fwd[j] - fwd[i])
                new = cost[seq[i - 1], seq[j]] + (rev[j] - rev[i])
                if j + 1 < m:
                    old += cost[seq[j], seq[j + 1]]
                    new += cost[seq[i], seq[j + 1]]
                if new - old < best_delta:
                    best_delta, move = new - old, (i, j)
        if move is None:
            break
        i, j = move
        best = best[:i - 1] + best[i - 1:j][::-1] + best[j:]
        moves += 1
    return TourResult("two_opt", best, tour_cost(cost, best, closed),
                      time.perf_counter() - started, {"moves": moves})


def nn_two_opt(cost: np.ndarray, closed: bool = True) -> TourResult:
    started = time.perf_counter()
    nn = nearest_neighbour(cost, closed)
    refined = two_opt(cost, nn.order, closed)
    return TourResult("nn_2opt", refined.order, refined.cost,
                      time.perf_counter() - started,
                      {"nn_cost": nn.cost, "moves": refined.info["moves"]})


def window_reorder_exact(cost: np.ndarray, order: Sequence[int], closed: bool = True,
                         window: int = 3, max_sweeps: int = 2) -> TourResult:
    """Classical twin of tsp_qaoa.solve_tour_qaoa_windowed: same windows and
    sweep/accept rule, but each window is re-ordered by enumerating window!
    permutations. Isolates what QAOA adds over the decomposition itself."""
    cost = np.asarray(cost, dtype=np.float64)
    n = n_sites(cost)
    validate_tour(order, n)
    window = min(window, n)
    started = time.perf_counter()
    best = list(order)
    improvements = 0
    for _ in range(max_sweeps):
        improved = False
        for s in range(0, n - window + 1):
            inner = best[s:s + window]
            prev = 0 if s == 0 else best[s - 1]
            nxt = best[s + window] if s + window < n else (0 if closed else None)
            current = path_cost(cost, prev, inner, nxt)
            cand = min(itertools.permutations(inner),
                       key=lambda p: path_cost(cost, prev, p, nxt))
            if path_cost(cost, prev, cand, nxt) < current - 1e-9:
                best[s:s + window] = list(cand)
                improved = True
                improvements += 1
        if not improved:
            break
    return TourResult("window_exact", best, tour_cost(cost, best, closed),
                      time.perf_counter() - started, {"improvements": improvements})


def random_tour_stats(cost: np.ndarray, closed: bool = True, samples: int = 1000,
                      seed: int = 0) -> Dict[str, float]:
    """Mean / min cost of uniformly random tours - a scale reference only."""
    n = n_sites(cost)
    rng = np.random.default_rng(seed)
    costs = [tour_cost(cost, (rng.permutation(n) + 1).tolist(), closed)
             for _ in range(samples)]
    return {"mean": float(np.mean(costs)), "min": float(np.min(costs))}


def best_of_random(cost: np.ndarray, samples: int, closed: bool = True,
                   seed: int = 0) -> TourResult:
    """Best of `samples` uniformly random tours. Used as a control for QAOA's
    best-of-feasible-shots decoding with the same number of candidates."""
    n = n_sites(cost)
    rng = np.random.default_rng(seed)
    started = time.perf_counter()
    best_order, best_cost = None, math.inf
    for _ in range(max(1, samples)):
        order = (rng.permutation(n) + 1).tolist()
        c = tour_cost(cost, order, closed)
        if c < best_cost:
            best_order, best_cost = order, c
    return TourResult("best_of_random", best_order, best_cost,
                      time.perf_counter() - started, {"samples": max(1, samples)})
