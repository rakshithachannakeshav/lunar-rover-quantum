#!/usr/bin/env python3
"""
quantum_queries.py
------------------
Count COST EVALUATIONS ("oracle queries") instead of seconds.

Dürr-Høyer minimum finding (1996) is Grover search run repeatedly with a falling
threshold: keep a candidate y; Grover-search for any item strictly cheaper than
y (BBHT schedule, since the number of cheaper items is unknown); on success
replace y and repeat. It needs O(sqrt(M)) oracle queries to find the minimum of
M items; classical exhaustive search needs M. Here M = N! visiting orders.

What is simulated: Grover's success probability is exact,
    P(success after j iterations) = sin^2((2j + 1) * theta),  sin(theta) = sqrt(t / M),
with t the number of strictly cheaper items. The measured item is then drawn
uniformly from the cheaper items (success) or from the rest (failure). Each
Grover iteration is one oracle query, plus one query to classically check the
measured item. No state vector is built, so this is exact but cheap.

READ THIS BEFORE QUOTING NUMBERS
  * It is a query-complexity statement, not a runtime one. A quantum oracle that
    evaluates a tour cost in superposition is a reversible circuit, so a 'query'
    costs far more than a classical table lookup.
  * It beats brute force (M evaluations). It does NOT beat Held-Karp, whose
    cost is a different kind of count (about N^2 * 2^N basic steps), nor good
    heuristics. The raw counts are therefore not directly comparable across
    model types; they are printed side by side for orientation only.
  * Query counts are for finding the optimum for the first time; the algorithm
    itself does not know when it has (it runs until its budget), so the
    published stopping bound is reported too.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence

import numpy as np

LAMBDA = 6.0 / 5.0


def published_budget(m: float) -> float:
    """Dürr-Høyer: 22.5 sqrt(M) + 1.4 lg^2 M queries give success prob >= 1/2."""
    return 22.5 * math.sqrt(m) + 1.4 * math.log2(m) ** 2


def _grover_succeeds(t: float, m: float, j: int, rng: np.random.Generator) -> bool:
    theta = math.asin(math.sqrt(min(1.0, t / m)))
    return rng.random() < math.sin((2 * j + 1) * theta) ** 2


def durr_hoyer_ranks(m: int, n_optimal: int, rng: np.random.Generator,
                     query_cap: Optional[float] = None) -> Dict:
    """Rank model: M items sorted by cost, the first `n_optimal` tie as optimal
    and all other costs are distinct. Item i (0-based, sorted) has t strictly
    cheaper items, with t = 0 for i < n_optimal and t = i otherwise. The
    algorithm's behaviour depends only on t, so this is exact for any cost
    function with that tie structure and works for M too large to tabulate
    (e.g. 20!)."""
    cap = query_cap or 200.0 * math.sqrt(m) + 100.0
    i = int(rng.random() * m)
    queries = 1.0
    mm = 1.0
    root = math.sqrt(m)
    while queries < cap:
        t = 0 if i < n_optimal else i
        if t == 0:
            return {"queries": queries, "found": True}
        j = int(rng.random() * math.ceil(mm))
        queries += j + 1
        if _grover_succeeds(t, m, j, rng):
            i = int(rng.random() * t)
            mm = 1.0
        else:
            mm = min(LAMBDA * mm, root)
    return {"queries": queries, "found": False}


def durr_hoyer_costs(costs: np.ndarray, rng: np.random.Generator,
                     query_cap: Optional[float] = None) -> Dict:
    """Same algorithm on an explicit cost array (handles ties exactly)."""
    costs = np.asarray(costs, dtype=float)
    m = len(costs)
    order = np.argsort(costs, kind="stable")
    sorted_costs = costs[order]
    cap = query_cap or 200.0 * math.sqrt(m) + 100.0
    y = int(rng.integers(m))
    queries = 1.0
    mm = 1.0
    root = math.sqrt(m)
    while queries < cap:
        t = int(np.searchsorted(sorted_costs, costs[y], side="left"))
        if t == 0:
            return {"queries": queries, "found": True}
        j = int(rng.random() * math.ceil(mm))
        queries += j + 1
        if _grover_succeeds(t, m, j, rng):
            y = int(order[int(rng.integers(t))])
            mm = 1.0
        else:
            mm = min(LAMBDA * mm, root)
    return {"queries": queries, "found": False}


def summarise_trials(results: Sequence[Dict], m: float) -> Dict:
    q = np.array([r["queries"] for r in results if r["found"]], dtype=float)
    budget = published_budget(m)
    return {
        "trials": len(results),
        "found_rate": len(q) / max(1, len(results)),
        "mean_queries": float(q.mean()) if len(q) else float("nan"),
        "median_queries": float(np.median(q)) if len(q) else float("nan"),
        "p90_queries": float(np.percentile(q, 90)) if len(q) else float("nan"),
        "within_published_budget": float((q <= budget).sum() / max(1, len(results))),
        "published_budget": budget,
        "sqrt_m": math.sqrt(m),
    }


def classical_exhaustive_queries(m: int, n_optimal: int = 1) -> Dict:
    """Brute force: M evaluations to be sure; (M + 1) / (n_opt + 1) on average to
    first SEE an optimum if tours are scanned in random order."""
    return {"verify_all": float(m), "mean_to_first_optimum": (m + 1) / (n_optimal + 1)}


def held_karp_steps(n: int) -> float:
    """Rough basic-step count of the Held-Karp DP: n^2 * 2^n. A different
    counting model from oracle queries; for orientation only."""
    return float(n * n * 2 ** n)


def loglog_slope(xs: Sequence[float], ys: Sequence[float]) -> float:
    """Least-squares slope of log y on log x (about 0.5 for sqrt scaling)."""
    lx = np.log(np.asarray(xs, dtype=float))
    ly = np.log(np.asarray(ys, dtype=float))
    return float(np.polyfit(lx, ly, 1)[0])
