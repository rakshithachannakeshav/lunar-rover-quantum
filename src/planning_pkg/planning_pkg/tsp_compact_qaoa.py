#!/usr/bin/env python3
"""
tsp_compact_qaoa.py
-------------------
Visiting-order QAOA with a COMPACT encoding: the tour is a permutation index
written in binary, so N sites need only ceil(log2 N!) qubits instead of N^2.

    N = 4: 5 qubits    N = 6: 10    N = 7: 13    N = 8: 16    N = 9: 19

(the one-hot QUBO in tsp_qubo needs 16 / 36 / 49 / 64 / 81). Index i decodes to
permutation i of 1..N in lexicographic order; indices >= N! wrap around
(i - N!), so EVERY bitstring decodes to a valid tour (P(feasible) = 1). Because
no probability mass is wasted on invalid tours, any gain over a uniformly random
bitstring is a gain in tour QUALITY, which is what the one-hot runs could not
isolate.

Honest limits:
  * Costs of all N! tours are applied as a lookup table (see qaoa_sim): a
    simulation shortcut, not a gate-level circuit.
  * Lexicographic indices share prefixes, so the landscape has some structure,
    but the standard X mixer does not know about permutations. Expect modest
    learning; this module measures how much.
  * Wrapped indices give a few tours double weight. `uniform_*` values below
    account for this exactly.

Metrics (exact, from the final state):
  p_opt, uniform_p_opt, amplification
  mean_ratio_qaoa / mean_ratio_uniform   E[tour cost] / optimum
  gap_closed   (E_uniform - E_qaoa) / (E_uniform - optimum): fraction of the
               average-vs-best gap that QAOA removes. 0 = same as random.
  shots_to_solution  samples for 99% chance of seeing an optimal tour
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from planning_pkg.qaoa_sim import (
    final_probs,
    sample_indices,
    shots_for_confidence,
    train_qaoa,
)

MAX_SITES = 9


def compact_qubits(n: int) -> int:
    return max(1, math.ceil(math.log2(math.factorial(n))))


@dataclass
class CompactProblem:
    n: int
    q: int
    perms: np.ndarray        # (N!, n) int8, lexicographic
    costs: np.ndarray        # (N!,) tour cost
    decode: np.ndarray       # (2^q,) bitstring -> permutation index
    weight: np.ndarray       # (N!,) how many bitstrings decode to each tour
    C: np.ndarray            # (2^q,) normalised cost in [0, 1] per bitstring
    opt_cost: float
    optimal: np.ndarray      # indices of optimal permutations


def tour_costs(cost: np.ndarray, perms: np.ndarray, closed: bool = True) -> np.ndarray:
    p = perms.astype(np.intp)
    total = cost[0, p[:, 0]] + cost[p[:, :-1], p[:, 1:]].sum(axis=1)
    if closed:
        total = total + cost[p[:, -1], 0]
    return total


def build_compact_problem(cost: np.ndarray, closed: bool = True) -> CompactProblem:
    cost = np.asarray(cost, dtype=np.float64)
    n = cost.shape[0] - 1
    if not 2 <= n <= MAX_SITES:
        raise ValueError(f"compact encoding supports 2..{MAX_SITES} sites; got {n}.")
    perms = np.array(list(itertools.permutations(range(1, n + 1))), dtype=np.int8)
    m = len(perms)
    costs = tour_costs(cost, perms, closed)
    q = compact_qubits(n)
    idx = np.arange(1 << q)
    decode = np.where(idx < m, idx, idx - m)
    weight = np.bincount(decode, minlength=m).astype(float)
    lo, hi = float(costs.min()), float(costs.max())
    norm = (costs - lo) / (hi - lo) if hi > lo else np.zeros_like(costs)
    opt = lo
    optimal = np.nonzero(costs <= lo + 1e-9 * max(1.0, abs(lo)))[0]
    return CompactProblem(n, q, perms, costs, decode, weight, norm[decode], opt, optimal)


def evaluate_distribution(prob: CompactProblem, probs: np.ndarray, shots: int,
                          seed: int) -> Dict:
    m = len(prob.costs)
    per_tour = np.bincount(prob.decode, weights=probs, minlength=m)
    uniform = prob.weight / prob.weight.sum()
    p_opt = float(per_tour[prob.optimal].sum())
    u_opt = float(uniform[prob.optimal].sum())
    e_q = float(np.dot(per_tour, prob.costs))
    e_u = float(np.dot(uniform, prob.costs))
    gap_closed = (e_u - e_q) / (e_u - prob.opt_cost) if e_u > prob.opt_cost else float("nan")
    rng = np.random.default_rng(seed + 777)
    drawn = prob.decode[sample_indices(probs, shots, rng)]
    best = float(prob.costs[drawn].min())
    rnd = prob.decode[rng.integers(0, 1 << prob.q, size=shots)]
    best_rnd = float(prob.costs[rnd].min())
    return {
        "p_opt": p_opt, "uniform_p_opt": u_opt, "amplification": p_opt / u_opt,
        "mean_ratio_qaoa": e_q / prob.opt_cost, "mean_ratio_uniform": e_u / prob.opt_cost,
        "gap_closed": gap_closed,
        "best_ratio": best / prob.opt_cost, "best_of_random_ratio": best_rnd / prob.opt_cost,
        "shots_to_solution": shots_for_confidence(p_opt),
        "uniform_shots_to_solution": shots_for_confidence(u_opt),
        "n_optimal": int(len(prob.optimal)),
    }


def run_compact_qaoa(cost: np.ndarray, p: int, alpha: float = 1.0, shots: int = 4096,
                     seed: int = 0, maxiter: int = 60, restarts: int = 1,
                     closed: bool = True, params: Optional[np.ndarray] = None,
                     problem: Optional[CompactProblem] = None) -> Dict:
    """Train (or, with `params`, just evaluate transferred angles) and report."""
    prob = problem or build_compact_problem(cost, closed)
    n_evals = 0
    if params is None:
        trained = train_qaoa(prob.C, p, alpha, maxiter=maxiter, restarts=restarts, seed=seed)
        params, n_evals = trained.params, trained.n_evals
    probs = final_probs(prob.C, params)
    out = evaluate_distribution(prob, probs, shots, seed)
    out.update({"n_sites": prob.n, "qubits": prob.q, "p": p, "alpha": alpha,
                "shots": shots, "n_evals": n_evals, "transferred": n_evals == 0,
                "params": [float(v) for v in params],
                "onehot_qubits": prob.n * prob.n})
    return out


def transfer_params(param_sets: Sequence[Sequence[float]]) -> np.ndarray:
    """Average angles found on training instances (same p), for reuse elsewhere."""
    arr = np.asarray(param_sets, dtype=float)
    if arr.ndim != 2:
        raise ValueError("param_sets must be a list of equal-length parameter vectors.")
    return arr.mean(axis=0)
