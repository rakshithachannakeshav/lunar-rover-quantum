#!/usr/bin/env python3
"""
orienteering_qaoa.py
--------------------
QAOA on the n-qubit orienteering problem (one qubit per site: visit / skip).

Uses the numpy simulator in qaoa_sim with the exact diagonal cost from
orienteering.qaoa_cost_diagonal (a simulation shortcut: the subset-cost table
is applied directly, not compiled to gates).

Reported per run (P_* are exact probabilities from the final state, not sampled):
  p_opt, p_feasible        probability mass on optimal / feasible subsets
  uniform_p_opt, ..._feas  the same for a uniformly random bitstring
  amplification            p_opt / uniform_p_opt
  feasible_enrichment      p_feasible / uniform_p_feasible
  conditional_amp          [p_opt / p_feasible] / [n_opt / n_feasible]
                           ~1 means: once a sample is feasible, QAOA picks an
                           optimal one no more often than a random feasible
                           subset would (i.e. it learned feasibility, not value)
  reward_ratio             best FEASIBLE sampled subset's reward / optimum
  shots_to_solution        samples for a 99% chance of seeing an optimum
With `params` given, no training happens (parameter transfer from another
instance); the angles are just evaluated.
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from planning_pkg.orienteering import (
    OrienteeringInstance,
    Tables,
    qaoa_cost_diagonal,
)
from planning_pkg.qaoa_sim import (
    final_probs,
    sample_indices,
    shots_for_confidence,
    train_qaoa,
)


def run_orienteering_qaoa(inst: OrienteeringInstance, tab: Tables, exact: Dict, p: int,
                          alpha: float = 1.0, shots: int = 4096, seed: int = 0,
                          maxiter: int = 80, restarts: int = 1,
                          params: Optional[np.ndarray] = None) -> Dict:
    C = qaoa_cost_diagonal(inst, tab)
    n_evals = 0
    if params is None:
        trained = train_qaoa(C, p, alpha, maxiter=maxiter, restarts=restarts, seed=seed)
        params, n_evals = trained.params, trained.n_evals
    probs = final_probs(C, params)

    size = 1 << inst.n
    feas = tab.feasible
    p_feasible = float(probs[feas].sum())
    p_opt = float(probs[exact["optimal_masks"]].sum())
    uniform_opt = exact["n_optimal"] / size
    uniform_feas = exact["n_feasible"] / size

    rng = np.random.default_rng(seed + 12345)
    drawn = sample_indices(probs, shots, rng)
    ok = drawn[feas[drawn]]
    if len(ok):
        order = np.lexsort((tab.T[ok], -tab.R[ok]))
        best = int(ok[order[0]])
        best_reward = float(tab.R[best])
    else:
        best, best_reward = 0, 0.0

    cond = (p_opt / p_feasible) / (exact["n_optimal"] / exact["n_feasible"]) \
        if p_feasible > 0 else float("nan")
    return {
        "p": p, "alpha": alpha, "qubits": inst.n, "shots": shots,
        "p_opt": p_opt, "p_feasible": p_feasible,
        "uniform_p_opt": uniform_opt, "uniform_p_feasible": uniform_feas,
        "amplification": p_opt / uniform_opt,
        "feasible_enrichment": p_feasible / uniform_feas,
        "conditional_amp": cond,
        "reward": best_reward, "reward_ratio": best_reward / exact["reward"],
        "found_optimum": best_reward >= exact["reward"] - 1e-9,
        "shots_to_solution": shots_for_confidence(p_opt),
        "n_evals": n_evals, "params": [float(v) for v in params],
        "transferred": n_evals == 0,
    }
