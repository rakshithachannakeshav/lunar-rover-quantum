#!/usr/bin/env python3
"""
orienteering.py
---------------
Which science sites should the rover visit before its battery runs out?

Instance: a depot (index 0) and n sites (indices 1..n) with a leg-cost matrix
`cost` (from A*, possibly asymmetric), a science reward per site, and a battery
`budget`. A chosen set S is feasible when the cheapest closed trip
depot -> S in some order -> depot costs at most `budget`. Maximise total reward.

Optional solar-illumination windows: site j can only be visited while it is lit,
i.e. the rover must ARRIVE no later than close[j]; arriving before open[j] means
waiting until open[j]. To keep a single resource, time = leg cost units and the
battery budget limits the whole mission duration INCLUDING waiting (the rover
keeps its heaters running). With no windows this is plain energy.

One dynamic programme (Held-Karp style, with waiting) gives the cheapest closed
trip for EVERY subset at once, so exact answers are cheap for n <= ~16:

    T[mask]  minimum mission time to visit exactly the sites in `mask`
             (inf if the windows make it impossible)
    R[mask]  total reward of `mask`

Classical baselines here read T[mask] as an oracle and count their lookups:
  greedy        add the site with best reward / extra time while it fits
  local_search  greedy, then add / drop / swap moves until none improves
  best_of_random_subsets   control with the same sample count as QAOA shots

Pure numpy; no rclpy, no Qiskit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np


@dataclass
class OrienteeringInstance:
    cost: np.ndarray                      # (n+1, n+1); 0 = depot; cost[a, b] = a -> b
    reward: np.ndarray                    # (n,)
    budget: float
    open: Optional[np.ndarray] = None     # (n,) earliest useful arrival (optional)
    close: Optional[np.ndarray] = None    # (n,) latest allowed arrival (optional)

    @property
    def n(self) -> int:
        return self.cost.shape[0] - 1

    @property
    def has_windows(self) -> bool:
        return self.open is not None or self.close is not None


def subset_times(cost: np.ndarray, open_: Optional[np.ndarray] = None,
                 close: Optional[np.ndarray] = None) -> np.ndarray:
    """T[mask] for every subset (bit j of mask = site j+1). T[0] = 0.

    dp[mask, j] = earliest time at which the rover can be at site j+1 having
    visited exactly `mask` (waiting allowed). Earliest arrival dominates
    because waiting is free, so keeping one value per (mask, j) is exact."""
    cost = np.asarray(cost, dtype=np.float64)
    n = cost.shape[0] - 1
    inf = math.inf
    op = np.zeros(n) if open_ is None else np.asarray(open_, dtype=np.float64)
    cl = np.full(n, inf) if close is None else np.asarray(close, dtype=np.float64)
    site = cost[1:, 1:]
    ks = np.arange(n)
    dp = np.full((1 << n, n), inf)
    for j in range(n):
        t = max(cost[0, j + 1], op[j])
        if t <= cl[j]:
            dp[1 << j, j] = t
    for mask in range(1, 1 << n):
        row = dp[mask]
        if not np.isfinite(row).any():
            continue
        cand = np.maximum(row[:, None] + site, op[None, :])
        cand[cand > cl[None, :]] = inf
        best = cand.min(axis=0)
        free = ((mask >> ks) & 1) == 0
        tgt_mask = mask | (1 << ks[free])
        tgt_k = ks[free]
        dp[tgt_mask, tgt_k] = np.minimum(dp[tgt_mask, tgt_k], best[free])
    total = (dp + cost[1:, 0][None, :]).min(axis=1)
    total[0] = 0.0
    return total


def subset_rewards(reward: np.ndarray) -> np.ndarray:
    n = len(reward)
    R = np.zeros(1 << n)
    for j in range(n):
        R[1 << j: 1 << (j + 1)] = R[: 1 << j] + reward[j]
    return R


@dataclass
class Tables:
    T: np.ndarray
    R: np.ndarray
    feasible: np.ndarray


def build_tables(inst: OrienteeringInstance) -> Tables:
    T = subset_times(inst.cost, inst.open, inst.close)
    R = subset_rewards(inst.reward)
    return Tables(T, R, T <= inst.budget + 1e-9)


def make_instance(cost: np.ndarray, seed: int, budget_frac: float = 0.5,
                  windows: bool = False, reward_range=(1, 10)) -> OrienteeringInstance:
    """Seeded rewards, a battery of `budget_frac` x the cheapest tour through ALL
    sites, and (optionally) a random lit window per site."""
    cost = np.asarray(cost, dtype=np.float64)
    n = cost.shape[0] - 1
    full_time = float(subset_times(cost)[(1 << n) - 1])
    for attempt in range(50):
        rng = np.random.default_rng(seed + 100003 * attempt)
        reward = rng.integers(reward_range[0], reward_range[1] + 1, size=n).astype(float)
        op = cl = None
        if windows:
            centre = rng.uniform(0.0, full_time, size=n)
            width = rng.uniform(0.35, 0.70, size=n) * full_time
            op = np.maximum(0.0, centre - width / 2)
            cl = centre + width / 2
        inst = OrienteeringInstance(cost, reward, budget_frac * full_time, op, cl)
        tab = build_tables(inst)
        if tab.feasible[1:].any():
            return inst
    raise ValueError("Could not build an instance with a feasible non-empty subset.")


def mask_to_sites(mask: int, n: int) -> List[int]:
    return [j + 1 for j in range(n) if (mask >> j) & 1]


@dataclass
class Solution:
    method: str
    mask: int
    reward: float
    time: float
    evals: int = 0


def solve_exact(inst: OrienteeringInstance, tab: Optional[Tables] = None) -> Dict:
    """Exhaustive optimum over all 2^n subsets (uses the DP tables)."""
    tab = tab or build_tables(inst)
    rew = np.where(tab.feasible, tab.R, -1.0)
    best = rew.max()
    optimal = np.nonzero(rew >= best - 1e-9)[0]
    pick = int(optimal[np.argmin(tab.T[optimal])])
    return {"reward": float(best), "mask": pick, "time": float(tab.T[pick]),
            "optimal_masks": optimal, "n_optimal": int(len(optimal)),
            "n_feasible": int(tab.feasible.sum())}


def greedy(inst: OrienteeringInstance, tab: Optional[Tables] = None) -> Solution:
    tab = tab or build_tables(inst)
    n, mask, evals = inst.n, 0, 0
    while True:
        best_k, best_ratio = -1, -1.0
        for k in range(n):
            if (mask >> k) & 1:
                continue
            new = mask | (1 << k)
            evals += 1
            if not tab.feasible[new]:
                continue
            extra = max(tab.T[new] - tab.T[mask], 1e-12)
            ratio = inst.reward[k] / extra
            if ratio > best_ratio:
                best_k, best_ratio = k, ratio
        if best_k < 0:
            break
        mask |= 1 << best_k
    return Solution("greedy", mask, float(tab.R[mask]), float(tab.T[mask]), evals)


def local_search(inst: OrienteeringInstance, start_mask: int = 0,
                 tab: Optional[Tables] = None) -> Solution:
    """Best-improvement add / drop / swap moves from `start_mask`."""
    tab = tab or build_tables(inst)
    n, mask, evals = inst.n, start_mask, 0
    while True:
        cur = (tab.R[mask], -tab.T[mask])
        best, best_key = None, cur
        candidates = []
        for k in range(n):
            bit = 1 << k
            if mask & bit:
                candidates.append(mask ^ bit)
                for m in range(n):
                    if not (mask >> m) & 1:
                        candidates.append((mask ^ bit) | (1 << m))
            else:
                candidates.append(mask | bit)
        for cand in candidates:
            evals += 1
            if not tab.feasible[cand]:
                continue
            key = (tab.R[cand], -tab.T[cand])
            if key > best_key:
                best, best_key = cand, key
        if best is None:
            break
        mask = best
    return Solution("greedy+local_search", mask, float(tab.R[mask]), float(tab.T[mask]), evals)


def greedy_local_search(inst: OrienteeringInstance,
                        tab: Optional[Tables] = None) -> Solution:
    tab = tab or build_tables(inst)
    g = greedy(inst, tab)
    ls = local_search(inst, g.mask, tab)
    ls.evals += g.evals
    return ls


def best_of_random_subsets(inst: OrienteeringInstance, samples: int, seed: int = 0,
                           tab: Optional[Tables] = None) -> Solution:
    """Best feasible reward among `samples` uniformly random subsets."""
    tab = tab or build_tables(inst)
    rng = np.random.default_rng(seed)
    masks = rng.integers(0, 1 << inst.n, size=max(1, samples))
    ok = tab.feasible[masks]
    if not ok.any():
        return Solution("best_of_random", 0, 0.0, 0.0, len(masks))
    cand = masks[ok]
    order = np.lexsort((tab.T[cand], -tab.R[cand]))
    pick = int(cand[order[0]])
    return Solution("best_of_random", pick, float(tab.R[pick]), float(tab.T[pick]), len(masks))


def qaoa_cost_diagonal(inst: OrienteeringInstance, tab: Tables) -> np.ndarray:
    """Cost of every bitstring for QAOA, scaled by total reward.

    Feasible set:   C = -reward / total          in [-1, 0]
    Infeasible set: C = (total - reward) / total + min(excess / budget, 1)  >= 0
    so every infeasible set is worse than every feasible one and the ground
    state is exactly the optimal feasible set. Built from the exact subset
    table (simulation shortcut, see qaoa_sim)."""
    total = float(tab.R[-1])
    with np.errstate(invalid="ignore"):
        excess = np.minimum((tab.T - inst.budget) / inst.budget, 1.0)
    excess = np.where(np.isfinite(excess), excess, 1.0)
    infeasible_cost = (total - tab.R) / total + np.maximum(excess, 0.0)
    return np.where(tab.feasible, -tab.R / total, infeasible_cost)
