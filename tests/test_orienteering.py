"""Tests for planning_pkg.orienteering (prize-collecting routing, exact DP)."""

import itertools
import math
import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg")
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.orienteering import (
    OrienteeringInstance,
    build_tables,
    greedy,
    greedy_local_search,
    make_instance,
    qaoa_cost_diagonal,
    solve_exact,
    subset_times,
)


def _cost(n, seed):
    rng = np.random.default_rng(seed)
    c = rng.random((n + 1, n + 1)) * 10 + 1
    np.fill_diagonal(c, 0)
    return c


def _brute_time(cost, sites):
    if not sites:
        return 0.0
    best = math.inf
    for perm in itertools.permutations(sites):
        t = cost[0, perm[0]] + sum(cost[a, b] for a, b in zip(perm, perm[1:])) + cost[perm[-1], 0]
        best = min(best, t)
    return best


def test_subset_times_match_brute_force():
    n = 5
    cost = _cost(n, 0)
    T = subset_times(cost)
    for mask in range(1, 1 << n):
        sites = [j + 1 for j in range(n) if (mask >> j) & 1]
        assert T[mask] == pytest.approx(_brute_time(cost, sites))
    assert T[0] == 0


def test_exact_is_feasible_and_not_beaten_by_heuristics():
    for seed in range(5):
        inst = make_instance(_cost(7, seed), seed, 0.5, windows=(seed % 2 == 0))
        tab = build_tables(inst)
        ex = solve_exact(inst, tab)
        assert tab.feasible[ex["mask"]]
        assert greedy(inst, tab).reward <= ex["reward"] + 1e-9
        assert greedy_local_search(inst, tab).reward <= ex["reward"] + 1e-9


def test_windows_never_help():
    cost = _cost(6, 4)
    plain = make_instance(cost, 4, 0.6)
    win = OrienteeringInstance(cost, plain.reward, plain.budget, np.full(6, 5.0), np.full(6, 30.0))
    assert (build_tables(win).T[1:] >= build_tables(plain).T[1:] - 1e-9).all()
    assert solve_exact(win)["reward"] <= solve_exact(plain)["reward"] + 1e-9


def test_qaoa_diagonal_prefers_optimum():
    inst = make_instance(_cost(6, 1), 1, 0.5)
    tab = build_tables(inst)
    C = qaoa_cost_diagonal(inst, tab)
    assert int(np.argmin(C)) in set(solve_exact(inst, tab)["optimal_masks"].tolist())
