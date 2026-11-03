#!/usr/bin/env python3
"""Tests for planning_pkg.quantum_local_search (Threshold-QAOA 2-opt local search)."""

import math
import os
import sys

import numpy as np
import pytest

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'planning_pkg'))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from planning_pkg import quantum_local_search as Q  # noqa: E402
from planning_pkg.instance_sets import synthetic_cost_matrix  # noqa: E402
from planning_pkg.tsp_baselines import held_karp  # noqa: E402


@pytest.mark.parametrize("n", [2, 3, 6, 9])
def test_neighbour_costs_match_brute_force_on_asymmetric_costs(n):
    cost = synthetic_cost_matrix(n, seed=n)
    assert not np.allclose(cost, cost.T)
    rng = np.random.default_rng(n)
    order = list(rng.permutation(np.arange(1, n + 1)))
    pairs = Q.two_opt_pairs(n)
    brute = [Q.tour_cost(cost, Q.apply_move(order, i, j)) for i, j in pairs]
    assert np.allclose(Q.neighbour_costs(cost, order, pairs), brute)


def test_grover_probability_matches_known_values():
    assert Q.grover_success_probability(0, 100, 5) == 0.0
    assert Q.grover_success_probability(1, 4, 1) == pytest.approx(1.0)   # one Grover step, N=4
    assert Q.grover_success_probability(10, 100, 0) == pytest.approx(0.1)


def test_first_marked_position_has_the_exact_mean():
    rng = np.random.default_rng(0)
    K, m = 60, 4
    xs = [Q._first_marked_position(K, m, rng) for _ in range(30000)]
    assert np.mean(xs) == pytest.approx((K - m) / (m + 1), rel=0.03)


def test_quantum_search_beats_classical_scan_when_improvers_are_rare():
    rng = np.random.default_rng(1)
    K, m = 1225, 1
    q = np.mean([Q.quantum_search_move(m, K, rng)[1] for _ in range(2000)])
    assert q < (K + 1) / (m + 1) / 4          # >4x fewer queries than classical


def test_quantum_search_rarely_stops_early():
    rng = np.random.default_rng(2)
    stops = sum(not Q.quantum_search_move(1, 4950, rng)[0] for _ in range(3000))
    assert stops / 3000 < 0.005


@pytest.mark.parametrize("descent", [Q.classical_descent, Q.quantum_descent])
def test_descents_reach_a_2opt_local_optimum(descent):
    cost = synthetic_cost_matrix(10, seed=3)
    pairs = Q.two_opt_pairs(10)
    r = descent(cost, np.random.default_rng(4))
    assert r.cost == pytest.approx(Q.tour_cost(cost, r.order))
    if r.true_local_optimum:
        assert Q.neighbour_costs(cost, r.order, pairs).min() >= r.cost - 1e-9
    assert r.queries >= 1


def test_both_descents_find_the_optimum_with_restarts():
    cost = synthetic_cost_matrix(7, seed=5)
    opt = held_karp(cost, True).cost
    for descent in (Q.classical_descent, Q.quantum_descent):
        q = Q.queries_to_target(descent, cost, opt * (1 + 1e-9), np.random.default_rng(6), cap=1e6)
        assert math.isfinite(q)


def test_descents_have_the_same_quality_distribution():
    """Both pick a uniformly random improving move, so final costs match in distribution."""
    cost = synthetic_cost_matrix(12, seed=7)
    rng = np.random.default_rng(8)
    c = [Q.classical_descent(cost, rng).cost for _ in range(300)]
    q = [Q.quantum_descent(cost, rng).cost for _ in range(300)]
    assert np.mean(q) == pytest.approx(np.mean(c), rel=0.02)
