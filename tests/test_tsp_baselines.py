#!/usr/bin/env python3
"""Tour validity and classical baselines for the multi-site problem."""

import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.tsp_baselines import (
    best_of_random,
    brute_force,
    held_karp,
    is_valid_tour,
    nearest_neighbour,
    nn_two_opt,
    tour_cost,
    two_opt,
    validate_tour,
    window_reorder_exact,
)


def random_cost(n_sites, seed, symmetric=True):
    rng = np.random.default_rng(seed)
    if symmetric:
        pts = rng.uniform(0, 40, (n_sites + 1, 2))
        return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)
    m = rng.uniform(1, 30, (n_sites + 1, n_sites + 1))
    np.fill_diagonal(m, 0.0)
    return m


def test_validate_tour():
    validate_tour([2, 1, 3], 3)
    assert is_valid_tour([3, 1, 2], 3)
    for bad in ([1, 2], [1, 2, 2], [0, 1, 2], [1, 2, 3, 4], [1, 2, 4]):
        assert not is_valid_tour(bad, 3)
        with pytest.raises(ValueError):
            validate_tour(bad, 3)


def test_tour_cost_closed_and_open():
    cost = np.array([[0, 1, 4], [1, 0, 2], [4, 2, 0]], dtype=float)
    assert tour_cost(cost, [1, 2], closed=True) == pytest.approx(1 + 2 + 4)
    assert tour_cost(cost, [1, 2], closed=False) == pytest.approx(1 + 2)


ALL_SOLVERS = [brute_force, held_karp, nearest_neighbour, nn_two_opt]


@pytest.mark.parametrize("n", [1, 2, 4, 7])
@pytest.mark.parametrize("closed", [True, False])
@pytest.mark.parametrize("solver", ALL_SOLVERS)
def test_every_solver_visits_every_site_once(solver, n, closed):
    cost = random_cost(n, n)
    result = solver(cost, closed=closed)
    validate_tour(result.order, n)
    assert result.cost == pytest.approx(tour_cost(cost, result.order, closed))


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("closed", [True, False])
@pytest.mark.parametrize("symmetric", [True, False])
def test_held_karp_matches_brute_force(seed, closed, symmetric):
    cost = random_cost(6, seed, symmetric)
    assert held_karp(cost, closed).cost == pytest.approx(brute_force(cost, closed).cost)


def test_brute_force_chunking_is_exact():
    cost = random_cost(7, 11)
    assert brute_force(cost, chunk=97).cost == pytest.approx(brute_force(cost).cost)
    assert brute_force(cost).info["evaluated"] == 5040


@pytest.mark.parametrize("seed", range(10))
@pytest.mark.parametrize("closed", [True, False])
def test_heuristics_never_beat_the_optimum_and_2opt_never_hurts(seed, closed):
    cost = random_cost(8, seed)
    opt = held_karp(cost, closed).cost
    nn = nearest_neighbour(cost, closed)
    refined = two_opt(cost, nn.order, closed)
    assert refined.cost <= nn.cost + 1e-9
    assert refined.cost >= opt - 1e-9
    assert nn_two_opt(cost, closed).cost == pytest.approx(refined.cost)


def test_two_opt_reaches_a_local_optimum():
    cost = random_cost(9, 3)
    result = two_opt(cost, list(range(1, 10)))
    order = result.order
    for i in range(8):
        for j in range(i + 1, 9):
            cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
            assert tour_cost(cost, cand) >= result.cost - 1e-9


def test_two_opt_rejects_invalid_seed():
    with pytest.raises(ValueError):
        two_opt(random_cost(4, 0), [1, 2, 2, 3])


@pytest.mark.parametrize("closed", [True, False])
def test_window_reorder_exact_is_valid_and_monotone(closed):
    cost = random_cost(8, 5)
    seed = nearest_neighbour(cost, closed)
    result = window_reorder_exact(cost, seed.order, closed, window=3)
    validate_tour(result.order, 8)
    assert result.cost <= seed.cost + 1e-9


def test_best_of_random_is_valid():
    cost = random_cost(5, 2)
    result = best_of_random(cost, samples=50, seed=1)
    validate_tour(result.order, 5)
    assert result.cost >= held_karp(cost).cost - 1e-9


def test_bad_matrix_raises():
    with pytest.raises(ValueError):
        held_karp(np.zeros((1, 1)))
    with pytest.raises(ValueError):
        nearest_neighbour(np.zeros((3, 2)))


# --- asymmetric leg costs (uphill != downhill) -------------------------------

def asymmetric_cost(n_sites, seed):
    """Euclidean legs plus a per-direction climb penalty, like the cost map."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(0, 40, (n_sites + 1, 2))
    z = rng.uniform(0, 10, n_sites + 1)
    dist = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)
    return dist + 3.0 * np.maximum(z[None, :] - z[:, None], 0.0)


def test_asymmetric_fixture_is_asymmetric():
    cost = asymmetric_cost(5, 0)
    assert not np.allclose(cost, cost.T)


def test_tour_cost_respects_direction():
    cost = np.array([[0, 1, 9], [9, 0, 1], [1, 9, 0]], dtype=float)
    assert tour_cost(cost, [1, 2]) == pytest.approx(3.0)   # 0->1->2->0
    assert tour_cost(cost, [2, 1]) == pytest.approx(27.0)  # 0->2->1->0


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("closed", [True, False])
def test_exact_solvers_agree_on_asymmetric(seed, closed):
    cost = asymmetric_cost(7, seed)
    bf = brute_force(cost, closed)
    hk = held_karp(cost, closed)
    assert hk.cost == pytest.approx(bf.cost)
    assert tour_cost(cost, hk.order, closed) == pytest.approx(hk.cost)


def test_nearest_neighbour_follows_outgoing_legs():
    # Cheap to leave the depot towards 2, expensive to come back from it.
    cost = np.array([[0, 5, 1], [5, 0, 5], [50, 5, 0]], dtype=float)
    assert nearest_neighbour(cost).order == [2, 1]


@pytest.mark.parametrize("seed", range(8))
@pytest.mark.parametrize("closed", [True, False])
def test_two_opt_delta_prices_reversed_segment(seed, closed):
    """The incremental delta must match a full recomputation: every accepted move
    lowers the true cost and the result is a true 2-opt local optimum."""
    cost = asymmetric_cost(8, seed)
    start = nearest_neighbour(cost, closed).order
    result = two_opt(cost, start, closed)
    validate_tour(result.order, 8)
    assert result.cost == pytest.approx(tour_cost(cost, result.order, closed))
    assert result.cost <= tour_cost(cost, start, closed) + 1e-9
    assert result.cost >= held_karp(cost, closed).cost - 1e-9
    order = result.order
    for i in range(7):
        for j in range(i + 1, 8):
            cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
            assert tour_cost(cost, cand, closed) >= result.cost - 1e-9


def test_two_opt_reversal_cost_is_not_symmetric_shortcut():
    """A symmetric-delta 2-opt would think reversing [1,2,3] is free (it only
    looks at the two boundary legs); here the reversed interior is much dearer."""
    cost = np.full((4, 4), 1.0)
    np.fill_diagonal(cost, 0.0)
    cost[1, 2] = cost[2, 3] = 0.1  # forward interior cheap
    cost[3, 2] = cost[2, 1] = 10.0  # reversed interior expensive
    cost[0, 3] = cost[1, 0] = 0.5   # boundary legs alone favour reversing
    # Boundary-only delta: (0.5 + 0.5) - (1 + 1) = -1 -> would reverse (wrongly).
    # True delta adds 20 - 0.2 for the reversed interior -> must keep the order.
    assert tour_cost(cost, [3, 2, 1]) > tour_cost(cost, [1, 2, 3])
    result = two_opt(cost, [1, 2, 3])
    assert result.order == [1, 2, 3]
    assert result.info["moves"] == 0


def test_count_optimal_orders():
    from planning_pkg.tsp_baselines import count_optimal_orders
    sym = random_cost(5, 1)
    assert count_optimal_orders(sym, closed=True) % 2 == 0  # tour + reverse
    asym = asymmetric_cost(5, 1)
    n_opt = count_optimal_orders(asym, closed=True)
    assert n_opt >= 1
    assert n_opt == sum(
        1 for o in __import__("itertools").permutations(range(1, 6))
        if tour_cost(asym, o) <= held_karp(asym).cost + 1e-9)


@pytest.mark.parametrize("closed", [True, False])
def test_held_karp_matches_brute_force_on_asymmetric(closed):
    for n in (1, 2, 5, 8):
        cost = random_cost(n, seed=n, symmetric=False)
        assert held_karp(cost, closed).cost == pytest.approx(brute_force(cost, closed).cost)


def test_held_karp_scales_to_larger_missions():
    cost = random_cost(16, seed=3)
    result = held_karp(cost)
    validate_tour(result.order, 16)
    assert result.cost == pytest.approx(tour_cost(cost, result.order))
    assert result.cost <= nn_two_opt(cost).cost + 1e-9
