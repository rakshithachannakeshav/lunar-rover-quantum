#!/usr/bin/env python3
"""QUBO construction for the multi-site visiting-order problem (no Qiskit needed)."""

import itertools
import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.quantum_optimizer import _cost_lookup, qubo_to_ising
from planning_pkg.tsp_baselines import held_karp, path_cost, tour_cost
from planning_pkg.tsp_qubo import (
    build_tsp_qubo,
    count_feasible_states,
    decode_bits,
    encode_order,
    repair_from_marginals,
)


def random_cost(n_sites, seed):
    pts = np.random.default_rng(seed).uniform(0, 40, (n_sites + 1, 2))
    return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)


def all_bitstrings(n):
    return [[(k >> i) & 1 for i in range(n)] for k in range(2 ** n)]


@pytest.mark.parametrize("n", [1, 2, 3, 4, 6])
def test_variable_count_is_n_squared(n):
    tq = build_tsp_qubo(random_cost(n, 0), range(1, n + 1))
    assert tq.qubo.num_variables == n * n
    assert len(tq.qubo.variable_names) == n * n
    assert all(i < j for i, j in tq.qubo.quadratic)


@pytest.mark.parametrize("closed", [True, False])
def test_feasible_energy_equals_scaled_tour_cost(closed):
    cost = random_cost(4, 1)
    tq = build_tsp_qubo(cost, range(1, 5), start=0, end=0 if closed else None)
    for order in itertools.permutations(range(1, 5)):
        energy = tq.qubo.value(encode_order(tq, order))
        assert energy * tq.scale == pytest.approx(tour_cost(cost, order, closed))


def test_window_energy_matches_fixed_endpoint_path_cost():
    cost = random_cost(6, 2)
    tq = build_tsp_qubo(cost, [2, 5, 3], start=4, end=1)
    for order in itertools.permutations([2, 5, 3]):
        energy = tq.qubo.value(encode_order(tq, order))
        assert energy * tq.scale == pytest.approx(path_cost(cost, 4, order, 1))


def test_encode_decode_roundtrip():
    tq = build_tsp_qubo(random_cost(4, 3), range(1, 5))
    for order in itertools.permutations(range(1, 5)):
        assert decode_bits(tq, encode_order(tq, order)) == list(order)


def test_decode_rejects_constraint_violations():
    tq = build_tsp_qubo(random_cost(3, 4), range(1, 4))
    valid = encode_order(tq, [2, 3, 1])
    assert decode_bits(tq, [0] * 9) is None
    assert decode_bits(tq, [1] * 9) is None
    for i in range(9):  # any single flip breaks the permutation matrix
        flipped = list(valid)
        flipped[i] ^= 1
        assert decode_bits(tq, flipped) is None


@pytest.mark.parametrize("seed", range(4))
@pytest.mark.parametrize("closed", [True, False])
def test_ground_state_is_the_optimal_tour(seed, closed):
    """Exhaustive over all 2^9 assignments (N=3): the minimum is feasible
    and equals the Held-Karp optimum; every feasible state is penalty-free."""
    cost = random_cost(3, seed)
    tq = build_tsp_qubo(cost, range(1, 4), start=0, end=0 if closed else None)
    energies = _cost_lookup(tq.qubo)
    best = int(np.argmin(energies))
    order = decode_bits(tq, [(best >> i) & 1 for i in range(9)])
    assert order is not None
    assert energies[best] * tq.scale == pytest.approx(held_karp(cost, closed).cost)

    feasible = {k for k in range(2 ** 9)
                if decode_bits(tq, [(k >> i) & 1 for i in range(9)]) is not None}
    assert len(feasible) == count_feasible_states(3)
    infeasible_min = min(energies[k] for k in range(2 ** 9) if k not in feasible)
    assert infeasible_min > energies[best]
    # Feasible states carry no penalty: energy <= legs * (max scaled leg = 1).
    n_legs = 3 + int(closed)
    assert max(energies[k] for k in feasible) <= n_legs + 1e-9


def test_ground_state_n4_closed():
    """N=4 closed tour: exhaustive over 2^16 states."""
    cost = random_cost(4, 7)
    tq = build_tsp_qubo(cost, range(1, 5))
    energies = _cost_lookup(tq.qubo)
    best = int(np.argmin(energies))
    order = decode_bits(tq, [(best >> i) & 1 for i in range(16)])
    assert order is not None
    assert tour_cost(cost, order) == pytest.approx(held_karp(cost).cost)


def test_ising_form_matches_qubo_up_to_constant():
    tq = build_tsp_qubo(random_cost(3, 5), range(1, 4))
    h, zz = qubo_to_ising(tq.qubo)
    values = []
    for bits in all_bitstrings(9):
        z = [1 - 2 * b for b in bits]  # x = (1 - z) / 2
        ising = sum(c * z[i] for i, c in h.items())
        ising += sum(c * z[i] * z[j] for (i, j), c in zz.items())
        values.append(tq.qubo.value(bits) - ising)
    assert np.ptp(values) == pytest.approx(0.0, abs=1e-9)


def test_costs_are_scaled_to_unit_max():
    cost = random_cost(4, 6) * 1000.0
    tq = build_tsp_qubo(cost, range(1, 5))
    assert tq.scale == pytest.approx(cost.max())
    assert max(abs(v) for v in tq.qubo.quadratic.values()) <= 2 * tq.penalty + 1e-9


def test_repair_produces_a_permutation():
    tq = build_tsp_qubo(random_cost(4, 8), range(1, 5))
    rng = np.random.default_rng(0)
    order = repair_from_marginals(tq, rng.uniform(size=16))
    assert sorted(order) == [1, 2, 3, 4]
    # Exact marginals of a valid state recover that state.
    assert repair_from_marginals(tq, encode_order(tq, [3, 1, 4, 2])) == [3, 1, 4, 2]


def test_invalid_inputs_raise():
    cost = random_cost(3, 9)
    with pytest.raises(ValueError):
        build_tsp_qubo(cost, [])
    with pytest.raises(ValueError):
        build_tsp_qubo(cost, [1, 1, 2])
    with pytest.raises(ValueError):
        build_tsp_qubo(cost, [1, 2, 9])
    with pytest.raises(ValueError):
        build_tsp_qubo(-cost, [1, 2, 3])
    with pytest.raises(ValueError):
        build_tsp_qubo(cost, [1, 2, 3], penalty=0.0)
    with pytest.raises(ValueError):
        encode_order(build_tsp_qubo(cost, [1, 2, 3]), [1, 2, 2])


@pytest.mark.parametrize("closed", [True, False])
def test_asymmetric_costs_are_encoded_per_direction(closed):
    rng = np.random.default_rng(3)
    cost = rng.uniform(1, 20, (5, 5))
    np.fill_diagonal(cost, 0.0)
    assert not np.allclose(cost, cost.T)
    tq = build_tsp_qubo(cost, range(1, 5), start=0, end=0 if closed else None)
    for order in itertools.permutations(range(1, 5)):
        energy = tq.qubo.value(encode_order(tq, order))
        assert energy * tq.scale == pytest.approx(tour_cost(cost, order, closed))
    energies = _cost_lookup(tq.qubo)
    best = decode_bits(tq, [(int(np.argmin(energies)) >> i) & 1 for i in range(16)])
    assert tour_cost(cost, best, closed) == pytest.approx(held_karp(cost, closed).cost)
