#!/usr/bin/env python3
"""QAOA circuit and solver checks on tiny instances (Qiskit-Aer, a few seconds)."""

import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

pytest.importorskip("qiskit_aer")

from planning_pkg.quantum_optimizer import _cost_lookup
from planning_pkg.tsp_baselines import held_karp, validate_tour
from planning_pkg.tsp_qaoa import (
    QaoaConfig,
    QubitLimitError,
    _interp,
    _StatevectorObjective,
    circuit_stats,
    solve_tour_qaoa,
)
from planning_pkg.tsp_qubo import build_tsp_qubo


def random_cost(n_sites, seed):
    pts = np.random.default_rng(seed).uniform(0, 40, (n_sites + 1, 2))
    return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)


def numpy_qaoa_probs(costs, n, gammas, betas):
    """Reference QAOA: diagonal phase exp(-i*gamma*C) then RX(2*beta) on every qubit."""
    psi = np.full(2 ** n, 2 ** (-n / 2), dtype=complex)
    rx = lambda b: np.array([[np.cos(b), -1j * np.sin(b)], [-1j * np.sin(b), np.cos(b)]])
    for g, b in zip(gammas, betas):
        psi = psi * np.exp(-1j * g * costs)
        for q in range(n):
            psi = psi.reshape(2 ** (n - q - 1), 2, 2 ** q)
            psi = np.einsum("ij,ajb->aib", rx(b), psi).reshape(-1)
    return np.abs(psi) ** 2


@pytest.mark.parametrize("reps", [1, 2])
def test_aer_circuit_implements_the_qubo(reps):
    tq = build_tsp_qubo(random_cost(3, 0), range(1, 4))
    obj = _StatevectorObjective(tq, reps, cvar_alpha=1.0, seed=1)
    params = np.random.default_rng(reps).uniform(0, 1.5, 2 * reps)
    expected = numpy_qaoa_probs(_cost_lookup(tq.qubo), 9, params[:reps], params[reps:])
    assert np.allclose(obj.probabilities(params), expected, atol=1e-8)


def test_cvar_is_at_most_the_expectation():
    tq = build_tsp_qubo(random_cost(3, 1), range(1, 4))
    params = np.array([0.4, 0.3])
    full = _StatevectorObjective(tq, 1, 1.0, 0)(params)
    tail = _StatevectorObjective(tq, 1, 0.1, 0)(params)
    assert tail <= full + 1e-12


def test_interp_extends_depth():
    out = _interp(np.array([0.5, 0.2]), 1)
    assert out.shape == (4,)
    assert np.allclose(out, [0.5, 0.5, 0.2, 0.2])


@pytest.mark.parametrize("n,reps", [(3, 1), (4, 1), (4, 2)])
def test_circuit_stats(n, reps):
    stats = circuit_stats(build_tsp_qubo(random_cost(n, 2), range(1, n + 1)), reps)
    assert stats["qubits"] == n * n
    assert stats["two_qubit_gates"] > 0 and stats["depth"] > 0
    single = circuit_stats(build_tsp_qubo(random_cost(n, 2), range(1, n + 1)), 1)
    assert stats["two_qubit_gates"] == reps * single["two_qubit_gates"]


@pytest.mark.parametrize("closed", [True, False])
def test_qaoa_returns_a_valid_tour(closed):
    cost = random_cost(3, 3)
    cfg = QaoaConfig(reps=1, maxiter=40, restarts=0, shots=2048, grid_points=6, seed=3)
    result = solve_tour_qaoa(cost, closed, cfg)
    validate_tour(result.order, 3)
    assert result.cost >= held_karp(cost, closed).cost - 1e-9
    assert 0.0 <= result.info["p_feasible"] <= 1.0
    assert result.info["qubits"] == 9


def test_spsa_optimizer_runs():
    cfg = QaoaConfig(reps=1, optimizer="SPSA", maxiter=10, restarts=0, shots=256, grid_points=4)
    validate_tour(solve_tour_qaoa(random_cost(2, 4), True, cfg).order, 2)


def test_qubit_limit_and_bad_config():
    with pytest.raises(QubitLimitError):
        solve_tour_qaoa(random_cost(5, 0), True, QaoaConfig(max_qubits=20))
    with pytest.raises(ValueError):
        solve_tour_qaoa(random_cost(2, 0), True, QaoaConfig(optimizer="Adam"))
    with pytest.raises(ValueError):
        solve_tour_qaoa(random_cost(2, 0), True, QaoaConfig(cvar_alpha=0.0))
