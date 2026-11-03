#!/usr/bin/env python3
"""Tests for planning_pkg.qaoa_warm_start (warm-started compact QAOA, angle transfer)."""

import math
import os
import sys

import numpy as np
import pytest

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'planning_pkg'))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from planning_pkg import qaoa_warm_start as W  # noqa: E402
from planning_pkg.instance_sets import synthetic_cost_matrix  # noqa: E402
from planning_pkg.qaoa_sim import final_probs  # noqa: E402
from planning_pkg.tsp_baselines import nn_two_opt  # noqa: E402
from planning_pkg.tsp_compact_qaoa import build_compact_problem  # noqa: E402


@pytest.fixture(scope="module")
def prob():
    return build_compact_problem(synthetic_cost_matrix(5, seed=2))


def test_initial_state_puts_qubit_k_on_bit_k():
    thetas = np.array([math.pi, 0.0, math.pi])      # |1>, |0>, |1>  -> index 0b101
    psi = W.initial_state(thetas)
    assert np.argmax(np.abs(psi)) == 0b101
    assert np.abs(psi[0b101]) == pytest.approx(1.0)


def test_eps_half_is_standard_qaoa_with_mirrored_beta(prob):
    q = prob.q
    params = np.array([0.7, 1.3, 0.4, 0.9])
    thetas = W.warm_start_thetas(0, q, 0.5)
    mirrored = np.concatenate([params[:2], -params[2:]])
    assert np.allclose(W.ws_probs(prob.C, params, thetas), final_probs(prob.C, mirrored))


def test_warm_start_state_is_a_mixer_eigenstate(prob):
    thetas = W.warm_start_thetas(17, prob.q, 0.2)
    psi0 = W.initial_state(thetas)
    after = W.ws_qaoa_state(np.zeros_like(prob.C), np.array([0.0, 0.8]), thetas)
    assert abs(np.vdot(psi0, after)) == pytest.approx(1.0)


def test_small_eps_concentrates_on_the_seed(prob):
    tour = nn_two_opt(synthetic_cost_matrix(5, seed=2), True).order
    idx = W.seed_index(prob.perms, tour)
    pr = W.ws_probs(prob.C, np.array([0.0, 0.0]), W.warm_start_thetas(idx, prob.q, 0.01))
    assert pr[idx] > 0.85


def test_seed_index_round_trips(prob):
    for i in (0, 7, len(prob.perms) - 1):
        assert W.seed_index(prob.perms, prob.perms[i]) == i


def test_training_lowers_the_mean_cost(prob):
    layers = W.train_layers(prob.C, 2, maxiter=30)
    uniform = float(np.mean(prob.C))
    assert layers[-1].objective < uniform
    assert layers[-1].cumulative_evals > layers[0].cumulative_evals
