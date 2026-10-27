"""Tests for planning_pkg.qaoa_sim (numpy QAOA simulator; no Qiskit needed)."""

import itertools
import math
import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg")
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from scipy.linalg import expm

from planning_pkg.qaoa_sim import (
    apply_x_mixer,
    final_probs,
    qaoa_state,
    shots_for_confidence,
    train_qaoa,
)


def _x_mixer_matrix(beta, q):
    X = np.array([[0, 1], [1, 0]], dtype=complex)
    m = expm(-1j * beta * X)
    full = np.array([[1]], dtype=complex)
    for _ in range(q):
        full = np.kron(full, m)
    return full


def test_mixer_matches_matrix_exponential():
    rng = np.random.default_rng(1)
    q = 4
    psi = rng.normal(size=1 << q) + 1j * rng.normal(size=1 << q)
    expect = _x_mixer_matrix(0.37, q) @ psi
    got = psi.copy()
    apply_x_mixer(got, 0.37, q)
    assert np.allclose(got, expect)


def test_state_is_normalised_and_probs_sum_to_one():
    C = np.random.default_rng(2).random(32)
    params = np.array([0.4, 0.9, 0.3, 0.2])
    assert abs(np.linalg.norm(qaoa_state(C, params)) - 1) < 1e-12
    assert abs(final_probs(C, params).sum() - 1) < 1e-9


def test_zero_angles_give_uniform():
    C = np.random.default_rng(3).random(16)
    assert np.allclose(final_probs(C, np.zeros(2)), 1 / 16)


def test_training_beats_uniform_on_simple_cost():
    q = 4
    C = np.array([bin(i).count("1") for i in range(1 << q)], dtype=float) / q
    res = train_qaoa(C, 2, maxiter=60, seed=0)
    probs = final_probs(C, res.params)
    assert probs @ C < C.mean() - 1e-3


def test_shots_for_confidence():
    assert shots_for_confidence(1.0) == 1
    assert 6 <= shots_for_confidence(0.5) <= 8
    assert shots_for_confidence(0.01) > shots_for_confidence(0.1)
