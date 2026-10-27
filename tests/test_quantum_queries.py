"""Tests for planning_pkg.quantum_queries (Durr-Hoyer query counting)."""

import itertools
import math
import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg")
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.quantum_queries import (
    classical_exhaustive_queries,
    durr_hoyer_costs,
    durr_hoyer_ranks,
    held_karp_steps,
    loglog_slope,
    published_budget,
    summarise_trials,
)


def test_finds_minimum_and_beats_brute_force():
    rng = np.random.default_rng(0)
    m = 4096
    res = [durr_hoyer_ranks(m, 1, rng) for _ in range(60)]
    s = summarise_trials(res, m)
    assert s["found_rate"] == 1.0
    assert s["mean_queries"] < m / 4
    assert s["mean_queries"] < published_budget(m)


def test_array_model_returns_true_minimum_reached():
    rng = np.random.default_rng(1)
    costs = np.random.default_rng(5).random(300)
    r = durr_hoyer_costs(costs, rng)
    assert r["found"] and r["queries"] > 0


def test_ties_are_all_optimal():
    rng = np.random.default_rng(2)
    costs = np.array([1.0, 1.0, 5.0, 7.0, 9.0, 11.0])
    assert all(durr_hoyer_costs(costs, rng)["found"] for _ in range(20))


def test_scaling_is_about_square_root():
    rng = np.random.default_rng(3)
    ms = [2 ** k for k in (8, 10, 12, 14, 16)]
    means = [np.mean([durr_hoyer_ranks(m, 1, rng)["queries"] for _ in range(80)]) for m in ms]
    assert 0.4 < loglog_slope(ms, means) < 0.65


def test_helpers():
    assert classical_exhaustive_queries(100)["verify_all"] == 100
    assert held_karp_steps(4) == 16 * 16
    assert loglog_slope([1, 4, 16], [1, 2, 4]) == pytest.approx(0.5)
