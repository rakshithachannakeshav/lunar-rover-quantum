"""Tests for planning_pkg.tsp_compact_qaoa (permutation-index encoding)."""

import itertools
import math
import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg")
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.tsp_baselines import held_karp
from planning_pkg.tsp_compact_qaoa import (
    build_compact_problem,
    compact_qubits,
    run_compact_qaoa,
    transfer_params,
)


def _cost(n, seed):
    rng = np.random.default_rng(seed)
    c = rng.random((n + 1, n + 1)) * 10
    np.fill_diagonal(c, 0)
    return c


def test_qubit_counts():
    assert [compact_qubits(n) for n in (4, 5, 6, 7, 8)] == [5, 7, 10, 13, 16]


def test_every_bitstring_is_a_valid_tour():
    prob = build_compact_problem(_cost(5, 0))
    assert prob.decode.min() == 0 and prob.decode.max() == 119
    assert prob.weight.sum() == 1 << prob.q


def test_optimum_matches_held_karp():
    c = _cost(6, 1)
    assert build_compact_problem(c).opt_cost == pytest.approx(held_karp(c).cost)


def test_run_reports_consistent_metrics():
    r = run_compact_qaoa(_cost(5, 2), 1, maxiter=15)
    assert 0 < r["p_opt"] <= 1 and r["qubits"] == 7 and r["onehot_qubits"] == 25
    assert r["mean_ratio_uniform"] >= 1.0
    t = run_compact_qaoa(_cost(5, 3), 1, params=transfer_params([r["params"], r["params"]]))
    assert t["transferred"]


def test_too_many_sites_rejected():
    with pytest.raises(ValueError):
        build_compact_problem(_cost(10, 0))
