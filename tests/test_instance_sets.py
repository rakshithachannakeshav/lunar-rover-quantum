"""Tests for planning_pkg.instance_sets (instance families and gap statistics)."""

import itertools
import math
import os
import sys

import numpy as np
import pytest

PKG_DIR = os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg")
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.instance_sets import (
    gap_record,
    multistart_two_opt,
    synthetic_cost_matrix,
)
from planning_pkg.tsp_baselines import held_karp, nn_two_opt


def test_synthetic_is_deterministic_and_asymmetric():
    a = synthetic_cost_matrix(6, 3, asym=8.0, relief=10.0)
    b = synthetic_cost_matrix(6, 3, asym=8.0, relief=10.0)
    assert a.shape == (7, 7) and np.array_equal(a, b)
    assert not np.allclose(a, a.T)
    assert np.allclose(np.diag(a), 0)


def test_symmetric_when_no_asymmetry():
    c = synthetic_cost_matrix(6, 1, clusters=1, asym=0.0, relief=0.0)
    assert np.allclose(c, c.T)


def test_gap_record_ordering():
    c = synthetic_cost_matrix(7, 2)
    r = gap_record(c, 2, True, 5)
    assert r.optimal <= r.multistart + 1e-9
    assert r.multistart <= r.nn_2opt + 1e-9
    assert r.nn_2opt_gap_pct >= -1e-9 and r.multistart_gap_pct >= -1e-9
    assert r.optimal == pytest.approx(held_karp(c).cost)


def test_multistart_not_worse_than_nn_2opt():
    c = synthetic_cost_matrix(8, 5)
    assert multistart_two_opt(c, 10).cost <= nn_two_opt(c).cost + 1e-9
