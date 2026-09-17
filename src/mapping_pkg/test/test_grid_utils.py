#!/usr/bin/env python3

# Copyright 2026 Rakshitha Channakeshav
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Unit tests for mapping_pkg grid utilities and classification."""

import math
import os
import tempfile

import cv2
from mapping_pkg.grid_utils import (
    bresenham,
    coord_to_index,
    grid_to_world,
    index_to_coord,
    is_in_bounds,
    log_odds_to_occupancy_grid,
    log_odds_to_probability,
    probability_to_log_odds,
    world_to_grid,
)
from mapping_pkg.terrain_storage import TerrainStorage
import numpy as np


def test_world_to_grid_and_round_trip():
    """Verify coordinate transforms between continuous world meters and discrete grid cells."""
    origin_x = -17.0
    origin_y = -15.0
    res = 0.05

    col, row = world_to_grid(origin_x, origin_y, origin_x, origin_y, res)
    assert col == 0
    assert row == 0

    wx, wy = grid_to_world(0, 0, origin_x, origin_y, res)
    assert math.isclose(wx, origin_x + 0.025, abs_tol=1e-6)
    assert math.isclose(wy, origin_y + 0.025, abs_tol=1e-6)

    for test_col in [0, 5, 25, 120, 599]:
        for test_row in [0, 10, 50, 300, 599]:
            x, y = grid_to_world(test_col, test_row, origin_x, origin_y, res)
            c_rt, r_rt = world_to_grid(x, y, origin_x, origin_y, res)
            assert (c_rt, r_rt) == (test_col, test_row)


def test_bounds_and_index_conversions():
    """Verify bounds checking and 1D <-> 2D coordinate indexing."""
    width, height = 600, 600

    assert is_in_bounds(0, 0, width, height) is True
    assert is_in_bounds(599, 599, width, height) is True
    assert is_in_bounds(-1, 0, width, height) is False
    assert is_in_bounds(0, -1, width, height) is False
    assert is_in_bounds(600, 10, width, height) is False
    assert is_in_bounds(10, 600, width, height) is False

    idx = coord_to_index(42, 17, width)
    assert idx == 17 * width + 42
    c, r = index_to_coord(idx, width)
    assert (c, r) == (42, 17)


def test_bresenham_line_trace():
    """Verify Bresenham line algorithm on cardinal and diagonal paths."""
    pts = bresenham(0, 0, 4, 0)
    assert pts == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]

    pts_v = bresenham(2, 1, 2, 4)
    assert pts_v == [(2, 1), (2, 2), (2, 3), (2, 4)]

    pts_d = bresenham(0, 0, 3, 3)
    assert pts_d == [(0, 0), (1, 1), (2, 2), (3, 3)]

    pts_rev = bresenham(3, 3, 0, 0)
    assert pts_rev[0] == (3, 3)
    assert pts_rev[-1] == (0, 0)
    assert len(pts_rev) == 4


def test_log_odds_conversion_boundaries():
    """Test log-odds to probability conversion and boundary limits."""
    assert math.isclose(log_odds_to_probability(0.0), 0.5, abs_tol=1e-6)
    assert math.isclose(probability_to_log_odds(0.5), 0.0, abs_tol=1e-6)

    p_high = log_odds_to_probability(30.0)
    assert math.isclose(p_high, 1.0, abs_tol=1e-4)

    p_low = log_odds_to_probability(-30.0)
    assert math.isclose(p_low, 0.0, abs_tol=1e-4)

    log_odds = np.array([
        [0.0, -2.0],
        [3.5, 0.0]
    ], dtype=np.float32)

    observed = np.array([
        [True, True],
        [True, False]
    ], dtype=bool)

    grid = log_odds_to_occupancy_grid(log_odds, observed)
    assert grid[0, 0] == 50
    assert grid[0, 1] < 15
    assert grid[1, 0] > 95
    assert grid[1, 1] == -1


def test_synthetic_crater_classification():
    """Feed synthetic obstacle ring and verify interior classified as crater (90)."""
    h, w = 120, 120
    cx, cy, r = 60, 60, 25

    raw_map = np.full((h, w), 10, dtype=np.int8)

    for row in range(h):
        for col in range(w):
            dist = math.hypot(col - cx, row - cy)
            if abs(dist - r) <= 1.5:
                raw_map[row, col] = 100

    terrain_grid = np.full((h, w), -1, dtype=np.int8)
    terrain_grid[(raw_map >= 0) & (raw_map < 50)] = 10
    terrain_grid[raw_map >= 50] = 100

    obstacle_mask = np.zeros((h, w), dtype=np.uint8)
    obstacle_mask[raw_map >= 50] = 255
    blurred = cv2.GaussianBlur(obstacle_mask, (5, 5), 1.5)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=20.0,
        param1=50.0,
        param2=15.0,
        minRadius=20,
        maxRadius=35
    )

    assert circles is not None, 'HoughCircles should detect the synthetic circular crater rim'
    detected = circles[0, 0]
    det_cx, det_cy, det_r = int(detected[0]), int(detected[1]), int(detected[2])

    assert abs(det_cx - cx) <= 3
    assert abs(det_cy - cy) <= 3
    assert abs(det_r - r) <= 3

    r_inner = int(det_r * 0.85)
    for row in range(max(0, det_cy - r_inner), min(h, det_cy + r_inner + 1)):
        for col in range(max(0, det_cx - r_inner), min(w, det_cx + r_inner + 1)):
            if (col - det_cx) ** 2 + (row - det_cy) ** 2 <= r_inner ** 2:
                if terrain_grid[row, col] in (10, 50):
                    terrain_grid[row, col] = 90

    assert terrain_grid[cy, cx] == 90
    assert terrain_grid[cy + 5, cx + 5] == 90
    assert terrain_grid[cy, cx + r] == 100
    assert terrain_grid[10, 10] == 10


def test_terrain_storage_persistence():
    """Verify TerrainStorage writes valid latest.npz and latest_meta.json files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        storage = TerrainStorage(output_dir=tmpdir, save_interval_sec=0.1)

        grid = np.array([
            [-1, 10, 50],
            [10, 90, 100],
            [10, 10, 10]
        ], dtype=np.int8)

        saved = storage.save(
            terrain_grid=grid,
            resolution=0.05,
            origin_x=-17.0,
            origin_y=-15.0,
            width=3,
            height=3
        )

        assert os.path.exists(saved['latest_npz'])
        assert os.path.exists(saved['latest_json'])

        with np.load(saved['latest_npz']) as loaded:
            np.testing.assert_array_equal(loaded['grid'], grid)
            assert float(loaded['resolution']) == 0.05
            assert float(loaded['origin_x']) == -17.0
            assert float(loaded['origin_y']) == -15.0
            assert int(loaded['width']) == 3
            assert int(loaded['height']) == 3
