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


"""Shared geometric and probabilistic utilities for occupancy grid mapping."""

import math
from typing import List, Tuple

import numpy as np


def world_to_grid(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float
) -> Tuple[int, int]:
    """Convert continuous world coordinates (x, y) in meters to discrete grid (col, row).

    Args:
        x: World X coordinate (m)
        y: World Y coordinate (m)
        origin_x: World X coordinate of grid origin (m)
        origin_y: World Y coordinate of grid origin (m)
        resolution: Grid cell size (m/cell)

    Returns:
        (col, row) integer cell indices.

    """
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor((y - origin_y) / resolution))
    return col, row


def grid_to_world(
    col: int,
    row: int,
    origin_x: float,
    origin_y: float,
    resolution: float
) -> Tuple[float, float]:
    """Convert discrete grid cell (col, row) to world coordinate center (x, y) in meters.

    Args:
        col: Grid column index (X)
        row: Grid row index (Y)
        origin_x: World X coordinate of grid origin (m)
        origin_y: World Y coordinate of grid origin (m)
        resolution: Grid cell size (m/cell)

    Returns:
        (x, y) center position of cell in meters.

    """
    x = origin_x + (float(col) + 0.5) * resolution
    y = origin_y + (float(row) + 0.5) * resolution
    return x, y


def is_in_bounds(col: int, row: int, width: int, height: int) -> bool:
    """Check if (col, row) lies within [0, width) and [0, height)."""
    return 0 <= col < width and 0 <= row < height


def coord_to_index(col: int, row: int, width: int) -> int:
    """Convert 2D (col, row) coordinate to flattened 1D row-major index."""
    return row * width + col


def index_to_coord(index: int, width: int) -> Tuple[int, int]:
    """Convert flattened 1D row-major index to 2D (col, row) coordinate."""
    col = index % width
    row = index // width
    return col, row


def bresenham(c0: int, r0: int, c1: int, r1: int) -> List[Tuple[int, int]]:
    """Compute all integer grid cells along the 2D segment from (c0, r0) to (c1, r1).

    Uses the standard integer-only Bresenham algorithm.

    Args:
        c0: Start column
        r0: Start row
        c1: End column
        r1: End row

    Returns:
        List of (col, row) tuples starting at (c0, r0) and ending at (c1, r1).

    """
    points: List[Tuple[int, int]] = []
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc - dr

    curr_c, curr_r = c0, r0
    while True:
        points.append((curr_c, curr_r))
        if curr_c == c1 and curr_r == r1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            curr_c += sc
        if e2 < dc:
            err += dc
            curr_r += sr

    return points


def log_odds_to_probability(log_odds_val: float) -> float:
    """Convert log-odds value into probability p in [0.0, 1.0].

    Formula: p = 1.0 - 1.0 / (1.0 + exp(log_odds_val))
    """
    if log_odds_val > 40.0:
        return 1.0
    if log_odds_val < -40.0:
        return 0.0
    return float(1.0 - 1.0 / (1.0 + math.exp(log_odds_val)))


def probability_to_log_odds(prob: float) -> float:
    """Convert probability p in (0.0, 1.0) into log-odds value."""
    p_clamped = min(max(prob, 1e-6), 1.0 - 1e-6)
    return float(math.log(p_clamped / (1.0 - p_clamped)))


def log_odds_to_occupancy_grid(
    log_odds: np.ndarray,
    observed_mask: np.ndarray
) -> np.ndarray:
    """Convert a 2D log-odds array and observed mask into standard OccupancyGrid int8 data.

    Values:
        -1: unobserved cell
        0..100: probability of occupancy (rounded to int)

    Args:
        log_odds: 2D numpy float array of accumulated log odds
        observed_mask: 2D numpy bool array indicating if cell has been observed

    Returns:
        2D numpy int8 array with values in [-1, 100].

    """
    probs = 1.0 / (1.0 + np.exp(-log_odds))
    data = np.full(log_odds.shape, -1, dtype=np.int8)

    obs = observed_mask
    if np.any(obs):
        scaled = np.clip(np.round(probs[obs] * 100.0), 0, 100).astype(np.int8)
        data[obs] = scaled

    return data
