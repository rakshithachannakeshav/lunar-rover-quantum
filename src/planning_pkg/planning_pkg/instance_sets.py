#!/usr/bin/env python3
"""
instance_sets.py
----------------
Instance families for the multi-site problems, including ones built to make
simple classical heuristics struggle.

Why: if nearest neighbour + 2-opt is already optimal, a quantum method has no
headroom to show anything. These tools produce (a) clustered sites, which trap
greedy tours, and (b) strongly asymmetric legs (uphill costs more than
downhill), which break the symmetric-tour intuition behind 2-opt.

Families
  terrain_random      sites sampled uniformly on the heightmap cost map
  terrain_clustered   sites sampled in a few tight clusters on the same map
  synthetic_clustered fast synthetic costs (clusters + relief + uphill term),
                      no heightmap needed; used by tests and the stress search

`stress_search` keeps only instances where nearest neighbour + 2-opt is worse
than a threshold. A set chosen that way is a STRESS SET, not a typical case:
always report it next to the unfiltered family and report how many instances had
to be tried (`n_tried` vs `n_kept`).

Also here: `multistart_two_opt` (2-opt from the NN tour plus random tours), the
fair classical answer to "2-opt gets stuck in local optima".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from planning_pkg.tsp_baselines import (
    TourResult,
    held_karp,
    n_sites,
    nearest_neighbour,
    nn_two_opt,
    tour_cost,
    two_opt,
)


# --------------------------------------------------------------------------
# Synthetic costs (fast; no terrain needed)
# --------------------------------------------------------------------------

def _relief(x: np.ndarray, y: np.ndarray, relief: float) -> np.ndarray:
    return relief * (np.sin(x / 17.0) + np.cos(y / 13.0) + 0.5 * np.sin((x + y) / 9.0))


def synthetic_cost_matrix(n: int, seed: int, clusters: int = 3, asym: float = 3.0,
                          relief: float = 6.0, w_slope: float = 1.0,
                          area: float = 100.0, cluster_sigma: float = 5.0) -> np.ndarray:
    """(n+1)x(n+1) leg costs; index 0 = depot.

    Points are scattered in `clusters` tight blobs (clusters <= 1: uniform).
    cost[a, b] = distance + w_slope * |dz| + asym * max(0, z_b - z_a), with z a
    smooth synthetic relief, so climbing is dearer than descending."""
    rng = np.random.default_rng(seed)
    pts = np.empty((n + 1, 2))
    pts[0] = rng.uniform(0.25 * area, 0.75 * area, size=2)
    if clusters <= 1:
        pts[1:] = rng.uniform(0.0, area, size=(n, 2))
    else:
        centres = rng.uniform(0.1 * area, 0.9 * area, size=(clusters, 2))
        for i in range(n):
            pts[i + 1] = centres[i % clusters] + rng.normal(0.0, cluster_sigma, size=2)
    z = _relief(pts[:, 0], pts[:, 1], relief)
    d = np.hypot(pts[:, None, 0] - pts[None, :, 0], pts[:, None, 1] - pts[None, :, 1])
    dz = z[None, :] - z[:, None]
    cost = d + w_slope * np.abs(dz) + asym * np.maximum(dz, 0.0)
    np.fill_diagonal(cost, 0.0)
    return cost


# --------------------------------------------------------------------------
# Clustered site sampling on the terrain graph
# --------------------------------------------------------------------------

def sample_sites_clustered(graph, n_sites: int, seed: int,
                           depot_xy: Tuple[float, float] = (0.0, 0.0),
                           n_clusters: int = 3, cluster_radius_m: float = 15.0,
                           min_separation_m: float = 6.0, max_radius_m: float = 60.0,
                           min_centre_separation_m: float = 25.0
                           ) -> List[Tuple[float, float]]:
    """Sites in `n_clusters` tight groups, assigned round-robin; every site is
    reachable from and back to the depot and >= min_separation_m from the others."""
    import networkx as nx
    from planning_pkg.classical_planning import resolve_node
    from planning_pkg.mission_planner import _node_xy

    depot = resolve_node(graph, *depot_xy)
    if graph.is_directed():
        reach = sorted((nx.descendants(graph, depot) & nx.ancestors(graph, depot)) | {depot})
    else:
        reach = sorted(nx.node_connected_component(graph, depot))
    dx, dy = _node_xy(graph, depot)
    xy = {v: _node_xy(graph, v) for v in reach}
    rng = np.random.default_rng(seed)
    near = [v for v in reach if math.hypot(xy[v][0] - dx, xy[v][1] - dy) <= max_radius_m]

    centres: List[Tuple[float, float]] = []
    for idx in rng.permutation(len(near)):
        cx, cy = xy[near[idx]]
        if all(math.hypot(cx - ox, cy - oy) >= min_centre_separation_m for ox, oy in centres):
            centres.append((cx, cy))
            if len(centres) == n_clusters:
                break
    if not centres:
        raise ValueError("No cluster centre could be placed.")

    chosen: List[Tuple[float, float]] = []
    for i in range(n_sites):
        cx, cy = centres[i % len(centres)]
        pool = [v for v in near if math.hypot(xy[v][0] - cx, xy[v][1] - cy) <= cluster_radius_m]
        placed = False
        for idx in rng.permutation(len(pool)):
            x, y = xy[pool[idx]]
            if all(math.hypot(x - px, y - py) >= min_separation_m
                   for px, py in chosen + [(dx, dy)]):
                chosen.append((x, y))
                placed = True
                break
        if not placed:
            raise ValueError(f"Could not place site {i + 1} of {n_sites} in its cluster.")
    return chosen


def terrain_cost_matrix(graph, cost_map, n: int, seed: int, family: str = "terrain_random",
                        max_radius_m: float = 60.0, min_separation_m: float = 10.0
                        ) -> np.ndarray:
    """A* leg-cost matrix for a sampled instance on the heightmap cost map."""
    from planning_pkg.mission_planner import build_instance, sample_sites

    if family == "terrain_random":
        sites = sample_sites(graph, n, seed, min_separation_m=min_separation_m,
                             max_radius_m=max_radius_m)
    elif family == "terrain_clustered":
        sites = sample_sites_clustered(graph, n, seed, max_radius_m=max_radius_m)
    else:
        raise ValueError(f"Unknown terrain family {family!r}.")
    return build_instance(graph, cost_map, (0.0, 0.0), sites).leg_costs


# --------------------------------------------------------------------------
# Classical: 2-opt with restarts, gap statistics, stress search
# --------------------------------------------------------------------------

def multistart_two_opt(cost: np.ndarray, restarts: int = 20, closed: bool = True,
                       seed: int = 0) -> TourResult:
    """Best of 2-opt from the NN tour and from `restarts` random tours.
    info['evals'] approximates neighbour evaluations: (moves + 1) * n(n-1)/2
    per 2-opt run (each move is priced in O(1) with prefix sums)."""
    n = n_sites(cost)
    rng = np.random.default_rng(seed)
    per_pass = n * (n - 1) / 2.0
    best = nn_two_opt(cost, closed)
    evals = (best.info.get("moves", 0) + 1) * per_pass
    for _ in range(restarts):
        start = (rng.permutation(n) + 1).tolist()
        r = two_opt(cost, start, closed)
        evals += (r.info.get("moves", 0) + 1) * per_pass
        if r.cost < best.cost - 1e-12:
            best = r
    return TourResult("multistart_two_opt", best.order, best.cost, 0.0,
                      {"restarts": restarts, "evals": evals})


@dataclass
class GapRecord:
    seed: int
    optimal: float
    nn: float
    nn_2opt: float
    multistart: float

    @property
    def nn_2opt_gap_pct(self) -> float:
        return 100.0 * (self.nn_2opt - self.optimal) / self.optimal

    @property
    def multistart_gap_pct(self) -> float:
        return 100.0 * (self.multistart - self.optimal) / self.optimal

    @property
    def nn_gap_pct(self) -> float:
        return 100.0 * (self.nn - self.optimal) / self.optimal


def gap_record(cost: np.ndarray, seed: int, closed: bool = True,
               restarts: int = 20) -> GapRecord:
    opt = held_karp(cost, closed).cost
    nn = nearest_neighbour(cost, closed).cost
    t2 = nn_two_opt(cost, closed).cost
    ms = multistart_two_opt(cost, restarts, closed, seed).cost
    return GapRecord(seed, opt, nn, t2, ms)


def stress_search(make_cost: Callable[[int], np.ndarray], seeds: Sequence[int],
                  gap_threshold_pct: float = 3.0, closed: bool = True,
                  restarts: int = 20) -> Dict:
    """Evaluate every seed; keep those where NN + 2-opt exceeds the threshold.

    Returns {'records': [GapRecord...], 'kept': [seeds], 'n_tried', 'n_kept'}.
    Report n_kept / n_tried: the kept set is selected for hardness."""
    records: List[GapRecord] = []
    for s in seeds:
        try:
            cost = make_cost(s)
        except ValueError:
            continue
        records.append(gap_record(cost, s, closed, restarts))
    kept = [r.seed for r in records if r.nn_2opt_gap_pct >= gap_threshold_pct]
    return {"records": records, "kept": kept, "n_tried": len(records), "n_kept": len(kept)}
