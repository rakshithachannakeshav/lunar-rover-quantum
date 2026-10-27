#!/usr/bin/env python3
"""Slope/shadow cost map, A* legs and the leg-cost matrix."""

import math
import os
import sys

import networkx as nx
import numpy as np
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.cost_map import (
    CostMapConfig,
    CostWeights,
    astar_leg,
    build_cost_map,
    cast_shadow,
    compute_leg_matrix,
    load_heightmap_elevation,
    synthetic_elevation,
)

HEIGHTMAP = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "src", "rover_simulation", "worlds", "heightmap.png"))


def flat_map(size=10, weights=None):
    config = CostMapConfig(weights=weights or CostWeights())
    return build_cost_map(np.zeros((size, size)), 1.0, 0.0, 0.0, config)


def test_flat_terrain_costs_pure_distance():
    cm = flat_map(weights=CostWeights(distance=1.0, slope=5.0, shadow=5.0))
    assert not cm.shadow.any()
    assert np.allclose(cm.slope_deg, 0.0)
    G = cm.to_graph()
    cost, path = astar_leg(G, cm, (0, 0), (3, 7))
    # Octile distance: 3 diagonal + 4 straight moves.
    assert cost == pytest.approx(3 * math.sqrt(2) + 4)
    assert path[0] == (0, 0) and path[-1] == (3, 7)


def test_ridge_casts_shadow_away_from_the_sun():
    z = np.zeros((5, 40))
    z[:, 20] = 2.0
    # Sun towards +x (east), 10 deg up: shadow reaches ~2/tan(10deg) = 11 cells west.
    shadow = cast_shadow(z, 1.0, sun_elevation_deg=10.0, sun_azimuth_deg=90.0)
    assert shadow[:, 12:20].all()
    assert not shadow[:, 21:].any()
    assert not shadow[:, :5].any()
    assert cast_shadow(z, 1.0, 90.0, 0.0).sum() == 0
    assert cast_shadow(z, 1.0, 0.0, 0.0).all()


def test_shadow_weight_only_penalises_shadowed_cells():
    z = synthetic_elevation(40, seed=1)
    base = build_cost_map(z, 1.0, 0.0, 0.0, CostMapConfig(weights=CostWeights(1, 1, 0)))
    dark = build_cost_map(z, 1.0, 0.0, 0.0, CostMapConfig(weights=CostWeights(1, 1, 3)))
    assert base.shadow.any() and not base.shadow.all()
    diff = dark.cell_cost() - base.cell_cost()
    lit = (base.shadow == 0) & base.traversable
    assert np.allclose(diff[lit], 0.0)
    assert np.allclose(diff[(base.shadow == 1) & base.traversable], 3.0)


def test_steep_cells_are_not_traversable():
    z = np.zeros((10, 10))
    z[:, 5:] = np.arange(5)[None, :] * 3.0  # ~71 deg ramp
    cm = build_cost_map(z, 1.0, 0.0, 0.0, CostMapConfig(max_slope_deg=25.0))
    G = cm.to_graph()
    assert not cm.traversable[:, 7].any()
    assert (0, 7) not in G
    assert G.number_of_nodes() == int(cm.traversable.sum())


def test_cell_world_roundtrip():
    cm = build_cost_map(np.zeros((8, 12)), 0.5, -3.0, 2.0)
    for cell in [(0, 0), (7, 11), (3, 5)]:
        assert cm.world_to_cell(*cm.cell_to_world(cell)) == cell
    assert cm.world_to_cell(-100.0, 100.0) == (7, 0)  # clamps


@pytest.mark.parametrize("weights", [CostWeights(1, 2, 1.5), CostWeights(0, 1, 1), CostWeights(3, 0, 0)])
def test_astar_matches_dijkstra(weights):
    """A* with the w_dist-scaled heuristic must stay optimal for any weights."""
    cm = build_cost_map(synthetic_elevation(40, seed=3), 1.0, 0.0, 0.0,
                        CostMapConfig(weights=weights))
    G = cm.to_graph()
    rng = np.random.default_rng(0)
    nodes = list(G.nodes)
    for _ in range(8):
        a, b = (nodes[i] for i in rng.choice(len(nodes), 2, replace=False))
        if not nx.has_path(G, a, b):
            continue
        cost, path = astar_leg(G, cm, a, b)
        assert cost == pytest.approx(nx.dijkstra_path_length(G, a, b, weight="weight"))
        assert nx.is_path(G, path)


def test_leg_matrix_paths_are_consistent():
    cm = build_cost_map(synthetic_elevation(30, seed=4), 1.0, 0.0, 0.0)
    G = cm.to_graph()
    comp = sorted(max(nx.strongly_connected_components(G), key=len))
    nodes = [comp[0], comp[len(comp) // 3], comp[2 * len(comp) // 3], comp[-1]]
    costs, paths = compute_leg_matrix(G, cm, nodes)
    assert np.allclose(np.diag(costs), 0.0)
    assert not np.allclose(costs, costs.T)  # uphill weight makes legs asymmetric
    for (i, j), path in paths.items():
        assert path[0] == nodes[i] and path[-1] == nodes[j]
        assert nx.path_weight(G, path, "weight") == pytest.approx(costs[i, j])


def test_unreachable_leg_raises():
    z = np.zeros((10, 10))
    z[:, 4:6] = np.array([0.0, 30.0])  # impassable wall
    cm = build_cost_map(z, 1.0, 0.0, 0.0)
    G = cm.to_graph()
    with pytest.raises(ValueError):
        astar_leg(G, cm, (0, 0), (0, 9))


def test_invalid_config_raises():
    with pytest.raises(ValueError):
        build_cost_map(np.zeros((4, 4)), 1.0, 0, 0, CostMapConfig(weights=CostWeights(-1, 1, 1)))
    with pytest.raises(ValueError):
        build_cost_map(np.zeros((4, 4)), 1.0, 0, 0, CostMapConfig(weights=CostWeights(0, 0, 0)))
    with pytest.raises(ValueError):
        build_cost_map(np.zeros((4, 4)), 0.0, 0, 0)


@pytest.mark.skipif(not os.path.exists(HEIGHTMAP), reason="simulation heightmap not present")
def test_simulation_heightmap_loads_at_coarse_resolution():
    z, ox, oy = load_heightmap_elevation(HEIGHTMAP, resolution_m=5.0)
    assert z.shape == (30, 30)
    assert ox == pytest.approx(-75.0) and oy == pytest.approx(-75.0)
    assert -5.0 <= z.min() <= z.max() <= 5.0


def test_uphill_costs_more_than_downhill():
    z = np.tile(np.arange(10, dtype=float) * 0.2, (5, 1))  # gentle ramp up in +x
    cm = build_cost_map(z, 1.0, 0.0, 0.0, CostMapConfig(weights=CostWeights(uphill=4.0)))
    G = cm.to_graph()
    up = G[(2, 3)][(2, 4)]["weight"]
    down = G[(2, 4)][(2, 3)]["weight"]
    assert up - down == pytest.approx(4.0 * 0.2)
    up_leg, _ = astar_leg(G, cm, (2, 0), (2, 9))
    down_leg, _ = astar_leg(G, cm, (2, 9), (2, 0))
    assert up_leg > down_leg


def test_zero_uphill_weight_is_symmetric():
    cm = build_cost_map(synthetic_elevation(20, seed=5), 1.0, 0.0, 0.0,
                        CostMapConfig(weights=CostWeights(uphill=0.0)))
    G = cm.to_graph()
    for u, v, w in list(G.edges(data="weight"))[:200]:
        assert G[v][u]["weight"] == pytest.approx(w)


@pytest.mark.parametrize("uphill", [0.0, 1.0, 10.0])
def test_astar_with_climb_heuristic_matches_dijkstra(uphill):
    cm = build_cost_map(synthetic_elevation(40, seed=6), 1.0, 0.0, 0.0,
                        CostMapConfig(weights=CostWeights(1.0, 1.0, 1.0, uphill)))
    G = cm.to_graph()
    rng = np.random.default_rng(1)
    nodes = list(G.nodes)
    for _ in range(8):
        a, b = (nodes[i] for i in rng.choice(len(nodes), 2, replace=False))
        if not nx.has_path(G, a, b):
            continue
        cost, _ = astar_leg(G, cm, a, b)
        assert cost == pytest.approx(nx.dijkstra_path_length(G, a, b, weight="weight"))
