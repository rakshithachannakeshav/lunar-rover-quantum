#!/usr/bin/env python3
"""Hybrid pipeline: sites -> A* legs -> visiting order -> stitched path."""

import os
import sys

import networkx as nx
import numpy as np
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "planning_pkg"))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.cost_map import build_cost_map, synthetic_elevation
from planning_pkg.mission_planner import (
    MAX_SITES,
    build_instance,
    plan_mission,
    sample_sites,
    save_mission_plan,
    solve_order,
    stitch_path,
)
from planning_pkg.tsp_baselines import held_karp, tour_cost, validate_tour
from planning_pkg.tsp_qaoa import QaoaConfig


@pytest.fixture(scope="module")
def terrain():
    cm = build_cost_map(synthetic_elevation(50, seed=2), 1.0, -25.0, -25.0)
    return cm, cm.to_graph()


def assert_path_is_connected(graph, path):
    for u, v in zip(path[:-1], path[1:]):
        assert graph.has_edge(u, v), f"{u} -> {v} is not an edge"


def test_sample_sites_are_separated_and_reachable(terrain):
    cm, G = terrain
    sites = sample_sites(G, 6, seed=0, min_separation_m=6.0)
    assert len(sites) == 6
    pts = np.array(sites + [(0.0, 0.0)])
    d = np.linalg.norm(pts[:, None] - pts[None], axis=-1) + np.eye(7) * 1e9
    assert d.min() >= 6.0 - 1.5  # depot is snapped to the nearest cell centre
    assert sample_sites(G, 6, seed=0, min_separation_m=6.0) == sites  # deterministic
    with pytest.raises(ValueError):
        sample_sites(G, 10, seed=0, min_separation_m=40.0)


@pytest.mark.parametrize("closed", [True, False])
@pytest.mark.parametrize("solver", ["exact", "nn_2opt"])
def test_classical_pipeline_visits_every_site_once(terrain, solver, closed):
    cm, G = terrain
    sites = sample_sites(G, 5, seed=1, min_separation_m=6.0)
    plan = plan_mission(G, cm, (0.0, 0.0), sites, solver=solver, closed=closed)
    validate_tour(plan.order, 5)
    assert_path_is_connected(G, plan.path_nodes)

    inst = build_instance(G, cm, (0.0, 0.0), sites)
    assert plan.path_nodes[0] == inst.nodes[0]
    if closed:
        assert plan.path_nodes[-1] == inst.nodes[0]
    # Each site node appears on the route, in the planned order.
    positions = [plan.path_nodes.index(inst.nodes[s]) for s in plan.order]
    assert positions == sorted(positions)
    assert sum(plan.leg_costs) == pytest.approx(plan.tour_cost)
    assert nx.path_weight(G, plan.path_nodes, "weight") == pytest.approx(plan.tour_cost)
    if solver == "exact":
        assert plan.tour_cost == pytest.approx(held_karp(inst.leg_costs, closed).cost)


def test_large_mission_exact_beats_heuristic(terrain):
    cm, G = terrain
    sites = sample_sites(G, 14, seed=6, min_separation_m=5.0)
    exact = plan_mission(G, cm, (0.0, 0.0), sites, solver="exact")
    heuristic = plan_mission(G, cm, (0.0, 0.0), sites, solver="nn_2opt")
    validate_tour(exact.order, 14)
    assert_path_is_connected(G, exact.path_nodes)
    assert exact.tour_cost <= heuristic.tour_cost + 1e-9


def test_stitch_rejects_invalid_order(terrain):
    cm, G = terrain
    inst = build_instance(G, cm, (0.0, 0.0), sample_sites(G, 3, seed=2, min_separation_m=6.0))
    with pytest.raises(ValueError):
        stitch_path(inst, [1, 1, 2])


def test_too_many_or_duplicate_sites_raise(terrain):
    cm, G = terrain
    with pytest.raises(ValueError):
        build_instance(G, cm, (0.0, 0.0), [(1.0, 1.0)] * (MAX_SITES + 1))
    with pytest.raises(ValueError):
        build_instance(G, cm, (0.0, 0.0), [(5.45, 5.45), (5.55, 5.55)])  # same cell
    with pytest.raises(ValueError):
        solve_order(np.zeros((3, 3)), solver="magic")


def test_save_mission_plan(terrain, tmp_path):
    cm, G = terrain
    plan = plan_mission(G, cm, (0.0, 0.0), sample_sites(G, 3, seed=3, min_separation_m=6.0),
                        solver="nn_2opt")
    saved = save_mission_plan(plan, str(tmp_path))
    assert os.path.exists(saved["latest"]) and os.path.exists(saved["history"])


def test_qaoa_pipeline_end_to_end(terrain):
    pytest.importorskip("qiskit_aer")
    cm, G = terrain
    sites = sample_sites(G, 3, seed=4, min_separation_m=6.0)
    cfg = QaoaConfig(reps=1, maxiter=40, restarts=0, shots=1024, grid_points=6)
    plan = plan_mission(G, cm, (0.0, 0.0), sites, solver="qaoa", qaoa=cfg)
    validate_tour(plan.order, 3)
    assert plan.solver_info["mode"] == "full"
    assert plan.solver_info["qubits"] == 9
    assert_path_is_connected(G, plan.path_nodes)


def test_qaoa_pipeline_falls_back_to_windows_above_qubit_limit(terrain):
    pytest.importorskip("qiskit_aer")
    cm, G = terrain
    sites = sample_sites(G, 5, seed=5, min_separation_m=6.0)
    cfg = QaoaConfig(reps=1, maxiter=30, restarts=0, shots=512, grid_points=5, max_qubits=9)
    inst = build_instance(G, cm, (0.0, 0.0), sites)
    result = solve_order(inst.leg_costs, "qaoa", True, cfg, window=3)
    validate_tour(result.order, 5)
    assert result.info["mode"] == "window"
    assert result.info["qubits"] == 9
    assert result.cost <= result.info["seed_cost"] + 1e-9
    assert result.cost == pytest.approx(tour_cost(inst.leg_costs, result.order))
