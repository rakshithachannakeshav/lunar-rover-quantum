#!/usr/bin/env python3
"""
tests/test_classical_planning.py
--------------------------------
Pure-Python unit tests for planning_pkg.classical_planning (Phase 7: classical
Dijkstra and A* path planning). Executed by pytest in CI (no ROS2 / Gazebo dependency).
"""

import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Tuple

import networkx as nx
import pytest

# Ensure src/planning_pkg is on python path for pure Python tests
PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'planning_pkg'))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.classical_planning import (
    PlanResult,
    compare,
    load_weighted_graph,
    main as cli_main,
    resolve_node,
    run_astar,
    run_dijkstra,
    save_plan_results,
)


@pytest.fixture
def synthetic_energy_graph() -> nx.Graph:
    """
    Construct a synthetic 4-node graph to test energy-weighted path planning.

    Topology and geometry:
        D (1.0, 1.0)
       / \
      /   \
     A --- B --- C
   (0,0) (1,0) (2,0)

    Edges and weights:
      - (A, B): weight = 1.5, d = 1.0
      - (B, C): weight = 2.0, d = 1.0
          Path A -> B -> C: total energy = 1.5 + 2.0 = 3.5, total distance = 2.0m.
      - (A, D): weight = 1.0, d = sqrt(2) ≈ 1.414m
      - (D, C): weight = 1.0, d = sqrt(2) ≈ 1.414m
          Path A -> D -> C: total energy = 1.0 + 1.0 = 2.0, total distance ≈ 2.828m.
      - (A, C): weight = 10.0 (high-cost direct edge)

    Ground Truth for start='A', goal='C':
      Energy-optimal path is uniquely: ['A', 'D', 'C']
      Optimal energy = 2.0
      Optimal distance = 2 * sqrt(2) ≈ 2.828427 m
      Number of waypoints = 3
    """
    G = nx.Graph()
    G.add_node('A', pos_x=0.0, pos_y=0.0, terrain_class=10, terrain_factor=1.0)
    G.add_node('B', pos_x=1.0, pos_y=0.0, terrain_class=50, terrain_factor=1.5)
    G.add_node('C', pos_x=2.0, pos_y=0.0, terrain_class=10, terrain_factor=1.0)
    G.add_node('D', pos_x=1.0, pos_y=1.0, terrain_class=10, terrain_factor=1.0)

    G.add_edge('A', 'B', weight=1.5)
    G.add_edge('B', 'C', weight=2.0)
    G.add_edge('A', 'D', weight=1.0)
    G.add_edge('D', 'C', weight=1.0)
    G.add_edge('A', 'C', weight=10.0)

    return G


@pytest.fixture
def disconnected_graph() -> nx.Graph:
    """Construct a graph with two disconnected components."""
    G = nx.Graph()
    G.add_node('N1', pos_x=0.0, pos_y=0.0)
    G.add_node('N2', pos_x=1.0, pos_y=0.0)
    G.add_edge('N1', 'N2', weight=1.0)

    G.add_node('ISLAND', pos_x=10.0, pos_y=10.0)
    return G


# ── 1. Coordinate Resolution Tests ─────────────────────────────────────────────

def test_resolve_node_snaps_to_nearest(synthetic_energy_graph):
    """Test snapping arbitrary world coordinates to the nearest graph node."""
    # (0.1, -0.05) is closest to 'A' (0, 0)
    assert resolve_node(synthetic_energy_graph, 0.1, -0.05) == 'A'
    # (0.9, 0.8) is closest to 'D' (1, 1)
    assert resolve_node(synthetic_energy_graph, 0.9, 0.8) == 'D'
    # (1.9, 0.05) is closest to 'C' (2, 0)
    assert resolve_node(synthetic_energy_graph, 1.9, 0.05) == 'C'


def test_resolve_node_empty_graph_raises():
    """Test that resolving on an empty graph raises ValueError."""
    empty_g = nx.Graph()
    with pytest.raises(ValueError, match="empty graph"):
        resolve_node(empty_g, 0.0, 0.0)


def test_resolve_node_out_of_bounds_raises(synthetic_energy_graph):
    """Test that coordinates absurdly far from the graph raise ValueError."""
    with pytest.raises(ValueError, match="too far"):
        resolve_node(synthetic_energy_graph, 5000.0, 5000.0)


# ── 2. Dijkstra Planning Tests ────────────────────────────────────────────────

def test_dijkstra_finds_known_optimal_path(synthetic_energy_graph):
    """Verify Dijkstra finds the exact hand-calculated optimal path and metrics."""
    result = run_dijkstra(synthetic_energy_graph, 'A', 'C')

    assert result.algorithm == "Dijkstra"
    assert result.path_nodes == ['A', 'D', 'C']
    assert math.isclose(result.total_energy, 2.0, abs_tol=1e-6)
    assert math.isclose(result.total_distance_m, 2 * math.sqrt(2), abs_tol=1e-6)
    assert result.num_waypoints == 3
    assert result.path_coords == [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
    assert result.runtime_ms >= 0.0
    assert result.timestamp is not None


def test_dijkstra_missing_node_raises(synthetic_energy_graph):
    """Verify informative ValueError when start or goal does not exist."""
    with pytest.raises(ValueError, match="Start node NON_EXISTENT not found"):
        run_dijkstra(synthetic_energy_graph, 'NON_EXISTENT', 'C')

    with pytest.raises(ValueError, match="Goal node NON_EXISTENT not found"):
        run_dijkstra(synthetic_energy_graph, 'A', 'NON_EXISTENT')


# ── 3. A* Planning Tests ──────────────────────────────────────────────────────

def test_astar_finds_optimal_path(synthetic_energy_graph):
    """Verify A* finds the optimal energy matching Dijkstra."""
    result = run_astar(synthetic_energy_graph, 'A', 'C')

    assert result.algorithm == "A*"
    assert result.path_nodes == ['A', 'D', 'C']
    assert math.isclose(result.total_energy, 2.0, abs_tol=1e-6)
    assert math.isclose(result.total_distance_m, 2 * math.sqrt(2), abs_tol=1e-6)
    assert result.num_waypoints == 3
    assert result.path_coords == [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
    assert result.runtime_ms >= 0.0


def test_astar_and_dijkstra_energy_equality(synthetic_energy_graph):
    """Verify both exact algorithms arrive at the same total energy."""
    dijkstra_res = run_dijkstra(synthetic_energy_graph, 'A', 'C')
    astar_res = run_astar(synthetic_energy_graph, 'A', 'C')

    assert math.isclose(dijkstra_res.total_energy, astar_res.total_energy, abs_tol=1e-9)


# ── 4. Disconnected Graph Handling ────────────────────────────────────────────

def test_disconnected_graph_raises_informative_error(disconnected_graph):
    """Verify that disconnected components raise descriptive ValueError without leaking NetworkXNoPath."""
    with pytest.raises(ValueError, match="no path found between start N1 and goal ISLAND"):
        run_dijkstra(disconnected_graph, 'N1', 'ISLAND')

    with pytest.raises(ValueError, match="no path found between start N1 and goal ISLAND"):
        run_astar(disconnected_graph, 'N1', 'ISLAND')


# ── 5. Comparison Summary Tests ───────────────────────────────────────────────

def test_compare_identical_paths(synthetic_energy_graph):
    """Test comparison output when both algorithms find the same path."""
    d_res = run_dijkstra(synthetic_energy_graph, 'A', 'C')
    a_res = run_astar(synthetic_energy_graph, 'A', 'C')

    comp = compare(d_res, a_res)
    assert comp['paths_identical'] is True
    assert math.isclose(comp['energy_delta'], 0.0, abs_tol=1e-9)
    assert math.isclose(comp['distance_delta_m'], 0.0, abs_tol=1e-9)
    assert 'speedup_factor' in comp
    assert comp['dijkstra']['waypoints'] == 3
    assert comp['astar']['waypoints'] == 3


def test_compare_differing_paths():
    """Test comparison summary when paths differ."""
    d_res = PlanResult(
        algorithm="Dijkstra", path_nodes=['A', 'B'], path_coords=[(0.0, 0.0), (1.0, 0.0)],
        total_energy=2.0, total_distance_m=1.0, num_waypoints=2, runtime_ms=1.5, timestamp="2026-09-13T00:00:00"
    )
    a_res = PlanResult(
        algorithm="A*", path_nodes=['A', 'D', 'B'], path_coords=[(0.0, 0.0), (0.5, 0.5), (1.0, 0.0)],
        total_energy=2.0, total_distance_m=1.414, num_waypoints=3, runtime_ms=0.8, timestamp="2026-09-13T00:00:00"
    )

    comp = compare(d_res, a_res)
    assert comp['paths_identical'] is False
    assert math.isclose(comp['energy_delta'], 0.0, abs_tol=1e-9)
    assert math.isclose(comp['distance_delta_m'], 0.414, abs_tol=1e-3)
    assert math.isclose(comp['runtime_delta_ms'], -0.7, abs_tol=1e-3)


# ── 6. Persistence & Serialization Tests ──────────────────────────────────────

def test_save_plan_results(tmp_path, synthetic_energy_graph):
    """Verify that save_plan_results writes latest_classical.json and a timestamped file."""
    d_res = run_dijkstra(synthetic_energy_graph, 'A', 'C')
    a_res = run_astar(synthetic_energy_graph, 'A', 'C')
    comp = compare(d_res, a_res)

    saved = save_plan_results(d_res, a_res, comp, out_dir=str(tmp_path))

    assert os.path.exists(saved['latest'])
    assert os.path.exists(saved['history'])

    with open(saved['latest'], 'r', encoding='utf-8') as f:
        data = json.load(f)

    assert "dijkstra" in data
    assert "astar" in data
    assert "comparison" in data
    assert data["dijkstra"]["path_nodes"] == ['A', 'D', 'C']
    assert data["astar"]["total_energy"] == 2.0
    assert data["comparison"]["paths_identical"] is True


def test_load_weighted_graph_round_trip(tmp_path, synthetic_energy_graph):
    """Test loading a graphml file via load_weighted_graph."""
    graphml_path = tmp_path / "test_graph.graphml"
    nx.write_graphml(synthetic_energy_graph, str(graphml_path))

    loaded_g = load_weighted_graph(str(graphml_path))
    assert loaded_g.number_of_nodes() == synthetic_energy_graph.number_of_nodes()
    assert loaded_g.number_of_edges() == synthetic_energy_graph.number_of_edges()

    # Verify planning still works on loaded graph
    d_res = run_dijkstra(loaded_g, 'A', 'C')
    assert math.isclose(d_res.total_energy, 2.0, abs_tol=1e-6)


def test_load_weighted_graph_non_existent_raises():
    """Verify FileNotFoundError if graph path does not exist."""
    with pytest.raises(FileNotFoundError, match="not found"):
        load_weighted_graph("non_existent_path.graphml")


# ── 7. CLI End-to-End Test ────────────────────────────────────────────────────

def test_cli_main_success(tmp_path, synthetic_energy_graph):
    """Test CLI main() invocation produces valid outputs."""
    graphml_path = tmp_path / "graph.graphml"
    nx.write_graphml(synthetic_energy_graph, str(graphml_path))

    out_dir = tmp_path / "paths"

    rc = cli_main([
        '--start-x', '0.0',
        '--start-y', '0.0',
        '--goal-x', '2.0',
        '--goal-y', '0.0',
        '--graph', str(graphml_path),
        '--out', str(out_dir),
    ])

    assert rc == 0
    assert (out_dir / "latest_classical.json").exists()
