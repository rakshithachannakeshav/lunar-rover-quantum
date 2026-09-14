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

"""
classical_planning.py
---------------------
Phase 7: Classical path planning using Dijkstra and A* algorithms over the
Phase 6 energy-weighted terrain graph (results/graphs/latest.graphml).

Pure Python - zero rclpy imports - runs standalone via:
    python -m planning_pkg.classical_planning --start-x <x> --start-y <y> --goal-x <gx> --goal-y <gy>
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import networkx as nx

from planning_pkg.graph_model import load_graph


@dataclass
class PlanResult:
    """Stores the path planning output for an algorithm."""

    algorithm: str
    path_nodes: List[Any]
    path_coords: List[Tuple[float, float]]
    total_energy: float
    total_distance_m: float
    num_waypoints: int
    runtime_ms: float
    timestamp: str

    def to_dict(self) -> Dict[str, Any]:
        """Convert result to a JSON-serializable dictionary."""
        d = asdict(self)
        # Ensure path_nodes are serializable (e.g. tuples -> list/str)
        d['path_nodes'] = [list(n) if isinstance(n, tuple) else n for n in self.path_nodes]
        return d


def load_weighted_graph(path: Union[str, Path] = "results/graphs/latest.graphml") -> nx.Graph:
    """Load an energy-weighted NetworkX graph from a GraphML file.

    Reuses Phase 6's `planning_pkg.graph_model.load_graph`.
    """
    path_str = str(path)
    if not os.path.exists(path_str):
        raise FileNotFoundError(f"Weighted graph file not found at: {path_str}")
    return load_graph(path_str)


def resolve_node(graph: nx.Graph, x: float, y: float, max_dist_factor: float = 5.0) -> Any:
    """Snap world coordinates (x, y) to the nearest traversable graph node.

    Args:
        graph: NetworkX graph with 'pos_x' and 'pos_y' node attributes.
        x: Target world x coordinate in meters.
        y: Target world y coordinate in meters.
        max_dist_factor: Threshold factor relative to the bounding box diagonal.

    Returns:
        The node ID of the nearest node.

    Raises:
        ValueError: If graph has no nodes, or if (x, y) is abnormally far from the graph.
    """
    if graph.number_of_nodes() == 0:
        raise ValueError("Cannot resolve node on an empty graph.")

    best_node = None
    min_dist_sq = float('inf')

    xs: List[float] = []
    ys: List[float] = []

    for n, data in graph.nodes(data=True):
        nx_val = float(data.get('pos_x', 0.0))
        ny_val = float(data.get('pos_y', 0.0))
        xs.append(nx_val)
        ys.append(ny_val)

        dist_sq = (nx_val - x) ** 2 + (ny_val - y) ** 2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            best_node = n

    # Calculate graph bounding box diagonal to guard against extreme out-of-bounds coordinates
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)
    diag = math.hypot(x_span, y_span)

    # Minimum threshold distance (at least 20m if graph has only 1 node or 0 span)
    threshold = max(diag * max_dist_factor, 20.0)
    nearest_dist = math.sqrt(min_dist_sq)

    if nearest_dist > threshold:
        raise ValueError(
            f"Coordinate ({x:.2f}, {y:.2f}) is too far from any graph node "
            f"(nearest is at distance {nearest_dist:.2f} m > threshold {threshold:.2f} m)."
        )

    return best_node


def _compute_path_metrics(
    graph: nx.Graph, path: List[Any], algorithm: str, runtime_ms: float
) -> PlanResult:
    """Compute total energy, cumulative Euclidean distance, and coordinates for a node sequence."""
    total_energy = 0.0
    total_dist = 0.0
    path_coords: List[Tuple[float, float]] = []

    for i, u in enumerate(path):
        u_data = graph.nodes[u]
        ux = float(u_data.get('pos_x', 0.0))
        uy = float(u_data.get('pos_y', 0.0))
        path_coords.append((ux, uy))

        if i > 0:
            v = path[i - 1]
            edge_data = graph.get_edge_data(v, u, {})
            edge_weight = float(edge_data.get('weight', 0.0))
            total_energy += edge_weight

            vx = float(graph.nodes[v].get('pos_x', 0.0))
            vy = float(graph.nodes[v].get('pos_y', 0.0))
            total_dist += math.hypot(ux - vx, uy - vy)

    return PlanResult(
        algorithm=algorithm,
        path_nodes=path,
        path_coords=path_coords,
        total_energy=total_energy,
        total_distance_m=total_dist,
        num_waypoints=len(path),
        runtime_ms=runtime_ms,
        timestamp=datetime.now().isoformat(),
    )


def run_dijkstra(graph: nx.Graph, start: Any, goal: Any) -> PlanResult:
    """Compute energy-optimal path using Dijkstra's algorithm.

    Args:
        graph: NetworkX graph with 'weight' on edges and 'pos_x'/'pos_y' on nodes.
        start: Start node ID.
        goal: Goal node ID.

    Returns:
        PlanResult dataclass containing the node sequence and metrics.

    Raises:
        ValueError: If start or goal node does not exist or no path connects them.
    """
    if start not in graph:
        raise ValueError(f"Start node {start} not found in graph.")
    if goal not in graph:
        raise ValueError(f"Goal node {goal} not found in graph.")

    start_time = time.perf_counter()
    try:
        path = nx.dijkstra_path(graph, source=start, target=goal, weight='weight')
    except nx.NetworkXNoPath as e:
        raise ValueError(
            f"Dijkstra failed: no path found between start {start} and goal {goal} "
            f"(disconnected terrain or obstacle barrier)."
        ) from e

    runtime_ms = (time.perf_counter() - start_time) * 1000.0
    return _compute_path_metrics(graph, path, algorithm="Dijkstra", runtime_ms=runtime_ms)


def run_astar(graph: nx.Graph, start: Any, goal: Any, heuristic: str = "euclidean") -> PlanResult:
    """Compute energy-optimal path using A* search.

    Heuristic Admissibility Justification:
    The edge energy formula defined in README.md Section 6 is:
        E(edge) = d * S(theta) * T(type) * R(r)
    where:
        - d = Euclidean distance between node centers (m)
        - S(theta) = 1.0 + 2.0 * |sin(theta)| >= 1.0 (slope factor)
        - T(type) in {flat: 1.0, rocky: 1.5, crater: 3.0} >= 1.0 (terrain type factor)
        - R(r) >= 1.0 (roughness factor)
    Since every multiplier is >= 1.0, E(edge) >= d for all edges.
    Therefore, the straight-line Euclidean distance between node u and the goal
        h(u, goal) = ||pos(u) - pos(goal)||_2
    is a strictly admissible (never overestimates remaining energy cost) and
    consistent lower bound on the true shortest path energy.

    Args:
        graph: NetworkX graph with 'weight' on edges and 'pos_x'/'pos_y' on nodes.
        start: Start node ID.
        goal: Goal node ID.
        heuristic: Heuristic function name ("euclidean").

    Returns:
        PlanResult dataclass containing the node sequence and metrics.

    Raises:
        ValueError: If start or goal node does not exist or no path connects them.
    """
    if start not in graph:
        raise ValueError(f"Start node {start} not found in graph.")
    if goal not in graph:
        raise ValueError(f"Goal node {goal} not found in graph.")

    goal_x = float(graph.nodes[goal].get('pos_x', 0.0))
    goal_y = float(graph.nodes[goal].get('pos_y', 0.0))

    def euclidean_heuristic(u: Any, v: Any) -> float:
        # Distance from node u to target v (goal)
        ux = float(graph.nodes[u].get('pos_x', 0.0))
        uy = float(graph.nodes[u].get('pos_y', 0.0))
        return math.hypot(goal_x - ux, goal_y - uy)

    h_fn = euclidean_heuristic if heuristic == "euclidean" else None

    start_time = time.perf_counter()
    try:
        path = nx.astar_path(graph, source=start, target=goal, heuristic=h_fn, weight='weight')
    except nx.NetworkXNoPath as e:
        raise ValueError(
            f"A* failed: no path found between start {start} and goal {goal} "
            f"(disconnected terrain or obstacle barrier)."
        ) from e

    runtime_ms = (time.perf_counter() - start_time) * 1000.0
    return _compute_path_metrics(graph, path, algorithm="A*", runtime_ms=runtime_ms)


def compare(dijkstra_res: PlanResult, astar_res: PlanResult) -> Dict[str, Any]:
    """Produce comparison summary between Dijkstra and A* planning results.

    Schema is clean, self-contained JSON for Phase 10 evaluation.
    """
    return {
        'energy_delta': astar_res.total_energy - dijkstra_res.total_energy,
        'abs_energy_delta': abs(astar_res.total_energy - dijkstra_res.total_energy),
        'distance_delta_m': astar_res.total_distance_m - dijkstra_res.total_distance_m,
        'runtime_delta_ms': astar_res.runtime_ms - dijkstra_res.runtime_ms,
        'speedup_factor': (
            dijkstra_res.runtime_ms / astar_res.runtime_ms
            if astar_res.runtime_ms > 0 else 1.0
        ),
        'paths_identical': dijkstra_res.path_nodes == astar_res.path_nodes,
        'dijkstra': {
            'energy': dijkstra_res.total_energy,
            'distance_m': dijkstra_res.total_distance_m,
            'waypoints': dijkstra_res.num_waypoints,
            'runtime_ms': dijkstra_res.runtime_ms,
        },
        'astar': {
            'energy': astar_res.total_energy,
            'distance_m': astar_res.total_distance_m,
            'waypoints': astar_res.num_waypoints,
            'runtime_ms': astar_res.runtime_ms,
        },
        'compared_at': datetime.now().isoformat(),
    }


def save_plan_results(
    dijkstra_res: PlanResult,
    astar_res: PlanResult,
    comparison: Dict[str, Any],
    out_dir: Union[str, Path] = "results/paths",
) -> Dict[str, str]:
    """Persist planning outputs to results/paths/latest_classical.json and a timestamped file."""
    out_dir_path = Path(out_dir)
    out_dir_path.mkdir(parents=True, exist_ok=True)

    data = {
        'dijkstra': dijkstra_res.to_dict(),
        'astar': astar_res.to_dict(),
        'comparison': comparison,
        'saved_at': datetime.now().isoformat(),
    }

    latest_path = out_dir_path / "latest_classical.json"
    with open(latest_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    history_path = out_dir_path / f"classical_{timestamp_str}.json"
    with open(history_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)

    return {
        'latest': str(latest_path),
        'history': str(history_path),
    }


def main(argv: Optional[List[str]] = None) -> int:
    """CLI entry point: run classical planning offline without ROS."""
    parser = argparse.ArgumentParser(
        description="Run classical Dijkstra and A* path planners on an energy-weighted terrain graph."
    )
    parser.add_argument('--start-x', type=float, default=0.0, help='Start world X coordinate in meters')
    parser.add_argument('--start-y', type=float, default=0.0, help='Start world Y coordinate in meters')
    parser.add_argument('--goal-x', type=float, required=True, help='Goal world X coordinate in meters')
    parser.add_argument('--goal-y', type=float, required=True, help='Goal world Y coordinate in meters')
    parser.add_argument(
        '--graph',
        type=str,
        default='results/graphs/latest.graphml',
        help='Path to the energy-weighted GraphML file',
    )
    parser.add_argument(
        '--out',
        type=str,
        default='results/paths',
        help='Directory to save output JSON files',
    )

    args = parser.parse_args(argv)

    print(f"[classical_planning] Loading graph: {args.graph}")
    try:
        G = load_weighted_graph(args.graph)
    except Exception as e:
        print(f"[classical_planning] ERROR: Failed to load graph: {e}")
        return 1

    print(f"[classical_planning] Loaded graph with {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

    try:
        start_node = resolve_node(G, args.start_x, args.start_y)
        goal_node = resolve_node(G, args.goal_x, args.goal_y)
    except Exception as e:
        print(f"[classical_planning] ERROR resolving coordinates: {e}")
        return 1

    print(f"[classical_planning] Resolved start ({args.start_x}, {args.start_y}) -> node {start_node}")
    print(f"[classical_planning] Resolved goal ({args.goal_x}, {args.goal_y}) -> node {goal_node}")

    try:
        dijkstra_res = run_dijkstra(G, start_node, goal_node)
        astar_res = run_astar(G, start_node, goal_node)
    except Exception as e:
        print(f"[classical_planning] ERROR during planning: {e}")
        return 1

    comp = compare(dijkstra_res, astar_res)
    saved = save_plan_results(dijkstra_res, astar_res, comp, out_dir=args.out)

    print(f"[classical_planning] Dijkstra: {dijkstra_res.total_energy:.2f} energy / "
          f"{dijkstra_res.num_waypoints} waypoints / {dijkstra_res.runtime_ms:.2f}ms")
    print(f"[classical_planning] A*:       {astar_res.total_energy:.2f} energy / "
          f"{astar_res.num_waypoints} waypoints / {astar_res.runtime_ms:.2f}ms")
    print(f"[classical_planning] Comparison: identical={comp['paths_identical']}, "
          f"energy_delta={comp['energy_delta']:.4f}")
    print(f"[classical_planning] Saved results to: {saved['latest']}")

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
