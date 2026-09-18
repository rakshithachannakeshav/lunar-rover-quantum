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
metrics.py
----------
Phase 10: compare the Phase 7 classical planners (Dijkstra, A*), the Phase 8
quantum planner (QAOA) and a distance-only baseline on the same energy graph.
Pure Python - no rclpy - runs standalone via:

    python -m evaluation_pkg.metrics --classical results/paths/latest_classical.json \
        --quantum results/paths/latest_quantum.json --graph results/graphs/latest.graphml

Honest framing: QAOA is expected to *match* the classical optimum, not beat it.
Any "energy saving" is reported against the distance-only baseline - the
shortest path by Euclidean length, re-priced with the real energy weights.
"""

import argparse
import json
import math
import os
import random
import statistics
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence

import networkx as nx

from .energy_model import DistanceTracker

CLASSICAL_KEYS = ('dijkstra', 'astar')
REFERENCE_PLANNER = 'astar'
QUANTUM_EXTRA_KEYS = (
    'qubits', 'shots', 'reps', 'reduced_edges', 'reduced_nodes',
    'valid_samples', 'sampled_candidates', 'fallback_to_classical', 'penalty',
)


@dataclass
class PlannerMetrics:
    """Metrics of one planned path."""

    name: str
    algorithm: str
    energy: float
    distance_m: float
    waypoints: int
    runtime_ms: float
    straight_line_m: float
    path_efficiency: Optional[float]
    energy_per_m: Optional[float]
    path_nodes: List[str]
    path_coords: List[List[float]]
    extras: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _normalise_node(raw: Any) -> str:
    """Return a node id in the form GraphML uses, e.g. [29, 33] -> '(29, 33)'."""
    if isinstance(raw, (list, tuple)):
        return str(tuple(raw))
    return str(raw)


def _straight_line(coords: Sequence[Sequence[float]]) -> float:
    if len(coords) < 2:
        return 0.0
    return math.hypot(coords[-1][0] - coords[0][0], coords[-1][1] - coords[0][1])


def _load_json(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Result file not found: {path}")
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def _metrics_from_plan(
    name: str, plan: Dict[str, Any], extra_keys: Sequence[str] = ()
) -> PlannerMetrics:
    energy = float(plan['total_energy'])
    distance = float(plan['total_distance_m'])
    coords = [[float(x), float(y)] for x, y in plan.get('path_coords', [])]
    straight = _straight_line(coords)
    return PlannerMetrics(
        name=name,
        algorithm=str(plan.get('algorithm', name)),
        energy=energy,
        distance_m=distance,
        waypoints=int(plan.get('num_waypoints', len(coords))),
        runtime_ms=float(plan.get('runtime_ms', 0.0)),
        straight_line_m=straight,
        path_efficiency=(straight / distance) if distance > 0.0 else None,
        energy_per_m=(energy / distance) if distance > 0.0 else None,
        path_nodes=[_normalise_node(n) for n in plan.get('path_nodes', [])],
        path_coords=coords,
        extras={k: plan[k] for k in extra_keys if k in plan},
    )


def load_planner_metrics(classical_json: str, quantum_json: str) -> Dict[str, PlannerMetrics]:
    """Load Dijkstra, A* and QAOA metrics from the Phase 7 / Phase 8 result files."""
    classical = _load_json(classical_json)
    quantum = _load_json(quantum_json)

    planners: Dict[str, PlannerMetrics] = {}
    for key in CLASSICAL_KEYS:
        if key not in classical:
            raise ValueError(f"'{key}' result missing from {classical_json}")
        planners[key] = _metrics_from_plan(key, classical[key])

    if 'quantum' not in quantum:
        raise ValueError(f"'quantum' result missing from {quantum_json}")
    planners['qaoa'] = _metrics_from_plan('qaoa', quantum['quantum'], QUANTUM_EXTRA_KEYS)
    return planners


def _node_xy(graph: nx.Graph, node: Any) -> List[float]:
    data = graph.nodes[node]
    return [float(data.get('pos_x', 0.0)), float(data.get('pos_y', 0.0))]


def distance_only_baseline(
    graph: nx.Graph, start: Any, goal: Any, name: str = 'distance_only'
) -> PlannerMetrics:
    """Shortest path by Euclidean length, priced with the real energy weights.

    This is what an energy-*unaware* planner would drive. Comparing its energy
    to the energy-optimal path shows what energy-aware planning saves.
    """
    def edge_length(u: Any, v: Any, _data: Dict[str, Any]) -> float:
        ux, uy = _node_xy(graph, u)
        vx, vy = _node_xy(graph, v)
        return math.hypot(vx - ux, vy - uy)

    started = time.perf_counter()
    try:
        path = nx.shortest_path(graph, start, goal, weight=edge_length)
    except (nx.NetworkXNoPath, nx.NodeNotFound) as exc:
        raise ValueError(f"No baseline path between {start!r} and {goal!r}: {exc}") from exc
    runtime_ms = (time.perf_counter() - started) * 1000.0

    coords = [_node_xy(graph, n) for n in path]
    distance = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(coords[:-1], coords[1:])
    )
    energy = _path_energy(graph, path)
    straight = _straight_line(coords)

    return PlannerMetrics(
        name=name,
        algorithm='Shortest distance (energy-unaware)',
        energy=energy,
        distance_m=distance,
        waypoints=len(path),
        runtime_ms=runtime_ms,
        straight_line_m=straight,
        path_efficiency=(straight / distance) if distance > 0.0 else None,
        energy_per_m=(energy / distance) if distance > 0.0 else None,
        path_nodes=[_normalise_node(n) for n in path],
        path_coords=coords,
    )


def _path_energy(graph: nx.Graph, path: Sequence[Any]) -> float:
    return sum(float(graph[u][v].get('weight', 0.0)) for u, v in zip(path[:-1], path[1:]))


def _sweep_summary(
    pairs: List[Dict[str, Any]], requested: int, seed: int, min_separation_m: float
) -> Dict[str, Any]:
    summary: Dict[str, Any] = {
        'n_pairs': len(pairs),
        'requested_pairs': requested,
        'seed': seed,
        'min_separation_m': min_separation_m,
        'mean_savings_pct': None,
        'median_savings_pct': None,
        'max_savings_pct': None,
        'differing_pairs': 0,
    }
    if pairs:
        savings = [p['savings_pct'] for p in pairs]
        summary.update({
            'mean_savings_pct': statistics.fmean(savings),
            'median_savings_pct': statistics.median(savings),
            'max_savings_pct': max(savings),
            'differing_pairs': sum(1 for x in savings if x > 1e-9),
        })
    return summary


def baseline_sweep(
    graph: nx.Graph, n_pairs: int = 200, seed: int = 7, min_separation_m: float = 3.0
) -> Dict[str, Any]:
    """Energy-optimal vs distance-only path over random start/goal pairs.

    One route can hide the effect (on flat terrain the two paths often coincide),
    so this samples many routes from the graph's largest connected component and
    reports the distribution of the energy saved. Deterministic for a given seed.
    """
    def edge_length(u: Any, v: Any, _data: Dict[str, Any]) -> float:
        ux, uy = _node_xy(graph, u)
        vx, vy = _node_xy(graph, v)
        return math.hypot(vx - ux, vy - uy)

    nodes: List[Any] = []
    if graph.number_of_nodes() >= 2:
        nodes = sorted(max(nx.connected_components(graph), key=len), key=str)

    pairs: List[Dict[str, Any]] = []
    if len(nodes) >= 2:
        rng = random.Random(seed)
        attempts = 0
        while len(pairs) < n_pairs and attempts < max(50, n_pairs * 20):
            attempts += 1
            start, goal = rng.sample(nodes, 2)
            sx, sy = _node_xy(graph, start)
            gx, gy = _node_xy(graph, goal)
            separation = math.hypot(gx - sx, gy - sy)
            if separation < min_separation_m:
                continue

            optimal = nx.dijkstra_path(graph, start, goal, weight='weight')
            shortest = nx.shortest_path(graph, start, goal, weight=edge_length)
            e_optimal = _path_energy(graph, optimal)
            e_distance = _path_energy(graph, shortest)
            pairs.append({
                'start': _normalise_node(start),
                'goal': _normalise_node(goal),
                'separation_m': separation,
                'energy_optimal': e_optimal,
                'energy_distance_only': e_distance,
                'savings_pct': ((e_distance - e_optimal) / e_distance * 100.0)
                if e_distance > 0.0 else 0.0,
            })

    return {'pairs': pairs, 'summary': _sweep_summary(pairs, n_pairs, seed, min_separation_m)}


def _summarise(planners: Dict[str, PlannerMetrics]) -> Dict[str, Any]:
    ref = planners[REFERENCE_PLANNER]
    qaoa = planners['qaoa']
    tol = 1e-6 * max(1.0, ref.energy)

    # Best energy among the three real planners; ties go to the fastest one.
    real = {k: planners[k] for k in ('dijkstra', 'astar', 'qaoa')}
    best_energy = min(p.energy for p in real.values())
    tied = {k: p for k, p in real.items() if p.energy <= best_energy + tol}
    best_planner = min(tied, key=lambda k: tied[k].runtime_ms)

    savings = None
    baseline = planners.get('distance_only')
    if baseline is not None and baseline.energy > 0.0:
        savings = round((baseline.energy - best_energy) / baseline.energy * 100.0, 6)

    return {
        'reference': REFERENCE_PLANNER,
        'best_energy_planner': best_planner,
        'best_energy': best_energy,
        'qaoa_matches_classical': abs(qaoa.energy - ref.energy) <= tol,
        'qaoa_energy_ratio': (qaoa.energy / ref.energy) if ref.energy > 0.0 else None,
        'qaoa_fallback': bool(qaoa.extras.get('fallback_to_classical', False)),
        'qaoa_runtime_ratio_vs_astar': (
            qaoa.runtime_ms / ref.runtime_ms if ref.runtime_ms > 0.0 else None
        ),
        'energy_savings_vs_distance_only_pct': savings,
    }


def build_comparison(
    classical_json: str,
    quantum_json: str,
    graph_path: Optional[str] = None,
    sweep_pairs: int = 0,
    sweep_seed: int = 7,
    sweep_min_separation_m: float = 3.0,
) -> Dict[str, Any]:
    """Build the full comparison document. The baseline and sweep need the graph file."""
    if sweep_pairs > 0 and graph_path is None:
        raise ValueError('The route sweep needs the energy graph (graph_path).')

    planners = load_planner_metrics(classical_json, quantum_json)
    ref = planners[REFERENCE_PLANNER]
    start = ref.path_nodes[0] if ref.path_nodes else None
    goal = ref.path_nodes[-1] if ref.path_nodes else None

    if graph_path is not None:
        if not os.path.exists(graph_path):
            raise FileNotFoundError(f"Graph file not found: {graph_path}")
        graph = nx.read_graphml(graph_path)
        if start not in graph or goal not in graph:
            raise ValueError(
                f"Start/goal node ({start}, {goal}) from the planner result is not in "
                f"{graph_path}; the graph was probably regenerated after planning."
            )
        planners['distance_only'] = distance_only_baseline(graph, start, goal)

    document = {
        'generated_at': datetime.now().isoformat(),
        'sources': {
            'classical': classical_json,
            'quantum': quantum_json,
            'graph': graph_path,
        },
        'start_node': start,
        'goal_node': goal,
        'planners': {name: p.to_dict() for name, p in planners.items()},
        'summary': _summarise(planners),
    }
    if sweep_pairs > 0:
        document['sweep'] = baseline_sweep(
            graph, n_pairs=sweep_pairs, seed=sweep_seed, min_separation_m=sweep_min_separation_m
        )
    return document


def save_comparison(comparison: Dict[str, Any], out_dir: str) -> Dict[str, str]:
    """Write latest_comparison.json plus a timestamped history copy."""
    os.makedirs(out_dir, exist_ok=True)
    latest = os.path.join(out_dir, 'latest_comparison.json')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    history = os.path.join(out_dir, f'comparison_{stamp}.json')
    for path in (latest, history):
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(comparison, f, indent=2)
    return {'latest': latest, 'history': history}


def execution_report(planned: PlannerMetrics, battery: Dict[str, Any]) -> Dict[str, Any]:
    """Compare a planned path with what the rover actually spent driving it.

    `execution_efficiency` (= planned distance / actual distance) is only
    meaningful once the rover has reached the goal; mid-run it just reflects
    how far along the path the rover is.
    """
    actual_m = float(battery.get('distance_m', 0.0))
    consumed_j = float(battery.get('consumed_j', 0.0))
    return {
        'planner': planned.name,
        'planned_energy': planned.energy,
        'planned_distance_m': planned.distance_m,
        'actual_distance_m': actual_m,
        'execution_efficiency': (planned.distance_m / actual_m) if actual_m > 0.0 else None,
        'consumed_j': consumed_j,
        'consumed_wh': consumed_j / 3600.0,
        'joules_per_m': (consumed_j / actual_m) if actual_m > 0.0 else None,
        'battery_percentage': battery.get('percentage'),
    }


class ExecutionMonitor:
    """Tracks one drive of a planned path: distance, energy since start, goal reached.

    Pure Python so the ROS evaluator node stays a thin wrapper. Energy is counted
    from the first battery sample, so any time the rover sits idle before it starts
    driving (e.g. while waiting for a plan) is included.
    """

    def __init__(self, planned: PlannerMetrics, goal_tolerance: float = 0.35):
        if not planned.path_coords:
            raise ValueError(f"Planner '{planned.name}' has no path coordinates to evaluate.")
        self.planned = planned
        self.goal_tolerance = goal_tolerance
        self.goal = (float(planned.path_coords[-1][0]), float(planned.path_coords[-1][1]))
        self._tracker = DistanceTracker()
        self._baseline_j: Optional[float] = None
        self.consumed_j = 0.0
        self.percentage: Optional[float] = None
        self.distance_to_goal_m: Optional[float] = None
        self.goal_reached = False

    def update_position(self, x: float, y: float) -> bool:
        """Feed an odometry sample. Returns True the first time the goal is reached."""
        if self.goal_reached:  # freeze the tally once the rover has arrived
            return False
        self._tracker.update(x, y)
        self.distance_to_goal_m = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if self.distance_to_goal_m <= self.goal_tolerance:
            self.goal_reached = True
            return True
        return False

    def update_battery(self, total_consumed_j: float, percentage: float) -> None:
        """Feed a battery sample (cumulative energy drawn by the battery model)."""
        if self._baseline_j is None:
            self._baseline_j = total_consumed_j
        if not self.goal_reached:
            self.consumed_j = max(0.0, total_consumed_j - self._baseline_j)
            self.percentage = percentage

    def report(self) -> Dict[str, Any]:
        result = execution_report(self.planned, {
            'distance_m': self._tracker.total_m,
            'consumed_j': self.consumed_j,
            'percentage': self.percentage,
        })
        result['goal_reached'] = self.goal_reached
        result['distance_to_goal_m'] = self.distance_to_goal_m
        result['goal'] = list(self.goal)
        return result


def main(argv: Optional[List[str]] = None) -> int:
    """CLI: compare planners and write results/energy_comparison/latest_comparison.json."""
    parser = argparse.ArgumentParser(
        description='Compare Dijkstra, A*, QAOA and a distance-only baseline.'
    )
    parser.add_argument('--classical', default='results/paths/latest_classical.json')
    parser.add_argument('--quantum', default='results/paths/latest_quantum.json')
    parser.add_argument('--graph', default='results/graphs/latest.graphml',
                        help='Energy graph, needed for the distance-only baseline.')
    parser.add_argument('--no-baseline', action='store_true',
                        help='Skip the distance-only baseline (no graph needed).')
    parser.add_argument('--sweep', type=int, default=0, metavar='N',
                        help='Also compare energy-optimal vs distance-only paths over N '
                             'random start/goal pairs on the graph (0 = skip).')
    parser.add_argument('--sweep-seed', type=int, default=7)
    parser.add_argument('--sweep-min-separation', type=float, default=3.0,
                        help='Minimum start-goal separation in metres for sweep pairs.')
    parser.add_argument('--out', default='results/energy_comparison')
    args = parser.parse_args(argv)

    graph_path: Optional[str] = None if args.no_baseline else args.graph
    if graph_path is not None and not os.path.exists(graph_path):
        print(f"[evaluation] WARNING: graph '{graph_path}' not found - "
              "skipping the distance-only baseline.")
        graph_path = None

    try:
        comparison = build_comparison(
            args.classical, args.quantum, graph_path,
            sweep_pairs=args.sweep, sweep_seed=args.sweep_seed,
            sweep_min_separation_m=args.sweep_min_separation,
        )
    except (FileNotFoundError, ValueError) as exc:
        print(f"[evaluation] ERROR: {exc}")
        return 1

    saved = save_comparison(comparison, args.out)

    for name, p in comparison['planners'].items():
        print(f"[evaluation] {name:<13} energy={p['energy']:.4f}  "
              f"distance={p['distance_m']:.3f} m  waypoints={p['waypoints']}  "
              f"runtime={p['runtime_ms']:.2f} ms")
    summary = comparison['summary']
    print(f"[evaluation] QAOA matches classical optimum: {summary['qaoa_matches_classical']} "
          f"(energy ratio {summary['qaoa_energy_ratio']}, fallback={summary['qaoa_fallback']})")
    if summary['energy_savings_vs_distance_only_pct'] is not None:
        print(f"[evaluation] energy saved vs distance-only path: "
              f"{summary['energy_savings_vs_distance_only_pct']:.2f} %")
    if 'sweep' in comparison:
        sw = comparison['sweep']['summary']
        if sw['n_pairs']:
            print(f"[evaluation] route sweep ({sw['n_pairs']} pairs): mean saving "
                  f"{sw['mean_savings_pct']:.3f} %, max {sw['max_savings_pct']:.3f} %, "
                  f"{sw['differing_pairs']} routes differ from the distance-only path")
        else:
            print('[evaluation] route sweep: no valid start/goal pairs found')
    print(f"[evaluation] Saved: {saved['latest']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
