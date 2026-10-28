#!/usr/bin/env python3
"""
mission_planner.py
------------------
Hybrid multi-site sample-collection planner (pure Python - no rclpy):

  1. cost map (distance + slope + shadow)          -> cost_map.CostMap / graph
  2. A* between every pair of {depot, sites}        -> leg-cost matrix + leg paths
  3. visiting order                                 -> QAOA (or a classical solver)
  4. stitch the chosen legs into one waypoint path  -> handed to navigation_pkg

Solvers for step 3:
  'qaoa'     full N^2-qubit QAOA when it fits in qaoa.max_qubits, otherwise
             QAOA windows over a nearest-neighbour seed tour (mode='window')
  'exact'    Held-Karp optimum
  'nn_2opt'  nearest neighbour + 2-opt

CLI:  python -m planning_pkg.mission_planner --sites 10,5 -20,30 ... [--solver qaoa]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import networkx as nx
import numpy as np

from planning_pkg.classical_planning import resolve_node
from planning_pkg.cost_map import (
    CostMap,
    CostMapConfig,
    CostWeights,
    build_cost_map,
    compute_leg_matrix,
    load_heightmap_elevation,
)
from planning_pkg.tsp_baselines import (
    TourResult,
    held_karp,
    nearest_neighbour,
    nn_two_opt,
    validate_tour,
)
from planning_pkg.tsp_qaoa import QaoaConfig, solve_tour_qaoa, solve_tour_qaoa_windowed

SOLVERS = ("qaoa", "exact", "nn_2opt")
MIN_SITES, MAX_SITES = 1, 20  # Held-Karp ('exact') needs O(2^N N) memory


@dataclass
class MissionInstance:
    """Depot + sites snapped to graph nodes, with all pairwise A* legs."""

    nodes: List[Any]
    coords: List[Tuple[float, float]]
    leg_costs: np.ndarray
    leg_paths: Dict[Tuple[int, int], List[Any]]
    astar_runtime_s: float

    @property
    def n_sites(self) -> int:
        return len(self.nodes) - 1


@dataclass
class MissionPlan:
    solver: str
    closed: bool
    order: List[int]
    site_visit_coords: List[Tuple[float, float]]
    tour_cost: float
    leg_costs: List[float]
    path_nodes: List[Any]
    path_coords: List[Tuple[float, float]]
    astar_runtime_s: float
    order_runtime_s: float
    solver_info: Dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["path_nodes"] = [list(n) if isinstance(n, tuple) else n for n in self.path_nodes]
        d["path_coords"] = [list(c) for c in self.path_coords]
        d["site_visit_coords"] = [list(c) for c in self.site_visit_coords]
        return d


def _node_xy(graph: nx.DiGraph, node: Any) -> Tuple[float, float]:
    data = graph.nodes[node]
    return float(data["pos_x"]), float(data["pos_y"])


def build_instance(graph: nx.DiGraph, cost_map: CostMap,
                   depot_xy: Tuple[float, float],
                   sites_xy: Sequence[Tuple[float, float]]) -> MissionInstance:
    if not MIN_SITES <= len(sites_xy) <= MAX_SITES:
        raise ValueError(f"Number of sites must be in [{MIN_SITES}, {MAX_SITES}].")
    nodes = [resolve_node(graph, x, y) for x, y in [tuple(depot_xy)] + list(sites_xy)]
    if len(set(nodes)) != len(nodes):
        raise ValueError("Two mission points snap to the same graph node; space them apart.")
    started = time.perf_counter()
    costs, paths = compute_leg_matrix(graph, cost_map, nodes)
    return MissionInstance(
        nodes=nodes,
        coords=[_node_xy(graph, n) for n in nodes],
        leg_costs=costs,
        leg_paths=paths,
        astar_runtime_s=time.perf_counter() - started,
    )


def sample_sites(graph: nx.DiGraph, n_sites: int, seed: int,
                 depot_xy: Tuple[float, float] = (0.0, 0.0),
                 min_separation_m: float = 10.0,
                 max_radius_m: Optional[float] = None) -> List[Tuple[float, float]]:
    """Random science sites the rover can reach from the depot and return from,
    pairwise >= min_separation_m apart (and from the depot), optionally within
    max_radius_m of it."""
    depot = resolve_node(graph, *depot_xy)
    if graph.is_directed():
        reachable = sorted((nx.descendants(graph, depot) & nx.ancestors(graph, depot))
                           | {depot})
    else:
        reachable = sorted(nx.node_connected_component(graph, depot))
    dx, dy = _node_xy(graph, depot)
    rng = np.random.default_rng(seed)
    chosen: List[Tuple[float, float]] = []
    for idx in rng.permutation(len(reachable)):
        x, y = _node_xy(graph, reachable[idx])
        if max_radius_m is not None and math.hypot(x - dx, y - dy) > max_radius_m:
            continue
        if all(math.hypot(x - px, y - py) >= min_separation_m
               for px, py in chosen + [(dx, dy)]):
            chosen.append((x, y))
            if len(chosen) == n_sites:
                return chosen
    raise ValueError(
        f"Could only place {len(chosen)} of {n_sites} sites with "
        f"min_separation_m={min_separation_m}."
    )


def solve_order(cost: np.ndarray, solver: str = "qaoa", closed: bool = True,
                qaoa: Optional[QaoaConfig] = None, window: int = 3) -> TourResult:
    if solver == "exact":
        return held_karp(cost, closed)
    if solver == "nn_2opt":
        return nn_two_opt(cost, closed)
    if solver != "qaoa":
        raise ValueError(f"solver must be one of {SOLVERS}; got {solver!r}.")
    qaoa = qaoa or QaoaConfig()
    n = cost.shape[0] - 1
    if n * n <= qaoa.max_qubits:
        result = solve_tour_qaoa(cost, closed, qaoa)
        result.info["mode"] = "full"
    else:
        seed = nearest_neighbour(cost, closed)
        result = solve_tour_qaoa_windowed(cost, seed.order, closed, qaoa, window=window)
        result.info["mode"] = "window"
    return result


def stitch_path(instance: MissionInstance, order: Sequence[int],
                closed: bool = True) -> List[Any]:
    """Concatenate the A* legs depot -> order... (-> depot) into one node path."""
    validate_tour(order, instance.n_sites)
    stops = [0] + list(order) + ([0] if closed else [])
    path: List[Any] = [instance.nodes[0]]
    for a, b in zip(stops[:-1], stops[1:]):
        path.extend(instance.leg_paths[(a, b)][1:])
    return path


def plan_mission(graph: nx.DiGraph, cost_map: CostMap,
                 depot_xy: Tuple[float, float],
                 sites_xy: Sequence[Tuple[float, float]],
                 solver: str = "qaoa", closed: bool = True,
                 qaoa: Optional[QaoaConfig] = None, window: int = 3) -> MissionPlan:
    instance = build_instance(graph, cost_map, depot_xy, sites_xy)
    result = solve_order(instance.leg_costs, solver, closed, qaoa, window)
    validate_tour(result.order, instance.n_sites)
    stops = [0] + result.order + ([0] if closed else [])
    path = stitch_path(instance, result.order, closed)
    info = {k: v for k, v in result.info.items() if k != "tour_counts"}
    return MissionPlan(
        solver=result.method,
        closed=closed,
        order=list(result.order),
        site_visit_coords=[instance.coords[s] for s in result.order],
        tour_cost=float(result.cost),
        leg_costs=[float(instance.leg_costs[a, b]) for a, b in zip(stops[:-1], stops[1:])],
        path_nodes=path,
        path_coords=[_node_xy(graph, n) for n in path],
        astar_runtime_s=instance.astar_runtime_s,
        order_runtime_s=result.runtime_s,
        solver_info=info,
    )


def save_mission_plan(plan: MissionPlan, out_dir: str = "results/missions") -> Dict[str, str]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    data = plan.to_dict()
    latest = out / "latest_mission.json"
    history = out / f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    for target in (latest, history):
        with target.open("w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
    return {"latest": str(latest), "history": str(history)}


def load_cost_map(heightmap: str, resolution_m: float = 1.0,
                  config: Optional[CostMapConfig] = None) -> Tuple[CostMap, nx.DiGraph]:
    config = config or CostMapConfig(resolution_m=resolution_m)
    elevation, ox, oy = load_heightmap_elevation(heightmap, config.resolution_m)
    cost_map = build_cost_map(elevation, config.resolution_m, ox, oy, config)
    return cost_map, cost_map.to_graph()


def _parse_xy(text: str) -> Tuple[float, float]:
    x, y = text.split(",")
    return float(x), float(y)


_XY = re.compile(r"^-[\d.]+,-?[\d.]+$")


def _protect_negative_xy(argv: Sequence[str]) -> List[str]:
    """argparse reads a pair like "-30,40" as an unknown option; a leading
    space keeps it positional (float() ignores the space)."""
    return [" " + a if _XY.match(a) else a for a in argv]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Plan a multi-site sample-collection mission.")
    parser.add_argument("--heightmap", default="src/rover_simulation/worlds/heightmap.png")
    parser.add_argument("--depot", type=_parse_xy, default=(0.0, 0.0), help="x,y")
    parser.add_argument("--sites", type=_parse_xy, nargs="+", required=True, help="x,y ...")
    parser.add_argument("--solver", choices=SOLVERS, default="qaoa")
    parser.add_argument("--open", action="store_true", help="Do not return to the depot.")
    parser.add_argument("--resolution", type=float, default=1.0)
    parser.add_argument("--w-dist", type=float, default=1.0)
    parser.add_argument("--w-slope", type=float, default=2.0)
    parser.add_argument("--w-shadow", type=float, default=1.5)
    parser.add_argument("--w-uphill", type=float, default=2.0,
                        help="Extra cost per metre of elevation gained.")
    parser.add_argument("--reps", type=int, default=1)
    parser.add_argument("--optimizer", default="COBYLA")
    parser.add_argument("--shots", type=int, default=4096)
    parser.add_argument("--max-qubits", type=int, default=20)
    parser.add_argument("--window", type=int, default=3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--out", default="results/missions")
    args = parser.parse_args(_protect_negative_xy(sys.argv[1:] if argv is None else argv))

    config = CostMapConfig(resolution_m=args.resolution,
                           weights=CostWeights(args.w_dist, args.w_slope, args.w_shadow,
                                               args.w_uphill))
    qaoa = QaoaConfig(reps=args.reps, optimizer=args.optimizer, shots=args.shots,
                      max_qubits=args.max_qubits, seed=args.seed)
    try:
        cost_map, graph = load_cost_map(args.heightmap, config=config)
        plan = plan_mission(graph, cost_map, args.depot, args.sites, args.solver,
                            closed=not args.open, qaoa=qaoa, window=args.window)
    except Exception as exc:
        print(f"[mission_planner] ERROR: {exc}")
        return 1
    saved = save_mission_plan(plan, args.out)
    print(f"[mission_planner] solver={plan.solver} order={plan.order} "
          f"cost={plan.tour_cost:.2f} waypoints={len(plan.path_coords)} "
          f"A*={plan.astar_runtime_s:.2f}s order={plan.order_runtime_s:.2f}s")
    print(f"[mission_planner] Saved: {saved['latest']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
