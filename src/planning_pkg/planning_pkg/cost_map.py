#!/usr/bin/env python3
"""
cost_map.py
-----------
Multi-criteria traversal cost map for multi-site mission planning.

A grid of elevation samples is turned into three per-cell layers:

  * slope    - local terrain slope (deg) from the elevation gradient,
  * shadow   - 1.0 where the cell is in cast shadow for a low sun (the lunar
               south-pole sun sits only a few degrees above the horizon), else 0.0,
  * traversable - slope <= max_slope_deg.

Cost per metre of travel through a cell is

    c(cell) = w_dist + w_slope * tan(slope)/tan(max_slope) + w_shadow * shadow

and a directed 8-connected edge a -> b between two traversable cells costs

    d * (c_a + c_b) / 2 + w_uphill * max(0, z_b - z_a)

so climbing costs more than descending and leg costs are asymmetric. Every
term except w_dist is >= 0, and any route must climb at least the net gain,
so h(u) = w_dist * ||u - goal|| + w_uphill * max(0, z_goal - z_u) is an
admissible, consistent A* heuristic for any non-negative weights.

The graph produced by `CostMap.to_graph` uses the same node attributes as the
Phase-6 graph (pos_x/pos_y, 'weight' on edges), so `classical_planning.resolve_node`
and the ROS path publishers work on it unchanged. Pure Python - no rclpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import networkx as nx
import numpy as np

from planning_pkg.graph_model import (
    HEIGHTMAP_WORLD_SIZE,
    HEIGHTMAP_Z_OFFSET,
    HEIGHTMAP_Z_SCALE,
    _load_heightmap_array,
)

Cell = Tuple[int, int]


@dataclass
class CostWeights:
    """Relative weights of the cost terms (all must be >= 0). `uphill` is the
    extra cost per metre of elevation gained along a directed edge."""

    distance: float = 1.0
    slope: float = 2.0
    shadow: float = 1.5
    uphill: float = 2.0

    def validate(self) -> None:
        for name in ("distance", "slope", "shadow", "uphill"):
            if getattr(self, name) < 0:
                raise ValueError(f"Cost weight '{name}' must be >= 0.")
        if self.distance == 0 and self.slope == 0 and self.shadow == 0:
            raise ValueError("At least one cost weight must be positive.")


@dataclass
class CostMapConfig:
    """Cost-map construction parameters."""

    resolution_m: float = 1.0
    max_slope_deg: float = 25.0
    sun_elevation_deg: float = 3.0
    # Direction *towards* the sun, compass-style: 0 = +y, 90 = +x.
    sun_azimuth_deg: float = 135.0
    weights: CostWeights = field(default_factory=CostWeights)


@dataclass
class CostMap:
    """Per-cell layers on a regular grid. Row index grows with world y."""

    elevation: np.ndarray
    slope_deg: np.ndarray
    shadow: np.ndarray
    traversable: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    config: CostMapConfig

    @property
    def shape(self) -> Tuple[int, int]:
        return self.elevation.shape

    @property
    def min_cost_per_metre(self) -> float:
        """Lower bound on per-metre cost; scales the admissible A* heuristic."""
        return self.config.weights.distance

    def cell_cost(self) -> np.ndarray:
        """Per-metre traversal cost of every cell (inf where not traversable)."""
        w = self.config.weights
        slope_term = np.tan(np.radians(self.slope_deg)) / math.tan(
            math.radians(self.config.max_slope_deg)
        )
        cost = w.distance + w.slope * slope_term + w.shadow * self.shadow
        return np.where(self.traversable, cost, np.inf)

    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        r, c = cell
        return (
            self.origin_x + (c + 0.5) * self.resolution,
            self.origin_y + (r + 0.5) * self.resolution,
        )

    def world_to_cell(self, x: float, y: float) -> Cell:
        h, w = self.shape
        c = int(math.floor((x - self.origin_x) / self.resolution))
        r = int(math.floor((y - self.origin_y) / self.resolution))
        return min(max(r, 0), h - 1), min(max(c, 0), w - 1)

    def to_graph(self) -> nx.DiGraph:
        """8-connected directed graph over traversable cells; every neighbour
        pair gets both directions, priced with the uphill term per direction."""
        cost = self.cell_cost()
        z = self.elevation
        w_up = self.config.weights.uphill
        h, w = self.shape
        G = nx.DiGraph()
        for r, c in zip(*np.nonzero(self.traversable)):
            x, y = self.cell_to_world((int(r), int(c)))
            G.add_node(
                (int(r), int(c)),
                pos_x=x,
                pos_y=y,
                elevation=float(self.elevation[r, c]),
                slope_deg=float(self.slope_deg[r, c]),
                shadow=float(self.shadow[r, c]),
            )

        # Half the neighbourhood covers every pair; each pair adds a->b and b->a.
        for dr, dc in ((0, 1), (1, -1), (1, 0), (1, 1)):
            d = self.resolution * math.hypot(dr, dc)
            r0 = np.arange(max(0, -dr), min(h, h - dr))
            c0 = np.arange(max(0, -dc), min(w, w - dc))
            rr, cc = np.meshgrid(r0, c0, indexing="ij")
            a = cost[rr, cc]
            b = cost[rr + dr, cc + dc]
            ok = np.isfinite(a) & np.isfinite(b)
            base = d * 0.5 * (a[ok] + b[ok])
            rise = z[rr + dr, cc + dc][ok] - z[rr, cc][ok]  # gain going a -> b
            fwd = base + w_up * np.maximum(rise, 0.0)
            bwd = base + w_up * np.maximum(-rise, 0.0)
            for r, c, wf, wb in zip(rr[ok], cc[ok], fwd, bwd):
                u, v = (int(r), int(c)), (int(r + dr), int(c + dc))
                G.add_edge(u, v, weight=float(wf))
                G.add_edge(v, u, weight=float(wb))
        return G


def slope_degrees(elevation: np.ndarray, resolution: float) -> np.ndarray:
    gy, gx = np.gradient(elevation, resolution)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def cast_shadow(
    elevation: np.ndarray,
    resolution: float,
    sun_elevation_deg: float,
    sun_azimuth_deg: float,
) -> np.ndarray:
    """Binary cast-shadow mask by marching each cell's ray towards the sun.

    A cell is shadowed if any terrain sample along the ray rises above the
    ray's height. Samples outside the grid never occlude.
    """
    if sun_elevation_deg >= 90.0:
        return np.zeros_like(elevation, dtype=np.float64)
    if sun_elevation_deg <= 0.0:
        return np.ones_like(elevation, dtype=np.float64)

    h, w = elevation.shape
    az = math.radians(sun_azimuth_deg)
    dx, dy = math.sin(az), math.cos(az)  # towards the sun, in cell units
    rise_per_cell = resolution * math.tan(math.radians(sun_elevation_deg))

    relief = float(elevation.max() - elevation.min())
    max_steps = int(min(math.hypot(h, w), math.ceil(relief / rise_per_cell) + 1))

    rows, cols = np.indices((h, w))
    shadow = np.zeros((h, w), dtype=bool)
    for k in range(1, max_steps + 1):
        rr = np.rint(rows + k * dy).astype(int)
        cc = np.rint(cols + k * dx).astype(int)
        inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        if not inside.any():
            break
        ray_z = elevation + k * rise_per_cell
        terrain_z = np.full((h, w), -np.inf)
        terrain_z[inside] = elevation[rr[inside], cc[inside]]
        shadow |= terrain_z > ray_z
    return shadow.astype(np.float64)


def build_cost_map(
    elevation: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    config: Optional[CostMapConfig] = None,
) -> CostMap:
    config = config or CostMapConfig()
    config.weights.validate()
    if resolution <= 0:
        raise ValueError("resolution must be positive.")
    if not 0.0 < config.max_slope_deg < 90.0:
        raise ValueError("max_slope_deg must be in (0, 90).")

    elevation = np.asarray(elevation, dtype=np.float64)
    slope = slope_degrees(elevation, resolution)
    shadow = cast_shadow(
        elevation, resolution, config.sun_elevation_deg, config.sun_azimuth_deg
    )
    return CostMap(
        elevation=elevation,
        slope_deg=slope,
        shadow=shadow,
        traversable=slope <= config.max_slope_deg,
        resolution=float(resolution),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        config=config,
    )


def load_heightmap_elevation(
    heightmap_png_path: str, resolution_m: float = 1.0
) -> Tuple[np.ndarray, float, float]:
    """Read worlds/heightmap.png and block-average it to resolution_m.

    Uses the same pixel->metre and world-extent convention as
    graph_model.sample_elevation (row 0 is world y = -HEIGHTMAP_WORLD_SIZE/2).
    Returns (elevation, origin_x, origin_y).
    """
    arr = _load_heightmap_array(heightmap_png_path)
    z = (arr / 65535.0) * HEIGHTMAP_Z_SCALE + HEIGHTMAP_Z_OFFSET
    n_cells = max(2, int(round(HEIGHTMAP_WORLD_SIZE / resolution_m)))
    rows = np.linspace(0, z.shape[0], n_cells + 1).astype(int)
    cols = np.linspace(0, z.shape[1], n_cells + 1).astype(int)
    out = np.empty((n_cells, n_cells))
    for i in range(n_cells):
        for j in range(n_cells):
            out[i, j] = z[rows[i]:rows[i + 1], cols[j]:cols[j + 1]].mean()
    half = HEIGHTMAP_WORLD_SIZE / 2.0
    return out, -half, -half


def synthetic_elevation(size: int = 60, seed: int = 0, n_craters: int = 8) -> np.ndarray:
    """Cratered test terrain in metres on a size x size grid (1 cell = 1 unit)."""
    rng = np.random.default_rng(seed)
    yy, xx = np.indices((size, size), dtype=np.float64)
    z = 0.4 * np.sin(xx / 7.0) * np.cos(yy / 9.0)
    for _ in range(n_craters):
        cx, cy = rng.uniform(0, size, 2)
        radius = rng.uniform(size / 20, size / 7)
        depth = rng.uniform(0.5, 2.0) * radius / 4
        d = np.hypot(xx - cx, yy - cy) / radius
        bowl = -depth * np.clip(1 - d ** 2, 0, None)
        rim = 0.35 * depth * np.exp(-((d - 1.0) ** 2) / 0.05)
        z += bowl + rim
    return z


def astar_leg(
    graph: nx.DiGraph, cost_map: CostMap, start: Any, goal: Any
) -> Tuple[float, List[Any]]:
    """Optimal directed leg (cost, node path) start -> goal using A* with

        h(u) = w_dist * ||u - goal|| + w_uphill * max(0, z_goal - z_u)

    which never overestimates (see module docstring). Raises ValueError if
    unreachable."""
    if start == goal:
        return 0.0, [start]
    goal_node = graph.nodes[goal]
    gx, gy = float(goal_node["pos_x"]), float(goal_node["pos_y"])
    gz = float(goal_node.get("elevation", 0.0))
    k = cost_map.min_cost_per_metre
    w_up = cost_map.config.weights.uphill

    def heuristic(u: Any, _goal: Any) -> float:
        node = graph.nodes[u]
        climb = max(0.0, gz - float(node.get("elevation", gz)))
        return k * math.hypot(gx - node["pos_x"], gy - node["pos_y"]) + w_up * climb

    try:
        path = nx.astar_path(graph, start, goal, heuristic=heuristic, weight="weight")
    except nx.NetworkXNoPath as exc:
        raise ValueError(f"No traversable path from {start} to {goal}.") from exc
    return float(nx.path_weight(graph, path, weight="weight")), path


def compute_leg_matrix(
    graph: nx.DiGraph, cost_map: CostMap, nodes: Sequence[Any]
) -> Tuple[np.ndarray, Dict[Tuple[int, int], List[Any]]]:
    """A* legs between every ordered pair of `nodes`. Costs are direction-
    dependent (uphill vs downhill), so (i, j) and (j, i) are searched
    separately and costs[i, j] need not equal costs[j, i]."""
    k = len(nodes)
    costs = np.zeros((k, k))
    paths: Dict[Tuple[int, int], List[Any]] = {}
    for i in range(k):
        for j in range(k):
            if i == j:
                paths[(i, i)] = [nodes[i]]
                continue
            costs[i, j], paths[(i, j)] = astar_leg(graph, cost_map, nodes[i], nodes[j])
    return costs, paths
