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
graph_model.py
--------------
Phase 6: turn a Phase-5 terrain-map snapshot (results/terrain_maps/latest.npz)
into an energy-weighted NetworkX graph, persisted to results/graphs/. Pure
Python - no rclpy - runs standalone via `python -m planning_pkg.graph_model`.
"""

import json
import math
import os
from dataclasses import dataclass
from datetime import datetime
from functools import lru_cache

import networkx as nx
import numpy as np
from PIL import Image

# Terrain factor table - must match the /terrain_map encoding
# (docs/PROGRESS.md / README.md "Interfaces / data contracts").
TERRAIN_FACTORS = {10: 1.0, 50: 1.5, 90: 3.0}
OBSTACLE_VALUE = 100
UNKNOWN_VALUE = -1
_TRAVERSABLE_CLASSES = (10, 50, 90)  # ascending cost order


@dataclass
class TerrainMap:
    """In-memory terrain snapshot loaded from a mapping_pkg latest.npz file."""

    grid: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    timestamp: str


def load_terrain_map(npz_path: str) -> TerrainMap:
    """Load a terrain snapshot written by mapping_pkg.terrain_storage.TerrainStorage."""
    with np.load(npz_path) as data:
        return TerrainMap(
            grid=np.array(data['grid']),
            resolution=float(data['resolution']),
            origin_x=float(data['origin_x']),
            origin_y=float(data['origin_y']),
            width=int(data['width']),
            height=int(data['height']),
            timestamp=str(data['timestamp']),
        )


def coarsen(grid: np.ndarray, cell_stride: int) -> np.ndarray:
    """Aggregate a raw terrain grid into cell_stride x cell_stride blocks.

    Rule per block: any obstacle cell -> block is obstacle (safety-first).
    Else majority unknown -> block is unknown. Else majority vote among the
    traversable classes, ties broken toward the higher-cost class.
    """
    height, width = grid.shape
    out_h = height // cell_stride
    out_w = width // cell_stride
    out = np.full((out_h, out_w), UNKNOWN_VALUE, dtype=np.int8)

    for br in range(out_h):
        for bc in range(out_w):
            block = grid[
                br * cell_stride:(br + 1) * cell_stride,
                bc * cell_stride:(bc + 1) * cell_stride,
            ]
            if np.any(block == OBSTACLE_VALUE):
                out[br, bc] = OBSTACLE_VALUE
                continue

            flat_block = block.flatten()
            n_total = flat_block.size
            n_unknown = int(np.sum(flat_block == UNKNOWN_VALUE))
            if n_unknown * 2 >= n_total:
                out[br, bc] = UNKNOWN_VALUE
                continue

            counts = {c: int(np.sum(flat_block == c)) for c in _TRAVERSABLE_CLASSES}
            best_count = max(counts.values())
            winners = [c for c, n in counts.items() if n == best_count]
            out[br, bc] = max(winners)

    return out


# worlds/heightmap.png convention (must match scripts/heightmap_to_collision_obj.py):
# a 16-bit pixel maps to world elevation via z = (pixel / 65535) * scale + offset,
# over a world square of HEIGHTMAP_WORLD_SIZE metres centred on the origin.
HEIGHTMAP_WORLD_SIZE = 150.0
HEIGHTMAP_Z_SCALE = 10.0
HEIGHTMAP_Z_OFFSET = -5.0


@lru_cache(maxsize=4)
def _load_heightmap_array(heightmap_png_path: str) -> np.ndarray:
    """Load + cache a heightmap PNG as a 2D float array (avoids re-opening the
    file for every graph edge - build_graph calls this once per node)."""
    img = Image.open(heightmap_png_path)
    arr = np.asarray(img)
    if arr.ndim == 3:
        arr = arr[:, :, 0]
    return arr.astype(np.float64)


def sample_elevation(heightmap_png_path: str, world_x: float, world_y: float) -> float:
    """Sample worlds/heightmap.png at a world (x, y) and return elevation (m).

    World (x, y) in [-HEIGHTMAP_WORLD_SIZE/2, HEIGHTMAP_WORLD_SIZE/2] maps to
    the full image; out-of-range coordinates clamp to the nearest edge pixel.
    """
    arr = _load_heightmap_array(heightmap_png_path)
    h, w = arr.shape
    half = HEIGHTMAP_WORLD_SIZE / 2.0

    col = round((world_x + half) / HEIGHTMAP_WORLD_SIZE * (w - 1))
    row = round((world_y + half) / HEIGHTMAP_WORLD_SIZE * (h - 1))
    col = min(max(col, 0), w - 1)
    row = min(max(row, 0), h - 1)

    pixel = arr[row, col]
    return float((pixel / 65535.0) * HEIGHTMAP_Z_SCALE + HEIGHTMAP_Z_OFFSET)


_NEIGHBOR_OFFSETS = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
]


def _block_center_world(block_row, block_col, cell_stride, resolution, origin_x, origin_y):
    """World (x, y) of a coarsened block's centre (generalises grid_to_world)."""
    x = origin_x + (block_col * cell_stride + cell_stride / 2.0) * resolution
    y = origin_y + (block_row * cell_stride + cell_stride / 2.0) * resolution
    return x, y


def build_graph(
    coarse_grid: np.ndarray,
    resolution: float,
    cell_stride: int,
    origin_x: float,
    origin_y: float,
    heightmap_png_path: str,
) -> nx.Graph:
    """Build an 8-connected, energy-weighted graph from a coarsened terrain grid.

    E(edge) = d * S(theta) * T_avg
      d       = Euclidean distance between block centres (m)
      S(theta)= 1.0 + 2.0 * |sin(theta)|, theta = atan2(|z_j - z_i|, d)
      T_avg   = mean of the two endpoint terrain factors
    """
    out_h, out_w = coarse_grid.shape
    G = nx.Graph()

    def is_traversable(r, c):
        return int(coarse_grid[r, c]) in TERRAIN_FACTORS

    for r in range(out_h):
        for c in range(out_w):
            if not is_traversable(r, c):
                continue
            x, y = _block_center_world(r, c, cell_stride, resolution, origin_x, origin_y)
            cls = int(coarse_grid[r, c])
            G.add_node((r, c), pos_x=x, pos_y=y, terrain_class=cls,
                       terrain_factor=TERRAIN_FACTORS[cls])

    for r in range(out_h):
        for c in range(out_w):
            if not is_traversable(r, c):
                continue
            for dr, dc in _NEIGHBOR_OFFSETS:
                nr, nc = r + dr, c + dc
                if nr < 0 or nr >= out_h or nc < 0 or nc >= out_w:
                    continue
                if not is_traversable(nr, nc):
                    continue
                if G.has_edge((r, c), (nr, nc)):
                    continue

                xi, yi = G.nodes[(r, c)]['pos_x'], G.nodes[(r, c)]['pos_y']
                xj, yj = G.nodes[(nr, nc)]['pos_x'], G.nodes[(nr, nc)]['pos_y']
                d = math.hypot(xj - xi, yj - yi)

                zi = sample_elevation(heightmap_png_path, xi, yi)
                zj = sample_elevation(heightmap_png_path, xj, yj)
                theta = math.atan2(abs(zj - zi), d) if d > 0 else 0.0
                slope_factor = 1.0 + 2.0 * abs(math.sin(theta))

                t_avg = (G.nodes[(r, c)]['terrain_factor']
                         + G.nodes[(nr, nc)]['terrain_factor']) / 2.0
                weight = d * slope_factor * t_avg
                G.add_edge((r, c), (nr, nc), weight=weight)

    return G


def save_graph(G: nx.Graph, out_dir: str, meta: dict) -> dict:
    """Persist a graph to <out_dir>/latest.graphml + latest_meta.json."""
    os.makedirs(out_dir, exist_ok=True)

    graphml_path = os.path.join(out_dir, 'latest.graphml')
    nx.write_graphml(G, graphml_path)

    weights = [d['weight'] for _, _, d in G.edges(data=True)]
    full_meta = dict(meta)
    full_meta.update({
        'node_count': G.number_of_nodes(),
        'edge_count': G.number_of_edges(),
        'weight_min': float(min(weights)) if weights else None,
        'weight_max': float(max(weights)) if weights else None,
        'weight_mean': float(sum(weights) / len(weights)) if weights else None,
        'generated_at': datetime.now().isoformat(),
    })
    meta_path = os.path.join(out_dir, 'latest_meta.json')
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(full_meta, f, indent=2)

    return {'graphml': graphml_path, 'meta_json': meta_path}


def load_graph(graphml_path: str) -> nx.Graph:
    """Load a graph previously written by save_graph."""
    return nx.read_graphml(graphml_path)


def main(argv=None) -> int:
    """CLI: build a graph from a terrain snapshot. `python -m planning_pkg.graph_model`."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Build an energy-weighted terrain graph from a Phase 5 terrain snapshot.'
    )
    parser.add_argument('--npz', required=True, help='Path to terrain latest.npz')
    parser.add_argument('--heightmap', required=True, help='Path to worlds/heightmap.png')
    parser.add_argument('--out', required=True, help='Output directory for latest.graphml/.json')
    parser.add_argument('--cell-stride', type=int, default=10,
                         help='Raw cells per graph block (default 10 -> 0.5 m spacing '
                              'at the current 0.05 m/cell resolution)')
    args = parser.parse_args(argv)

    tm = load_terrain_map(args.npz)
    coarse = coarsen(tm.grid, args.cell_stride)
    G = build_graph(coarse, tm.resolution, args.cell_stride, tm.origin_x, tm.origin_y,
                     args.heightmap)
    paths = save_graph(G, args.out, meta={
        'resolution': tm.resolution,
        'cell_stride': args.cell_stride,
        'source_timestamp': tm.timestamp,
    })

    print(f"Graph written: {paths['graphml']} "
          f"({G.number_of_nodes()} nodes, {G.number_of_edges()} edges)")
    print(f"Meta written: {paths['meta_json']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
