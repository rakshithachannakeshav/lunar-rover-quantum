#!/usr/bin/env python3
"""
tests/test_graph_model.py
--------------------------
Pure-Python unit tests for planning_pkg.graph_model (Phase 6: energy-weighted
terrain graph). Executed by pytest in CI (no ROS2 / Gazebo dependency).
"""

import json
import math
import os
import sys
import tempfile

import numpy as np
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'planning_pkg'))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from planning_pkg.graph_model import (  # noqa: E402
    coarsen,
    load_terrain_map,
)


def test_load_terrain_map_reads_npz_fields():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path = os.path.join(tmpdir, 'latest.npz')
        grid = np.full((6, 6), 10, dtype=np.int8)
        np.savez_compressed(
            npz_path, grid=grid, resolution=0.05, origin_x=-1.0, origin_y=-1.0,
            width=6, height=6, timestamp='2026-09-11T00:00:00'
        )
        tm = load_terrain_map(npz_path)
        assert tm.resolution == 0.05
        assert tm.origin_x == -1.0
        assert tm.origin_y == -1.0
        assert tm.width == 6
        assert tm.height == 6
        assert tm.timestamp == '2026-09-11T00:00:00'
        np.testing.assert_array_equal(tm.grid, grid)


def test_coarsen_majority_vote():
    grid = np.array([
        [10, 10, 50, 50],
        [10, 50, 50, 50],
        [10, 10, 10, 10],
        [10, 10, 10, 10],
    ], dtype=np.int8)
    coarse = coarsen(grid, cell_stride=2)
    assert coarse.shape == (2, 2)
    assert coarse[0, 0] == 10   # block [[10,10],[10,50]] -> majority flat
    assert coarse[0, 1] == 50   # block [[50,50],[50,50]] -> all rocky
    assert coarse[1, 0] == 10
    assert coarse[1, 1] == 10


def test_coarsen_any_obstacle_excludes_block():
    grid = np.array([
        [10, 10],
        [10, 100],
    ], dtype=np.int8)
    coarse = coarsen(grid, cell_stride=2)
    assert coarse[0, 0] == 100  # obstacle wins even as a minority


def test_coarsen_majority_unknown_excludes_block():
    grid = np.array([
        [-1, -1],
        [-1, 10],
    ], dtype=np.int8)
    coarse = coarsen(grid, cell_stride=2)
    assert coarse[0, 0] == -1


def test_coarsen_tie_breaks_toward_higher_cost_class():
    grid = np.array([
        [10, 50],
        [10, 50],
    ], dtype=np.int8)
    coarse = coarsen(grid, cell_stride=2)
    assert coarse[0, 0] == 50  # 2 flat / 2 rocky tie -> higher-cost class wins


def test_sample_elevation_matches_z_formula(tmp_path):
    from PIL import Image
    from planning_pkg.graph_model import sample_elevation

    arr = np.full((8, 8), 32768, dtype=np.uint16)  # uniform mid-grey
    png_path = tmp_path / 'flat_heightmap.png'
    Image.fromarray(arr, mode='I;16').save(str(png_path))

    z = sample_elevation(str(png_path), world_x=0.0, world_y=0.0)
    expected = (32768 / 65535.0) * 10.0 - 5.0
    assert math.isclose(z, expected, abs_tol=1e-3)


def test_sample_elevation_out_of_bounds_clamps_to_edge(tmp_path):
    from PIL import Image
    from planning_pkg.graph_model import sample_elevation

    arr = np.full((8, 8), 65535, dtype=np.uint16)  # all-white -> z = +5.0
    png_path = tmp_path / 'white_heightmap.png'
    Image.fromarray(arr, mode='I;16').save(str(png_path))

    z = sample_elevation(str(png_path), world_x=1000.0, world_y=-1000.0)
    assert math.isclose(z, 5.0, abs_tol=1e-3)


def test_build_graph_excludes_obstacle_and_unknown_nodes(tmp_path):
    from PIL import Image
    from planning_pkg.graph_model import build_graph

    arr = np.full((4, 4), 32768, dtype=np.uint16)
    png_path = tmp_path / 'flat.png'
    Image.fromarray(arr, mode='I;16').save(str(png_path))

    coarse = np.array([
        [10, 10, 100],
        [10, -1, 10],
        [10, 10, 10],
    ], dtype=np.int8)
    G = build_graph(
        coarse, resolution=0.5, cell_stride=1, origin_x=0.0, origin_y=0.0,
        heightmap_png_path=str(png_path),
    )

    assert (0, 2) not in G.nodes   # obstacle
    assert (1, 1) not in G.nodes   # unknown
    assert len(G.nodes) == 7

    node_set = set(G.nodes)
    for u, v in G.edges:
        assert u in node_set and v in node_set


def test_build_graph_flat_zero_slope_weight_matches_formula(tmp_path):
    from PIL import Image
    from planning_pkg.graph_model import build_graph

    arr = np.full((4, 4), 32768, dtype=np.uint16)  # uniform -> zero slope everywhere
    png_path = tmp_path / 'flat2.png'
    Image.fromarray(arr, mode='I;16').save(str(png_path))

    coarse = np.array([[10, 10]], dtype=np.int8)
    G = build_graph(
        coarse, resolution=0.5, cell_stride=1, origin_x=0.0, origin_y=0.0,
        heightmap_png_path=str(png_path),
    )
    w = G.edges[(0, 0), (0, 1)]['weight']
    # d = 0.5 m, S(theta) = 1.0 (zero slope), T_avg = 1.0 (both flat)
    assert math.isclose(w, 0.5, abs_tol=1e-6)


def test_build_graph_node_attrs():
    from PIL import Image
    import tempfile as _tempfile
    from planning_pkg.graph_model import build_graph

    with _tempfile.TemporaryDirectory() as tmpdir:
        png_path = os.path.join(tmpdir, 'flat.png')
        Image.fromarray(np.full((4, 4), 32768, dtype=np.uint16), mode='I;16').save(png_path)
        coarse = np.array([[50]], dtype=np.int8)
        G = build_graph(
            coarse, resolution=0.5, cell_stride=2, origin_x=-1.0, origin_y=-1.0,
            heightmap_png_path=png_path,
        )
        node = G.nodes[(0, 0)]
        assert node['terrain_class'] == 50
        assert math.isclose(node['terrain_factor'], 1.5, abs_tol=1e-9)
        # block centre: origin + (0*stride + stride/2) * resolution = -1.0 + 1*0.5 = -0.5
        assert math.isclose(node['pos_x'], -0.5, abs_tol=1e-9)
        assert math.isclose(node['pos_y'], -0.5, abs_tol=1e-9)


def test_save_and_load_graph_round_trip(tmp_path):
    import networkx as nx
    from planning_pkg.graph_model import load_graph, save_graph

    G = nx.Graph()
    G.add_node('a', pos_x=0.0, pos_y=0.0, terrain_class=10, terrain_factor=1.0)
    G.add_node('b', pos_x=1.0, pos_y=0.0, terrain_class=10, terrain_factor=1.0)
    G.add_edge('a', 'b', weight=1.0)

    paths = save_graph(G, str(tmp_path), meta={
        'resolution': 0.5, 'cell_stride': 1, 'source_timestamp': '2026-09-11T00:00:00',
    })
    assert os.path.exists(paths['graphml'])
    assert os.path.exists(paths['meta_json'])

    with open(paths['meta_json']) as f:
        meta = json.load(f)
    assert meta['node_count'] == 2
    assert meta['edge_count'] == 1
    assert math.isclose(meta['weight_mean'], 1.0, abs_tol=1e-9)
    assert meta['resolution'] == 0.5
    assert meta['cell_stride'] == 1

    G2 = load_graph(paths['graphml'])
    assert G2.number_of_nodes() == 2
    assert G2.number_of_edges() == 1
    edge_data = list(G2.edges(data=True))[0][2]
    assert math.isclose(float(edge_data['weight']), 1.0, abs_tol=1e-6)


def test_save_graph_empty_graph_does_not_crash(tmp_path):
    import networkx as nx
    from planning_pkg.graph_model import save_graph

    G = nx.Graph()
    paths = save_graph(G, str(tmp_path), meta={'resolution': 0.5, 'cell_stride': 1,
                                                 'source_timestamp': 'none'})
    with open(paths['meta_json']) as f:
        meta = json.load(f)
    assert meta['node_count'] == 0
    assert meta['edge_count'] == 0
    assert meta['weight_min'] is None


def test_cli_main_writes_graph_files(tmp_path):
    from PIL import Image
    from planning_pkg.graph_model import main

    heightmap_path = tmp_path / 'heightmap.png'
    Image.fromarray(np.full((4, 4), 32768, dtype=np.uint16), mode='I;16').save(str(heightmap_path))

    npz_path = tmp_path / 'latest.npz'
    grid = np.full((4, 4), 10, dtype=np.int8)
    np.savez_compressed(
        str(npz_path), grid=grid, resolution=0.5, origin_x=0.0, origin_y=0.0,
        width=4, height=4, timestamp='2026-09-11T00:00:00',
    )

    out_dir = tmp_path / 'graphs'
    rc = main([
        '--npz', str(npz_path),
        '--heightmap', str(heightmap_path),
        '--out', str(out_dir),
        '--cell-stride', '2',
    ])
    assert rc == 0
    assert (out_dir / 'latest.graphml').exists()
    assert (out_dir / 'latest_meta.json').exists()

    with open(out_dir / 'latest_meta.json') as f:
        meta = json.load(f)
    assert meta['node_count'] == 4  # 4x4 grid / stride 2 -> 2x2 blocks, all flat
    assert meta['cell_stride'] == 2
