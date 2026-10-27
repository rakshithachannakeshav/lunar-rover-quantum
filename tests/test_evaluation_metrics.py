#!/usr/bin/env python3
"""
tests/test_evaluation_metrics.py
--------------------------------
Pure-Python unit tests for evaluation_pkg.metrics (Phase 10: classical vs
quantum vs distance-only comparison). No ROS 2 / Gazebo dependency.
"""

import json
import math
import os
import sys

import networkx as nx
import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'evaluation_pkg'))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

NAV_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'navigation_pkg'))
if NAV_DIR not in sys.path:
    sys.path.insert(0, NAV_DIR)

from evaluation_pkg.metrics import (  # noqa: E402
    DEFAULT_GOAL_TOLERANCE_M,
    ExecutionMonitor,
    build_comparison,
    distance_only_baseline,
    execution_report,
    load_planner_metrics,
    main as cli_main,
    save_comparison,
)

DETOUR_DISTANCE = 2.0 * math.sqrt(2.0)


@pytest.fixture
def graph() -> nx.Graph:
    """Direct S-G edge is short but expensive; the S-M-G detour is longer but cheap.

      S(0,0) --- G(2,0)     direct: distance 2.0, energy 5.0
        \\       /
         M(1,1)             detour: distance 2.828, energy 1.5 + 1.5 = 3.0
    """
    g = nx.Graph()
    g.add_node('S', pos_x=0.0, pos_y=0.0)
    g.add_node('M', pos_x=1.0, pos_y=1.0)
    g.add_node('G', pos_x=2.0, pos_y=0.0)
    g.add_edge('S', 'G', weight=5.0)
    g.add_edge('S', 'M', weight=1.5)
    g.add_edge('M', 'G', weight=1.5)
    return g


def _plan(algorithm, runtime_ms):
    return {
        'algorithm': algorithm,
        'path_nodes': ['S', 'M', 'G'],
        'path_coords': [[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]],
        'total_energy': 3.0,
        'total_distance_m': DETOUR_DISTANCE,
        'num_waypoints': 3,
        'runtime_ms': runtime_ms,
        'timestamp': '2026-09-18T00:00:00',
    }


def _quantum(fallback=False, energy=3.0):
    plan = _plan('QAOA', 5000.0)
    plan.update({
        'total_energy': energy,
        'qubits': 11, 'shots': 2048, 'reps': 1, 'reduced_edges': 8,
        'valid_samples': 24, 'sampled_candidates': 2048,
        'fallback_to_classical': fallback,
    })
    return plan


@pytest.fixture
def result_files(tmp_path):
    classical = tmp_path / 'latest_classical.json'
    classical.write_text(json.dumps({
        'dijkstra': _plan('Dijkstra', 1.0),
        'astar': _plan('A*', 0.4),
    }), encoding='utf-8')
    quantum = tmp_path / 'latest_quantum.json'
    quantum.write_text(json.dumps({'quantum': _quantum()}), encoding='utf-8')
    return str(classical), str(quantum)


def test_load_planner_metrics_reads_all_three(result_files):
    planners = load_planner_metrics(*result_files)
    assert set(planners) == {'dijkstra', 'astar', 'qaoa'}

    astar = planners['astar']
    assert astar.energy == pytest.approx(3.0)
    assert astar.distance_m == pytest.approx(DETOUR_DISTANCE)
    assert astar.waypoints == 3
    assert astar.runtime_ms == pytest.approx(0.4)
    assert astar.straight_line_m == pytest.approx(2.0)
    assert astar.path_efficiency == pytest.approx(2.0 / DETOUR_DISTANCE)
    assert astar.energy_per_m == pytest.approx(3.0 / DETOUR_DISTANCE)

    qaoa = planners['qaoa']
    assert qaoa.extras['qubits'] == 11
    assert qaoa.extras['fallback_to_classical'] is False


def test_list_node_ids_are_normalised_to_graphml_form(tmp_path):
    plan = _plan('A*', 1.0)
    plan['path_nodes'] = [[29, 33], [30, 34]]
    classical = tmp_path / 'c.json'
    classical.write_text(json.dumps({'dijkstra': plan, 'astar': plan}), encoding='utf-8')
    quantum = tmp_path / 'q.json'
    quantum.write_text(json.dumps({'quantum': _quantum()}), encoding='utf-8')

    planners = load_planner_metrics(str(classical), str(quantum))
    assert planners['astar'].path_nodes == ['(29, 33)', '(30, 34)']


def test_missing_result_file_raises_informative_error(tmp_path):
    with pytest.raises(FileNotFoundError, match='not found'):
        load_planner_metrics(str(tmp_path / 'nope.json'), str(tmp_path / 'nope2.json'))


def test_distance_only_baseline_takes_short_but_costly_path(graph):
    baseline = distance_only_baseline(graph, 'S', 'G')
    assert baseline.name == 'distance_only'
    assert baseline.path_nodes == ['S', 'G']
    assert baseline.distance_m == pytest.approx(2.0)
    assert baseline.energy == pytest.approx(5.0)  # re-priced with the real weight
    assert baseline.path_efficiency == pytest.approx(1.0)


def test_build_comparison_reports_savings_vs_distance_only(result_files, graph, tmp_path):
    graph_path = tmp_path / 'g.graphml'
    nx.write_graphml(graph, str(graph_path))

    comparison = build_comparison(*result_files, graph_path=str(graph_path))
    summary = comparison['summary']

    assert set(comparison['planners']) == {'dijkstra', 'astar', 'qaoa', 'distance_only'}
    assert comparison['start_node'] == 'S'
    assert comparison['goal_node'] == 'G'
    assert summary['reference'] == 'astar'
    assert summary['qaoa_matches_classical'] is True
    assert summary['qaoa_energy_ratio'] == pytest.approx(1.0)
    assert summary['qaoa_fallback'] is False
    assert summary['qaoa_runtime_ratio_vs_astar'] == pytest.approx(5000.0 / 0.4)
    assert summary['best_energy_planner'] == 'astar'  # ties -> lowest runtime
    # (5.0 - 3.0) / 5.0 = 40 %
    assert summary['energy_savings_vs_distance_only_pct'] == pytest.approx(40.0)


def test_build_comparison_without_graph_skips_baseline(result_files):
    comparison = build_comparison(*result_files, graph_path=None)
    assert 'distance_only' not in comparison['planners']
    assert comparison['summary']['energy_savings_vs_distance_only_pct'] is None


def test_qaoa_fallback_is_flagged_not_hidden(tmp_path):
    classical = tmp_path / 'c.json'
    classical.write_text(json.dumps({
        'dijkstra': _plan('Dijkstra', 1.0), 'astar': _plan('A*', 0.4),
    }), encoding='utf-8')
    quantum = tmp_path / 'q.json'
    quantum.write_text(json.dumps({'quantum': _quantum(fallback=True)}), encoding='utf-8')

    summary = build_comparison(str(classical), str(quantum))['summary']
    assert summary['qaoa_fallback'] is True


def test_qaoa_worse_than_classical_is_not_a_match(tmp_path):
    classical = tmp_path / 'c.json'
    classical.write_text(json.dumps({
        'dijkstra': _plan('Dijkstra', 1.0), 'astar': _plan('A*', 0.4),
    }), encoding='utf-8')
    quantum = tmp_path / 'q.json'
    quantum.write_text(json.dumps({'quantum': _quantum(energy=3.6)}), encoding='utf-8')

    summary = build_comparison(str(classical), str(quantum))['summary']
    assert summary['qaoa_matches_classical'] is False
    assert summary['qaoa_energy_ratio'] == pytest.approx(1.2)


def test_save_comparison_writes_latest_and_history(result_files, tmp_path):
    comparison = build_comparison(*result_files)
    out_dir = tmp_path / 'energy_comparison'
    saved = save_comparison(comparison, str(out_dir))

    assert os.path.exists(saved['latest'])
    assert os.path.exists(saved['history'])
    with open(saved['latest'], encoding='utf-8') as f:
        data = json.load(f)
    assert data['summary']['reference'] == 'astar'
    assert data['planners']['qaoa']['extras']['qubits'] == 11


def test_execution_report_compares_planned_and_actual(result_files):
    planned = load_planner_metrics(*result_files)['astar']
    battery = {'distance_m': 3.0, 'consumed_j': 150.0, 'percentage': 0.99}

    report = execution_report(planned, battery)
    assert report['planner'] == 'astar'
    assert report['planned_distance_m'] == pytest.approx(DETOUR_DISTANCE)
    assert report['actual_distance_m'] == pytest.approx(3.0)
    assert report['execution_efficiency'] == pytest.approx(DETOUR_DISTANCE / 3.0)
    assert report['joules_per_m'] == pytest.approx(50.0)
    assert report['consumed_wh'] == pytest.approx(150.0 / 3600.0)


def test_execution_report_before_the_rover_moves(result_files):
    planned = load_planner_metrics(*result_files)['astar']
    report = execution_report(planned, {'distance_m': 0.0, 'consumed_j': 0.0, 'percentage': 1.0})
    assert report['execution_efficiency'] is None
    assert report['joules_per_m'] is None


def test_execution_monitor_tracks_a_full_drive(result_files):
    planned = load_planner_metrics(*result_files)['astar']
    monitor = ExecutionMonitor(planned, goal_tolerance=0.3)
    assert monitor.goal == (2.0, 0.0)

    monitor.update_battery(total_consumed_j=100.0, percentage=1.0)  # baseline sample
    assert monitor.update_position(0.0, 0.0) is False
    monitor.update_battery(total_consumed_j=160.0, percentage=0.99)
    assert monitor.update_position(1.0, 1.0) is False
    monitor.update_battery(total_consumed_j=220.0, percentage=0.98)
    assert monitor.update_position(2.0, 0.1) is True  # inside tolerance of (2, 0)
    assert monitor.goal_reached

    report = monitor.report()
    assert report['goal_reached'] is True
    assert report['consumed_j'] == pytest.approx(120.0)  # relative to the first sample
    assert report['actual_distance_m'] == pytest.approx(math.sqrt(2.0) + math.hypot(1.0, 0.9))
    assert report['battery_percentage'] == pytest.approx(0.98)
    assert report['distance_to_goal_m'] == pytest.approx(0.1)


def test_execution_monitor_freezes_after_arrival(result_files):
    planned = load_planner_metrics(*result_files)['astar']
    monitor = ExecutionMonitor(planned, goal_tolerance=0.3)
    monitor.update_battery(0.0, 1.0)
    monitor.update_position(2.0, 0.0)
    monitor.update_battery(50.0, 0.9)
    frozen = monitor.report()

    assert monitor.update_position(5.0, 5.0) is False  # reported once only
    monitor.update_battery(500.0, 0.5)
    assert monitor.report() == frozen


def test_evaluator_tolerance_is_not_tighter_than_the_executors():
    """If the executor stops at 0.5 m but the evaluator needs 0.35 m, arrival is never seen."""
    from navigation_pkg.path_following import DEFAULT_GOAL_TOLERANCE

    assert DEFAULT_GOAL_TOLERANCE_M >= DEFAULT_GOAL_TOLERANCE


def test_execution_monitor_rejects_pathless_planner(result_files):
    planned = load_planner_metrics(*result_files)['astar']
    planned.path_coords = []
    with pytest.raises(ValueError):
        ExecutionMonitor(planned)


def test_cli_main_writes_comparison(result_files, graph, tmp_path):
    graph_path = tmp_path / 'g.graphml'
    nx.write_graphml(graph, str(graph_path))
    out_dir = tmp_path / 'out'

    rc = cli_main([
        '--classical', result_files[0],
        '--quantum', result_files[1],
        '--graph', str(graph_path),
        '--out', str(out_dir),
    ])
    assert rc == 0
    assert (out_dir / 'latest_comparison.json').exists()


def test_cli_returns_error_code_on_missing_input(tmp_path):
    rc = cli_main([
        '--classical', str(tmp_path / 'nope.json'),
        '--quantum', str(tmp_path / 'nope2.json'),
        '--out', str(tmp_path / 'out'),
    ])
    assert rc == 1


# ── Route sweep: energy-aware vs distance-only over many start/goal pairs ─────

@pytest.fixture
def rough_band_grid() -> nx.Graph:
    """5x5 grid; the middle column (x == 2, y in 1..3) is rough terrain (factor 10).

    Going straight across is shortest by distance but expensive; going around
    through the open top/bottom rows is longer but cheaper.
    """
    factor = {}
    g = nx.Graph()
    for x in range(5):
        for y in range(5):
            node = f'{x},{y}'
            factor[node] = 10.0 if (x == 2 and 1 <= y <= 3) else 1.0
            g.add_node(node, pos_x=float(x), pos_y=float(y))
    for x in range(5):
        for y in range(5):
            for dx, dy in ((1, 0), (0, 1)):
                nx_, ny_ = x + dx, y + dy
                if nx_ < 5 and ny_ < 5:
                    a, b = f'{x},{y}', f'{nx_},{ny_}'
                    g.add_edge(a, b, weight=1.0 * (factor[a] + factor[b]) / 2.0)
    return g


def test_sweep_finds_routes_where_energy_aware_planning_saves_energy(rough_band_grid):
    from evaluation_pkg.metrics import baseline_sweep

    sweep = baseline_sweep(rough_band_grid, n_pairs=40, seed=7, min_separation_m=2.0)
    summary = sweep['summary']

    assert summary['n_pairs'] == 40
    assert len(sweep['pairs']) == 40
    assert summary['differing_pairs'] > 0
    assert summary['max_savings_pct'] > 0.0
    assert 0.0 <= summary['mean_savings_pct'] <= summary['max_savings_pct']
    # Energy-optimal can never cost more than the distance-only path.
    assert all(p['savings_pct'] >= -1e-9 for p in sweep['pairs'])
    assert all(p['separation_m'] >= 2.0 for p in sweep['pairs'])


def test_sweep_is_deterministic_for_a_seed(rough_band_grid):
    from evaluation_pkg.metrics import baseline_sweep

    a = baseline_sweep(rough_band_grid, n_pairs=15, seed=3, min_separation_m=2.0)
    b = baseline_sweep(rough_band_grid, n_pairs=15, seed=3, min_separation_m=2.0)
    assert a['pairs'] == b['pairs']


def test_sweep_with_impossible_separation_returns_empty_summary(rough_band_grid):
    from evaluation_pkg.metrics import baseline_sweep

    sweep = baseline_sweep(rough_band_grid, n_pairs=10, seed=1, min_separation_m=1000.0)
    assert sweep['pairs'] == []
    assert sweep['summary']['n_pairs'] == 0
    assert sweep['summary']['mean_savings_pct'] is None


def test_build_comparison_includes_sweep_when_requested(result_files, graph, tmp_path):
    graph_path = tmp_path / 'g.graphml'
    nx.write_graphml(graph, str(graph_path))

    comparison = build_comparison(
        *result_files, graph_path=str(graph_path), sweep_pairs=5, sweep_min_separation_m=1.0
    )
    assert 'sweep' in comparison
    assert comparison['sweep']['summary']['n_pairs'] >= 0


def test_build_comparison_sweep_requires_a_graph(result_files):
    with pytest.raises(ValueError, match='graph'):
        build_comparison(*result_files, graph_path=None, sweep_pairs=5)
