#!/usr/bin/env python3
"""
plot_energy_comparison.py
-------------------------
Phase 10 figures, drawn from results/energy_comparison/latest_comparison.json
(written by `python -m evaluation_pkg.metrics`).

    python scripts/plot_energy_comparison.py
    python scripts/plot_energy_comparison.py --theme dark --out results/energy_comparison

Writes, next to the comparison file:
    energy_comparison.png   energy / path length / runtime per planner (one route)
    route_sweep.png         energy saved vs a distance-only path over many routes
    path_overlay.png        the planned paths (and the biggest-saving route) on the graph
    comparison_table.csv    the same numbers as a table (the accessible view)

Colours follow the project dataviz palette (validated for colour-vision
deficiency in light and dark): planners take fixed categorical slots, the
energy-unaware baseline is neutral grey.
"""

import argparse
import csv
import json
import math
import os
import sys

import matplotlib

matplotlib.use('Agg')  # always headless: figures are files, not windows

import matplotlib.pyplot as plt  # noqa: E402
import networkx as nx  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'evaluation_pkg')
)

THEMES = {
    'light': {
        'surface': '#fcfcfb', 'text_primary': '#0b0b0b', 'text_secondary': '#52514e',
        'muted': '#898781', 'grid': '#e1e0d9', 'axis': '#c3c2b7',
        'slot1': '#2a78d6', 'slot2': '#eb6834', 'slot3': '#1baf7a',
    },
    'dark': {
        'surface': '#1a1a19', 'text_primary': '#ffffff', 'text_secondary': '#c3c2b7',
        'muted': '#898781', 'grid': '#2c2c2a', 'axis': '#383835',
        'slot1': '#3987e5', 'slot2': '#d95926', 'slot3': '#199e70',
    },
}

# Fixed order = fixed colour slot. The baseline is not a categorical series: grey.
PLANNER_ORDER = ('dijkstra', 'astar', 'qaoa', 'distance_only')
PLANNER_SLOT = {'dijkstra': 'slot1', 'astar': 'slot2', 'qaoa': 'slot3', 'distance_only': 'muted'}
PLANNER_LABEL = {
    'dijkstra': 'Dijkstra', 'astar': 'A*', 'qaoa': 'QAOA (simulated)',
    'distance_only': 'Shortest distance (energy-unaware)',
}
PLANNER_TICK = {'dijkstra': 'Dijkstra', 'astar': 'A*', 'qaoa': 'QAOA', 'distance_only': 'Distance\nonly'}


def _color(theme, planner):
    return THEMES[theme][PLANNER_SLOT[planner]]


def _style_axes(ax, theme):
    t = THEMES[theme]
    ax.set_facecolor(t['surface'])
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(t['axis'])
        ax.spines[side].set_linewidth(1.0)
    ax.tick_params(colors=t['muted'], labelcolor=t['text_secondary'], length=0, labelsize=9)
    ax.grid(axis='y', color=t['grid'], linewidth=0.8, linestyle='-')  # solid hairlines
    ax.set_axisbelow(True)


def _new_figure(theme, size):
    fig = plt.figure(figsize=size, dpi=150)
    fig.patch.set_facecolor(THEMES[theme]['surface'])
    return fig


def _headline(fig, theme, title, subtitle):
    t = THEMES[theme]
    fig.text(0.012, 0.965, title, ha='left', va='top', fontsize=13.5,
             fontweight='bold', color=t['text_primary'])
    fig.text(0.012, 0.905, subtitle, ha='left', va='top', fontsize=9.5,
             color=t['text_secondary'])


def _qaoa_headline(summary):
    ratio = summary.get('qaoa_energy_ratio')
    slower = summary.get('qaoa_runtime_ratio_vs_astar')
    slow_txt = f' but takes {slower:,.0f}× longer than A*' if slower else ''
    if summary.get('qaoa_fallback'):
        return ('QAOA fell back to the classical path - its numbers are not an '
                'independent quantum result')
    if summary.get('qaoa_matches_classical'):
        return f'QAOA finds the classical optimum (energy ratio {ratio:.2f}){slow_txt}'
    return f'QAOA energy is {ratio:.2f}× the classical optimum{slow_txt}'


def _fmt_runtime(v):
    return f'{v:,.0f}' if v >= 100 else f'{v:.2f}'


def plot_planner_bars(comparison, theme, out_path):
    t = THEMES[theme]
    planners = comparison['planners']
    names = [n for n in PLANNER_ORDER if n in planners]
    panels = [
        ('energy', 'Energy cost (weighted units)', lambda v: f'{v:.3f}', False),
        ('distance_m', 'Path length (m)', lambda v: f'{v:.2f}', False),
        ('runtime_ms', 'Runtime (ms, log scale)', _fmt_runtime, True),
    ]

    fig = _new_figure(theme, (11, 4.9))
    for i, (key, title, fmt, log) in enumerate(panels):
        ax = fig.add_subplot(1, 3, i + 1)
        _style_axes(ax, theme)
        values = [float(planners[n][key]) for n in names]
        xs = range(len(names))
        ax.bar(xs, values, width=0.55, color=[_color(theme, n) for n in names], zorder=3)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([PLANNER_TICK[n] for n in names])
        ax.set_title(title, loc='left', fontsize=10, color=t['text_secondary'], pad=8)

        if log:
            ax.set_yscale('log')
            positive = [v for v in values if v > 0] or [1.0]
            ax.set_ylim(min(positive) / 4.0, max(positive) * 6.0)
            top = max(positive) * 1.25
        else:
            peak = max(values) or 1.0
            ax.set_ylim(0, peak * 1.2)
            top = peak * 0.02
        for x, v in zip(xs, values):
            y = v * 1.12 if log else v + top
            ax.text(x, y, fmt(v), ha='center', va='bottom', fontsize=9, color=t['text_primary'])

    handles = [Patch(facecolor=_color(theme, n), label=PLANNER_LABEL[n]) for n in names]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.005, 0.86), ncol=len(handles),
               frameon=False, labelcolor=t['text_secondary'], fontsize=9)
    start, goal = comparison.get('start_node'), comparison.get('goal_node')
    _headline(fig, theme, _qaoa_headline(comparison['summary']),
              f'One route, every planner: {start} → {goal}. '
              f'Energy is cost-weighted distance; QAOA runs on a CPU simulator.')
    fig.subplots_adjust(top=0.70, bottom=0.12, left=0.06, right=0.985, wspace=0.28)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_route_sweep(comparison, theme, out_path):
    sweep = comparison.get('sweep')
    if not sweep or not sweep['pairs']:
        return False

    t = THEMES[theme]
    savings = np.array([p['savings_pct'] for p in sweep['pairs']], dtype=float)
    s = sweep['summary']

    fig = _new_figure(theme, (9, 4.9))
    ax = fig.add_subplot(1, 1, 1)
    _style_axes(ax, theme)
    upper = max(float(savings.max()), 1e-6) * 1.0001
    ax.hist(savings, bins=np.linspace(0.0, upper, 21), color=t['slot1'],
            edgecolor=t['surface'], linewidth=1.5, zorder=3)
    ax.set_xlabel('Energy saved vs the shortest-distance path (%)', color=t['text_secondary'],
                  fontsize=10)
    ax.set_ylabel('Routes', color=t['text_secondary'], fontsize=10)

    ax.set_xlim(-0.02 * upper, upper * 1.03)
    top = ax.get_ylim()[1]
    bin_w = upper / 20.0
    # The mean sits inside the tallest bar, so its label goes just right of that bar;
    # the max line is at the far edge, so its label goes to the left of the line.
    ax.axvline(s['mean_savings_pct'], color=t['text_secondary'], linewidth=1.0, zorder=4)
    ax.text(bin_w * 1.15, top * 0.93, f"← mean {s['mean_savings_pct']:.2f} %", ha='left',
            va='top', fontsize=9.5, color=t['text_primary'])
    ax.axvline(s['max_savings_pct'], color=t['text_secondary'], linewidth=1.0, zorder=4)
    ax.text(s['max_savings_pct'] - 0.01 * upper, top * 0.55, f"max {s['max_savings_pct']:.2f} %",
            ha='right', va='top', fontsize=9.5, color=t['text_primary'])

    _headline(
        fig, theme,
        f"Energy-aware planning saves {s['mean_savings_pct']:.2f} % on average, "
        f"up to {s['max_savings_pct']:.1f} % on a single route",
        f"{s['n_pairs']} random routes at least {s['min_separation_m']:g} m apart on the real terrain "
        f"graph; {s['differing_pairs']} of them differ from the shortest-distance path.")
    fig.subplots_adjust(top=0.80, bottom=0.14, left=0.08, right=0.97)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    return True


def _padded_limits(coords, pad=1.2):
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)


def _draw_nodes(ax, graph, xlim, ylim, theme):
    pts = [(float(d['pos_x']), float(d['pos_y'])) for _, d in graph.nodes(data=True)
           if 'pos_x' in d and xlim[0] <= float(d['pos_x']) <= xlim[1]
           and ylim[0] <= float(d['pos_y']) <= ylim[1]]
    if pts:
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=8, color=THEMES[theme]['grid'],
                   zorder=1, linewidths=0)


def _mark_endpoints(ax, coords, theme):
    t = THEMES[theme]
    ax.scatter([coords[0][0]], [coords[0][1]], s=70, color=t['text_primary'], edgecolor=t['surface'],
               linewidth=2, zorder=8)
    ax.scatter([coords[-1][0]], [coords[-1][1]], s=150, marker='*', color=t['text_primary'],
               edgecolor=t['surface'], linewidth=1.5, zorder=8)


def _finish_map_axes(ax, theme, xlim, ylim, title):
    t = THEMES[theme]
    ax.set_facecolor(t['surface'])
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect('equal')
    ax.set_title(title, loc='left', fontsize=10, color=t['text_secondary'], pad=8)
    ax.tick_params(colors=t['muted'], labelcolor=t['text_secondary'], length=0, labelsize=8.5)
    ax.set_xlabel('x (m)', color=t['text_secondary'], fontsize=9)
    ax.set_ylabel('y (m)', color=t['text_secondary'], fontsize=9)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(t['axis'])


def _largest_saving_route(graph, sweep):
    """Recompute both paths for the sweep pair with the biggest saving."""
    from evaluation_pkg.metrics import distance_only_baseline

    best = max(sweep['pairs'], key=lambda p: p['savings_pct'])
    if best['savings_pct'] <= 1e-9:
        return None
    start, goal = best['start'], best['goal']
    if start not in graph or goal not in graph:
        return None
    optimal = nx.dijkstra_path(graph, start, goal, weight='weight')
    coords = [[float(graph.nodes[n]['pos_x']), float(graph.nodes[n]['pos_y'])] for n in optimal]
    return best, coords, distance_only_baseline(graph, start, goal).path_coords


def plot_path_overlay(comparison, theme, out_path, graph=None):
    t = THEMES[theme]
    planners = comparison['planners']
    names = [n for n in PLANNER_ORDER if n in planners]
    route_coords = planners['astar']['path_coords']
    if not route_coords:
        return False

    extra = None
    if graph is not None and comparison.get('sweep') and comparison['sweep']['pairs']:
        extra = _largest_saving_route(graph, comparison['sweep'])

    ncols = 2 if extra else 1
    fig = _new_figure(theme, (6.4 * ncols + 0.6, 5.6))
    ax = fig.add_subplot(1, ncols, 1)
    xlim, ylim = _padded_limits(route_coords)
    if graph is not None:
        _draw_nodes(ax, graph, xlim, ylim, theme)

    # Thickest line at the bottom, thinnest on top, so identical paths all stay visible.
    width = {'qaoa': 7.0, 'dijkstra': 4.5, 'astar': 2.2, 'distance_only': 1.4}
    for z, n in enumerate(width):
        if n not in planners:
            continue
        xs = [c[0] for c in planners[n]['path_coords']]
        ys = [c[1] for c in planners[n]['path_coords']]
        ax.plot(xs, ys, color=_color(theme, n), linewidth=width[n], solid_capstyle='round',
                linestyle='--' if n == 'distance_only' else '-', zorder=3 + z)
    _mark_endpoints(ax, route_coords, theme)
    _finish_map_axes(ax, theme, xlim, ylim, 'The evaluated route (all planners overlaid)')

    if extra:
        best, optimal_coords, distance_coords = extra
        ax2 = fig.add_subplot(1, 2, 2)
        xlim2, ylim2 = _padded_limits(optimal_coords + distance_coords)
        _draw_nodes(ax2, graph, xlim2, ylim2, theme)
        ax2.plot([c[0] for c in distance_coords], [c[1] for c in distance_coords],
                 color=t['muted'], linewidth=1.8, linestyle='--', zorder=3)
        ax2.plot([c[0] for c in optimal_coords], [c[1] for c in optimal_coords],
                 color=t['slot1'], linewidth=2.4, zorder=4)
        _mark_endpoints(ax2, optimal_coords, theme)
        _finish_map_axes(ax2, theme, xlim2, ylim2,
                         f"Biggest saving in the sweep: {best['savings_pct']:.1f} % less energy")

    handles = [Line2D([0], [0], color=_color(theme, n), linewidth=3.0,
                      linestyle='--' if n == 'distance_only' else '-', label=PLANNER_LABEL[n])
               for n in names]
    fig.legend(handles=handles, loc='lower center', ncol=len(handles), frameon=False,
               labelcolor=t['text_secondary'], fontsize=9)
    _headline(fig, theme, 'Planned paths on the terrain graph',
              'Dots are traversable graph nodes; the circle is the start, the star the goal.')
    fig.subplots_adjust(top=0.82, bottom=0.22, left=0.07, right=0.98, wspace=0.22)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    return True


TABLE_COLUMNS = ('planner', 'energy', 'distance_m', 'waypoints', 'runtime_ms',
                 'path_efficiency', 'energy_per_m')


def write_table(comparison, out_path):
    """The accessible table view of the same numbers, as CSV (and printed)."""
    rows = []
    for name in PLANNER_ORDER:
        if name in comparison['planners']:
            p = comparison['planners'][name]
            rows.append([name] + [p[c] for c in TABLE_COLUMNS[1:]])
    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows(rows)

    print(f"{'planner':<14}{'energy':>10}{'dist_m':>10}{'wpts':>6}{'runtime_ms':>13}"
          f"{'efficiency':>12}{'energy/m':>10}")
    for r in rows:
        eff = '-' if r[5] is None else f'{r[5]:.3f}'
        epm = '-' if r[6] is None else f'{r[6]:.3f}'
        print(f'{r[0]:<14}{r[1]:>10.4f}{r[2]:>10.3f}{r[3]:>6}{r[4]:>13.2f}{eff:>12}{epm:>10}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Plot the Phase 10 energy comparison.')
    parser.add_argument('--comparison', default='results/energy_comparison/latest_comparison.json')
    parser.add_argument('--graph', default=None,
                        help='Energy graph for the path overlay (default: the one recorded in '
                             'the comparison file).')
    parser.add_argument('--out', default=None,
                        help='Output directory (default: next to the comparison file).')
    parser.add_argument('--theme', choices=sorted(THEMES), default='light')
    args = parser.parse_args(argv)

    if not os.path.exists(args.comparison):
        print(f"Error: comparison file '{args.comparison}' not found - run "
              "`python -m evaluation_pkg.metrics` first.")
        return 1
    with open(args.comparison, 'r', encoding='utf-8') as f:
        comparison = json.load(f)

    out_dir = args.out or os.path.dirname(os.path.abspath(args.comparison))
    os.makedirs(out_dir, exist_ok=True)
    suffix = '' if args.theme == 'light' else f'_{args.theme}'

    graph = None
    graph_path = args.graph or (comparison.get('sources') or {}).get('graph')
    if graph_path and os.path.exists(graph_path):
        graph = nx.read_graphml(graph_path)

    written = []
    path = os.path.join(out_dir, f'energy_comparison{suffix}.png')
    plot_planner_bars(comparison, args.theme, path)
    written.append(path)

    path = os.path.join(out_dir, f'route_sweep{suffix}.png')
    if plot_route_sweep(comparison, args.theme, path):
        written.append(path)
    else:
        print('No route sweep in the comparison file - run evaluation_pkg.metrics with --sweep N.')

    path = os.path.join(out_dir, f'path_overlay{suffix}.png')
    if plot_path_overlay(comparison, args.theme, path, graph):
        written.append(path)

    table = os.path.join(out_dir, 'comparison_table.csv')
    write_table(comparison, table)
    written.append(table)

    for w in written:
        print(f'wrote {w}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
