#!/usr/bin/env python3
"""
visualize_plan.py
-----------------
Visualizes classical path planning results (Phase 7) overlaid on the
energy-weighted terrain graph.

Usage:
    python scripts/visualize_plan.py
    python scripts/visualize_plan.py --json results/paths/latest_classical.json --out results/paths/plan_viz.png
"""

import argparse
import json
import os
import sys
from pathlib import Path
import matplotlib
# Use non-interactive backend by default so it works in headless/WSL environments without error
if 'DISPLAY' not in os.environ and sys.platform != 'win32':
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import networkx as nx


def main():
    parser = argparse.ArgumentParser(description="Visualize Phase 7 Classical Path Planning Results.")
    parser.add_argument(
        '--json',
        type=str,
        default='results/paths/latest_classical.json',
        help='Path to the classical plan JSON output',
    )
    parser.add_argument(
        '--graph',
        type=str,
        default='results/graphs/latest.graphml',
        help='Path to the energy-weighted GraphML file',
    )
    parser.add_argument(
        '--out',
        type=str,
        default='results/paths/latest_classical_plot.png',
        help='Path to save the output plot image',
    )
    parser.add_argument(
        '--show',
        action='store_true',
        help='Display the plot window interactively (if GUI is available)',
    )
    args = parser.parse_args()

    if not os.path.exists(args.json):
        print(f"Error: JSON file '{args.json}' not found.")
        sys.exit(1)

    with open(args.json, 'r') as f:
        data = json.load(f)

    fig, ax = plt.subplots(figsize=(10, 8), dpi=150)
    fig.patch.set_facecolor('#1e1e24')
    ax.set_facecolor('#141418')

    # 1. Plot background graph nodes if graphml exists
    if os.path.exists(args.graph):
        print(f"Loading terrain graph: {args.graph}")
        G = nx.read_graphml(args.graph)
        xs = [float(d['pos_x']) for _, d in G.nodes(data=True) if 'pos_x' in d]
        ys = [float(d['pos_y']) for _, d in G.nodes(data=True) if 'pos_y' in d]
        ax.scatter(xs, ys, s=12, color='#44475a', alpha=0.35, label='Traversable Nodes')

    # 2. Extract paths
    astar = data.get('astar', {})
    dijkstra = data.get('dijkstra', {})
    comp = data.get('comparison', {})

    a_coords = astar.get('path_coords', [])
    d_coords = dijkstra.get('path_coords', [])

    # Plot Dijkstra path (slightly thicker dashed underlay)
    if d_coords:
        dx = [p[0] for p in d_coords]
        dy = [p[1] for p in d_coords]
        ax.plot(
            dx, dy,
            linestyle='--', color='#ff79c6', linewidth=2.5, alpha=0.8,
            label=f"Dijkstra ({dijkstra.get('total_energy', 0):.2f} J, {dijkstra.get('runtime_ms', 0):.2f} ms)"
        )

    # Plot A* path
    if a_coords:
        ax_pts = [p[0] for p in a_coords]
        ay_pts = [p[1] for p in a_coords]
        ax.plot(
            ax_pts, ay_pts,
            linestyle='-', color='#50fa7b', linewidth=2.0, marker='o', markersize=4,
            label=f"A* ({astar.get('total_energy', 0):.2f} J, {astar.get('runtime_ms', 0):.2f} ms)"
        )

        # Start and Goal markers
        ax.scatter([ax_pts[0]], [ay_pts[0]], color='#8be9fd', s=130, marker='D', edgecolor='white', linewidth=1.5, zorder=5, label='Start')
        ax.scatter([ax_pts[-1]], [ay_pts[-1]], color='#ff5555', s=160, marker='*', edgecolor='white', linewidth=1.5, zorder=5, label='Goal')

    # 3. Aesthetics and text box
    ax.set_title("Phase 7: Classical Path Planning (A* vs Dijkstra)", color='white', fontsize=14, pad=12, fontweight='bold')
    ax.set_xlabel("X Position (meters)", color='white', fontsize=11)
    ax.set_ylabel("Y Position (meters)", color='white', fontsize=11)
    ax.tick_params(colors='white')
    for spine in ax.spines.values():
        spine.set_color('#6272a4')

    speedup = comp.get('speedup_factor', 1.0)
    identical = comp.get('paths_identical', False)
    metrics_text = (
        f"A* Energy: {astar.get('total_energy', 0):.2f} J\n"
        f"A* Distance: {astar.get('total_distance_m', 0):.2f} m\n"
        f"Waypoints: {astar.get('num_waypoints', 0)}\n"
        f"A* Speedup: {speedup:.1f}x\n"
        f"Paths Match: {identical}"
    )
    ax.text(
        0.03, 0.97, metrics_text,
        transform=ax.transAxes, verticalalignment='top',
        fontsize=10, color='white', family='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#282a36', edgecolor='#6272a4', alpha=0.9)
    )

    ax.legend(loc='lower right', facecolor='#282a36', edgecolor='#6272a4', labelcolor='white')
    ax.grid(True, linestyle=':', alpha=0.25, color='gray')

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(str(out_path), facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"Visualization saved to: {out_path.resolve()}")

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
