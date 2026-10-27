#!/usr/bin/env python3
"""
plot_multi_site.py
------------------
Figures for scripts/benchmark_multi_site.py. Can be re-run on a saved CSV:

    python scripts/plot_multi_site.py results/multi_site/benchmark_raw.csv
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

# Reference categorical palette, fixed slot order (light surface).
SURFACE = "#fcfcfb"
INK, INK_2, MUTED, GRID, AXIS = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SLOTS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300",
         "#6250d6", "#e34948"]

# Colour follows the method, in every figure.
METHOD_STYLE = {
    "nearest_neighbour": ("Nearest neighbour", SLOTS[0], "o"),
    "nn_2opt": ("NN + 2-opt", SLOTS[1], "s"),
    "qaoa_window_a1": ("QAOA windows, CVaR 1.0", SLOTS[2], "D"),
    "window_exact": ("Exact windows (NN seed)", SLOTS[3], "^"),
    "qaoa_window_a0.5": ("QAOA windows, CVaR 0.5", SLOTS[4], "D"),
    "qaoa_window_a0.2": ("QAOA windows, CVaR 0.2", SLOTS[5], "D"),
    "held_karp": ("Held-Karp (exact)", SLOTS[6], "v"),
    "brute_force": ("Brute force (exact)", SLOTS[7], "P"),
}
FULL_QAOA = r"qaoa_p\d+"  # matches qaoa_p1 (size-only rows) and qaoa_p1_a0.5 (runs)
QAOA_FULL = {1: SLOTS[0], 2: SLOTS[1], 3: SLOTS[2], 4: SLOTS[3]}


def _style(ax, title, ylabel, xlabel="Number of sites N"):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=11, color=INK, pad=10)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8)


def _figure(ncols=1, width=7.0):
    fig, axes = plt.subplots(1, ncols, figsize=(width, 4.2), facecolor=SURFACE,
                             squeeze=False)
    return fig, axes[0]


def _line_with_label(ax, x, y, label, color, marker):
    ax.plot(x, y, color=color, linewidth=2, marker=marker, markersize=6,
            markeredgecolor=SURFACE, markeredgewidth=1.2, label=label)
    ax._end_labels = getattr(ax, "_end_labels", []) + [(x[-1], y[-1], label)]


def _place_end_labels(ax, min_gap_px=12):
    """Direct labels at line ends, nudged apart vertically so equal values don't collide."""
    items = getattr(ax, "_end_labels", [])
    if not items:
        return
    ax.figure.canvas.draw()
    to_px = ax.transData.transform
    placed = sorted(((to_px((x, y))[1], x, y, label) for x, y, label in items))
    last = -np.inf
    px_to_pt = 72.0 / ax.figure.dpi
    for py, x, y, label in placed:
        target = max(py, last + min_gap_px)
        last = target
        ax.annotate(label, (x, y), xytext=(8, (target - py) * px_to_pt),
                    textcoords="offset points", va="center", fontsize=8, color=INK_2)


def _integer_x(ax):
    ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))


def plot_approx_ratio(df, out):
    fig, (ax,) = _figure(width=8.5)
    ok = df[df["status"] == "ok"]
    for method in ("nearest_neighbour", "nn_2opt", "qaoa_window_a1", "window_exact",
                   "qaoa_window_a0.5", "qaoa_window_a0.2"):
        sub = ok[ok["method"] == method]
        if sub.empty:
            continue
        label, color, marker = METHOD_STYLE[method]
        ax.scatter(sub["n_sites"], sub["approx_ratio"], color=color, s=14, alpha=0.35,
                   linewidths=0)
        mean = sub.groupby("n_sites")["approx_ratio"].mean()
        _line_with_label(ax, mean.index.to_numpy(), mean.to_numpy(), label, color, marker)
    ax.axhline(1.0, color=AXIS, linewidth=1)
    _style(ax, "Tour cost / optimal (lower is better; dots = seeds, line = mean)",
           "Approximation ratio")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.set_xlim(right=ax.get_xlim()[1] + 1.2)
    _integer_x(ax)
    fig.tight_layout()
    _place_end_labels(ax)
    path = out / "approx_ratio_vs_n.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def plot_runtime(df, out):
    fig, (ax,) = _figure(width=8.5)
    ok = df[df["status"] == "ok"]
    methods = ["held_karp", "brute_force", "nn_2opt", "qaoa_window_a1", "window_exact"]
    for method in methods:
        sub = ok[ok["method"] == method]
        if sub.empty:
            continue
        label, color, marker = METHOD_STYLE[method]
        mean = sub.groupby("n_sites")["runtime_s"].mean()
        _line_with_label(ax, mean.index.to_numpy(), mean.to_numpy(), label, color, marker)
    for p, color in QAOA_FULL.items():
        sub = ok[ok["method"].str.match(rf"qaoa_p{p}_a")]
        if sub.empty:
            continue
        mean = sub.groupby("n_sites")["runtime_s"].mean()
        ax.scatter(mean.index, mean.to_numpy(), marker="*", s=120, color=color,
                   edgecolor=INK, linewidth=0.6, zorder=5, label=f"Full QAOA p={p} (mean over CVaR)")
    ax.set_yscale("log")
    _style(ax, "Mean solve time for the visiting order (A* legs excluded)", "Seconds (log)")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.set_xlim(right=ax.get_xlim()[1] + 1.2)
    _integer_x(ax)
    fig.tight_layout()
    _place_end_labels(ax)
    path = out / "runtime_vs_n.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def plot_resources(df, out):
    full = df[df["method"].str.match(FULL_QAOA)]
    if full.empty:
        return None
    fig, (ax1, ax2) = _figure(ncols=2, width=10)
    q = full.groupby("n_sites")["qubits"].max()
    ax1.plot(q.index, q.to_numpy(), color=SLOTS[0], linewidth=2, marker="o", markersize=6)
    for n, v in q.items():
        ax1.annotate(f"{int(v)}", (n, v), xytext=(0, 6), textcoords="offset points",
                     ha="center", fontsize=8, color=INK_2)
    _style(ax1, "Qubits for the full QUBO (N²)", "Qubits")

    for p, color in QAOA_FULL.items():
        sub = full[full["p"] == p]
        if sub.empty:
            continue
        cx = sub.groupby("n_sites")["two_qubit_gates"].max()
        _line_with_label(ax2, cx.index.to_numpy(), cx.to_numpy(), f"p={p}", color, "o")
    ax2.set_yscale("log")
    _style(ax2, "CX gates (all-to-all, before routing)", "CX count (log)")
    ax2.set_xlim(right=ax2.get_xlim()[1] + 0.8)
    _integer_x(ax1)
    _integer_x(ax2)
    fig.tight_layout()
    _place_end_labels(ax2)
    path = out / "qaoa_resources_vs_n.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def plot_qaoa_distribution(df, out):
    """Where the full QAOA ran: feasibility / optimal-tour probability vs uniform."""
    full = df[df["method"].str.match(FULL_QAOA) & (df["status"] == "ok")]
    if full.empty:
        return None
    fig, (ax1, ax2) = _figure(ncols=2, width=10)
    labels, groups = [], []
    for (n, p, a), sub in full.groupby(["n_sites", "p", "cvar_alpha"]):
        labels.append(f"N={int(n)} p={int(p)}\nCVaR {a:g}")
        groups.append(sub)
    x = np.arange(len(groups))
    w = 0.38
    for ax, col, ucol, title in (
        (ax1, "p_feasible", "uniform_p_feasible", "P(sample is a valid tour)"),
        (ax2, "p_optimal", "uniform_p_optimal", "P(sample is an optimal tour)"),
    ):
        qa = [g[col].mean() for g in groups]
        un = [g[ucol].mean() for g in groups]
        ax.bar(x - w / 2, qa, w, color=SLOTS[0], label="QAOA (mean over seeds)",
               edgecolor=SURFACE, linewidth=2)
        same_n = len(set(un)) == 1
        ax.bar(x + w / 2, un, w, color=SLOTS[1],
               label="Uniform random bitstring" + (f" ({un[0]:.4%})" if same_n else ""),
               edgecolor=SURFACE, linewidth=2)
        for xi, v, u in zip(x, qa, un):
            ax.annotate(f"{v:.2%}", (xi - w / 2, v), xytext=(0, 3),
                        textcoords="offset points", ha="center", fontsize=8, color=INK_2)
            if not same_n:
                ax.annotate(f"{u:.4%}", (xi + w / 2, u), xytext=(0, 3),
                            textcoords="offset points", ha="center", fontsize=8, color=INK_2)
        ax.set_yscale("log")
        positive = [v for v in qa + un if v > 0]
        ax.set_ylim(min(positive) / 5, max(positive) * 15)  # headroom for the legend
        ax.set_xticks(x, labels)
        _style(ax, title, "Probability per shot (log)", xlabel="")
        ax.legend(frameon=False, fontsize=8, loc="upper right")
    path = out / "qaoa_sampling_vs_uniform.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def plot_cost_map(cost_map, out):
    if cost_map is None:
        return None
    fig, axes = _figure(ncols=3, width=13)
    h, w = cost_map.shape
    extent = [cost_map.origin_x, cost_map.origin_x + w * cost_map.resolution,
              cost_map.origin_y, cost_map.origin_y + h * cost_map.resolution]
    layers = [
        (cost_map.slope_deg, "Slope (deg)", "Blues"),
        (cost_map.shadow, "Cast shadow (1 = dark)", "Greys"),
        (np.where(np.isfinite(cost_map.cell_cost()), cost_map.cell_cost(), np.nan),
         "Cost per metre (white = impassable)", "Blues"),
    ]
    for ax, (data, title, cmap) in zip(axes, layers):
        im = ax.imshow(data, origin="lower", extent=extent, cmap=cmap)
        _style(ax, title, "y (m)", "x (m)")
        ax.grid(False)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    path = out / "cost_map_layers.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def make_plots(df: pd.DataFrame, cost_map, out: Path):
    out = Path(out)
    paths = [plot_approx_ratio(df, out), plot_runtime(df, out), plot_resources(df, out),
             plot_qaoa_distribution(df, out), plot_cost_map(cost_map, out)]
    return [p for p in paths if p is not None]


if __name__ == "__main__":
    csv = Path(sys.argv[1] if len(sys.argv) > 1 else "results/multi_site/benchmark_raw.csv")
    for p in make_plots(pd.read_csv(csv), None, csv.parent):
        print(p)
