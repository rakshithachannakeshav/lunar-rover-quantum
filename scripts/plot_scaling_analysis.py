#!/usr/bin/env python3
"""
plot_scaling_analysis.py
------------------------
Visualizes quantum resource scaling for lunar-rover mission planning:
1. Qubits vs N (One-hot N^2 vs Compact ceil(log2 N!) vs Orienteering N vs Windowed).
2. Circuit 2-qubit gate / CX scaling vs N.
3. Statevector simulation memory vs N (complex128: 16 bytes per amplitude).

Outputs: results/scaling_analysis.png and docs/scaling_analysis.png
"""

import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Palette consistent with project
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SLOTS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#6250d6", "#e34948"]


def _style(ax, title, ylabel, xlabel="Number of sites (N)"):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=11, color=INK, pad=10, fontweight="bold")
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)
    ax.grid(True, color=GRID, linewidth=0.6, linestyle="--")
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8)


def generate_scaling_plot(out_paths):
    ns = np.arange(3, 15)

    # 1. Qubits
    q_one_hot = ns ** 2
    q_compact = [math.ceil(math.log2(math.factorial(n))) for n in ns]
    q_orienteering = ns
    q_windowed = [9 if n >= 3 else n**2 for n in ns]  # 3-site window -> 3^2 = 9 qubits

    # 2. Statevector Memory (bytes) = 2^q * 16 bytes (complex128)
    # Clamp extreme exponents for plotting
    mem_one_hot = [16.0 * (2.0 ** float(q)) for q in q_one_hot]
    mem_compact = [16.0 * (2.0 ** float(q)) for q in q_compact]
    mem_orienteering = [16.0 * (2.0 ** float(q)) for q in q_orienteering]

    # 3. CX gates (approximate for p=1)
    # One-hot full QUBO has O(N^4) quadratic penalty terms + leg cost pairs:
    # Specifically, pairwise cost terms between consecutive time slots: N * N*(N-1) terms
    # + time conflict penalties: N * (N*(N-1)/2)
    # + site conflict penalties: N * (N*(N-1)/2)
    # Total quadratic terms ~ N^3 * 2. Each quadratic ZZ term in QAOA takes 2 CX gates.
    cx_one_hot_p1 = [2 * (n * (n * (n - 1)) + 2 * n * (n * (n - 1) // 2)) for n in ns]
    # Compact QAOA (standard X mixer, lookup-table cost compilation would take O(N! * log N!) gates on hardware)
    # For full circuit synthesis, diagonal cost operator on q qubits requires ~ 2 * 2^q CNOTs without shortcut!
    cx_compact_synth_p1 = [2 * (2 ** q) for q in q_compact]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), facecolor=SURFACE)

    # Subplot 1: Qubits vs N
    ax1 = axes[0]
    ax1.plot(ns, q_one_hot, label="Full TSP One-Hot ($N^2$)", color=SLOTS[0], marker="o", linewidth=2)
    ax1.plot(ns, q_compact, label=r"Compact TSP ($\lceil\log_2 N!\rceil$)", color=SLOTS[1], marker="s", linewidth=2)
    ax1.plot(ns, q_orienteering, label="Orienteering ($N$)", color=SLOTS[2], marker="^", linewidth=2)
    ax1.plot(ns, q_windowed, label="Windowed QAOA (9 qubits)", color=SLOTS[3], linestyle=":", linewidth=2)
    ax1.axhline(20, color=SLOTS[6], linestyle="--", alpha=0.7, label="20-Qubit Sim Cap")
    _style(ax1, "Qubit Requirements vs Problem Size", "Qubits (linear scale)")
    ax1.set_yscale("log")
    ax1.set_ylim(2, 250)
    ax1.legend(frameon=False, fontsize=8, loc="upper left")

    # Subplot 2: Circuit 2-Qubit Gates (p=1)
    ax2 = axes[1]
    ax2.plot(ns, cx_one_hot_p1, label="One-Hot QUBO (Analytical ZZ pairs)", color=SLOTS[0], marker="o", linewidth=2)
    ax2.plot(ns, cx_compact_synth_p1, label="Compact Gate Synthesis (Worst-case)", color=SLOTS[1], marker="s", linestyle="--", linewidth=2)
    _style(ax2, "Two-Qubit CX Gates (p=1)", "Estimated CX Gate Count (log scale)")
    ax2.set_yscale("log")
    ax2.legend(frameon=False, fontsize=8, loc="upper left")

    # Subplot 3: Statevector Simulation Memory
    ax3 = axes[2]
    ax3.plot(ns, mem_one_hot, label="One-Hot ($N^2$ qubits)", color=SLOTS[0], marker="o", linewidth=2)
    ax3.plot(ns, mem_compact, label=r"Compact ($\lceil\log_2 N!\rceil$ qubits)", color=SLOTS[1], marker="s", linewidth=2)
    ax3.plot(ns, mem_orienteering, label="Orienteering ($N$ qubits)", color=SLOTS[2], marker="^", linewidth=2)
    
    # Reference thresholds
    ax3.axhline(16 * 1024 * 1024, color=MUTED, linestyle="--", alpha=0.8)
    ax3.text(3.2, 25 * 1024 * 1024, "16 MB (20 qubits)", fontsize=7.5, color=INK_2)
    ax3.axhline(16 * (1024**3), color=SLOTS[6], linestyle="--", alpha=0.8)
    ax3.text(3.2, 25 * (1024**3), "16 GB (30 qubits - RAM limit)", fontsize=7.5, color=SLOTS[6])

    _style(ax3, "Statevector Memory vs Problem Size", "RAM Required (Bytes, log scale)")
    ax3.set_yscale("log")
    ax3.set_ylim(1e2, 1e22)
    
    # Custom y-ticks for memory
    yticks = [1e3, 1e6, 1e9, 1e12, 1e15, 1e18, 1e21]
    yticklabels = ["1 KB", "1 MB", "1 GB", "1 TB", "1 PB", "1 EB", "1 ZB"]
    ax3.set_yticks(yticks)
    ax3.set_yticklabels(yticklabels)
    ax3.legend(frameon=False, fontsize=8, loc="lower right")

    fig.tight_layout()
    for p in out_paths:
        p.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(p, dpi=160, facecolor=SURFACE)
        print(f"Saved scaling plot to: {p}")
    plt.close(fig)


if __name__ == "__main__":
    out1 = Path("results/scaling_analysis.png")
    out2 = Path("docs/scaling_analysis.png")
    generate_scaling_plot([out1, out2])

