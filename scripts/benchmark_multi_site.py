#!/usr/bin/env python3
"""
benchmark_multi_site.py
-----------------------
QAOA vs classical baselines on the multi-site visiting-order problem.

For every N in [--n-min, --n-max] and every seed: sample N science sites on the
slope/shadow/uphill cost map of the simulation heightmap, build the (asymmetric)
A* leg-cost matrix, then solve the order with

  held_karp          exact reference (every N)
  brute_force        exact, N <= --brute-force-max (cross-check + runtime)
  nearest_neighbour  greedy
  nn_2opt            nearest neighbour + 2-opt to a local optimum
  window_exact       NN tour, sliding windows re-ordered by enumeration
  random_mean        mean cost of 1000 random tours (scale reference)
and, for every CVaR alpha in --cvar-alpha:
  qaoa_p{p}_a{alpha}        full N^2-qubit QAOA, only if N^2 <= --max-qubits
  best_of_random@qaoa_p{p}_a{alpha}
                            control: best of k uniform random tours, k = the
                            number of feasible QAOA shots (equal samples)
  qaoa_window_a{alpha}      NN tour, sliding windows re-ordered by QAOA (p = first --reps)
Full-QUBO circuits too large to simulate get one `qaoa_p{p}` row per (N, p)
with status=not_simulated and their qubit / CX / depth counts.

Writes results/multi_site/{benchmark_raw.csv, benchmark_summary.csv,
run_config.json, FINDINGS.md, *.png}.

    python scripts/benchmark_multi_site.py                         # full run
    python scripts/benchmark_multi_site.py --n-max 5 --seeds 0 --reps 1 --cvar-alpha 1.0
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "planning_pkg"))

from planning_pkg.cost_map import CostMapConfig, CostWeights  # noqa: E402
from planning_pkg.mission_planner import build_instance, load_cost_map, sample_sites  # noqa: E402
from planning_pkg.tsp_baselines import (  # noqa: E402
    best_of_random,
    brute_force,
    count_optimal_orders,
    held_karp,
    nearest_neighbour,
    nn_two_opt,
    random_tour_stats,
    tour_cost,
    window_reorder_exact,
)
from planning_pkg.tsp_qaoa import (  # noqa: E402
    QaoaConfig,
    circuit_stats,
    solve_tour_qaoa,
    solve_tour_qaoa_windowed,
)
from planning_pkg.tsp_qubo import build_tsp_qubo, count_feasible_states  # noqa: E402

TOL = 1e-9


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--n-min", type=int, default=4)
    p.add_argument("--n-max", type=int, default=8)
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    p.add_argument("--heightmap", default=str(ROOT / "src/rover_simulation/worlds/heightmap.png"))
    p.add_argument("--resolution", type=float, default=1.0)
    p.add_argument("--w-dist", type=float, default=1.0)
    p.add_argument("--w-slope", type=float, default=2.0)
    p.add_argument("--w-shadow", type=float, default=1.5)
    p.add_argument("--w-uphill", type=float, default=2.0)
    p.add_argument("--sun-elevation", type=float, default=3.0)
    p.add_argument("--sun-azimuth", type=float, default=135.0)
    p.add_argument("--min-separation", type=float, default=10.0)
    p.add_argument("--max-radius", type=float, default=60.0)
    p.add_argument("--open", action="store_true", help="Open missions (no return to depot).")
    p.add_argument("--reps", type=int, nargs="+", default=[1, 2], help="QAOA depths p.")
    p.add_argument("--cvar-alpha", type=float, nargs="+", default=[1.0, 0.5, 0.2])
    p.add_argument("--optimizer", default="COBYLA")
    p.add_argument("--maxiter", type=int, default=150)
    p.add_argument("--restarts", type=int, default=2)
    p.add_argument("--shots", type=int, default=4096)
    p.add_argument("--penalty", type=float, default=1.5)
    p.add_argument("--max-qubits", type=int, default=20)
    p.add_argument("--window", type=int, default=3)
    p.add_argument("--brute-force-max", type=int, default=10)
    p.add_argument("--no-window", action="store_true", help="Skip the windowed QAOA runs.")
    p.add_argument("--out", default=str(ROOT / "results" / "multi_site"))
    return p.parse_args(argv)


def alpha_tag(alpha: float) -> str:
    return f"a{alpha:g}"


def run(args):
    closed = not args.open
    config = CostMapConfig(
        resolution_m=args.resolution,
        sun_elevation_deg=args.sun_elevation,
        sun_azimuth_deg=args.sun_azimuth,
        weights=CostWeights(args.w_dist, args.w_slope, args.w_shadow, args.w_uphill),
    )
    cost_map, graph = load_cost_map(args.heightmap, config=config)
    print(f"cost map {cost_map.shape} @ {args.resolution} m, {graph.number_of_nodes()} nodes, "
          f"shadow {cost_map.shadow.mean():.0%}", flush=True)

    stats_cache = {}
    rows = []
    for n in range(args.n_min, args.n_max + 1):
        for seed in args.seeds:
            sites = sample_sites(graph, n, seed, min_separation_m=args.min_separation,
                                 max_radius_m=args.max_radius)
            inst = build_instance(graph, cost_map, (0.0, 0.0), sites)
            cost = inst.leg_costs
            exact = held_karp(cost, closed)
            opt = exact.cost
            n_opt = count_optimal_orders(cost, closed)
            base = {"n_sites": n, "seed": seed, "closed": closed, "optimal_cost": opt,
                    "n_optimal_orders": n_opt, "astar_legs_s": inst.astar_runtime_s,
                    "leg_asymmetry": float(np.abs(cost - cost.T).max() / cost.max())}

            def add(method, result=None, **extra):
                row = dict(base, method=method, status="ok")
                if result is not None:
                    row.update(tour_cost=result.cost, runtime_s=result.runtime_s,
                               tour=" ".join(map(str, result.order)))
                row.update(extra)
                if row.get("tour_cost") == row.get("tour_cost") and "tour_cost" in row:
                    row["approx_ratio"] = row["tour_cost"] / opt
                    row["gap_pct"] = 100.0 * (row["tour_cost"] - opt) / opt
                rows.append(row)
                print(f"N={n} seed={seed} {method:<30} cost={row.get('tour_cost', float('nan')):9.2f} "
                      f"ratio={row.get('approx_ratio', float('nan')):.4f} "
                      f"t={row.get('runtime_s', float('nan')):.3f}s {row['status']}", flush=True)
                return row

            add("held_karp", exact)
            if n <= args.brute_force_max:
                bf = brute_force(cost, closed)
                assert abs(bf.cost - opt) <= TOL * max(1.0, opt), "brute force != Held-Karp"
                add("brute_force", bf)
            nn = nearest_neighbour(cost, closed)
            add("nearest_neighbour", nn)
            add("nn_2opt", nn_two_opt(cost, closed))
            add("random_mean", tour_cost=random_tour_stats(cost, closed, seed=seed)["mean"])
            wx = window_reorder_exact(cost, nn.order, closed, args.window)
            add("window_exact", wx, seed_cost=nn.cost, improved_seed=wx.cost < nn.cost - TOL)

            qubits = n * n
            for p in args.reps:
                if qubits > args.max_qubits:
                    if (n, p) not in stats_cache:
                        tq = build_tsp_qubo(cost, range(1, n + 1), 0, 0 if closed else None)
                        stats_cache[(n, p)] = circuit_stats(tq, p)
                    s = stats_cache[(n, p)]
                    add(f"qaoa_p{p}", status="not_simulated", p=p, qubits=qubits,
                        circuit_depth=s["depth"], two_qubit_gates=s["two_qubit_gates"])

            for alpha in args.cvar_alpha:
                tag = alpha_tag(alpha)
                for p in args.reps:
                    if qubits > args.max_qubits:
                        continue
                    cfg = QaoaConfig(reps=p, optimizer=args.optimizer, maxiter=args.maxiter,
                                     restarts=args.restarts, shots=args.shots, seed=seed,
                                     penalty=args.penalty, cvar_alpha=alpha,
                                     max_qubits=args.max_qubits)
                    q = solve_tour_qaoa(cost, closed, cfg)
                    info = q.info
                    optimal_shots = sum(
                        c for t, c in info["tour_counts"].items()
                        if tour_cost(cost, [int(v) for v in t.split(",")], closed)
                        <= opt + TOL * max(1.0, opt))
                    feasible_shots = int(round(info["p_feasible"] * args.shots))
                    uniform_p_opt = n_opt / 2.0 ** qubits
                    p_opt = optimal_shots / args.shots
                    ctrl = best_of_random(cost, max(1, feasible_shots), closed, seed=seed)
                    add(f"qaoa_p{p}_{tag}", q, p=p, cvar_alpha=alpha, optimizer=args.optimizer,
                        shots=args.shots, qubits=qubits, circuit_depth=info["depth"],
                        two_qubit_gates=info["two_qubit_gates"], p_feasible=info["p_feasible"],
                        uniform_p_feasible=count_feasible_states(n) / 2.0 ** qubits,
                        p_optimal=p_opt, uniform_p_optimal=uniform_p_opt,
                        amplification=p_opt / uniform_p_opt,
                        mean_feasible_ratio=info["mean_feasible_cost"] / opt,
                        distinct_feasible_tours=len(info["tour_counts"]),
                        repaired=info["repaired"], n_evals=info["n_evals"],
                        control_samples=ctrl.info["samples"],
                        ratio_vs_best_of_random=q.cost / ctrl.cost)
                    add(f"best_of_random@qaoa_p{p}_{tag}", ctrl, p=p, cvar_alpha=alpha,
                        samples=ctrl.info["samples"])

                if not args.no_window:
                    p = args.reps[0]
                    cfg = QaoaConfig(reps=p, optimizer=args.optimizer, maxiter=args.maxiter,
                                     restarts=args.restarts, shots=args.shots, seed=seed,
                                     penalty=args.penalty, cvar_alpha=alpha,
                                     max_qubits=args.max_qubits)
                    w = solve_tour_qaoa_windowed(cost, nn.order, closed, cfg, window=args.window)
                    i = w.info
                    add(f"qaoa_window_{tag}", w, p=p, cvar_alpha=alpha, qubits=i["qubits"],
                        circuit_depth=i["depth"], two_qubit_gates=i["two_qubit_gates"],
                        p_feasible=i["p_feasible"], repaired=i["repaired_windows"] > 0,
                        windows_solved=i["windows_solved"], n_evals=i["n_evals"],
                        seed_cost=nn.cost, improved_seed=w.cost < nn.cost - TOL,
                        matches_window_exact=abs(w.cost - wx.cost) <= TOL * max(1.0, opt),
                        ratio_vs_window_exact=w.cost / wx.cost)
    return pd.DataFrame(rows), cost_map


def summarise(df):
    df = df.copy()
    for col in ("qubits", "circuit_depth", "p_feasible", "p_optimal", "amplification",
                "ratio_vs_best_of_random", "improved_seed"):
        if col not in df:
            df[col] = np.nan
    ok = df[df["status"] == "ok"]
    agg = ok.groupby(["n_sites", "method"]).agg(
        runs=("seed", "count"),
        approx_ratio_mean=("approx_ratio", "mean"),
        approx_ratio_max=("approx_ratio", "max"),
        gap_pct_mean=("gap_pct", "mean"),
        optimal_hits=("gap_pct", lambda g: int((g.abs() < 1e-6).sum())),
        runtime_s_mean=("runtime_s", "mean"),
        p_feasible_mean=("p_feasible", "mean"),
        p_optimal_mean=("p_optimal", "mean"),
        amplification_mean=("amplification", "mean"),
        ratio_vs_best_of_random_mean=("ratio_vs_best_of_random", "mean"),
        improved_seed_count=("improved_seed", lambda s: int((s == True).sum())),  # noqa: E712
    ).reset_index()
    sizes = df.groupby(["n_sites", "method"]).agg(
        qubits=("qubits", "max"), circuit_depth=("circuit_depth", "max"),
        two_qubit_gates=("two_qubit_gates", "max") if "two_qubit_gates" in df else ("seed", "size"),
        status=("status", "first"),
    ).reset_index()
    return agg.merge(sizes, on=["n_sites", "method"], how="outer").sort_values(
        ["n_sites", "method"])


# ----------------------------------------------------------------------------- findings

def _fmt(x, spec=".3f"):
    return "-" if x is None or x != x else format(x, spec)


def _table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def write_findings(df: pd.DataFrame, args, out: Path, wall_s: float) -> Path:
    ok = df[df["status"] == "ok"].copy()
    key = ["n_sites", "seed"]
    nn2 = ok[ok["method"] == "nn_2opt"].set_index(key)["tour_cost"]
    nn2_t = ok[ok["method"] == "nn_2opt"].set_index(key)["runtime_s"]
    n_seeds = len(args.seeds)
    full = ok[ok["method"].str.match(r"qaoa_p\d+_a")]
    win = ok[ok["method"].str.startswith("qaoa_window_")]
    wx = ok[ok["method"] == "window_exact"].set_index(key)
    lines = []
    lines.append("# Multi-site visiting order: QAOA vs classical baselines\n")
    lines.append(
        f"Generated by `scripts/benchmark_multi_site.py` on {datetime.now():%Y-%m-%d %H:%M} "
        f"({wall_s / 60:.0f} min). N = {args.n_min}..{args.n_max}, seeds {args.seeds}, "
        f"{'closed tours (return to depot)' if not args.open else 'open missions'}. "
        f"Leg costs: A* on the simulation heightmap cost map (w_dist={args.w_dist}, "
        f"w_slope={args.w_slope}, w_shadow={args.w_shadow}, w_uphill={args.w_uphill}), "
        f"so legs are **asymmetric** (uphill != downhill; median max |c_ij - c_ji| / max c = "
        f"{ok.groupby(key)['leg_asymmetry'].first().median():.2f}).\n")
    lines.append(
        f"QAOA: Qiskit-Aer statevector training ({args.optimizer}, maxiter {args.maxiter}, "
        f"{args.restarts} random restarts + grid/INTERP start), {args.shots} shots, "
        f"p in {args.reps}, CVaR alpha in {args.cvar_alpha}, penalty {args.penalty}. "
        f"The full QUBO uses N^2 qubits; only N^2 <= {args.max_qubits} is simulated "
        f"(N <= {int(args.max_qubits ** 0.5)}). The answer is the cheapest feasible sampled tour.\n")

    # ---- where QAOA loses (plain statements, computed) ----
    lines.append("## Where QAOA loses\n")
    bullets = []
    if not full.empty:
        f = full.join(nn2.rename("nn2"), on=key).join(nn2_t.rename("nn2_t"), on=key)
        worse = int((f["tour_cost"] > f["nn2"] + TOL).sum())
        better = int((f["tour_cost"] < f["nn2"] - TOL).sum())
        sim_n = sorted(int(v) for v in f["n_sites"].unique())
        hk = ok[ok["method"] == "held_karp"].set_index(key)["tour_cost"]
        inst = nn2.index[nn2.index.get_level_values(0).isin(sim_n)]
        nn2_opt = int(((nn2.loc[inst] - hk.loc[inst]).abs() <= TOL * hk.loc[inst]).sum())
        verdict = ("never beats" if better == 0 else f"beats on only {better} runs and otherwise does not beat")
        bullets.append(
            f"**Full QAOA {verdict} NN + 2-opt**: worse on {worse} of {len(f)} runs, "
            f"better on {better}, tied on {len(f) - worse - better}. NN + 2-opt alone is "
            f"already optimal on {nn2_opt} of {len(inst)} instances at N = "
            f"{', '.join(map(str, sim_n))}.")
        slow = (f["runtime_s"] / f["nn2_t"].clip(lower=1e-6)).median()
        bullets.append(
            f"**Runtime:** full QAOA takes {f['runtime_s'].median():.1f} s median per instance "
            f"vs {f['nn2_t'].median() * 1e3:.2f} ms for NN + 2-opt "
            f"(~{slow:,.0f}x slower), on a simulator, before any hardware queue or noise.")
        bvr = f["ratio_vs_best_of_random"]
        bullets.append(
            f"**Against random sampling with the same number of candidates** "
            f"(best of k uniform random tours, k = QAOA's feasible shots): QAOA is better in "
            f"{int((bvr < 1 - TOL).sum())}, equal in {int((bvr.sub(1).abs() <= TOL).sum())}, "
            f"worse in {int((bvr > 1 + TOL).sum())} of {len(f)} runs. At N = {sim_n[-1]} there are "
            f"only {count_feasible_states(sim_n[-1])} orders, so a few dozen feasible samples "
            f"already hit the optimum by chance; QAOA's best-sample cost is not evidence of "
            f"quantum advantage.")
        mfr = f["mean_feasible_ratio"].median()
        rnd = ok[ok["method"] == "random_mean"]
        rnd = rnd[rnd["n_sites"].isin(f["n_sites"].unique())]["approx_ratio"].median()
        near_random = mfr >= rnd - 0.02
        bullets.append(
            ("**Tour quality inside the QAOA distribution is near random:** "
             if near_random else "**Tour quality inside the QAOA distribution:** ")
            + f"the mean feasible sampled tour is {mfr:.3f}x optimal (median over runs) vs "
            f"{rnd:.3f}x for a uniformly random tour."
            + (" QAOA mostly learns the constraints (feasibility), not the cost."
               if near_random else ""))
        bullets.append(
            f"**Scaling:** the full QUBO needs N^2 qubits - "
            + ", ".join(f"N={n}: {n * n}" for n in range(args.n_min, args.n_max + 1))
            + f". Nothing above N = {int(args.max_qubits ** 0.5)} was simulated; "
            "see `qaoa_resources_vs_n.png` for CX counts (all-to-all connectivity, no routing).")
    if not win.empty:
        w = win.join(nn2.rename("nn2"), on=key)
        worse_w = int((w["tour_cost"] > w["nn2"] + TOL).sum())
        better_w = int((w["tour_cost"] < w["nn2"] - TOL).sum())
        bullets.append(
            f"**QAOA windows vs NN + 2-opt:** worse on {worse_w}, better on {better_w}, "
            f"equal on {len(w) - worse_w - better_w} of {len(w)} runs.")
        same = int(w["matches_window_exact"].astype(bool).sum())
        bullets.append(
            ("**QAOA windows add nothing over the decomposition itself:** "
             if same == len(w) else "**QAOA windows vs the same decomposition solved exactly:** ")
            + f"the same windows "
            f"solved by enumerating {args.window}! orders (`window_exact`, microseconds) reach the "
            f"same cost on {same} of {len(w)} runs"
            + (f" and a better one on {len(w) - same}." if same < len(w) else "."))
    lines += [f"- {b}" for b in bullets]
    lines.append("")

    # ---- table: full QAOA ----
    if not full.empty:
        lines.append("## Full QAOA (N^2 qubits) by depth p and CVaR alpha\n")
        rows = []
        for (n, p, a), g in full.groupby(["n_sites", "p", "cvar_alpha"]):
            g = g.join(nn2.rename("nn2"), on=key)
            rows.append([
                int(n), int(p), f"{a:g}",
                f"{int((g['gap_pct'].abs() < 1e-6).sum())}/{len(g)}",
                _fmt(g["approx_ratio"].mean(), ".4f"),
                _fmt(g["p_feasible"].mean(), ".4f"),
                _fmt(g["p_optimal"].mean(), ".5f"),
                _fmt(g["uniform_p_optimal"].mean(), ".2e"),
                _fmt(g["amplification"].mean(), ",.0f"),
                _fmt((g["p_feasible"] / g["uniform_p_feasible"]).mean(), ",.0f"),
                _fmt((g["p_optimal"] / g["p_feasible"].where(g["p_feasible"] > 0)
                      / (g["n_optimal_orders"] / count_feasible_states(int(n)))).mean(), ".2f"),
                _fmt(g["mean_feasible_ratio"].mean(), ".3f"),
                _fmt(g["ratio_vs_best_of_random"].mean(), ".4f"),
                f"{int((g['tour_cost'] > g['nn2'] + TOL).sum())}/{len(g)}",
                _fmt(g["runtime_s"].mean(), ".1f"),
            ])
        lines.append(_table(
            ["N", "p", "alpha", "optimal", "ratio", "P(feasible)", "P(optimal)",
             "uniform P(opt)", "amplification", "feasible enrichment", "conditional amp.",
             "mean feasible ratio", "vs best-of-random",
             "worse than NN+2opt", "time (s)"], rows))
        lines.append(
            "\n`amplification` = P(optimal) / uniform P(optimal), where uniform is "
            "(number of optimal orders) / 2^(N^2) - the chance a random bitstring is an optimal "
            "tour. It mixes two effects, so they are split out: `feasible enrichment` = "
            "P(feasible) / (N!/2^(N^2)) and `conditional amp.` = P(optimal | feasible) / "
            "(optimal orders / N!). A conditional amplification near 1 means that, once a sample "
            "is a valid order, QAOA picks the optimal one no more often than a random valid "
            "order would - the gain is in satisfying the constraints, not in the tour cost.\n")

    # ---- table: windows ----
    if not win.empty:
        lines.append(f"## QAOA windows (window = {args.window} sites, NN seed) vs classical\n")
        rows = []
        for (n, a), g in win.groupby(["n_sites", "cvar_alpha"]):
            g = g.join(nn2.rename("nn2"), on=key)
            ex = wx.loc[g.set_index(key).index]
            rows.append([
                int(n), f"{a:g}",
                f"{int(g['improved_seed'].astype(bool).sum())}/{len(g)}",
                f"{int(ex['improved_seed'].astype(bool).sum())}/{len(g)}",
                _fmt(g["approx_ratio"].mean(), ".4f"),
                _fmt(ex["approx_ratio"].mean(), ".4f"),
                _fmt(g["nn2"].div(g["optimal_cost"]).mean(), ".4f"),
                f"{int(g['matches_window_exact'].astype(bool).sum())}/{len(g)}",
                f"{int((g['tour_cost'] > g['nn2'] + TOL).sum())}/{len(g)}",
                _fmt(g["p_feasible"].mean(), ".3f"),
                _fmt(g["runtime_s"].mean(), ".1f"),
            ])
        lines.append(_table(
            ["N", "alpha", "QAOA improved NN seed", "exact-window improved NN seed",
             "QAOA-window ratio", "exact-window ratio", "NN+2opt ratio",
             "QAOA = exact window", "QAOA worse than NN+2opt", "P(feasible)", "time (s)"],
            rows))
        lines.append("")

    # ---- table: classical ----
    lines.append("## Classical baselines (approximation ratio, mean / worst; optimal hits)\n")
    rows = []
    for n, g in ok.groupby("n_sites"):
        cells = [int(n)]
        for m in ("nearest_neighbour", "nn_2opt", "window_exact", "random_mean"):
            s = g[g["method"] == m]
            hits = int((s["gap_pct"].abs() < 1e-6).sum())
            cells.append(f"{s['approx_ratio'].mean():.4f} / {s['approx_ratio'].max():.4f}"
                         + ("" if m == "random_mean" else f" ({hits}/{len(s)})"))
        hk = g[g["method"] == "held_karp"]["runtime_s"].mean()
        cells.append(f"{hk * 1e3:.1f}")
        rows.append(cells)
    lines.append(_table(["N", "nearest neighbour", "NN + 2-opt", "exact windows",
                         "random tour (mean)", "Held-Karp ms"], rows))
    lines.append(
        "\nAll exact numbers are Held-Karp, cross-checked against brute force for every "
        "instance. No baseline was weakened: 2-opt runs to a local optimum with "
        "direction-aware move pricing; Held-Karp and brute force are exact on the asymmetric "
        "matrices.\n")
    path = out / "FINDINGS.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def main(argv=None):
    args = parse_args(argv)
    if not 1 <= args.n_min <= args.n_max <= 10:
        raise SystemExit("--n-min/--n-max must satisfy 1 <= n_min <= n_max <= 10")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    df, cost_map = run(args)
    wall = time.perf_counter() - started
    df.to_csv(out / "benchmark_raw.csv", index=False)
    summarise(df).to_csv(out / "benchmark_summary.csv", index=False)
    with open(out / "run_config.json", "w", encoding="utf-8") as f:
        json.dump({**vars(args), "generated_at": datetime.now().isoformat(),
                   "wall_time_s": wall}, f, indent=2)
    print(f"findings: {write_findings(df, args, out, wall)}")

    from plot_multi_site import make_plots  # same directory
    for path in make_plots(df, cost_map, out):
        print(f"plot: {path}")
    print(f"wrote {out / 'benchmark_raw.csv'} ({len(df)} rows) in {wall:.0f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
