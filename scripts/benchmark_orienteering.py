#!/usr/bin/env python3
"""
benchmark_orienteering.py
-------------------------
Which science sites to visit under a battery budget: exact vs classical
heuristics vs QAOA (n qubits, one per site).

Per (source, mode, n, seed): build a leg-cost matrix, seed rewards and a budget
(a fraction of the cheapest tour through ALL sites), then solve with

  exact_dp              the optimum (one DP gives every subset's cheapest trip)
  greedy                best reward / extra-time first
  greedy_local_search   greedy + add / drop / swap moves
  best_of_random        best feasible of k random subsets (k = QAOA shots)
  qaoa_p{p}_a{alpha}    QAOA trained per instance
  qaoa_transfer_...     angles averaged over the smallest-n instances and reused
                        on larger n with NO training (parameter transfer)

sources: synthetic_clustered, terrain_clustered, terrain_random
modes:   plain (energy budget only) | windows (sites only lit for a window)

QAOA here is simulated with the exact subset-cost table as a diagonal (see
planning_pkg.qaoa_sim): a simulation shortcut. Writes
results/orienteering/{benchmark_raw.csv, benchmark_summary.csv, FINDINGS.md,
run_config.json}.

    python scripts/benchmark_orienteering.py                          # full
    python scripts/benchmark_orienteering.py --ns 6 8 --seeds 0 1 --ps 1 --sources synthetic_clustered   # quick
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
sys.path.insert(0, str(Path(__file__).resolve().parent))

from planning_pkg.instance_sets import synthetic_cost_matrix, terrain_cost_matrix  # noqa: E402
from planning_pkg.orienteering import (  # noqa: E402
    best_of_random_subsets,
    build_tables,
    greedy,
    greedy_local_search,
    make_instance,
    solve_exact,
)
from planning_pkg.orienteering_qaoa import run_orienteering_qaoa  # noqa: E402
from planning_pkg.qaoa_sim import shots_for_confidence  # noqa: E402
from report_utils import md_table, sci  # noqa: E402

TOL = 1e-9


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--sources", nargs="+",
                   default=["synthetic_clustered", "terrain_clustered"],
                   choices=["synthetic_clustered", "terrain_clustered", "terrain_random"])
    p.add_argument("--modes", nargs="+", default=["plain", "windows"],
                   choices=["plain", "windows"])
    p.add_argument("--ns", type=int, nargs="+", default=[6, 8, 10, 12, 14])
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    p.add_argument("--budget-frac", type=float, default=0.5)
    p.add_argument("--ps", type=int, nargs="+", default=[1, 2, 3])
    p.add_argument("--alphas", type=float, nargs="+", default=[1.0, 0.5])
    p.add_argument("--shots", type=int, default=4096)
    p.add_argument("--maxiter", type=int, default=80)
    p.add_argument("--restarts", type=int, default=1)
    p.add_argument("--heightmap", default=str(ROOT / "src/rover_simulation/worlds/heightmap.png"))
    p.add_argument("--w-uphill", type=float, default=30.0)
    p.add_argument("--asym", type=float, default=8.0, help="synthetic uphill weight")
    p.add_argument("--relief", type=float, default=10.0, help="synthetic relief height")
    p.add_argument("--out", default=str(ROOT / "results" / "orienteering"))
    return p.parse_args(argv)


def cost_function(source: str, args, cache: dict):
    if source == "synthetic_clustered":
        return lambda n, seed: synthetic_cost_matrix(
            n, seed, clusters=3, asym=args.asym, relief=args.relief)
    from planning_pkg.cost_map import CostMapConfig, CostWeights
    from planning_pkg.mission_planner import load_cost_map
    if "terrain" not in cache:
        cfg = CostMapConfig(resolution_m=1.0, sun_elevation_deg=3.0, sun_azimuth_deg=135.0,
                            weights=CostWeights(1.0, 2.0, 1.5, args.w_uphill))
        cache["terrain"] = load_cost_map(args.heightmap, config=cfg)
    cm, graph = cache["terrain"]
    return lambda n, seed: terrain_cost_matrix(graph, cm, n, seed, family=source)


def run(args) -> pd.DataFrame:
    rows, skipped = [], 0
    cache: dict = {}
    n_min = min(args.ns)
    for source in args.sources:
        make_cost = cost_function(source, args, cache)
        for mode in args.modes:
            train_params: dict = {}
            for n in sorted(args.ns):
                for seed in args.seeds:
                    try:
                        cost = make_cost(n, seed)
                        inst = make_instance(cost, seed, args.budget_frac, windows=(mode == "windows"))
                    except ValueError as exc:
                        skipped += 1
                        print(f"skip {source}/{mode} n={n} seed={seed}: {exc}", flush=True)
                        continue
                    t0 = time.perf_counter()
                    tab = build_tables(inst)
                    ex = solve_exact(inst, tab)
                    dp_s = time.perf_counter() - t0
                    base = {"source": source, "mode": mode, "n": n, "seed": seed,
                            "budget_frac": args.budget_frac, "opt_reward": ex["reward"],
                            "n_optimal": ex["n_optimal"], "n_feasible": ex["n_feasible"],
                            "feasible_frac": ex["n_feasible"] / (1 << n), "exact_dp_s": dp_s}

                    def add(method, **kw):
                        row = dict(base, method=method)
                        row.update(kw)
                        if "reward" in row and "reward_ratio" not in row:
                            row["reward_ratio"] = row["reward"] / ex["reward"]
                        rows.append(row)

                    add("exact_dp", reward=ex["reward"], evals=1 << n)
                    g = greedy(inst, tab)
                    add("greedy", reward=g.reward, evals=g.evals)
                    gl = greedy_local_search(inst, tab)
                    add("greedy_local_search", reward=gl.reward, evals=gl.evals)
                    rnd = best_of_random_subsets(inst, args.shots, seed, tab)
                    add("best_of_random", reward=rnd.reward, evals=rnd.evals)

                    uniform_shots = shots_for_confidence(ex["n_optimal"] / (1 << n))
                    for p in args.ps:
                        for alpha in args.alphas:
                            r = run_orienteering_qaoa(inst, tab, ex, p, alpha, args.shots, seed,
                                                      args.maxiter, args.restarts)
                            add(f"qaoa_p{p}_a{alpha:g}", p=p, alpha=alpha,
                                uniform_shots_to_solution=uniform_shots,
                                **{k: v for k, v in r.items()
                                   if k not in ("params", "p", "alpha", "shots", "qubits")})
                            if n == n_min:
                                train_params.setdefault((p, alpha), []).append(r["params"])
                            elif (p, alpha) in train_params:
                                avg = np.mean(train_params[(p, alpha)], axis=0)
                                t = run_orienteering_qaoa(inst, tab, ex, p, alpha, args.shots,
                                                          seed, params=avg)
                                add(f"qaoa_transfer_p{p}_a{alpha:g}", p=p, alpha=alpha,
                                    uniform_shots_to_solution=uniform_shots,
                                    **{k: v for k, v in t.items()
                                       if k not in ("params", "p", "alpha", "shots", "qubits")})
                    print(f"{source}/{mode} n={n} seed={seed}: opt={ex['reward']:.0f} "
                          f"(x{ex['n_optimal']}), feasible {ex['n_feasible']}/{1 << n}, "
                          f"greedy={g.reward:.0f}, greedy+LS={gl.reward:.0f}, DP {dp_s:.2f}s",
                          flush=True)
    df = pd.DataFrame(rows)
    df.attrs["skipped"] = skipped
    return df


def summarise(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["hit"] = df["reward_ratio"] >= 1.0 - TOL
    cols = {c: (c, "mean") for c in
            ["reward_ratio", "p_opt", "uniform_p_opt", "amplification", "conditional_amp",
             "feasible_enrichment", "shots_to_solution", "uniform_shots_to_solution",
             "evals", "n_evals"] if c in df}
    agg = df.groupby(["source", "mode", "n", "method"]).agg(
        runs=("seed", "count"), optimal_hits=("hit", "sum"), **cols).reset_index()
    return agg.sort_values(["source", "mode", "n", "method"])


def write_findings(df: pd.DataFrame, summary: pd.DataFrame, args, wall_s: float) -> str:
    L = []
    L.append("# Orienteering: which sites to visit under a battery budget\n")
    L.append(f"Generated by `scripts/benchmark_orienteering.py` on "
             f"{datetime.now():%Y-%m-%d %H:%M} ({wall_s / 60:.1f} min). "
             f"n = {sorted(args.ns)}, seeds {args.seeds}, budget = {args.budget_frac:g} x the cheapest "
             f"tour through all sites, sources {args.sources}, modes {args.modes}. "
             f"Rewards are made-up integers 1..10. Uphill weight: terrain {args.w_uphill:g}, "
             f"synthetic {args.asym:g}.\n")
    L.append(f"QAOA: numpy statevector, one qubit per site, p in {args.ps}, CVaR alpha in "
             f"{args.alphas}, COBYLA maxiter {args.maxiter}, {args.shots} shots. The cost of every subset "
             f"comes from the exact DP table and is applied as a diagonal: a **simulation shortcut**, "
             f"not a gate-level circuit. 'plain' = energy budget only; 'windows' = each site is only lit "
             f"for a window and time includes waiting.\n")

    ok = df[df["method"].isin(["exact_dp", "greedy", "greedy_local_search"])]
    piv = ok.pivot_table(index=["source", "mode", "n"], columns="method",
                         values="reward_ratio", aggfunc="mean").reset_index()
    hits = ok.assign(hit=ok["reward_ratio"] >= 1 - TOL).pivot_table(
        index=["source", "mode", "n"], columns="method", values="hit", aggfunc="sum").reset_index()
    runs = df[df["method"] == "exact_dp"].groupby(["source", "mode", "n"]).agg(
        runs=("seed", "count"), feasible=("feasible_frac", "mean")).reset_index()
    t1 = runs.merge(piv, on=["source", "mode", "n"]).merge(
        hits, on=["source", "mode", "n"], suffixes=("", "_hits"))
    L.append("## 1. Do the classical heuristics leave room? (reward ratio to the optimum, 1.00 = optimal)\n")
    L.append(_t1(t1))
    miss = int(((df["method"] == "greedy_local_search") & (df["reward_ratio"] < 1 - TOL)).sum())
    tot = int((df["method"] == "greedy_local_search").sum())
    dp_max = df.loc[df["method"] == "exact_dp", "exact_dp_s"].max()
    L.append(f"\nGreedy + local search is suboptimal on {miss} of {tot} instances. The exact DP solves "
             f"every instance in at most {dp_max:.2f} s, so a heuristic miss is not a reason to need "
             f"a quantum method at these sizes.\n")

    q = df[df["method"].str.startswith("qaoa_p")]
    if len(q):
        g = q.groupby(["source", "mode", "n", "method"]).agg(
            p_opt=("p_opt", "mean"), uni=("uniform_p_opt", "mean"), amp=("amplification", "mean"),
            cond=("conditional_amp", "mean"), train=("n_evals", "mean"),
            ratio=("reward_ratio", "mean")).reset_index()
        g["s99"] = g["p_opt"].map(shots_for_confidence)
        g["u99"] = g["uni"].map(shots_for_confidence)
        best = g.sort_values("p_opt", ascending=False).groupby(["source", "mode", "n"]).head(1)
        best = best.sort_values(["source", "mode", "n"])
        best["brute"] = 2.0 ** best["n"]
        L.append("\n## 2. QAOA at its best configuration per setting (exact probabilities, mean over seeds)\n")
        L.append("`P(opt)` = probability one shot is an optimal subset. `amplification` = P(opt) / uniform. "
                 "`conditional amp.` ~ 1 means QAOA picks optimal among FEASIBLE subsets no better than random. "
                 "`QAOA shots` / `random shots` = samples for a 99% chance of seeing an optimum, computed from the mean P(opt), excluding training. "
                 "`brute force` = subset-cost lookups to check all 2^n subsets.\n")
        L.append(md_table(best, [
            ("source", "source", "{}"), ("mode", "mode", "{}"), ("n", "n", "{}"),
            ("method", "best config", "{}"), ("p_opt", "P(opt)", "{:.4f}"),
            ("uni", "uniform P(opt)", "{:.5f}"), ("amp", "amplification", "{:.1f}"),
            ("cond", "conditional amp.", "{:.2f}"), ("s99", "QAOA shots", sci),
            ("u99", "random shots", sci), ("train", "training evals", sci),
            ("brute", "brute force", sci)]))
        beats = int((g["amp"] > 1).sum())
        top_n = int(best["n"].max())
        row = best[best["n"] == top_n]
        train_mean = float(row["train"].mean())
        brute_top = 2.0 ** top_n
        shots_per_eval = 1000
        L.append(f"\nQAOA raised P(optimal) above a uniformly random bitstring in {beats} of {len(g)} "
                 f"(setting, config) groups. That is a gain over random sampling only. For total work, "
                 f"training also counts: at n = {top_n} the best configurations used a mean of "
                 f"{train_mean:,.0f} circuit evaluations to train. If each needs {shots_per_eval:,} shots on "
                 f"hardware (an assumption), that is about {train_mean * shots_per_eval:,.0f} shots, against "
                 f"{brute_top:,.0f} subset-cost lookups for brute force and a single DP pass for the exact "
                 f"answer.\n")

        tr = q.copy()
        tr["transfer"] = tr["transferred"].astype(bool)
        t = tr[tr["n"] > min(args.ns)].groupby(["source", "mode", "n", "p", "alpha", "transfer"])[
            "p_opt"].mean().unstack("transfer").reset_index()
        if {True, False} <= set(t.columns):
            t = t.rename(columns={False: "trained", True: "transfer"})
            t["ratio"] = t["transfer"] / t["trained"]
            L.append("\n## 3. Parameter transfer (angles from the smallest n, reused with no training)\n")
            L.append(md_table(t.sort_values(["source", "mode", "n", "p", "alpha"]), [
                ("source", "source", "{}"), ("mode", "mode", "{}"), ("n", "n", "{}"),
                ("p", "p", "{}"), ("alpha", "alpha", "{:g}"),
                ("trained", "P(opt) trained", "{:.4f}"), ("transfer", "P(opt) transferred", "{:.4f}"),
                ("ratio", "transfer / trained", "{:.2f}")]))
            L.append(f"\nMedian transfer / trained ratio: {t['ratio'].median():.2f} "
                     f"(1.00 = transfer is as good as per-instance training).\n")

    L.append("\n## What this does and does not show\n")
    L.append("- The exact DP finds the optimum on every instance, fast. No heuristic or quantum method "
             "beats it, and none is claimed to.\n"
             "- Any QAOA gain reported above is against uniformly random sampling or a specific "
             "heuristic, never against the exact solver.\n"
             "- The subset-cost table is built classically and applied directly; real hardware would "
             "need that cost compiled into gates.\n"
             "- Rewards, windows and budgets are synthetic.\n")
    return "\n".join(L)


def _t1(t1: pd.DataFrame) -> str:
    return md_table(t1, [
        ("source", "source", "{}"), ("mode", "mode", "{}"), ("n", "n", "{}"),
        ("runs", "runs", "{}"), ("feasible", "feasible subsets", "{:.1%}"),
        ("greedy", "greedy ratio", "{:.3f}"), ("greedy_hits", "greedy optimal", "{:.0f}"),
        ("greedy_local_search", "greedy+LS ratio", "{:.3f}"),
        ("greedy_local_search_hits", "greedy+LS optimal", "{:.0f}")])


def main(argv=None):
    args = parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    df = run(args)
    summary = summarise(df)
    df.to_csv(out / "benchmark_raw.csv", index=False)
    summary.to_csv(out / "benchmark_summary.csv", index=False)
    wall = time.perf_counter() - started
    with open(out / "run_config.json", "w", encoding="utf-8") as f:
        json.dump({**vars(args), "generated_at": datetime.now().isoformat(),
                   "wall_time_s": wall, "skipped_instances": df.attrs.get("skipped", 0)}, f, indent=2)
    (out / "FINDINGS.md").write_text(write_findings(df, summary, args, wall), encoding="utf-8")
    print(f"wrote {out} ({len(df)} rows) in {wall:.0f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
