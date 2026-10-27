#!/usr/bin/env python3
"""
benchmark_hard_and_compact.py
-----------------------------
Visiting-order problem, three experiments (subcommands write CSVs, `report`
turns them into FINDINGS.md):

  gaps      Do hard instances exist? For each family and N, how far above the
            optimum are nearest neighbour, NN + 2-opt, and 2-opt with 20 random
            restarts? Also marks the STRESS instances (NN + 2-opt >= threshold).
  compact   QAOA with the compact permutation encoding (ceil(log2 N!) qubits)
            on plain and stress instances: P(optimal), amplification, how much
            of the average-vs-best gap QAOA closes, depth p, CVaR alpha, and
            parameter transfer (angles trained at one N, reused at larger N).
  queries   Cost evaluations ("oracle queries"): Durr-Hoyer minimum finding
            (simulated) vs brute force, with Held-Karp steps for orientation.
  report    Combine everything in --out into FINDINGS.md.

Families (see planning_pkg.instance_sets):
  terrain_random_up2 / _up30   random sites on the heightmap, uphill weight 2 / 30
  terrain_clustered_up30       clustered sites on the heightmap, uphill weight 30
  synthetic_clustered_asym8    synthetic clusters + relief + strong uphill cost
  synthetic_uniform_sym        synthetic uniform symmetric costs (an easy control)

    python scripts/benchmark_hard_and_compact.py gaps --family synthetic_clustered_asym8
    python scripts/benchmark_hard_and_compact.py compact --family synthetic_clustered_asym8
    python scripts/benchmark_hard_and_compact.py queries
    python scripts/benchmark_hard_and_compact.py report

The compact encoding and its QAOA are simulated with the tour costs applied as a
lookup table (planning_pkg.qaoa_sim): a simulation shortcut, not a gate circuit.
"""

from __future__ import annotations

import argparse
import glob
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "planning_pkg"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from planning_pkg.instance_sets import (  # noqa: E402
    gap_record,
    synthetic_cost_matrix,
    terrain_cost_matrix,
)
from planning_pkg.qaoa_sim import shots_for_confidence  # noqa: E402
from planning_pkg.quantum_queries import (  # noqa: E402
    classical_exhaustive_queries,
    durr_hoyer_costs,
    durr_hoyer_ranks,
    held_karp_steps,
    loglog_slope,
    summarise_trials,
)
from planning_pkg.tsp_baselines import nn_two_opt  # noqa: E402
from planning_pkg.tsp_compact_qaoa import (  # noqa: E402
    build_compact_problem,
    run_compact_qaoa,
    transfer_params,
)
from report_utils import md_table, sci  # noqa: E402

FAMILIES = {
    "terrain_random_up2": dict(kind="terrain", family="terrain_random", w_uphill=2.0),
    "terrain_random_up30": dict(kind="terrain", family="terrain_random", w_uphill=30.0),
    "terrain_clustered_up30": dict(kind="terrain", family="terrain_clustered", w_uphill=30.0),
    "synthetic_clustered_asym8": dict(kind="synthetic", clusters=3, asym=8.0, relief=10.0),
    "synthetic_uniform_sym": dict(kind="synthetic", clusters=1, asym=0.0, relief=0.0),
}
HEIGHTMAP = str(ROOT / "src/rover_simulation/worlds/heightmap.png")
_terrain_cache: dict = {}


def cost_fn(name: str):
    spec = FAMILIES[name]
    if spec["kind"] == "synthetic":
        return lambda n, seed: synthetic_cost_matrix(
            n, seed, clusters=spec["clusters"], asym=spec["asym"], relief=spec["relief"])
    from planning_pkg.cost_map import CostMapConfig, CostWeights
    from planning_pkg.mission_planner import load_cost_map
    key = spec["w_uphill"]
    if key not in _terrain_cache:
        cfg = CostMapConfig(resolution_m=1.0, sun_elevation_deg=3.0, sun_azimuth_deg=135.0,
                            weights=CostWeights(1.0, 2.0, 1.5, key))
        _terrain_cache[key] = load_cost_map(HEIGHTMAP, config=cfg)
    cm, graph = _terrain_cache[key]
    return lambda n, seed: terrain_cost_matrix(graph, cm, n, seed, family=spec["family"])


def default_seed_count(name: str) -> int:
    return 40 if FAMILIES[name]["kind"] == "synthetic" else 10


# ---------------------------------------------------------------- gaps

def cmd_gaps(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    make = cost_fn(args.family)
    seeds = list(range(args.seed_count or default_seed_count(args.family)))
    rows = []
    for n in args.ns:
        for seed in seeds:
            try:
                cost = make(n, seed)
            except ValueError as exc:
                print(f"skip {args.family} n={n} seed={seed}: {exc}", flush=True)
                continue
            r = gap_record(cost, seed, True, args.restarts)
            rows.append({"family": args.family, "n": n, "seed": seed, "optimal": r.optimal,
                         "nn_gap_pct": r.nn_gap_pct, "nn2opt_gap_pct": r.nn_2opt_gap_pct,
                         "multistart_gap_pct": r.multistart_gap_pct,
                         "stress": r.nn_2opt_gap_pct >= args.threshold,
                         "threshold_pct": args.threshold, "restarts": args.restarts})
        sub = [r for r in rows if r["n"] == n]
        if sub:
            print(f"{args.family} n={n}: {len(sub)} instances, NN+2opt non-optimal "
                  f"{sum(r['nn2opt_gap_pct'] > 1e-6 for r in sub)}, stress "
                  f"{sum(r['stress'] for r in sub)}", flush=True)
    pd.DataFrame(rows).to_csv(out / f"gaps_{args.family}.csv", index=False)


# ---------------------------------------------------------------- compact

def _instances(family: str, n: int, plain_seeds, stress_seeds, make):
    for label, seeds in (("plain", plain_seeds), ("stress", stress_seeds)):
        for seed in seeds:
            try:
                yield label, seed, make(n, seed)
            except ValueError:
                continue


def cmd_compact(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    make = cost_fn(args.family)
    gaps_path = out / f"gaps_{args.family}.csv"
    gaps = pd.read_csv(gaps_path) if gaps_path.exists() else pd.DataFrame()
    rows = []
    trained: dict = {}
    for n in sorted(args.ns):
        plain = list(range(args.seeds))
        stress = []
        if len(gaps):
            stress = gaps[(gaps["n"] == n) & gaps["stress"]]["seed"].tolist()[:args.seeds]
        ps = args.ps if n < args.large_n else args.ps_large
        for label, seed, cost in _instances(args.family, n, plain, stress, make):
            prob = build_compact_problem(cost)
            nn2 = nn_two_opt(cost).cost / prob.opt_cost
            for alpha in args.alphas:
                for p in ps:
                    t0 = time.perf_counter()
                    r = run_compact_qaoa(cost, p, alpha, args.shots, seed, args.maxiter,
                                         args.restarts, problem=prob)
                    r.pop("params_unused", None)
                    row = {k: v for k, v in r.items() if k != "params"}
                    row.update({"family": args.family, "set": label, "n": n, "seed": seed,
                                "opt_cost": prob.opt_cost, "nn2opt_ratio": nn2,
                                "train_s": time.perf_counter() - t0})
                    rows.append(row)
                    if label == "plain" and n == args.train_n:
                        trained.setdefault((p, alpha), []).append(r["params"])
                    if n > args.train_n and (p, alpha) in trained:
                        avg = transfer_params(trained[(p, alpha)])
                        t = run_compact_qaoa(cost, p, alpha, args.shots, seed, params=avg,
                                             problem=prob)
                        row2 = {k: v for k, v in t.items() if k != "params"}
                        row2.update({"family": args.family, "set": label, "n": n, "seed": seed,
                                     "opt_cost": prob.opt_cost, "nn2opt_ratio": nn2,
                                     "train_s": 0.0})
                        rows.append(row2)
            print(f"{args.family}/{label} n={n} seed={seed}: opt {prob.opt_cost:.1f}, "
                  f"NN+2opt ratio {nn2:.3f}, q={prob.q}", flush=True)
    pd.DataFrame(rows).to_csv(out / f"compact_{args.family}.csv", index=False)


# ---------------------------------------------------------------- queries

def cmd_queries(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    rows = []
    for n_opt in (1, 2):
        for n in range(args.n_min, args.n_max + 1):
            m = math.factorial(n)
            res = [durr_hoyer_ranks(m, n_opt, rng) for _ in range(args.trials)]
            s = summarise_trials(res, m)
            c = classical_exhaustive_queries(m, n_opt)
            rows.append({"model": "rank", "n": n, "M": m, "n_optimal": n_opt, **s,
                         "classical_verify_all": c["verify_all"],
                         "classical_mean_first_optimum": c["mean_to_first_optimum"],
                         "held_karp_steps": held_karp_steps(n)})
            print(f"rank n={n} n_opt={n_opt}: mean queries {s['mean_queries']:.0f} "
                  f"(sqrt M {s['sqrt_m']:.0f})", flush=True)
    make = cost_fn("synthetic_clustered_asym8")
    for n in range(4, 9):
        cost = make(n, 0)
        prob = build_compact_problem(cost)
        m = len(prob.costs)
        res = [durr_hoyer_costs(prob.costs, rng) for _ in range(args.trials)]
        s = summarise_trials(res, m)
        c = classical_exhaustive_queries(m, len(prob.optimal))
        rows.append({"model": "array", "n": n, "M": m, "n_optimal": len(prob.optimal), **s,
                     "classical_verify_all": c["verify_all"],
                     "classical_mean_first_optimum": c["mean_to_first_optimum"],
                     "held_karp_steps": held_karp_steps(n)})
    pd.DataFrame(rows).to_csv(out / "queries.csv", index=False)


# ---------------------------------------------------------------- report

def _load(out: Path, prefix: str) -> pd.DataFrame:
    frames = [pd.read_csv(p) for p in sorted(glob.glob(str(out / f"{prefix}_*.csv")))]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def cmd_report(args):
    out = Path(args.out)
    gaps = _load(out, "gaps")
    comp = _load(out, "compact")
    qpath = out / "queries.csv"
    queries = pd.read_csv(qpath) if qpath.exists() else pd.DataFrame()
    L = ["# Hard instances, compact-encoding QAOA, and query counts\n",
         f"Generated by `scripts/benchmark_hard_and_compact.py report` on {datetime.now():%Y-%m-%d %H:%M}.\n"]

    if len(gaps):
        thr = float(gaps["threshold_pct"].iloc[0])
        a = gaps.groupby(["family", "n"]).agg(
            instances=("seed", "count"),
            nn=("nn_gap_pct", "mean"),
            nn2=("nn2opt_gap_pct", "mean"), nn2_max=("nn2opt_gap_pct", "max"),
            nn2_bad=("nn2opt_gap_pct", lambda g: int((g > 1e-6).sum())),
            ms=("multistart_gap_pct", "mean"), ms_max=("multistart_gap_pct", "max"),
            ms_bad=("multistart_gap_pct", lambda g: int((g > 1e-6).sum())),
            stress=("stress", "sum")).reset_index()
        L.append("## 1. Can instances be hard for simple heuristics?\n")
        L.append("Gap = 100 x (cost - optimum) / optimum. `NN+2opt` = nearest neighbour then 2-opt. "
                 "`2opt x20` = best of 2-opt from the NN tour and 20 random tours. "
                 f"`stress` = instances where NN+2opt is at least {thr:g}% above optimal. "
                 "Optimum from Held-Karp.\n")
        L.append(md_table(a, [
            ("family", "family", "{}"), ("n", "n", "{}"), ("instances", "instances", "{}"),
            ("nn", "NN gap % (mean)", "{:.2f}"),
            ("nn2", "NN+2opt gap % (mean)", "{:.2f}"), ("nn2_max", "max", "{:.2f}"),
            ("nn2_bad", "non-optimal", "{}"),
            ("ms", "2opt x20 gap % (mean)", "{:.2f}"), ("ms_max", "max", "{:.2f}"),
            ("ms_bad", "non-optimal", "{}"), ("stress", "stress", "{}")]))
        tot = len(gaps)
        bad = int((gaps["nn2opt_gap_pct"] > 1e-6).sum())
        fixed = int(((gaps["nn2opt_gap_pct"] > 1e-6) & (gaps["multistart_gap_pct"] <= 1e-6)).sum())
        L.append(f"\nNN+2opt is non-optimal on {bad} of {tot} instances; 20 random restarts repair "
                 f"{fixed} of those {bad}. A stress set is chosen for hardness: always show it next to "
                 f"the unfiltered rows above, and quote `stress` out of `instances` as the selection rate.\n")

    if len(comp):
        tr = comp[~comp["transferred"]]
        g = tr.groupby(["family", "set", "n", "p", "alpha"]).agg(
            p_opt=("p_opt", "mean"), uni=("uniform_p_opt", "mean"), amp=("amplification", "mean"),
            gap=("gap_closed", "mean"), eq=("mean_ratio_qaoa", "mean"),
            eu=("mean_ratio_uniform", "mean"), runs=("seed", "count"),
            qubits=("qubits", "first"), onehot=("onehot_qubits", "first")).reset_index()
        g["s99"] = g["p_opt"].map(shots_for_confidence)
        g["u99"] = g["uni"].map(shots_for_confidence)
        best = g.sort_values("p_opt", ascending=False).groupby(["family", "set", "n"]).head(1)
        best = best.sort_values(["family", "set", "n"])
        best["config"] = "p=" + best["p"].astype(int).astype(str) + ", a=" + best["alpha"].map("{:g}".format)
        best["tours"] = best["n"].map(math.factorial)
        L.append("\n## 2. QAOA with the compact encoding (best configuration per row, exact probabilities)\n")
        L.append("Every bitstring decodes to a valid tour, so a gain over random is a gain in tour quality. "
                 "`gap closed` = fraction of the (random-average minus optimum) cost gap that QAOA removes "
                 "in expectation (0 = no better than random, 1 = always optimal). `shots` = samples for a "
                 "99% chance of seeing an optimal tour, from the mean P(opt), excluding training. "
                 "`qubits` compact vs one-hot encoding. The best configuration is picked after seeing the results, "
                 "so these rows are optimistic; the depth table below averages everything. "
                 "Training cost (circuit evaluations) is NOT included in `QAOA shots`.\n")
        L.append(md_table(best, [
            ("family", "family", "{}"), ("set", "set", "{}"), ("n", "N", "{}"),
            ("qubits", "qubits", "{}"), ("onehot", "one-hot qubits", "{}"), ("config", "best config", "{}"),
            ("runs", "runs", "{}"), ("p_opt", "P(opt)", "{:.5f}"), ("uni", "uniform", "{:.5f}"),
            ("amp", "amplification", "{:.1f}"), ("gap", "gap closed", "{:.2f}"),
            ("s99", "QAOA shots", sci), ("u99", "random shots", sci), ("tours", "brute force N!", sci)]))
        pos = int((g["gap"] > 0).sum())
        L.append(f"\nQAOA closed a positive share of the cost gap in {pos} of {len(g)} "
                 f"(family, set, N, p, alpha) groups; the median `gap closed` is {g['gap'].median():.2f}.\n")

        depth = tr.assign(cfg="p" + tr["p"].astype(int).astype(str) + " a" + tr["alpha"].map("{:g}".format))
        d = depth.pivot_table(index="n", columns="cfg", values="gap_closed", aggfunc="mean")
        d = d[sorted(d.columns)].reset_index()
        L.append("\n### Effect of depth p and CVaR alpha (mean `gap closed`, all families and sets)\n")
        L.append(md_table(d, [("n", "N", "{:.0f}")] + [(c, c, "{:.2f}") for c in d.columns if c != "n"]))

        xf = comp[comp["n"] > comp[~comp["transferred"]]["n"].min()]
        if xf["transferred"].any():
            t = xf.groupby(["n", "p", "alpha", "transferred"])["gap_closed"].mean().unstack("transferred")
            if {True, False} <= set(t.columns):
                t = t.rename(columns={False: "trained", True: "transfer"}).reset_index()
                L.append("\n### Parameter transfer (angles trained on the smallest N, reused with no training)\n")
                L.append(md_table(t, [("n", "N", "{:.0f}"), ("p", "p", "{:.0f}"), ("alpha", "alpha", "{:g}"),
                                      ("trained", "gap closed (trained)", "{:.2f}"),
                                      ("transfer", "gap closed (transfer)", "{:.2f}")]))
                L.append(f"\nMedian transfer - trained difference in `gap closed`: "
                         f"{(t['transfer'] - t['trained']).median():+.2f}.\n")

    if len(queries):
        r = queries[(queries["model"] == "rank") & (queries["n_optimal"] == 1)].copy()
        r = r.sort_values("n")
        slope = loglog_slope(r["M"], r["mean_queries"])
        r["ratio"] = r["mean_queries"] / r["sqrt_m"]
        L.append("\n## 3. Cost evaluations (oracle queries), not seconds\n")
        L.append("Durr-Hoyer minimum finding, simulated with the exact Grover success probability (see "
                 "`planning_pkg/quantum_queries.py`). M = N! orders, one optimal order (asymmetric costs; "
                 "a symmetric instance has two, a tour and its reverse, which only helps slightly). `queries` = mean queries until an optimal order is first reached "
                 f"({int(r['trials'].iloc[0])} trials). `brute force` = M evaluations. `Held-Karp steps` "
                 "= N^2 2^N basic steps: a different counting model, shown for orientation only.\n")
        if len(comp):
            tr = comp[~comp["transferred"]]
            q = tr.groupby(["n", "p", "alpha"]).agg(p_opt=("p_opt", "mean"), uni=("uniform_p_opt", "mean")).reset_index()
            bq = q.sort_values("p_opt", ascending=False).groupby("n").head(1)
            bq["qaoa_shots"] = bq["p_opt"].map(shots_for_confidence)
            bq["rand_shots"] = bq["uni"].map(shots_for_confidence)
            r = r.merge(bq[["n", "qaoa_shots", "rand_shots"]], on="n", how="left")
        else:
            r["qaoa_shots"] = np.nan
            r["rand_shots"] = np.nan
        L.append(md_table(r, [
            ("n", "N", "{}"), ("M", "M = N!", sci), ("classical_verify_all", "brute force", sci),
            ("mean_queries", "Durr-Hoyer queries", sci), ("ratio", "queries / sqrt(M)", "{:.2f}"),
            ("within_published_budget", "found within published bound", "{:.2f}"),
            ("held_karp_steps", "Held-Karp steps", sci),
            ("qaoa_shots", "QAOA shots (best cfg)", sci), ("rand_shots", "random-sampling shots", sci)]))
        L.append(f"\nFitted log-log slope of queries against M: {slope:.3f} (theory: 0.5), i.e. a quadratic "
                 f"query speedup over brute force. Queries are about {r['ratio'].mean():.1f} x sqrt(M).\n")
        cross = r[r["mean_queries"] > r["held_karp_steps"]]
        if len(cross):
            L.append(f"On raw counts, Held-Karp steps fall below Durr-Hoyer queries from N = "
                     f"{int(cross['n'].min())}. That comparison mixes unlike units and is not a "
                     f"statement about speed.\n")
        else:
            L.append("On raw counts, Durr-Hoyer queries stay below Held-Karp steps over this N range, "
                     "but the units differ: one quantum query evaluates a whole tour cost in a "
                     "reversible circuit, one Held-Karp step is a table update.\n")

    L.append("\n## What this does and does not show\n")
    L.append("- Durr-Hoyer's quadratic speedup is over brute-force search. It does not beat Held-Karp "
             "or good heuristics, and no wall-clock claim is made.\n"
             "- QAOA is simulated with the tour costs applied as a lookup table, a simulation "
             "shortcut; real hardware would need those costs compiled into gates.\n"
             "- Stress sets are selected for heuristic failure and are not typical instances.\n"
             "- Synthetic families use made-up geometry and relief; terrain families use the "
             "simulation heightmap.\n")
    (out / "FINDINGS.md").write_text("\n".join(L), encoding="utf-8")
    print(f"wrote {out / 'FINDINGS.md'}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    default_out = str(ROOT / "results" / "ordering_hard")

    g = sub.add_parser("gaps")
    g.add_argument("--family", required=True, choices=sorted(FAMILIES))
    g.add_argument("--ns", type=int, nargs="+", default=[5, 6, 7, 8, 9, 10])
    g.add_argument("--seed-count", type=int, default=0)
    g.add_argument("--threshold", type=float, default=3.0)
    g.add_argument("--restarts", type=int, default=20)
    g.add_argument("--out", default=default_out)
    g.set_defaults(fn=cmd_gaps)

    c = sub.add_parser("compact")
    c.add_argument("--family", required=True, choices=sorted(FAMILIES))
    c.add_argument("--ns", type=int, nargs="+", default=[5, 6, 7, 8])
    c.add_argument("--seeds", type=int, default=4)
    c.add_argument("--ps", type=int, nargs="+", default=[1, 2, 3, 4])
    c.add_argument("--ps-large", type=int, nargs="+", default=[1, 2, 3])
    c.add_argument("--large-n", type=int, default=8)
    c.add_argument("--alphas", type=float, nargs="+", default=[1.0, 0.2])
    c.add_argument("--shots", type=int, default=4096)
    c.add_argument("--maxiter", type=int, default=60)
    c.add_argument("--restarts", type=int, default=1)
    c.add_argument("--train-n", type=int, default=6)
    c.add_argument("--out", default=default_out)
    c.set_defaults(fn=cmd_compact)

    q = sub.add_parser("queries")
    q.add_argument("--n-min", type=int, default=4)
    q.add_argument("--n-max", type=int, default=20)
    q.add_argument("--trials", type=int, default=300)
    q.add_argument("--seed", type=int, default=0)
    q.add_argument("--out", default=default_out)
    q.set_defaults(fn=cmd_queries)

    r = sub.add_parser("report")
    r.add_argument("--out", default=default_out)
    r.set_defaults(fn=cmd_report)

    args = ap.parse_args(argv)
    args.fn(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
