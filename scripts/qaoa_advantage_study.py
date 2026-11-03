#!/usr/bin/env python3
"""
QAOA variants vs classical on the multi-site lunar mission (follow-up to
qaoa_mission_study.py, which found that plain compact QAOA loses to 2-opt).

Phases (each writes results/qaoa_advantage_study/<phase>.json):

  ls        Threshold-QAOA local search (planning_pkg.quantum_local_search) vs the
            same classical black-box 2-opt, on the terrain instances (4-20 sites).
            Metric: queries until a tour <= target is seen (target = optimum from
            Held-Karp for N <= 15, else a top-5 % tour: the 5th percentile of the
            final costs of all descents, both methods pooled).
  scale     same race on synthetic lunar-slope instances, 10-100 sites.
  warm      warm-started regularised compact QAOA (planning_pkg.qaoa_warm_start),
            seeded with a black-box 2-opt descent, on instances where that seed is
            not already optimal: does QAOA improve on the tour it was given?
  transfer  deep compact QAOA (p up to 10) with angles trained on 6-7 sites and
            reused, untrained, on 8-10 sites.
  report    scaling-exponent fits + ADVANTAGE_TABLES.md from the JSON files.

One query = one tour-energy evaluation (same as qaoa_vs_classical_queries.py).
Quantum local search counts 2 queries per Grover/Th-QAOA layer (compute +
uncompute) plus 1 per measured candidate; change with --oracle-calls.

    python scripts/qaoa_advantage_study.py --phase all            # full run
    python scripts/qaoa_advantage_study.py --phase all --quick    # smoke test
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src" / "planning_pkg"))

import qaoa_mission_study as S  # noqa: E402
from planning_pkg import quantum_local_search as Q  # noqa: E402
from planning_pkg import qaoa_warm_start as W  # noqa: E402
from planning_pkg import tsp_compact_qaoa as TC  # noqa: E402
from planning_pkg.instance_sets import synthetic_cost_matrix  # noqa: E402
from planning_pkg.qaoa_sim import shots_for_confidence  # noqa: E402
from planning_pkg.tsp_baselines import held_karp  # noqa: E402

OUT = ROOT / "results" / "qaoa_advantage_study"
OLD = ROOT / "results" / "qaoa_mission_study"
EXACT_MAX = 15            # Held-Karp optimum up to this many sites
OPT_TOL = 1 + 1e-9


def q99(xs):
    xs = [x for x in xs if math.isfinite(x)]
    return float(np.percentile(xs, 99)) if xs else math.inf


def _dump(name, rows):
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(rows, open(OUT / f"{name}.json", "w"), indent=1)
    print(f"wrote {OUT / (name + '.json')} ({len(rows)} rows)", flush=True)


# ------------------------------------------------------------------ local search
def race(cost, target, runs, seed, oracle_calls, cap):
    out = {}
    for name, fn, kw in (("classical", Q.classical_descent, {}),
                         ("quantum", Q.quantum_descent, {"oracle_calls_per_layer": oracle_calls})):
        qs = [Q.queries_to_target(fn, cost, target, np.random.default_rng(10_000 * seed + k),
                                  cap=cap, **kw) for k in range(runs)]
        out[name] = {"median": float(np.median(qs)), "q99": q99(qs),
                     "unsolved": int(sum(not math.isfinite(x) for x in qs))}
    out["speedup_median"] = out["classical"]["median"] / out["quantum"]["median"]
    out["speedup_q99"] = out["classical"]["q99"] / out["quantum"]["q99"]
    return out


def descent_stats(cost, runs, seed, oracle_calls):
    res = {}
    for name, fn, kw in (("classical", Q.classical_descent, {}),
                         ("quantum", Q.quantum_descent, {"oracle_calls_per_layer": oracle_calls})):
        rs = [fn(cost, np.random.default_rng(20_000 * seed + k), **kw) for k in range(runs)]
        res[name] = rs
    pooled = [r.cost for v in res.values() for r in v]
    best = min(pooled)
    out = {"best_known": best, "top5pct_cost": float(np.percentile(pooled, 5))}
    for name, rs in res.items():
        out[name] = {"queries_median": float(np.median([r.queries for r in rs])),
                     "cost_ratio_mean": float(np.mean([r.cost for r in rs])) / best,
                     "false_stop_rate": float(np.mean([not r.true_local_optimum for r in rs]))}
    out["descent_speedup"] = out["classical"]["queries_median"] / out["quantum"]["queries_median"]
    return out


def ls_job(args):
    family, n, seed, runs, oracle_calls, cap = args
    t0 = time.perf_counter()
    cost = S.terrain_instance(n, seed) if family == "terrain" else synthetic_cost_matrix(n, seed)
    row = {"family": family, "n_sites": n, "seed": seed, "moves_K": n * (n - 1) // 2}
    d = descent_stats(cost, max(50, runs // 2), seed, oracle_calls)
    row["descent"] = d
    if n <= EXACT_MAX:
        target, row["target"] = held_karp(cost, True).cost * OPT_TOL, "optimum"
    else:
        target, row["target"] = d["top5pct_cost"] * OPT_TOL, "top 5%"
    row["target_cost"] = float(target)
    row["race"] = race(cost, target, runs, seed, oracle_calls, cap)
    row["seconds"] = time.perf_counter() - t0
    print(f"[{family}] N={n} seed={seed}: classical {row['race']['classical']['median']:.0f} | "
          f"quantum {row['race']['quantum']['median']:.0f} queries (median) -> "
          f"{row['race']['speedup_median']:.2f}x", flush=True)
    return row


# ------------------------------------------------------------------ warm start
def warm_job(args):
    n, seed, eps_list, pmax, maxiter = args
    cost = S.terrain_instance(n, seed)
    prob = TC.build_compact_problem(cost, closed=True)
    seed_run = Q.classical_descent(cost, np.random.default_rng(seed))
    seed_cost = seed_run.cost
    rows = []
    base = {"n_sites": n, "seed": seed, "qubits": prob.q, "opt": prob.opt_cost,
            "seed_cost": seed_cost, "seed_ratio": seed_cost / prob.opt_cost,
            "seed_queries": seed_run.queries,
            "seed_optimal": bool(seed_cost <= prob.opt_cost * OPT_TOL)}
    idx = W.seed_index(prob.perms, seed_run.order)
    for eps in eps_list:
        thetas = W.warm_start_thetas(idx, prob.q, eps)
        for lr in W.train_layers(prob.C, pmax, thetas, maxiter=maxiter):
            m = W.distribution_metrics(prob, W.ws_probs(prob.C, lr.params, thetas), seed_cost)
            shots = shots_for_confidence(m["p_opt"])
            rows.append({**base, "eps": eps, "p": lr.p, "train_evals": lr.cumulative_evals, **m,
                         "shots_to_solution": shots,
                         "total_queries_no_training": seed_run.queries + shots,
                         "total_queries_with_training":
                             seed_run.queries + shots + lr.cumulative_evals * S.SHOTS_PER_EVAL})
    best = min((r for r in rows), key=lambda r: r["total_queries_no_training"])
    print(f"[warm] N={n} seed={seed} seed_ratio={base['seed_ratio']:.3f} "
          f"best p_opt={best['p_opt']:.3f} (eps={best['eps']}, p={best['p']})", flush=True)
    return rows


# ------------------------------------------------------------------ transfer
def transfer_phase(train_ns, test_ns, seeds, pmax, maxiter, jobs):
    TC.MAX_SITES = max(TC.MAX_SITES, max(test_ns))
    with Pool(jobs) as pool:
        trained = pool.map(_train_job, [(n, s, pmax, maxiter) for n in train_ns for s in seeds])
    angles = {p: TC.transfer_params([t[p - 1] for t in trained]) for p in range(1, pmax + 1)}
    with Pool(jobs) as pool:
        rows = sum(pool.map(_transfer_eval_job,
                            [(n, s, pmax, {p: a.tolist() for p, a in angles.items()})
                             for n in test_ns for s in seeds]), [])
    return rows, {p: a.tolist() for p, a in angles.items()}


def _train_job(args):
    n, seed, pmax, maxiter = args
    prob = TC.build_compact_problem(S.terrain_instance(n, seed), closed=True)
    return [lr.params for lr in W.train_layers(prob.C, pmax, None, maxiter=maxiter)]


def _transfer_eval_job(args):
    n, seed, pmax, angles = args
    TC.MAX_SITES = max(TC.MAX_SITES, n)
    prob = TC.build_compact_problem(S.terrain_instance(n, seed), closed=True)
    u_opt = float((prob.weight / prob.weight.sum())[prob.optimal].sum())
    rows = []
    for p in range(1, pmax + 1):
        m = W.distribution_metrics(prob, W.ws_probs(prob.C, np.array(angles[p]), None))
        rows.append({"n_sites": n, "seed": seed, "qubits": prob.q, "p": p, **m,
                     "uniform_p_opt": u_opt, "amplification": m["p_opt"] / u_opt,
                     "shots_to_solution": shots_for_confidence(m["p_opt"]),
                     "train_evals_on_this_instance": 0})
    print(f"[transfer] N={n} seed={seed}: amplification at p={pmax} "
          f"{rows[-1]['amplification']:.1f}x", flush=True)
    return rows


# ------------------------------------------------------------------ report
def fit(xs, ys, log_x):
    xs, ys = np.asarray(xs, float), np.asarray(ys, float)
    ok = np.isfinite(ys) & (ys > 0)
    if ok.sum() < 2:
        return None
    X = np.log(xs[ok]) if log_x else xs[ok]
    k, a = np.polyfit(X, np.log(ys[ok]), 1)
    pred = a + k * X
    ss = np.sum((np.log(ys[ok]) - pred) ** 2)
    st = np.sum((np.log(ys[ok]) - np.log(ys[ok]).mean()) ** 2)
    return {"slope": float(k), "r2": float(1 - ss / st) if st > 0 else 1.0}


def _median_by_n(rows, key):
    by = {}
    for r in rows:
        by.setdefault(r["n_sites"], []).append(key(r))
    return {n: float(np.median(v)) for n, v in sorted(by.items())}


def report():
    load = lambda name: json.load(open(OUT / f"{name}.json")) if (OUT / f"{name}.json").exists() else []
    ls, scale, warm, transfer = load("ls"), load("scale"), load("warm"), load("transfer")
    race_old = json.load(open(OLD / "query_race.json")) if (OLD / "query_race.json").exists() else []
    L = ["# QAOA variants vs classical: result tables", "",
         "Generated by `scripts/qaoa_advantage_study.py`. One query = one tour-energy evaluation.",
         "Quantum local search counts 2 oracle queries per Th-QAOA layer + 1 per measurement.",
         "Medians over runs and instances unless noted. Simulated, ideal (noise-free) hardware.", ""]
    fits = {}
    for title, rows in (("terrain instances", ls), ("synthetic lunar-slope instances", scale)):
        if not rows:
            continue
        L += [f"## Threshold-QAOA local search vs classical 2-opt: {title}", "",
              "| N sites | moves K | target | classical queries | quantum queries | speedup (median) | speedup (q99) | descent speedup | false-stop rate |",
              "|---|---|---|---|---|---|---|---|---|"]
        by = {}
        for r in rows:
            by.setdefault(r["n_sites"], []).append(r)
        for n, rs in sorted(by.items()):
            med = lambda f: float(np.median([f(r) for r in rs]))
            L.append("| %d | %d | %s | %.0f | %.0f | %.2fx | %.2fx | %.2fx | %.4f |" % (
                n, rs[0]["moves_K"], rs[0]["target"],
                med(lambda r: r["race"]["classical"]["median"]), med(lambda r: r["race"]["quantum"]["median"]),
                med(lambda r: r["race"]["speedup_median"]), med(lambda r: r["race"]["speedup_q99"]),
                med(lambda r: r["descent"]["descent_speedup"]),
                med(lambda r: r["descent"]["quantum"]["false_stop_rate"])))
        L.append("")
        for name in ("classical", "quantum"):
            mb = _median_by_n(rows, lambda r, nm=name: r["race"][nm]["median"])
            fits[f"{title}: {name} local search"] = fit(list(mb), list(mb.values()), log_x=True)
    if race_old:
        for key, label in (("two_opt_median", "2-opt (query_race.json)"),
                           ("annealing_median", "simulated annealing (query_race.json)"),
                           ("qaoa_queries_no_training", "compact QAOA, no training (query_race.json)"),
                           ("random_queries", "random sampling (query_race.json)")):
            mb = _median_by_n([r for r in race_old if r["n_sites"] >= 6], lambda r, k=key: r[k])
            fits[label] = fit(list(mb), list(mb.values()), log_x=True)
    if transfer:
        best = {}
        for r in transfer:
            best.setdefault(r["p"], []).append(r)
        pmax = max(best)
        L += ["## Deep compact QAOA with transferred angles (no training on these instances)", "",
              "| N sites | qubits | " + " | ".join(f"p={p}" for p in sorted(best)) + " |",
              "|---|---|" + "---|" * len(best)]
        for n in sorted({r["n_sites"] for r in transfer}):
            cells = [np.median([r["amplification"] for r in transfer if r["n_sites"] == n and r["p"] == p])
                     for p in sorted(best)]
            q = [r["qubits"] for r in transfer if r["n_sites"] == n][0]
            L.append(f"| {n} | {q} | " + " | ".join("%.1fx" % c for c in cells) + " |")
        L += ["", "Values: P(optimal) amplification over uniform random sampling.", ""]
        mb = _median_by_n([r for r in transfer if r["p"] == pmax], lambda r: r["shots_to_solution"])
        fits[f"compact QAOA, transferred p={pmax}"] = fit(list(mb), list(mb.values()), log_x=True)
    if warm:
        L += ["## Warm-started regularised compact QAOA (seed = black-box 2-opt descent)", "",
              "Instances where the seed was already optimal are excluded (nothing to improve).", "",
              "| N sites | instances | eps | best p | seed / opt | P(optimal) | P(better than seed) | seed+QAOA queries (no training) |",
              "|---|---|---|---|---|---|---|---|"]
        keep = [r for r in warm if not r["seed_optimal"]]
        for n in sorted({r["n_sites"] for r in keep}):
            for eps in sorted({r["eps"] for r in keep}):
                rs = [r for r in keep if r["n_sites"] == n and r["eps"] == eps]
                inst = sorted({r["seed"] for r in rs})
                per = [min((r for r in rs if r["seed"] == s), key=lambda r: r["total_queries_no_training"])
                       for s in inst]
                L.append("| %d | %d | %.2f | %d | %.3f | %.3f | %.3f | %.0f |" % (
                    n, len(inst), eps, int(np.median([r["p"] for r in per])),
                    np.median([r["seed_ratio"] for r in per]), np.median([r["p_opt"] for r in per]),
                    np.median([r["p_better_than_seed"] for r in per]),
                    np.median([r["total_queries_no_training"] for r in per])))
        n_opt = len({(r["n_sites"], r["seed"]) for r in warm if r["seed_optimal"]})
        L += ["", f"Seed already optimal on {n_opt} of "
              f"{len({(r['n_sites'], r['seed']) for r in warm})} instances.", ""]
    L += ["## Scaling fits: log(queries to target) = a + k log(N)", "",
          "| method | exponent k | R^2 |", "|---|---|---|"]
    for name, f in fits.items():
        if f:
            L.append("| %s | %.2f | %.3f |" % (name, f["slope"], f["r2"]))
    L += ["", "A smaller exponent means queries grow more slowly with the number of sites.",
          "Factorial growth (compact QAOA, random) shows as an exponent that keeps rising with N;",
          "the fit over a short range understates it.", ""]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "ADVANTAGE_TABLES.md").write_text("\n".join(L), encoding="utf-8")
    json.dump(fits, open(OUT / "scaling_fits.json", "w"), indent=1)
    print("\n".join(L))


# ------------------------------------------------------------------ main
def main(argv=None):
    global OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--phase", default="all", choices=["ls", "scale", "warm", "transfer", "report", "all"])
    ap.add_argument("--quick", action="store_true", help="tiny settings for a smoke test")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--runs", type=int, default=200, help="independent runs per method per instance")
    ap.add_argument("--oracle-calls", type=float, default=2.0, help="queries per Th-QAOA layer")
    ap.add_argument("--cap", type=float, default=2e7, help="query cap per run")
    ap.add_argument("--terrain-ns", default="4,6,8,10,12,14,16,18,20")
    ap.add_argument("--scale-ns", default="10,20,30,40,60,80,100")
    ap.add_argument("--warm-ns", default="5,6,7,8,9")
    ap.add_argument("--warm-eps", default="0.1,0.25,0.4")
    ap.add_argument("--warm-pmax", type=int, default=5)
    ap.add_argument("--transfer-train-ns", default="6,7")
    ap.add_argument("--transfer-test-ns", default="8,9,10", help="10 sites = 22 qubits")
    ap.add_argument("--transfer-pmax", type=int, default=10)
    ap.add_argument("--maxiter", type=int, default=60)
    ap.add_argument("--out", default=None, help="output directory (default results/qaoa_advantage_study)")
    a = ap.parse_args(argv)
    if a.out:
        OUT = Path(a.out)
    ints = lambda s: [int(x) for x in s.split(",") if x]
    if a.quick:
        a.seeds, a.runs, a.cap, a.maxiter = 1, 20, 1e6, 20
        a.terrain_ns, a.scale_ns, a.warm_ns = "6,8", "10,20", "5,6"
        a.warm_eps, a.warm_pmax = "0.25", 2
        a.transfer_train_ns, a.transfer_test_ns, a.transfer_pmax = "5", "6", 3
    seeds = list(range(a.seeds))
    phases = ["ls", "scale", "warm", "transfer", "report"] if a.phase == "all" else [a.phase]
    t0 = time.perf_counter()
    for ph in phases:
        print(f"=== phase {ph}", flush=True)
        if ph in ("ls", "scale"):
            family, ns = ("terrain", ints(a.terrain_ns)) if ph == "ls" else ("synthetic", ints(a.scale_ns))
            jobs = [(family, n, s, a.runs, a.oracle_calls, a.cap) for n in ns for s in seeds]
            with Pool(a.jobs) as pool:
                _dump(ph, pool.map(ls_job, jobs, chunksize=1))
        elif ph == "warm":
            eps = [float(x) for x in a.warm_eps.split(",")]
            with Pool(a.jobs) as pool:
                rows = pool.map(warm_job, [(n, s, eps, a.warm_pmax, a.maxiter)
                                           for n in ints(a.warm_ns) for s in seeds], chunksize=1)
            _dump("warm", sum(rows, []))
        elif ph == "transfer":
            rows, angles = transfer_phase(ints(a.transfer_train_ns), ints(a.transfer_test_ns), seeds,
                                          a.transfer_pmax, a.maxiter, a.jobs)
            _dump("transfer", rows)
            _dump("transfer_angles", [angles])
        else:
            report()
    print(f"done in {time.perf_counter() - t0:.0f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
