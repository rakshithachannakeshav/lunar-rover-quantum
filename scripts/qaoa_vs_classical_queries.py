#!/usr/bin/env python3
"""
Black-box query race on the SAME terrain instances as qaoa_mission_study.py.

One "query" = evaluating the energy of one complete tour (what one QAOA shot
gives you). Classical black-box solvers below see only tour energies, exactly
like QAOA. Metric: queries needed for a 99 % chance of having seen an optimal
tour (99th percentile over independent runs). QAOA numbers are read from
results/qaoa_mission_study/compact.json, with and without training cost.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import qaoa_mission_study as S  # noqa: E402

RUNS = 300
OUT = ROOT / "results" / "qaoa_mission_study"


class Oracle:
    def __init__(self, cost):
        self.cost, self.queries = cost, 0

    def __call__(self, order):
        self.queries += 1
        return S.tour_cost(self.cost, order)


def two_opt_restarts(oracle, n, opt, rng, cap=10**7):
    """Random-restart 2-opt (first improvement, random neighbour order)."""
    while oracle.queries < cap:
        tour = list(rng.permutation(np.arange(1, n + 1)))
        best = oracle(tour)
        if best <= opt * (1 + 1e-9):
            return oracle.queries
        improved = True
        while improved:
            improved = False
            pairs = [(i, j) for i in range(n - 1) for j in range(i + 1, n)]
            for k in rng.permutation(len(pairs)):
                i, j = pairs[k]
                cand = tour[:i] + tour[i:j + 1][::-1] + tour[j + 1:]
                c = oracle(cand)
                if c < best - 1e-12:
                    tour, best, improved = cand, c, True
                    if best <= opt * (1 + 1e-9):
                        return oracle.queries
                    break
    return math.inf


def annealing(oracle, n, opt, rng, steps=4000, cap=10**7):
    """Simulated annealing with reversal/swap moves, restarted until optimal."""
    while oracle.queries < cap:
        tour = list(rng.permutation(np.arange(1, n + 1)))
        cur = oracle(tour)
        if cur <= opt * (1 + 1e-9):
            return oracle.queries
        t0, t1 = 0.3 * cur / n, 1e-4 * cur / n
        for s in range(steps):
            temp = t0 * (t1 / t0) ** (s / steps)
            i, j = sorted(rng.choice(n, 2, replace=False))
            cand = (tour[:i] + tour[i:j + 1][::-1] + tour[j + 1:]) if rng.random() < 0.5 else \
                [tour[j] if k == i else tour[i] if k == j else tour[k] for k in range(n)]
            c = oracle(cand)
            if c <= opt * (1 + 1e-9):
                return oracle.queries
            if c < cur or rng.random() < math.exp(-(c - cur) / temp):
                tour, cur = cand, c
    return math.inf


def q99(xs):
    return float(np.percentile(xs, 99))


def main():
    compact = json.load(open(OUT / "compact.json"))
    best_p = {}
    for r in compact:
        key = (r["n_sites"], r["seed"])
        if key not in best_p or r["p"] > best_p[key]["p"]:
            best_p[key] = r
    rows = []
    for (n, seed), r in sorted(best_p.items()):
        cost = S.terrain_instance(n, seed)
        opt = S.classical_refs(cost)["opt"]
        assert abs(opt - r["opt"]) < 1e-6 * opt
        res = {}
        for name, fn in (("two_opt", two_opt_restarts), ("annealing", annealing)):
            qs = []
            for k in range(RUNS):
                o = Oracle(cost)
                qs.append(fn(o, n, opt, np.random.default_rng(1000 * seed + k)))
            res[name] = {"median": float(np.median(qs)), "q99": q99(qs)}
        rows.append({
            "n_sites": n, "seed": seed, "qubits": r["qubits"], "p": r["p"],
            "qaoa_queries_no_training": r["shots_to_solution"],
            "qaoa_queries_with_training": r["hw_total_samples"],
            "random_queries": r["uniform_shots_to_solution"],
            "two_opt_q99": res["two_opt"]["q99"], "two_opt_median": res["two_opt"]["median"],
            "annealing_q99": res["annealing"]["q99"], "annealing_median": res["annealing"]["median"],
        })
        print(f"N={n} seed={seed}: QAOA {r['shots_to_solution']:.0f} (no training) | "
              f"2-opt {res['two_opt']['q99']:.0f} | SA {res['annealing']['q99']:.0f} | "
              f"random {r['uniform_shots_to_solution']:.0f}", flush=True)
    json.dump(rows, open(OUT / "query_race.json", "w"), indent=1)
    print("query race done")


if __name__ == "__main__":
    main()
