#!/usr/bin/env python3
"""
qaoa_mission_study.py -- independent study of QAOA on the multi-site lunar mission
(visit N science sites from the base and return; asymmetric uphill-aware costs).

QAOA ALONE: no classical seed tour, no windowing, no Hungarian/marginal repair,
no fallback. Classical solvers are run only as references, after the fact.

Phases (results in results/qaoa_mission_study/):
  compact   compact encoding, ceil(log2 N!) qubits, every bitstring is a tour.
            N = 4..10, p = 1..5. Ideal statevector.
  onehot    standard position-encoded QUBO (Lucas 2014), N^2 qubits.
            N = 3, 4 (p = 1..3) and N = 5 (25 qubits, p = 1).
  hardware  gate counts of the one-hot circuit (what a real device must run)
            and noisy Aer simulations of trained one-hot circuits.

Simulation caveat: a statevector simulator stores the cost of every one of the
2^q basis states, i.e. it enumerates all answers. CPU time therefore measures
the simulator, not QAOA; we report sample counts and circuit sizes instead.

Usage (repo root):  python3 scripts/qaoa_mission_study.py --phase all
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "planning_pkg"))

from planning_pkg import tsp_compact_qaoa  # noqa: E402
from planning_pkg.mission_planner import build_instance, load_cost_map, sample_sites  # noqa: E402
from planning_pkg.qaoa_sim import Objective, _interp, final_probs, shots_for_confidence  # noqa: E402
from planning_pkg.quantum_optimizer import _cost_lookup  # noqa: E402
from planning_pkg.tsp_baselines import held_karp, nn_two_opt  # noqa: E402
from planning_pkg.tsp_qubo import build_tsp_qubo, encode_order  # noqa: E402

OUT = ROOT / "results" / "qaoa_mission_study"
HEIGHTMAP = ROOT / "src" / "rover_simulation" / "worlds" / "heightmap.png"
SHOTS_PER_EVAL = 1000   # shots a real device needs to estimate <C> once
TARGET_CONF = 0.99      # "solved" = 99 % chance to have sampled an optimal tour

tsp_compact_qaoa.MAX_SITES = 10  # N = 10 -> 22 qubits, fits in RAM


# ----------------------------------------------------------------- instances
_GRAPH = None


def terrain_instance(n_sites: int, seed: int) -> np.ndarray:
    """(N+1)x(N+1) leg-energy matrix, index 0 = base at (0, 0)."""
    global _GRAPH
    if _GRAPH is None:
        _GRAPH = load_cost_map(str(HEIGHTMAP))
    cost_map, graph = _GRAPH
    sites = sample_sites(graph, n_sites, seed, min_separation_m=8.0, max_radius_m=45.0)
    return build_instance(graph, cost_map, (0.0, 0.0), sites).leg_costs


def tour_cost(cost: np.ndarray, order) -> float:
    full = [0] + list(order) + [0]
    return float(sum(cost[a, b] for a, b in zip(full[:-1], full[1:])))


def classical_refs(cost: np.ndarray) -> dict:
    t = time.perf_counter(); hk = held_karp(cost, True); t_hk = time.perf_counter() - t
    t = time.perf_counter(); nn = nn_two_opt(cost, True); t_nn = time.perf_counter() - t
    return {"opt": float(hk.cost), "held_karp_ms": t_hk * 1e3,
            "nn2opt_ratio": float(nn.cost) / float(hk.cost), "nn2opt_ms": t_nn * 1e3}


# ----------------------------------------------------------------- training
def train_by_layer(C: np.ndarray, pmax: int, maxiter: int = 60,
                   gammas=None, betas=None):
    """Grid search at p=1, COBYLA, then INTERP to each deeper layer.
    Yields (p, params, objective, cumulative_evals) for every p."""
    obj = Objective(C, 1.0)
    gs = np.linspace(0.05, 3.5, 10) if gammas is None else gammas
    bs = np.linspace(0.05, math.pi / 2, 8) if betas is None else betas
    _, g0, b0 = min((obj(np.array([g, b])), g, b) for g in gs for b in bs)
    res = minimize(obj, np.array([g0, b0]), method="COBYLA", options={"maxiter": maxiter})
    x = np.asarray(res.x, float)
    out = [(1, x, float(res.fun), obj.n_evals)]
    for p in range(2, pmax + 1):
        res = minimize(obj, _interp(x, p - 1), method="COBYLA", options={"maxiter": maxiter})
        x = np.asarray(res.x, float)
        out.append((p, x, float(res.fun), obj.n_evals))
    return out


# ----------------------------------------------------------------- compact
def compact_job(args):
    n, seed, pmax = args
    cost = terrain_instance(n, seed)
    ref = classical_refs(cost)
    prob = tsp_compact_qaoa.build_compact_problem(cost, closed=True)
    assert abs(prob.opt_cost - ref["opt"]) < 1e-6 * ref["opt"], "encoding/optimum mismatch"
    rows = []
    for p, params, _, evals in train_by_layer(prob.C, pmax):
        probs = final_probs(prob.C, params)
        m = tsp_compact_qaoa.evaluate_distribution(prob, probs, SHOTS_PER_EVAL, seed)
        rows.append({
            "encoding": "compact", "n_sites": n, "seed": seed, "qubits": prob.q, "p": p,
            "train_evals": evals, **ref,
            "p_opt": m["p_opt"], "uniform_p_opt": m["uniform_p_opt"],
            "amplification": m["amplification"],
            "mean_ratio_qaoa": m["mean_ratio_qaoa"], "mean_ratio_uniform": m["mean_ratio_uniform"],
            "gap_closed": m["gap_closed"],
            "shots_to_solution": m["shots_to_solution"],
            "uniform_shots_to_solution": m["uniform_shots_to_solution"],
            "hw_total_samples": evals * SHOTS_PER_EVAL + m["shots_to_solution"],
        })
    print(f"[compact] N={n} seed={seed} done", flush=True)
    return rows


# ----------------------------------------------------------------- one-hot
def onehot_problem(cost: np.ndarray):
    k = cost.shape[0] - 1
    tq = build_tsp_qubo(cost, list(range(1, k + 1)), start=0, end=0, penalty=1.5)
    C = _cost_lookup(tq.qubo)                    # QUBO value of every bitstring
    perms = list(itertools.permutations(range(1, k + 1)))
    costs = np.array([tour_cost(cost, o) for o in perms])
    idx = np.array([sum(b << i for i, b in enumerate(encode_order(tq, list(o)))) for o in perms])
    opt = costs.min()
    return tq, C, idx, costs, idx[costs <= opt * (1 + 1e-9)]


def onehot_job(args):
    n, seed, pmax = args
    cost = terrain_instance(n, seed)
    ref = classical_refs(cost)
    tq, C, feas_idx, feas_costs, opt_idx = onehot_problem(cost)
    q = tq.qubo.num_variables
    scale = float(C.std())
    Cs = C / scale                                # angle scale only; same ground states
    rows, params_out = [], {}
    for p, params, _, evals in train_by_layer(Cs, pmax, gammas=np.geomspace(0.02, 3.0, 12)):
        probs = final_probs(Cs, params)
        p_feas = float(probs[feas_idx].sum())
        p_opt = float(probs[opt_idx].sum())
        u_feas = len(feas_idx) / 2 ** q
        u_opt = len(opt_idx) / 2 ** q
        pf = probs[feas_idx]
        mean_feas_ratio = float((pf * feas_costs).sum() / pf.sum() / ref["opt"]) if pf.sum() > 0 else float("nan")
        rows.append({
            "encoding": "onehot", "n_sites": n, "seed": seed, "qubits": q, "p": p,
            "train_evals": evals, **ref,
            "p_feasible": p_feas, "uniform_p_feasible": u_feas, "feasible_amplification": p_feas / u_feas,
            "p_opt": p_opt, "uniform_p_opt": u_opt, "amplification": p_opt / u_opt,
            "mean_ratio_feasible_qaoa": mean_feas_ratio,
            "mean_ratio_feasible_uniform": float(feas_costs.mean() / ref["opt"]),
            "shots_to_solution": shots_for_confidence(p_opt),
            "uniform_shots_to_solution": shots_for_confidence(u_opt),
            "hw_total_samples": evals * SHOTS_PER_EVAL + shots_for_confidence(p_opt),
        })
        params_out[p] = {"gammas": list(params[:p] / scale), "betas": list(params[p:])}
    print(f"[onehot] N={n} seed={seed} q={q} done", flush=True)
    return rows, {"n_sites": n, "seed": seed, "params": params_out}


# ----------------------------------------------------------------- hardware
def hardware_phase(onehot_params):
    from qiskit import transpile
    from qiskit_aer import AerSimulator
    from qiskit_aer.noise import NoiseModel, depolarizing_error, ReadoutError
    from planning_pkg.tsp_qaoa import build_qaoa_circuit

    gates = []
    for n in range(3, 9):
        cost = np.ones((n + 1, n + 1)) - np.eye(n + 1)
        tq = build_tsp_qubo(cost, list(range(1, n + 1)), 0, 0, 1.5)
        for p in (1, 2, 3):
            qc, g, b = build_qaoa_circuit(tq, p)
            qc = qc.assign_parameters({**{x: 0.1 for x in g}, **{x: 0.1 for x in b}})
            t = transpile(qc, basis_gates=["rz", "rx", "h", "cx"], optimization_level=1)
            cx = int(t.count_ops().get("cx", 0))
            gates.append({"n_sites": n, "qubits": n * n, "p": p, "cx": cx, "depth": int(t.depth()),
                          "fidelity_cx_1e-3": (1 - 1e-3) ** cx, "fidelity_cx_5e-3": (1 - 5e-3) ** cx})
    print("[hardware] gate counts done", flush=True)

    noisy = []
    for item in onehot_params:
        n, seed = item["n_sites"], item["seed"]
        if n > 4 or seed != 0:
            continue
        cost = terrain_instance(n, seed)
        tq, _, feas_idx, _, opt_idx = onehot_problem(cost)
        q = tq.qubo.num_variables
        feas, opt = set(int(i) for i in feas_idx), set(int(i) for i in opt_idx)
        for p_str, ang in item["params"].items():
            p = int(p_str)
            qc, g, b = build_qaoa_circuit(tq, p, measure=True)
            qc = qc.assign_parameters({**dict(zip(g, ang["gammas"])), **dict(zip(b, ang["betas"]))})
            for eps in (0.0, 1e-3, 5e-3, 1e-2):
                if eps == 0.0:
                    sim = AerSimulator()
                else:
                    nm = NoiseModel()
                    nm.add_all_qubit_quantum_error(depolarizing_error(eps / 10, 1), ["rz", "rx", "h"])
                    nm.add_all_qubit_quantum_error(depolarizing_error(eps, 2), ["cx"])
                    nm.add_all_qubit_readout_error(ReadoutError([[1 - eps, eps], [eps, 1 - eps]]))
                    sim = AerSimulator(noise_model=nm)
                t = transpile(qc, basis_gates=["rz", "rx", "h", "cx"], optimization_level=1)
                shots = 20000
                counts = sim.run(t, shots=shots, seed_simulator=seed + 11).result().get_counts()
                pf = po = 0
                for s, c in counts.items():
                    i = int(s.replace(" ", ""), 2)   # qiskit string is q[n-1]..q[0] = binary of index
                    pf += c * (i in feas)
                    po += c * (i in opt)
                noisy.append({"n_sites": n, "qubits": q, "p": p, "cx": int(t.count_ops().get("cx", 0)),
                              "error_2q": eps, "p_feasible": pf / shots, "p_opt": po / shots,
                              "uniform_p_feasible": len(feas) / 2 ** q, "uniform_p_opt": len(opt) / 2 ** q})
            print(f"[hardware] noisy N={n} p={p} done", flush=True)
    return gates, noisy


# ----------------------------------------------------------------- main
def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="all", choices=["compact", "onehot", "hardware", "all"])
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--seeds", type=int, default=4)
    args = ap.parse_args(argv)
    OUT.mkdir(parents=True, exist_ok=True)

    if args.phase in ("compact", "all"):
        jobs = [(n, s, 5 if n <= 8 else 3) for n in range(4, 11)
                for s in range(args.seeds if n < 10 else min(3, args.seeds))]
        jobs.sort(key=lambda j: -j[0])          # biggest first for load balance
        with Pool(args.workers) as pool:
            rows = [r for rs in pool.imap_unordered(compact_job, jobs) for r in rs]
        json.dump(rows, open(OUT / "compact.json", "w"), indent=1)

    if args.phase in ("onehot", "all"):
        jobs = [(n, s, 3) for n in (3, 4) for s in range(args.seeds)] + [(5, 0, 1)]
        with Pool(min(args.workers, len(jobs))) as pool:
            res = pool.map(onehot_job, jobs)
        json.dump([r for rs, _ in res for r in rs], open(OUT / "onehot.json", "w"), indent=1)
        json.dump([p for _, p in res], open(OUT / "onehot_params.json", "w"), indent=1)

    if args.phase in ("hardware", "all"):
        params = json.load(open(OUT / "onehot_params.json"))
        gates, noisy = hardware_phase(params)
        json.dump(gates, open(OUT / "hw_gates.json", "w"), indent=1)
        json.dump(noisy, open(OUT / "hw_noisy.json", "w"), indent=1)
    print("study done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
