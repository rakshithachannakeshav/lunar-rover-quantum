#!/usr/bin/env python3
"""
GPU scale-up of the compact-encoding QAOA mission study (CuPy, complex64).

N = 11 -> 26 qubits, N = 12 -> 29 qubits (2^29 amplitudes = 4 GiB on the GPU).
Same instances, encoding, training schedule and metrics as qaoa_mission_study.py;
also runs the black-box query race (2-opt restarts, simulated annealing).

  python3 scripts/qaoa_gpu_scaleup.py --validate        # GPU vs CPU check at N=8
  python3 scripts/qaoa_gpu_scaleup.py --sizes 11 12 --seeds 3
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cupy as cp
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import qaoa_mission_study as S  # noqa: E402
import qaoa_vs_classical_queries as R  # noqa: E402
from planning_pkg.qaoa_sim import _interp, shots_for_confidence  # noqa: E402

OUT = S.OUT

_phase = cp.ElementwiseKernel(
    "float32 C, float32 g", "raw complex64 psi",
    "float a = -g * C; psi[i] = psi[i] * complex<float>(cosf(a), sinf(a));", "qaoa_phase")

_mix = cp.ElementwiseKernel(
    "int64 k, float32 c, float32 s", "raw complex64 psi",
    """
    long long low = i & ((1LL << k) - 1);
    long long i0 = ((i >> k) << (k + 1)) | low;
    long long i1 = i0 | (1LL << k);
    complex<float> a = psi[i0], b = psi[i1];
    complex<float> ms(0.f, -s);
    psi[i0] = c * a + ms * b;
    psi[i1] = ms * a + c * b;
    """, "qaoa_mix")

_expect = cp.ReductionKernel(
    "complex64 psi, float32 w", "float64 out",
    "(double)(psi.real() * psi.real() + psi.imag() * psi.imag()) * (double)w",
    "a + b", "out = a", "0", "qaoa_expect")

_pmask = cp.ReductionKernel(
    "complex64 psi, bool m", "float64 out",
    "m ? (double)(psi.real() * psi.real() + psi.imag() * psi.imag()) : 0.0",
    "a + b", "out = a", "0", "qaoa_pmask")

_norm = cp.ReductionKernel(
    "complex64 psi", "float64 out",
    "(double)(psi.real() * psi.real() + psi.imag() * psi.imag())",
    "a + b", "out = a", "0", "qaoa_norm")


def build_costs_gpu(cost: np.ndarray, chunk: int = 1 << 23):
    """Tour energy for every bitstring b (b -> permutation index b or b - N!, lexicographic)."""
    n = cost.shape[0] - 1
    m = math.factorial(n)
    q = max(1, math.ceil(math.log2(m)))
    costd = cp.asarray(cost, dtype=cp.float64)
    facts = [math.factorial(n - 1 - k) for k in range(n)]
    out = cp.empty(1 << q, dtype=cp.float32)
    for start in range(0, 1 << q, chunk):
        b = cp.arange(start, min(start + chunk, 1 << q), dtype=cp.int64)
        r = cp.where(b < m, b, b - m)
        rows = cp.arange(len(b))
        avail = cp.ones((len(b), n), dtype=cp.int32)
        total = cp.zeros(len(b), dtype=cp.float64)
        prev = cp.zeros(len(b), dtype=cp.int64)
        for k in range(n):
            d = r // facts[k]
            r = r % facts[k]
            pos = cp.argmax(cp.cumsum(avail, axis=1) == (d[:, None] + 1), axis=1)
            avail[rows, pos] = 0
            cur = pos + 1
            total += costd[prev, cur]
            prev = cur
        total += costd[prev, 0]
        out[start:start + len(b)] = total.astype(cp.float32)
        del b, r, rows, avail, total, prev
    return out, m, q


class GpuQaoa:
    def __init__(self, cost: np.ndarray, opt: float):
        t = time.perf_counter()
        costs, self.m, self.q = build_costs_gpu(cost)
        self.lo, self.hi = float(costs.min()), float(costs.max())
        assert abs(self.lo - opt) <= 1e-5 * opt, f"GPU min {self.lo} != exact optimum {opt}"
        self.opt = opt
        self.mask = costs <= self.lo * (1 + 1e-6)
        self.u_opt = float(self.mask.mean())
        self.e_u = float(costs.mean(dtype=cp.float64))
        costs -= self.lo
        costs /= (self.hi - self.lo)
        self.C = costs                       # normalised cost in [0, 1], in place
        self.psi = cp.empty(1 << self.q, dtype=cp.complex64)
        self.n_evals = 0
        self.build_s = time.perf_counter() - t

    def state(self, params):
        p = len(params) // 2
        self.psi.fill(np.complex64(1.0 / math.sqrt(1 << self.q)))
        half = 1 << (self.q - 1)
        for layer in range(p):
            _phase(self.C, np.float32(params[layer]), self.psi)
            beta = float(params[p + layer])
            c, s = np.float32(math.cos(beta)), np.float32(math.sin(beta))
            for k in range(self.q):
                _mix(np.int64(k), c, s, self.psi, size=half)

    def objective(self, params):
        self.n_evals += 1
        self.state(params)
        return float(_expect(self.psi, self.C))

    def metrics(self, params):
        self.state(params)
        norm = float(_norm(self.psi))       # ~1; divides out float32 drift
        c_mean = float(_expect(self.psi, self.C)) / norm
        p_opt = float(_pmask(self.psi, self.mask)) / norm
        e_q = self.lo + (self.hi - self.lo) * c_mean
        return {
            "p_opt": p_opt, "uniform_p_opt": self.u_opt, "amplification": p_opt / self.u_opt,
            "mean_ratio_qaoa": e_q / self.opt, "mean_ratio_uniform": self.e_u / self.opt,
            "gap_closed": (self.e_u - e_q) / (self.e_u - self.opt),
            "shots_to_solution": shots_for_confidence(p_opt),
            "uniform_shots_to_solution": shots_for_confidence(self.u_opt),
        }


def train_by_layer(g: GpuQaoa, pmax: int, maxiter: int = 60):
    gs, bs = np.linspace(0.05, 3.5, 10), np.linspace(0.05, math.pi / 2, 8)
    _, g0, b0 = min((g.objective(np.array([a, b])), a, b) for a in gs for b in bs)
    res = minimize(g.objective, np.array([g0, b0]), method="COBYLA", options={"maxiter": maxiter})
    x = np.asarray(res.x, float)
    out = [(1, x, g.n_evals)]
    for p in range(2, pmax + 1):
        res = minimize(g.objective, _interp(x, p - 1), method="COBYLA", options={"maxiter": maxiter})
        x = np.asarray(res.x, float)
        out.append((p, x, g.n_evals))
    return out


def race(cost, n, opt, seed, runs=300):
    res = {}
    for name, fn in (("two_opt", R.two_opt_restarts), ("annealing", R.annealing)):
        qs = [fn(R.Oracle(cost), n, opt, np.random.default_rng(1000 * seed + k)) for k in range(runs)]
        res[name] = (float(np.percentile(qs, 99)), float(np.median(qs)))
    return res


def validate():
    """GPU simulator vs the CPU (float64) simulator on the same N=8 instance."""
    from planning_pkg import tsp_compact_qaoa as T
    from planning_pkg.qaoa_sim import final_probs, Objective
    cost = S.terrain_instance(8, 0)
    ref = S.classical_refs(cost)
    prob = T.build_compact_problem(cost)
    g = GpuQaoa(cost, ref["opt"])
    dc = float(cp.abs(g.C - cp.asarray(prob.C, dtype=cp.float32)).max())
    print(f"N=8: cost table max |GPU-CPU| = {dc:.2e}  (q={g.q})")
    obj = Objective(prob.C)
    for params in (np.array([0.7, 0.3]), np.array([0.5, 1.1, 0.4, 0.2]), np.array([1.0, 0.6, 2.0, 0.3, 0.9, 0.1])):
        cpu = obj(params)
        gpu = g.objective(params)
        probs = final_probs(prob.C, params)
        p_cpu = T.evaluate_distribution(prob, probs, 10, 0)["p_opt"]
        p_gpu = g.metrics(params)["p_opt"]
        print(f"  p={len(params)//2}: <C> cpu={cpu:.6f} gpu={gpu:.6f} | P(opt) cpu={p_cpu:.6f} gpu={p_gpu:.6f}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--sizes", type=int, nargs="+", default=[11, 12])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--pmax", type=int, default=3)
    args = ap.parse_args(argv)
    if args.validate:
        validate()
        return 0
    out_path = OUT / "gpu_scaleup.json"
    rows = json.load(open(out_path)) if out_path.exists() else []
    done = {(r["n_sites"], r["seed"]) for r in rows}
    for n in args.sizes:
        for seed in range(args.seeds):
            if (n, seed) in done:
                continue
            t0 = time.perf_counter()
            cost = S.terrain_instance(n, seed)
            ref = S.classical_refs(cost)
            g = GpuQaoa(cost, ref["opt"])
            print(f"N={n} seed={seed}: q={g.q}, cost table built in {g.build_s:.1f}s", flush=True)
            trained = train_by_layer(g, args.pmax)
            rc = race(cost, n, ref["opt"], seed)
            for p, params, evals in trained:
                m = g.metrics(params)
                rows.append({"encoding": "compact_gpu", "n_sites": n, "seed": seed, "qubits": g.q,
                             "p": p, "train_evals": evals, **ref, **m,
                             "hw_total_samples": evals * S.SHOTS_PER_EVAL + m["shots_to_solution"],
                             "two_opt_q99": rc["two_opt"][0], "annealing_q99": rc["annealing"][0]})
                print(f"   p={p}: P(opt)={m['p_opt']:.3e} (random {m['uniform_p_opt']:.3e}, "
                      f"{m['amplification']:.1f}x) | QAOA {m['shots_to_solution']:.0f} vs "
                      f"2-opt {rc['two_opt'][0]:.0f} / SA {rc['annealing'][0]:.0f} queries", flush=True)
            json.dump(rows, open(out_path, "w"), indent=1)
            print(f"   done in {time.perf_counter() - t0:.0f}s", flush=True)
            del g
            cp.get_default_memory_pool().free_all_blocks()
    print("gpu scale-up done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
