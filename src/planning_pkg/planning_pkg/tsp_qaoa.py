#!/usr/bin/env python3
"""
tsp_qaoa.py
-----------
QAOA (Qiskit-Aer) solver for the visiting-order QUBO in tsp_qubo.

Pipeline for one QUBO:
  1. Ising form via quantum_optimizer.qubo_to_ising; a parameterised circuit
     H^n, then p layers of [RZ(2*gamma*h_i), RZZ(2*gamma*J_ij), RX(2*beta)].
  2. Angles are trained on the exact Aer statevector, minimising <H> (or the
     CVaR_alpha of H when cvar_alpha < 1). Layer 1 starts from the best point
     of a coarse (gamma, beta) grid; layer p > 1 starts from the INTERP
     extrapolation of the layer p-1 optimum (Zhou et al., PRX 10, 021067), plus
     `restarts` random starts. Optimiser: any scipy.optimize.minimize method
     or 'SPSA'.
  3. The trained circuit is sampled with `shots` on AerSimulator. The answer is
     the cheapest *feasible* sampled tour. If no sample is feasible, the tour is
     repaired from per-qubit marginals and flagged `repaired=True` - such a
     result owes as much to classical post-processing as to QAOA.

The full QUBO needs N^2 qubits; statevector simulation is limited by
`max_qubits` (default 20, i.e. N <= 4). For larger N, `solve_tour_qaoa_windowed`
re-orders consecutive windows of `window` sites of a seed tour with QAOA
(window^2 qubits each) - a decomposition heuristic, reported separately.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize

from planning_pkg.quantum_optimizer import _cost_lookup, _decode_bitstring, qubo_to_ising
from planning_pkg.tsp_baselines import TourResult, n_sites, path_cost, tour_cost, validate_tour
from planning_pkg.tsp_qubo import (
    TspQubo,
    build_tsp_qubo,
    decode_bits,
    repair_from_marginals,
)

SCIPY_OPTIMIZERS = ("COBYLA", "Nelder-Mead", "Powell", "L-BFGS-B", "SLSQP")


@dataclass
class QaoaConfig:
    reps: int = 1
    optimizer: str = "COBYLA"
    maxiter: int = 150
    restarts: int = 2
    shots: int = 4096
    seed: int = 7
    penalty: float = 1.5
    cvar_alpha: float = 1.0
    max_qubits: int = 20
    grid_points: int = 12

    def validate(self) -> None:
        if self.reps < 1:
            raise ValueError("reps must be >= 1.")
        if self.shots < 1:
            raise ValueError("shots must be >= 1.")
        if not 0.0 < self.cvar_alpha <= 1.0:
            raise ValueError("cvar_alpha must be in (0, 1].")
        if self.optimizer not in SCIPY_OPTIMIZERS + ("SPSA",):
            raise ValueError(
                f"optimizer must be one of {SCIPY_OPTIMIZERS + ('SPSA',)}; got {self.optimizer!r}."
            )


class QubitLimitError(ValueError):
    """The QUBO needs more qubits than the configured simulation limit."""


@dataclass
class QaoaRun:
    """Outcome of QAOA on one QUBO (full tour or a single window)."""

    order: List[int]
    cost: float
    repaired: bool
    p_feasible: float
    tour_counts: Dict[Tuple[int, ...], int]
    mean_feasible_cost: float
    expectation: float
    params: List[float]
    n_evals: int
    qubits: int
    depth: int
    two_qubit_gates: int
    runtime_s: float

    def summary(self) -> Dict[str, Any]:
        d = asdict(self)
        d.pop("tour_counts")
        return d


def _require_aer():
    try:
        from qiskit import QuantumCircuit, transpile
        from qiskit.circuit import ParameterVector
        from qiskit_aer import AerSimulator
    except ImportError as exc:
        raise RuntimeError(
            "QAOA needs qiskit and qiskit-aer: pip install -r requirements.txt"
        ) from exc
    return QuantumCircuit, transpile, ParameterVector, AerSimulator


def build_qaoa_circuit(tq: TspQubo, reps: int, measure: bool = False):
    """Parameterised QAOA ansatz. Returns (circuit, gammas, betas)."""
    QuantumCircuit, _, ParameterVector, _ = _require_aer()
    n = tq.qubo.num_variables
    h, zz = qubo_to_ising(tq.qubo)
    gammas = ParameterVector("gamma", reps)
    betas = ParameterVector("beta", reps)
    qc = QuantumCircuit(n)
    qc.h(range(n))
    for layer in range(reps):
        for i, coeff in sorted(h.items()):
            if coeff != 0.0:
                qc.rz(2.0 * gammas[layer] * coeff, i)
        for (i, j), coeff in sorted(zz.items()):
            if coeff != 0.0:
                qc.rzz(2.0 * gammas[layer] * coeff, i, j)
        for i in range(n):
            qc.rx(2.0 * betas[layer], i)
    if measure:
        qc.measure_all()
    return qc, gammas, betas


def circuit_stats(tq: TspQubo, reps: int) -> Dict[str, int]:
    """Qubits, depth and CX count after decomposing to {h, rz, rx, cx}.

    No hardware coupling map is assumed (all-to-all), so routing overhead on
    real devices would come on top of these numbers.
    """
    _, transpile, _, _ = _require_aer()
    qc, _, _ = build_qaoa_circuit(tq, reps, measure=False)
    basis = transpile(qc, basis_gates=["h", "rz", "rx", "cx"], optimization_level=0)
    ops = basis.count_ops()
    return {
        "qubits": qc.num_qubits,
        "depth": int(basis.depth()),
        "two_qubit_gates": int(ops.get("cx", 0)),
    }


class _StatevectorObjective:
    """<H> or CVaR_alpha(H) of the QAOA state, from the Aer statevector."""

    def __init__(self, tq: TspQubo, reps: int, cvar_alpha: float, seed: int):
        _, transpile, _, AerSimulator = _require_aer()
        qc, self.gammas, self.betas = build_qaoa_circuit(tq, reps)
        qc.save_statevector()
        self.sim = AerSimulator(method="statevector", seed_simulator=seed)
        self.compiled = transpile(qc, self.sim)
        self.reps = reps
        self.costs = _cost_lookup(tq.qubo)
        self.order = np.argsort(self.costs, kind="stable")
        self.cvar_alpha = cvar_alpha
        self.n_evals = 0

    def probabilities(self, params: np.ndarray) -> np.ndarray:
        binding = dict(zip(self.gammas, params[:self.reps]))
        binding.update(zip(self.betas, params[self.reps:]))
        bound = self.compiled.assign_parameters(binding)
        sv = self.sim.run(bound).result().get_statevector()
        return np.abs(np.asarray(sv)) ** 2

    def __call__(self, params: np.ndarray) -> float:
        self.n_evals += 1
        probs = self.probabilities(np.asarray(params, dtype=float))
        if self.cvar_alpha >= 1.0:
            return float(np.dot(probs, self.costs))
        p_sorted = probs[self.order]
        c_sorted = self.costs[self.order]
        cum = np.cumsum(p_sorted)
        take = np.minimum(p_sorted, np.maximum(self.cvar_alpha - (cum - p_sorted), 0.0))
        return float(np.dot(take, c_sorted) / self.cvar_alpha)


def _spsa(fun, x0: np.ndarray, maxiter: int, rng: np.random.Generator) -> Tuple[np.ndarray, float]:
    """Plain SPSA (Spall 1998 gain schedule) returning the best point seen."""
    x = np.array(x0, dtype=float)
    a, c, alpha, gamma_exp, big_a = 0.2, 0.1, 0.602, 0.101, 0.1 * maxiter
    best_x, best_f = x.copy(), fun(x)
    for k in range(maxiter):
        ak = a / (k + 1 + big_a) ** alpha
        ck = c / (k + 1) ** gamma_exp
        delta = rng.choice([-1.0, 1.0], size=x.shape)
        grad = (fun(x + ck * delta) - fun(x - ck * delta)) / (2.0 * ck) * delta
        x = x - ak * grad
        f = fun(x)
        if f < best_f:
            best_x, best_f = x.copy(), f
    return best_x, best_f


def _local_minimize(fun, x0: np.ndarray, cfg: QaoaConfig,
                    rng: np.random.Generator) -> Tuple[np.ndarray, float]:
    if cfg.optimizer == "SPSA":
        return _spsa(fun, x0, cfg.maxiter, rng)
    res = minimize(fun, x0, method=cfg.optimizer, options={"maxiter": cfg.maxiter})
    return np.asarray(res.x, dtype=float), float(res.fun)


def _interp(params: np.ndarray, p: int) -> np.ndarray:
    """INTERP initialisation: extend optimal depth-p angles to depth p+1."""
    out = []
    for vec in (params[:p], params[p:]):
        ext = np.concatenate([[0.0], vec, [0.0]])
        new = [((i - 1) / p) * ext[i - 1] + ((p - i + 1) / p) * ext[i]
               for i in range(1, p + 2)]
        out.append(new)
    return np.array(out[0] + out[1])


def train_qaoa(tq: TspQubo, cfg: QaoaConfig) -> Tuple[np.ndarray, float, int]:
    """Optimise angles for depth cfg.reps. Returns (params, objective, n_evals)."""
    rng = np.random.default_rng(cfg.seed)
    total_evals = 0

    # Depth 1: coarse grid, then local refinement.
    obj = _StatevectorObjective(tq, 1, cfg.cvar_alpha, cfg.seed)
    grid_g = np.linspace(0.05, math.pi / 2, cfg.grid_points)
    grid_b = np.linspace(0.05, math.pi / 2, cfg.grid_points)
    scores = [(obj(np.array([g, b])), g, b) for g in grid_g for b in grid_b]
    _, g0, b0 = min(scores)
    best_x, best_f = _local_minimize(obj, np.array([g0, b0]), cfg, rng)
    total_evals += obj.n_evals

    for p in range(2, cfg.reps + 1):
        obj = _StatevectorObjective(tq, p, cfg.cvar_alpha, cfg.seed)
        best_x, best_f = _local_minimize(obj, _interp(best_x, p - 1), cfg, rng)
        total_evals += obj.n_evals

    if cfg.restarts > 0:
        obj = _StatevectorObjective(tq, cfg.reps, cfg.cvar_alpha, cfg.seed)
        for _ in range(cfg.restarts):
            x0 = np.concatenate([rng.uniform(0, math.pi / 2, cfg.reps),
                                 rng.uniform(0, math.pi / 2, cfg.reps)])
            x, f = _local_minimize(obj, x0, cfg, rng)
            if f < best_f:
                best_x, best_f = x, f
        total_evals += obj.n_evals
    return best_x, best_f, total_evals


def run_qaoa_on_qubo(cost: np.ndarray, tq: TspQubo, cfg: QaoaConfig) -> QaoaRun:
    """Train, sample and decode QAOA for one QUBO."""
    cfg.validate()
    n = tq.qubo.num_variables
    if n > cfg.max_qubits:
        raise QubitLimitError(
            f"QUBO needs {n} qubits > max_qubits={cfg.max_qubits} (statevector limit)."
        )
    _, transpile, _, AerSimulator = _require_aer()
    started = time.perf_counter()

    params, objective, n_evals = train_qaoa(tq, cfg)

    qc, gammas, betas = build_qaoa_circuit(tq, cfg.reps, measure=True)
    binding = dict(zip(gammas, params[:cfg.reps]))
    binding.update(zip(betas, params[cfg.reps:]))
    sim = AerSimulator(seed_simulator=cfg.seed)
    compiled = transpile(qc.assign_parameters(binding), sim)
    counts = sim.run(compiled, shots=cfg.shots).result().get_counts()

    tour_counts: Dict[Tuple[int, ...], int] = {}
    marginals = np.zeros(n)
    for bitstring, count in counts.items():
        bits = _decode_bitstring(bitstring, n)
        marginals += count * np.asarray(bits)
        order = decode_bits(tq, bits)
        if order is not None:
            tour_counts[tuple(order)] = tour_counts.get(tuple(order), 0) + count
    marginals /= cfg.shots

    def leg_cost(order: Sequence[int]) -> float:
        return path_cost(cost, tq.start, order, tq.end)

    feasible_shots = sum(tour_counts.values())
    if tour_counts:
        best = min(tour_counts, key=lambda o: (leg_cost(o), o))
        order, repaired = list(best), False
        mean_cost = sum(leg_cost(o) * c for o, c in tour_counts.items()) / feasible_shots
    else:
        order, repaired = repair_from_marginals(tq, marginals), True
        mean_cost = float("nan")

    stats = circuit_stats(tq, cfg.reps)
    return QaoaRun(
        order=order,
        cost=leg_cost(order),
        repaired=repaired,
        p_feasible=feasible_shots / cfg.shots,
        tour_counts=tour_counts,
        mean_feasible_cost=float(mean_cost),
        expectation=float(objective),
        params=[float(v) for v in params],
        n_evals=n_evals,
        qubits=stats["qubits"],
        depth=stats["depth"],
        two_qubit_gates=stats["two_qubit_gates"],
        runtime_s=time.perf_counter() - started,
    )


def solve_tour_qaoa(cost: np.ndarray, closed: bool = True,
                    cfg: Optional[QaoaConfig] = None) -> TourResult:
    """QAOA on the full N-site QUBO (N^2 qubits). Raises QubitLimitError."""
    cfg = cfg or QaoaConfig()
    n = n_sites(cost)
    tq = build_tsp_qubo(cost, range(1, n + 1), start=0, end=0 if closed else None,
                        penalty=cfg.penalty)
    run = run_qaoa_on_qubo(cost, tq, cfg)
    validate_tour(run.order, n)
    info = run.summary()
    info["tour_counts"] = {",".join(map(str, o)): c for o, c in run.tour_counts.items()}
    return TourResult("qaoa", run.order, tour_cost(cost, run.order, closed),
                      run.runtime_s, info)


def solve_tour_qaoa_windowed(
    cost: np.ndarray,
    initial_order: Sequence[int],
    closed: bool = True,
    cfg: Optional[QaoaConfig] = None,
    window: int = 3,
    max_sweeps: int = 2,
) -> TourResult:
    """Improve a seed tour by QAOA-reordering each window of `window` consecutive
    sites, with the window's neighbours held fixed. A window's new order is
    kept only if it is strictly cheaper. Sweeps repeat until no window improves
    or `max_sweeps` is reached."""
    cfg = cfg or QaoaConfig()
    n = n_sites(cost)
    validate_tour(initial_order, n)
    window = min(window, n)
    if window < 2:
        raise ValueError("window must be >= 2.")
    started = time.perf_counter()
    order = list(initial_order)
    runs: List[QaoaRun] = []
    improvements = 0
    sweeps = 0
    for sweeps in range(1, max_sweeps + 1):
        improved = False
        for s in range(0, n - window + 1):
            inner = order[s:s + window]
            prev = 0 if s == 0 else order[s - 1]
            if s + window < n:
                nxt: Optional[int] = order[s + window]
            else:
                nxt = 0 if closed else None
            tq = build_tsp_qubo(cost, inner, start=prev, end=nxt, penalty=cfg.penalty)
            run = run_qaoa_on_qubo(cost, tq, cfg)
            runs.append(run)
            if run.cost < path_cost(cost, prev, inner, nxt) - 1e-9:
                order[s:s + window] = run.order
                improved = True
                improvements += 1
        if not improved:
            break
    validate_tour(order, n)
    info = {
        "window": window,
        "windows_solved": len(runs),
        "sweeps": sweeps,
        "improvements": improvements,
        "seed_cost": tour_cost(cost, initial_order, closed),
        "qubits": max(r.qubits for r in runs),
        "depth": max(r.depth for r in runs),
        "two_qubit_gates": max(r.two_qubit_gates for r in runs),
        "p_feasible": float(np.mean([r.p_feasible for r in runs])),
        "repaired_windows": int(sum(r.repaired for r in runs)),
        "n_evals": int(sum(r.n_evals for r in runs)),
    }
    return TourResult("qaoa_window", order, tour_cost(cost, order, closed),
                      time.perf_counter() - started, info)
