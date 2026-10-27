#!/usr/bin/env python3
"""
qaoa_sim.py
-----------
Small numpy QAOA simulator for a cost that is *diagonal* in the computational
basis: you give the cost of every one of the 2^q bitstrings as an array C and
it runs  |psi> = prod_l [ exp(-i beta_l sum_k X_k) exp(-i gamma_l C) ] |+>^q.

SIMULATION SHORTCUT (be explicit about this in any write-up): applying
exp(-i gamma C) as a lookup table needs the cost of all 2^q states in memory.
It is exact and fast to simulate (q up to ~20), but on real hardware C would
have to be compiled into gates, whose depth is not what this file measures.
This module needs only numpy and scipy (no Qiskit), so it runs in CI.

Parameters are a flat vector [gamma_1..gamma_p, beta_1..beta_p].
Training: coarse (gamma, beta) grid at p = 1, local COBYLA refinement, INTERP
extrapolation to deeper layers (Zhou et al., PRX 10, 021067), then a few random
restarts at the final depth. The objective is <C> or CVaR_alpha(C).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from scipy.optimize import minimize


def num_qubits(C: np.ndarray) -> int:
    size = len(C)
    q = size.bit_length() - 1
    if size != 1 << q or q < 1:
        raise ValueError("C must have length 2^q with q >= 1.")
    return q


def apply_x_mixer(psi: np.ndarray, beta: float, q: int) -> None:
    """In place: apply exp(-i beta X) to every qubit (bit k = axis of size 2)."""
    c = math.cos(beta)
    s = -1j * math.sin(beta)
    for k in range(q):
        v = psi.reshape(-1, 2, 1 << k)
        a = v[:, 0, :]
        b = v[:, 1, :]
        new0 = c * a + s * b
        new1 = s * a + c * b
        v[:, 0, :] = new0
        v[:, 1, :] = new1


def qaoa_state(C: np.ndarray, params: np.ndarray) -> np.ndarray:
    """Final state for angles `params` = [gammas, betas] (equal halves)."""
    q = num_qubits(C)
    params = np.asarray(params, dtype=float)
    p = len(params) // 2
    psi = np.full(1 << q, 1.0 / math.sqrt(1 << q), dtype=np.complex128)
    for layer in range(p):
        psi *= np.exp(-1j * params[layer] * C)
        apply_x_mixer(psi, float(params[p + layer]), q)
    return psi


def final_probs(C: np.ndarray, params: np.ndarray) -> np.ndarray:
    probs = np.abs(qaoa_state(C, params)) ** 2
    return probs / probs.sum()


class Objective:
    """<C> (alpha = 1) or CVaR_alpha(C) of the QAOA state; counts evaluations."""

    def __init__(self, C: np.ndarray, alpha: float = 1.0):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1].")
        self.C = np.asarray(C, dtype=float)
        self.alpha = alpha
        self.order = np.argsort(self.C, kind="stable")
        self.sorted_C = self.C[self.order]
        self.n_evals = 0

    def __call__(self, params: np.ndarray) -> float:
        self.n_evals += 1
        probs = np.abs(qaoa_state(self.C, params)) ** 2
        if self.alpha >= 1.0:
            return float(np.dot(probs, self.C))
        ps = probs[self.order]
        cum = np.cumsum(ps)
        take = np.minimum(ps, np.maximum(self.alpha - (cum - ps), 0.0))
        return float(np.dot(take, self.sorted_C) / self.alpha)


def _interp(params: np.ndarray, p: int) -> np.ndarray:
    out = []
    for vec in (params[:p], params[p:]):
        ext = np.concatenate([[0.0], vec, [0.0]])
        out.append([((i - 1) / p) * ext[i - 1] + ((p - i + 1) / p) * ext[i]
                    for i in range(1, p + 2)])
    return np.array(out[0] + out[1])


@dataclass
class TrainResult:
    params: np.ndarray
    objective: float
    n_evals: int


def train_qaoa(C: np.ndarray, p: int, alpha: float = 1.0, maxiter: int = 80,
               restarts: int = 1, grid: Tuple[int, int] = (10, 8),
               gamma_max: float = 3.5, seed: int = 0) -> TrainResult:
    """Optimise the 2p angles. Evaluations are counted in the result."""
    if p < 1:
        raise ValueError("p must be >= 1.")
    rng = np.random.default_rng(seed)
    obj = Objective(C, alpha)

    gs = np.linspace(0.05, gamma_max, grid[0])
    bs = np.linspace(0.05, math.pi / 2, grid[1])
    scored = [(obj(np.array([g, b])), g, b) for g in gs for b in bs]
    _, g0, b0 = min(scored)
    res = minimize(obj, np.array([g0, b0]), method="COBYLA", options={"maxiter": maxiter})
    best_x, best_f = np.asarray(res.x, float), float(res.fun)

    for layer in range(2, p + 1):
        x0 = _interp(best_x, layer - 1)
        res = minimize(obj, x0, method="COBYLA", options={"maxiter": maxiter})
        best_x, best_f = np.asarray(res.x, float), float(res.fun)

    for _ in range(restarts):
        x0 = np.concatenate([rng.uniform(0.05, gamma_max, p),
                             rng.uniform(0.05, math.pi / 2, p)])
        res = minimize(obj, x0, method="COBYLA", options={"maxiter": maxiter})
        if res.fun < best_f:
            best_x, best_f = np.asarray(res.x, float), float(res.fun)
    return TrainResult(best_x, best_f, obj.n_evals)


def sample_indices(probs: np.ndarray, shots: int, rng: np.random.Generator) -> np.ndarray:
    """`shots` measured bitstring indices from the distribution `probs`."""
    probs = np.asarray(probs, dtype=float)
    return rng.choice(len(probs), size=shots, p=probs / probs.sum())


def shots_for_confidence(p_success: float, confidence: float = 0.99) -> float:
    """Samples needed to see an optimal answer at least once with `confidence`,
    if each sample is optimal with probability p_success."""
    if p_success <= 0.0:
        return math.inf
    if p_success >= 1.0:
        return 1.0
    return math.log(1.0 - confidence) / math.log(1.0 - p_success)
