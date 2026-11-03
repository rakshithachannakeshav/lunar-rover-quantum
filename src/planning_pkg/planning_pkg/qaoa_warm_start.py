#!/usr/bin/env python3
"""
qaoa_warm_start.py
------------------
Warm-started, regularised QAOA on the compact (permutation-index) encoding, and
deep-circuit angle transfer. Companion to qaoa_sim.py (same conventions: bit k of
a basis-state index is qubit k; params = [gamma_1..gamma_p, beta_1..beta_p]).

Warm start (Egger, Marecek & Woerner, Quantum 5, 479 (2021)), regularised as in
RWS-QAOA: each qubit starts tilted towards the bit of a classical seed tour,

    |psi_0> = (x)_k  R_Y(theta_k)|0>,   sin^2(theta_k / 2) = c_k,
    c_k = 1 - eps if bit k of the seed index is 1 else eps,

and the mixer rotates about that same axis, so |psi_0> is its ground state:

    U_M(beta) = (x)_k  R_Y(theta_k) R_Z(-2 beta) R_Y(-theta_k).

eps (0 < eps <= 0.5) is the regulariser: eps -> 0 pins the state to the seed
(QAOA cannot move), eps = 0.5 is the ordinary |+> start and X-like mixer.

Fair use: the seed is nearest-neighbour + 2-opt. The question this module answers
is whether QAOA improves on the tour it was given, so report it on instances
where the seed is NOT already optimal, and count the seed's own queries.

Angle transfer: angles trained on small instances (normalised costs in [0, 1]
make them comparable across sizes) are averaged per depth and reused unchanged
on bigger instances, so deep circuits (p up to ~10) need no training there.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import minimize

from planning_pkg.qaoa_sim import _interp, apply_x_mixer, num_qubits


def seed_index(perms: np.ndarray, order: Sequence[int]) -> int:
    """Lexicographic index of tour `order` among the compact encoding's perms."""
    hit = np.nonzero((perms == np.asarray(order, dtype=perms.dtype)).all(axis=1))[0]
    if len(hit) != 1:
        raise ValueError("order is not one of the encoded permutations.")
    return int(hit[0])


def warm_start_thetas(index: int, q: int, eps: float) -> np.ndarray:
    if not 0.0 < eps <= 0.5:
        raise ValueError("eps must be in (0, 0.5].")
    bits = (index >> np.arange(q)) & 1
    c = np.where(bits == 1, 1.0 - eps, eps)
    return 2.0 * np.arcsin(np.sqrt(c))


def _apply_single_qubit(psi: np.ndarray, U: np.ndarray, k: int) -> None:
    v = psi.reshape(-1, 2, 1 << k)
    a = v[:, 0, :].copy()
    b = v[:, 1, :]
    v[:, 0, :] = U[0, 0] * a + U[0, 1] * b
    v[:, 1, :] = U[1, 0] * a + U[1, 1] * b


def _ry(t: float) -> np.ndarray:
    c, s = math.cos(t / 2), math.sin(t / 2)
    return np.array([[c, -s], [s, c]], dtype=np.complex128)


def _rz(t: float) -> np.ndarray:
    return np.array([[np.exp(-0.5j * t), 0], [0, np.exp(0.5j * t)]], dtype=np.complex128)


def initial_state(thetas: np.ndarray) -> np.ndarray:
    psi = np.ones(1, dtype=np.complex128)
    for t in thetas:                         # qubit k is bit k: build little-endian
        psi = np.kron(np.array([math.cos(t / 2), math.sin(t / 2)]), psi)
    return psi


def ws_qaoa_state(C: np.ndarray, params: np.ndarray, thetas: Optional[np.ndarray]) -> np.ndarray:
    """QAOA state; thetas=None gives the standard |+> start and X mixer."""
    q = num_qubits(C)
    params = np.asarray(params, dtype=float)
    p = len(params) // 2
    if thetas is None:
        psi = np.full(1 << q, 1.0 / math.sqrt(1 << q), dtype=np.complex128)
    else:
        psi = initial_state(thetas)
    for layer in range(p):
        psi *= np.exp(-1j * params[layer] * C)
        beta = float(params[p + layer])
        if thetas is None:
            apply_x_mixer(psi, beta, q)
        else:
            rz = _rz(-2.0 * beta)
            for k, t in enumerate(thetas):
                _apply_single_qubit(psi, _ry(t) @ rz @ _ry(-t), k)
    return psi


def ws_probs(C: np.ndarray, params: np.ndarray, thetas: Optional[np.ndarray]) -> np.ndarray:
    pr = np.abs(ws_qaoa_state(C, params, thetas)) ** 2
    return pr / pr.sum()


class WsObjective:
    """<C> of the (warm-started) QAOA state; counts evaluations."""

    def __init__(self, C: np.ndarray, thetas: Optional[np.ndarray]):
        self.C = np.asarray(C, dtype=float)
        self.thetas = thetas
        self.n_evals = 0

    def __call__(self, params: np.ndarray) -> float:
        self.n_evals += 1
        return float(np.dot(ws_probs(self.C, params, self.thetas), self.C))


@dataclass
class LayerResult:
    p: int
    params: np.ndarray
    objective: float
    cumulative_evals: int


def train_layers(C: np.ndarray, pmax: int, thetas: Optional[np.ndarray] = None,
                 maxiter: int = 60, grid=(10, 8), gamma_max: float = 3.5) -> List[LayerResult]:
    """Grid + COBYLA at p = 1, then INTERP + COBYLA at every deeper p."""
    obj = WsObjective(C, thetas)
    gs = np.linspace(0.05, gamma_max, grid[0])
    bs = np.linspace(0.05, math.pi / 2, grid[1])
    _, g0, b0 = min((obj(np.array([g, b])), g, b) for g in gs for b in bs)
    res = minimize(obj, np.array([g0, b0]), method="COBYLA", options={"maxiter": maxiter})
    x = np.asarray(res.x, float)
    out = [LayerResult(1, x, float(res.fun), obj.n_evals)]
    for p in range(2, pmax + 1):
        res = minimize(obj, _interp(x, p - 1), method="COBYLA", options={"maxiter": maxiter})
        x = np.asarray(res.x, float)
        out.append(LayerResult(p, x, float(res.fun), obj.n_evals))
    return out


def distribution_metrics(prob, probs: np.ndarray, seed_cost: Optional[float] = None) -> Dict:
    """P(optimal), P(better than the seed) and mean cost ratio for a compact problem."""
    m = len(prob.costs)
    per_tour = np.bincount(prob.decode, weights=probs, minlength=m)
    out = {
        "p_opt": float(per_tour[prob.optimal].sum()),
        "mean_ratio": float(np.dot(per_tour, prob.costs)) / prob.opt_cost,
    }
    if seed_cost is not None:
        better = prob.costs < seed_cost - 1e-9 * max(1.0, seed_cost)
        out["p_better_than_seed"] = float(per_tour[better].sum())
    return out
