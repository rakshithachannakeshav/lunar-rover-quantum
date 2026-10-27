# Empirical Results, Resource Scaling, and Defensible Limitations

This document synthesizes the experimental benchmarks, theoretical complexity scalings, and physical limitations of the quantum navigation modules developed in the `lunar-rover-quantum` project.

---

## 1. Executive Summary of Experimental Findings

> [!IMPORTANT]
> **No Quantum Advantage Claimed**: Across point-to-point path planning, multi-site traveling salesperson routing (TSP), and constrained science-site orienteering, the simulated quantum algorithms (QAOA and Dürr-Høyer minimum finding) **do not outperform state-of-the-art classical algorithms**. Classical exact methods (Dijkstra, A*, Held-Karp dynamic programming, and orienteering DP) and polynomial-time heuristics (Nearest Neighbour with 2-opt and random restarts) consistently deliver equal or superior solution quality in orders-of-magnitude less runtime and without classical training overhead.

| Formulation | Problem Size | Classical Baseline Metric | Quantum (QAOA / Dürr-Høyer) Metric | Quantum vs. Classical Reality |
|---|---|---|---|---|
| **Point-to-Point Path Selection** | 8–12 macro edges (corridor around A*) | A* runtime: **~0.25 ms**; Energy: 100% optimal | QAOA runtime: **~5.5 s** (~22,000× slower); Energy ratio: **1.0000** | QAOA matches the A* baseline cost exactly but provides zero energy improvement over A*. |
| **Energy vs. Distance-Only Baseline** | 200 random terrain routes | Mean energy saving: **0.24%** (max 8.5%) | Identical to classical energy-aware A* | Savings against Euclidean shortest path are small because the simulated DART heightmap terrain is flat. |
| **Multi-Site TSP (Full One-Hot QAOA)** | $N=4$ sites ($N^2 = 16$ qubits) | NN + 2-opt: **0.23 ms**, 5/5 optimal; Held-Karp: **0.5 ms** | QAOA: **14.7 s** (~77,000× slower); 2/15 runs suboptimal; ratio: 1.000–1.034 | QAOA never beats NN + 2-opt (tied 13/15, worse 2/15). Best-of-random sampling with identical sample count matches QAOA. |
| **Multi-Site TSP (Windowed QAOA)** | $N=5 \dots 8$ sites (9 qubits, 3-site sliding window) | NN + 2-opt ratio: **1.000–1.011**; Exact window: **<1 ms** | QAOA window ratio: **1.000–1.029**; Worse than NN+2-opt on **16/75** runs | QAOA window achieves the exact same cost as classical window enumeration on 74/75 runs; classical 2-opt remains superior. |
| **Science Orienteering (Energy Budget)** | $n=6 \dots 14$ sites ($n$ qubits, exact DP cost) | Exact DP: **<0.42 s** on all instances; Greedy+LS suboptimal on 15/80 | QAOA amplifies optimal subset **6× to 200×** over uniform random bitstrings | QAOA beats random sampling, but conditional amplification among feasible subsets is only **1.05×–14.5×** (mostly 1–3×). Training requires ~400 circuit evaluations vs. 1 pass of exact DP. |
| **Compact-Encoding QAOA** | $N=5 \dots 8$ sites ($\lceil \log_2 N! \rceil$ qubits: 7 to 16 qubits) | Held-Karp: **<12 ms**; 2-opt $\times 20$ restarts: 100% optimal | Closes **10%–27%** of gap-to-optimal at $N=8$, up to **70%** at $N=5–7$ | Every bitstring is valid (no constraint penalties). Median gap closed across all runs is **0.25**. Parameter transfer between problem sizes **fails** (gap closed drops below 0). |
| **Oracle Query Complexity (Dürr-Høyer)** | $M = N!$ permutations ($N=4 \dots 20$) | Brute-force: $M$ queries; Held-Karp: $O(N^2 2^N)$ operations | Dürr-Høyer queries: **$\approx 2.7 \sqrt{N!}$**; Empirical exponent: **0.499** | Confirms theoretical quadratic speedup over *brute-force search only*. Does not beat Held-Karp dynamic programming or heuristics. |

---

## 2. Multi-Site Traveling Salesperson Problem (TSP)

### 2.1 One-Hot Encoding ($N^2$ Qubits)
In the standard one-hot permutation encoding, a binary variable $x_{i,t} = 1$ indicates that site $i$ is visited at time step $t$. This formulation requires:
- **Qubit Count**: $Q = N^2$. For $N=4 \implies 16$ qubits; for $N=5 \implies 25$ qubits; for $N=8 \implies 64$ qubits.
- **Valid Permutation Subspace**: Out of $2^{N^2}$ possible bitstrings, exactly $N!$ represent valid tours. At $N=4$, this is $24 / 65536 \approx 0.0366\%$.
- **Objective Function**:
  $$\min \sum_{t=0}^{N-1} \sum_{i \ne j} c_{ij} x_{i,t} x_{j,(t+1)\bmod N} + \lambda \sum_{t} \left(1 - \sum_i x_{i,t}\right)^2 + \lambda \sum_{i} \left(1 - \sum_t x_{i,t}\right)^2$$
  where leg costs $c_{ij}$ are asymmetric (computed via A* on the heightmap cost map incorporating distance, slope, and solar shadow).

### 2.2 Detailed Full QAOA Experimental Breakdown ($N=4$, 16 Qubits)
Empirical results across 15 benchmark runs (depth $p=1$, CVaR $\alpha \in \{0.2, 0.5, 1.0\}$, 4096 shots, Qiskit `AerSimulator`):
- **Feasible Tour Enrichment**: QAOA increases the probability of sampling a valid permutation from the uniform random baseline of $0.0366\%$ to $1.09\% - 2.05\%$ (an enrichment factor of $30\times - 62\times$).
- **Cost Discrimination (Conditional Amplification)**:
  $$\text{Conditional Amplification} = \frac{P(\text{optimal} \mid \text{feasible})}{P_{\text{uniform}}(\text{optimal} \mid \text{feasible})} \approx 0.54 - 1.08$$
  A conditional amplification near $1.0$ proves that **once a sampled bitstring satisfies the tour constraints, QAOA chooses the optimal tour no more frequently than picking a valid tour uniformly at random**.
- **Mean Feasible Tour Quality**: The expected tour cost ratio among feasible samples drawn from QAOA is $1.138\times$ optimal, compared to $1.142\times$ optimal for a uniformly random permutation. QAOA primarily learns to satisfy the row/column constraint penalties, not to optimize path cost.
- **Comparison to Classical Heuristics**: Classical Nearest Neighbour with 2-opt found the exact optimum on 5/5 instances in $0.23\text{ ms}$, whereas QAOA took $14.7\text{ s}$ per instance and failed to reach the optimum on 2 of 15 runs.

---

## 3. Compact Permutation Encoding QAOA

### 3.1 Mathematical Formulation
To overcome the $N^2$ scaling and the constraint-penalty burden of one-hot QUBOs, the compact encoding maps each integer permutation index $k \in \{0, 1, \dots, N!-1\}$ to an unconstrained binary string of length:
$$Q_{\text{compact}} = \left\lceil \log_2(N!) \right\rceil$$
For bitstrings representing integers $k \ge N!$, costs are wrapped cyclically ($k \bmod N!$). 
- At $N=8$, compact QAOA requires **16 qubits** instead of 64 qubits.
- **Every bitstring maps directly to a valid tour**. No penalty multipliers or Lagrange constraints are required.
- Standard transverse-field $X$ mixer: $H_M = \sum_i X_i$.

### 3.2 Performance and Parameter Optimization
The cost gap closed is defined as:
$$\text{Gap Closed} = \frac{\mathbb{E}[C_{\text{uniform}}] - \mathbb{E}[C_{\text{QAOA}}]}{\mathbb{E}[C_{\text{uniform}}] - C_{\text{optimal}}}$$

| Number of Sites ($N$) | Qubits Compact | Qubits One-Hot | Best Config | $P(\text{opt})$ QAOA | $P(\text{opt})$ Uniform | Amplification | Gap Closed | QAOA Shots ($99\%$) | Brute Force ($N!$) |
|---|---|---|---|---|---|---|---|---|---|
| **$N=5$** | 7 | 25 | $p=4, \alpha=0.2$ | 0.1003 | 0.0156 | $6.4\times$ | 0.50 | 44 | 120 |
| **$N=6$** | 10 | 36 | $p=4, \alpha=0.2$ | 0.0413 | 0.0024 | $17.0\times$ | 0.58 | 109 | 720 |
| **$N=7$** | 13 | 49 | $p=4, \alpha=1.0$ | 0.0088 | 0.00046 | $19.2\times$ | 0.69 | 520 | 5,040 |
| **$N=8$** | 16 | 64 | $p=3, \alpha=0.2$ | 0.00025 | 0.00005 | $5.3\times$ | 0.22 | 18,592 | 40,320 |

> [!NOTE]
> **Optimistic Best-Configuration Bias**: The "best configuration" rows were selected *post hoc* by identifying the highest-performing hyperparameters ($p, \alpha$) on the evaluated test set. When averaged across all depths ($p=1 \dots 4$) and CVaR cutoffs, the median gap closed is **0.25**.

### 3.3 Depth-Averaged Gap Closed Table
Averaged across all synthetic and terrain instance families:

| Problem Size ($N$) | $p=1, \alpha=0.2$ | $p=1, \alpha=1.0$ | $p=2, \alpha=0.2$ | $p=2, \alpha=1.0$ | $p=3, \alpha=0.2$ | $p=3, \alpha=1.0$ | $p=4, \alpha=0.2$ | $p=4, \alpha=1.0$ |
|---|---|---|---|---|---|---|---|---|
| **$N=5$** | 0.20 | 0.24 | 0.38 | 0.46 | 0.37 | 0.49 | 0.41 | 0.51 |
| **$N=6$** | 0.14 | 0.17 | 0.32 | 0.26 | 0.44 | 0.36 | 0.42 | 0.40 |
| **$N=7$** | 0.15 | 0.13 | 0.20 | 0.23 | 0.35 | 0.49 | 0.38 | 0.55 |
| **$N=8$** | 0.06 | 0.12 | 0.10 | 0.16 | 0.15 | 0.21 | — | — |

### 3.4 Parameter Transferability Failure
To test whether classical training overhead could be eliminated by transferring optimal angles $(\boldsymbol{\gamma}, \boldsymbol{\beta})$ trained on $N=6$ to larger instances ($N=7$ and $N=8$):
- **Transferred Gap Closed at $N=7$**: Mean gap closed dropped to $-0.04$ (worse than uniform random sampling).
- **Transferred Gap Closed at $N=8$**: Mean gap closed dropped to $-0.04$.
- **Median Degradation**: $\Delta(\text{gap closed}) = -0.23$.
- **Conclusion**: The phase landscape of the compact encoding is deeply sensitive to $N$ and the lexicographical index topology; parameters do not generalize across problem dimensions.

---

## 4. Science Site Orienteering Under Battery Budget

The rover must select a subset of candidate scientific targets $S \subseteq \{1, \dots, n\}$ that maximizes cumulative scientific value $\sum_{i \in S} R_i$ subject to the total transit energy remaining below the battery allowance $B$:
$$\text{Cost}(x) = -\sum_{i=1}^n R_i x_i + \lambda \max\left(0, \text{TourEnergy}(x) - B\right)^2$$
- Each site is assigned 1 qubit ($Q = n$).
- Transit costs between subsets are evaluated via exact dynamic programming.
- Exact classical DP solves all instances up to $n=14$ in **$<0.42\text{ s}$**.

### 4.1 Heuristic Failure vs. Exact DP
Classical Greedy with 2-opt local search is suboptimal on **15 out of 80** benchmark instances. However, because exact DP solves the problem instantly, heuristic shortcomings do not justify quantum intervention.

### 4.2 QAOA Subset Selection Results

| Sites ($n$) | Mode | Best Config | $P(\text{opt})$ | Uniform $P(\text{opt})$ | Feasible Subsets | Amplification | Conditional Amp. | QAOA Shots ($99\%$) | Training Evals | Brute Force ($2^n$) |
|---|---|---|---|---|---|---|---|---|---|---|
| **6** | plain | $p=3, \alpha=0.5$ | 0.0912 | 0.0156 | 15.0% | $5.8\times$ | 1.38 | 48 | 378 | 64 |
| **8** | plain | $p=2, \alpha=1.0$ | 0.0243 | 0.0039 | 7.9% | $6.2\times$ | 1.15 | 187 | 306 | 256 |
| **10** | plain | $p=3, \alpha=0.5$ | 0.0317 | 0.0010 | 4.1% | $32.5\times$ | 2.64 | 143 | 399 | 1,024 |
| **12** | plain | $p=3, \alpha=1.0$ | 0.0075 | 0.00024 | 3.4% | $30.7\times$ | 3.17 | 612 | 398 | 4,096 |
| **14** | plain | $p=3, \alpha=1.0$ | 0.0123 | 0.00006 | 2.0% | $201.9\times$ | 14.52 | 371 | 400 | 16,384 |

**The Hidden Overhead**: At $n=14$, QAOA requires 371 post-training measurement shots to achieve a 99% probability of observing the optimal subset. However, training the parameters via classical COBYLA requires **~400 statevector evaluations**. On real hardware with 1,000 shots per training iteration, this equates to **~400,000 physical shots**, compared to only 16,384 evaluations for complete classical brute force and 1 pass for exact DP.

---

## 5. Quantum Query Complexity (Dürr-Høyer Algorithm)

Simulating the Dürr-Høyer minimum-finding algorithm using the exact analytical Grover rotation probability yields:
$$\text{Mean Queries} \approx 2.7 \sqrt{N!}$$
- **Fitted Slope on $\log(\text{Queries})$ vs $\log(N!)$**: **0.499** (matches theoretical $\mathcal{O}(\sqrt{M})$ prediction).
- **Comparison to Held-Karp**: Held-Karp requires $O(N^2 2^N)$ arithmetic steps. While raw query counts for Dürr-Høyer drop below Held-Karp step counts around $N=17$, this comparison mixes abstract oracle queries with arithmetic operations. Each quantum oracle evaluation requires executing a coherent quantum circuit of depth proportional to the cost evaluation, introducing substantial physical overhead.

---

## 6. Quantum Resource Scaling Analysis

The figure below (generated by `scripts/plot_scaling_analysis.py`) highlights the resource scaling limits across all studied formulations:

![Quantum Resource Scaling](file:///e:/Downloads/lunar-rover-quantum%20(2)%20(1)/lunar-rover-quantum/results/scaling_analysis.png)

### 6.1 Statevector Memory Limits
A statevector of $Q$ qubits requires $2^Q$ complex amplitudes. Using standard 128-bit complex floats (`np.complex128`), the statevector memory is:
$$\text{Memory} = 16 \times 2^Q \text{ bytes}$$
- **$Q = 16$ ($N=4$ one-hot, $N=8$ compact)**: $16 \times 65,536 = 1.05\text{ MB}$ (readily simulated on laptop CPU).
- **$Q = 20$**: $16\text{ MB}$ (practical interactive simulation threshold).
- **$Q = 25$ ($N=5$ one-hot)**: $536.8\text{ MB}$.
- **$Q = 30$**: $17.18\text{ GB}$ (standard workstation RAM ceiling).
- **$Q = 36$ ($N=6$ one-hot)**: $1.1\text{ TB}$.
- **$Q = 64$ ($N=8$ one-hot)**: $2.95 \times 10^{20}\text{ bytes} \approx 295,000\text{ PB}$ (physically impossible to simulate).

### 6.2 The Simulation Shortcut
> [!WARNING]
> In `qaoa_sim.py` and `orienteering_qaoa.py`, the cost Hamiltonian phase-separation step is applied as a **classical diagonal statevector lookup table**:
> $$|\psi'\rangle = e^{-i \gamma C(z)} |\psi\rangle$$
> This is a **simulation shortcut**, not a gate-level circuit. On fault-tolerant quantum hardware, evaluating $C(z)$ for the compact encoding would require synthesizing a binary-to-permutation decoder circuit and an arithmetic accumulator, demanding thousands of coherent Toffoli and CNOT gates. The qubit savings of the compact encoding represent a state-space compression, but not a gate-level efficiency gain.

---

## 7. What Would Have to Change for Quantum to Help?

To achieve true, defensible quantum utility in planetary rover navigation:
1. **Gate-Level Hardware Synthesis**: Efficient circuit synthesis for cost Hamiltonians without exponential ancilla or lookup-table overhead.
2. **Error Mitigation and Fault Tolerance**: Physical quantum processors with two-qubit error rates $<10^{-4}$ and coherence times supporting circuit depths $p \gg 10$.
3. **Problem Scales Beyond Classical Exact Solvability**: Problem sizes $N > 40$ where Held-Karp ($O(N^2 2^N)$) is computationally intractable, combined with complex constraint landscapes where local search heuristics (such as 2-opt and Lin-Kernighan) reliably fail.
4. **Co-Design with Physical Sensors**: Incorporating genuine real-time uncertainty distributions directly into the quantum state amplitudes rather than static pre-computed cost matrices.

