# Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation
## Final-Year Capstone Engineering Project Report

---

**Project Title:** Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation  
**Academic Context:** Final-Year Major Capstone Engineering Project (B.E. / B.Tech, October 2026)    
**Target Platform:** ROS 2 Jazzy, Gazebo Harmonic 8.x, Qiskit Aer / NumPy Statevector Simulation  
**Repository & Codebase:** `lunar-rover-quantum`  
**Test Suite Status:** **301 Pure-Python Unit Tests Passing** (5 ROS/Gazebo tests deselected, 19.20s)  

---

## Executive Abstract

This project investigates the application of quantum optimization—specifically the Quantum Approximate Optimization Algorithm (QAOA) and simulated quantum minimum finding (Dürr-Høyer)—to autonomous lunar rover mission planning within a robotic simulation environment (ROS 2 Jazzy and Gazebo Harmonic). The simulation employs a flat box terrain collision (as Gazebo Harmonic with the DART physics engine cannot collide heightmaps), with elevation slope and terrain hazards integrated via heightmap-based cost models. We integrate a four-layer autonomy stack spanning LiDAR/IMU perception, hazard-aware traversability mapping, energy-weighted graph extraction, hybrid quantum/classical path generation, closed-loop trajectory execution, and continuous battery monitoring: Phases 1–8 are verified in simulation; Phase 9 (path executor) was refactored after its initial live run and verified via pure-Python unit tests; Phase 10 ROS nodes (`battery_monitor_node`, `evaluator_node`) were tested against stubbed `rclpy`; and Phase 11 (real Arjuna/Jetson robot deployment) is scoped as future work.

To evaluate whether quantum computing provides tangible utility for surface planetary exploration, we benchmark classical exact solvers (Dijkstra, A*, Held-Karp dynamic programming, orienteering DP) and heuristics (Nearest Neighbour with 2-opt and random restarts) against multiple quantum formulations:
1. Local corridor point-to-point path selection ($8\text{--}12$ macro-edges),
2. Multi-site Traveling Salesperson Problem (TSP) using a full one-hot quadratic unconstrained binary optimization (QUBO) formulation ($N^2$ qubits),
3. Windowed QAOA utilizing sliding 3-site sub-tours ($9$ qubits),
4. Science site orienteering under battery capacity constraints ($n$ qubits), and
5. An unconstrained compact permutation encoding ($\lceil \log_2 N! \rceil$ qubits).

**Core Finding:** The honest and defensible conclusion of this investigation is that **no quantum advantage is achieved**. Classical exact dynamic programming and simple multi-start local search heuristics consistently match or outperform QAOA across all problem formulations in both solution quality and execution speed:
- In point-to-point planning on the single evaluated route, QAOA returns identical energy costs to A* (ratio 1.0000) but is $\approx 22,000\times$ slower ($5.5\text{ s}$ vs. $0.25\text{ ms}$).
- In one-hot multi-site TSP ($N=4$, $16$ qubits), QAOA is $\approx 77,000\times$ slower than classical 2-opt, never beats 2-opt (tied 13/15, worse 2/15), and exhibits a conditional amplification of $\approx 0.54\text{--}1.08$, indicating that QAOA primarily learns to satisfy feasibility constraints rather than optimizing tour costs.
- In orienteering, exact dynamic programming solves every evaluated instance up to $14$ sites in $\le 0.42\text{ s}$, whereas QAOA requires $\approx 400$ classical training iterations to achieve moderate probability amplification.
- In compact permutation encoding, closing $10\%\text{--}27\%$ of the cost gap at $N=8$ relies on classical training, and parameter transfer across problem sizes is no better than random sampling (mean gap closed approx $-0.045$).
- Query complexity analysis of the Dürr-Høyer algorithm confirms the theoretical quadratic speedup ($\approx 2.7\sqrt{N!}$, fitted exponent $0.499$) over *brute force only*, which counts oracle evaluations and is fundamentally distinct from classical dynamic programming operations ($O(N^2 2^N)$).

We conclude that for near-term rover navigation ($N \le 20$), classical algorithms remain overwhelmingly superior. We identify explicit hardware and theoretical thresholds that would be required for quantum algorithms to offer genuine utility.

---

## 1. Introduction and Objectives

Autonomous planetary exploration rovers operating in extreme environments (such as the lunar South Pole) face severe energy constraints. Traversal across steep slopes, unlit shadows, and boulder fields rapidly drains onboard batteries. Path planning algorithms must therefore compute energy-minimal trajectories rather than simply shortest-distance geometric paths.

With the advent of Noisy Intermediate-Scale Quantum (NISQ) hardware, variational quantum algorithms like QAOA have been proposed for combinatorial optimization. However, academic literature often reports isolated proof-of-concept quantum demonstrations on synthetic toy graphs without realistic robotic constraints, fair classical comparisons, or disclosure of classical simulation shortcuts.

### 1.1 Project Objectives & Scope
1. **Robotic Autonomy Integration:** Build and integrate a ROS 2 Jazzy and Gazebo Harmonic simulation architecture for an autonomous lunar rover, including differential-drive kinematics, LiDAR obstacle detection, IMU inclinometry, and closed-loop motor control. Due to Gazebo DART physics limitations with heightmaps, the simulation terrain uses a flat box collision, and elevation slope is modeled through heightmap cost mapping. Phases 1–8 are verified in simulation; Phase 9 is verified in unit tests after executor refactoring; Phase 10 ROS nodes are verified against stubbed `rclpy`; Phase 11 physical deployment on an Arjuna AMR with Jetson Nano is documented as future work.
2. **Terrain-Aware Energy Modeling:** Formulate energy cost models incorporating Euclidean distance, slope incline/decline (gravity gradient), surface roughness, and solar illumination.
3. **Rigorous Quantum vs. Classical Benchmarking:** Formulate, implement, and benchmark QAOA across multiple problem encodings, measuring approximation ratios, sampling probability distributions, classical training overhead, and quantum query scaling against industry-standard classical solvers.
4. **Defensible Deliverable:** Present an uncompromised, honest evaluation of quantum utility for space robotics, establishing exact computational limits without exaggerating quantum capabilities.

---

## 2. System Architecture & Robotics Pipeline

The system is organized into a modular four-layer architecture connected through standardized ROS 2 topics and persisted file artifacts.

```
+-----------------------------------------------------------------------------------+
| LAYER 1: PERCEPTION (sensor_pkg)                                                  |
|   LiDAR (/scan, 10 Hz)         --> lidar_processor    --> /terrain/pointcloud     |
|   IMU (/imu/data, 50 Hz)       --> imu_processor      --> /imu/slope              |
|   Encoders (/joint_states)     --> encoder_processor  --> /odom (Kinematic Diff)  |
+-----------------------------------------------------------------------------------+
                                         |
+-----------------------------------------------------------------------------------+
| LAYER 2: MAPPING & PLANNING (mapping_pkg, planning_pkg)                           |
|   /scan + TF                   --> occupancy_grid_node    --> /map                |
|   /map + /scan                 --> terrain_classifier_node--> /terrain_map (.npz) |
|   latest.npz + heightmap.png   --> graph_model (CLI)      --> latest.graphml      |
|   latest.graphml               --> classical_planner      --> /path/classical     |
|   latest.graphml + /odom       --> quantum_optimizer      --> /path/quantum       |
|   heightmap.png + site coords  --> multi_site_mission     --> /path/mission       |
+-----------------------------------------------------------------------------------+
                                         |
+-----------------------------------------------------------------------------------+
| LAYER 3: EXECUTION (navigation_pkg)                                               |
|   /path/* + /odom              --> path_executor          --> /cmd_vel            |
+-----------------------------------------------------------------------------------+
                                         |
+-----------------------------------------------------------------------------------+
| LAYER 4: EVALUATION (evaluation_pkg)                                              |
|   /cmd_vel + /odom             --> battery_monitor        --> /battery/status     |
|   /odom + /battery/status      --> evaluator_node         --> /metrics            |
|   Offline Path Logs            --> evaluation_pkg.metrics --> comparison.json     |
+-----------------------------------------------------------------------------------+
```

### 2.1 Component Specifications

| Package | Node / Module | Input Topics / Files | Output Topics / Files | Implementation Details |
|---|---|---|---|---|
| `sensor_pkg` | `lidar_processor` | `/scan` | `/terrain/pointcloud`, `/terrain/roughness` | 360-sample ray casting, local beam variance computation |
| `sensor_pkg` | `imu_processor` | `/imu/data` | `/imu/slope`, `/imu/terrain_status` | Pitch/roll decomposition into surface gradient |
| `sensor_pkg` | `encoder_processor` | `/joint_states` | `/odom`, `odom->base_footprint` TF | Dead-reckoned 2D differential-drive odometry |
| `mapping_pkg` | `occupancy_grid_node` | `/scan`, TF | `/map` | 2D occupancy grid ($600 \times 600$ cells @ $0.05\text{ m}$) |
| `mapping_pkg` | `terrain_classifier_node` | `/map`, `/scan` | `/terrain_map`, `latest.npz` | Hough transform crater detection and roughness classification |
| `planning_pkg` | `graph_model` | `latest.npz`, `heightmap.png` | `latest.graphml` | Coarsened 8-connected grid graph with asymmetric energy weights |
| `planning_pkg` | `classical_planner` | `latest.graphml` | `/path/classical`, `/path/classical/dijkstra` | Priority-queue Dijkstra and Euclidean-heuristic A* |
| `planning_pkg` | `quantum_optimizer` | `latest.graphml`, `/odom` | `/path/quantum`, `latest_quantum.json` | Local corridor QUBO formulation solved via Qiskit Aer QAOA |
| `planning_pkg` | `multi_site_mission` | `heightmap.png`, sites | `/path/mission` | Full & windowed multi-site visiting order pipeline |
| `navigation_pkg`| `path_executor` | `/path/*`, `/odom` | `/cmd_vel` | Proportional heading & distance controller (0.5 m arrival tolerance) |
| `evaluation_pkg`| `battery_monitor` | `/cmd_vel`, `/odom` | `/battery/status` | Empirical discharge model: $P = P_{\text{idle}} + k_v\|v\| + k_\omega\|\omega\|$ |
| `evaluation_pkg`| `evaluator_node` | `/odom`, `/battery/status` | `/metrics`, `latest_execution.json` | Real-time distance tracking and energy tallying upon arrival |

---

## 3. Mathematical Formulations

### 3.1 Terrain Energy & Cost Models

The codebase implements two distinct models for path pricing, strictly separated by mission phase:

#### (A) Point-to-Point Graph Energy Model (`planning_pkg.graph_model`, `build_graph`)
Used for Phases 6–10 in coarsened grid path planning across the mapped terrain. For an undirected edge between coarsened nodes $u = (r_u, c_u)$ and $v = (r_v, c_v)$ at spacing $d(u, v) = \sqrt{(x_v - x_u)^2 + (y_v - y_u)^2}$:
$$E(u, v) = d(u, v) \cdot S(\theta) \cdot T_{\text{avg}}$$
where:
- $\theta = \arctan\left(\frac{|z_v - z_u|}{d(u, v)}\right)$ is the inclination angle, with elevation $z$ sampled from `worlds/heightmap.png` via `sample_elevation`,
- $S(\theta) = 1.0 + 2.0 |\sin \theta|$ is the slope factor,
- $T_{\text{avg}} = \frac{T(u) + T(v)}{2}$ is the arithmetic mean of the two endpoint terrain classification factors from `TERRAIN_FACTORS`:
  $$T \in \{\text{flat}: 1.0, \text{rocky}: 1.5, \text{crater}: 3.0, \text{obstacle}: \infty\}$$
*Note:* The point-to-point graph model has **no roughness multiplier** and **no directional uphill penalty**; edge weights are symmetric ($E(u, v) = E(v, u)$).

#### (B) Multi-Site Traversability Cost Map (`planning_pkg.cost_map`, `CostMap.cell_cost` and `CostMap.to_graph`)
Used for multi-site TSP and science orienteering to price directed legs $c_{ij}$ between arbitrary lunar coordinates via directed A* (`cost_map.astar_leg`).
1. **Per-Cell Traversal Cost (`CostMap.cell_cost`):**
   $$\text{Cost}_{\text{cell}} = w_{\text{dist}} + w_{\text{slope}} \frac{\tan(\theta_{\text{slope}})}{\tan(\theta_{\text{max}})} + w_{\text{shadow}} \cdot S_{\text{shadow}}$$
   where $\theta_{\text{slope}}$ is the local gradient in degrees, $S_{\text{shadow}} \in \{0.0, 1.0\}$ is ray-traced cast shadow, and cells with $\theta_{\text{slope}} > \theta_{\text{max}}$ are non-traversable ($\infty$).
2. **Directed Transition Cost between Adjacent Cells (`CostMap.to_graph`):**
   $$\text{Cost}(u \to v) = d(u, v) \cdot \frac{\text{Cost}_{\text{cell}}(u) + \text{Cost}_{\text{cell}}(v)}{2} + w_{\text{uphill}} \max(0, z_v - z_u)$$
   The term $w_{\text{uphill}} \max(0, z_v - z_u)$ introduces directional physical asymmetry ($c_{ij} \ne c_{ji}$): driving uphill gains potential energy against gravity and consumes additional power, whereas driving downhill does not incur the penalty.
3. **Committed Weight Parameters (from `results/multi_site/run_config.json`):**
   - $w_{\text{dist}} = 1.0$, $w_{\text{slope}} = 2.0$, $w_{\text{shadow}} = 1.5$,
   - $w_{\text{uphill}} = 30.0$ for lunar terrain instances ($8.0$ for synthetic clustered instances),
   - $\theta_{\text{max}} = 25.0^\circ$, sun elevation $= 3.0^\circ$, sun azimuth $= 135.0^\circ$.

### 3.2 Point-to-Point Path Selection (Corridor QUBO)
To render QAOA tractable on classical simulators, a sub-graph corridor consisting of $8\text{--}12$ candidate macro-edges is extracted around the initial classical A* backbone. Each binary variable $x_e \in \{0, 1\}$ represents whether macro-edge $e$ is traversed:
$$\min_{\mathbf{x}} \sum_{e \in \mathcal{E}} w_e x_e + \lambda_{\text{start}} \left(1 - \sum_{e \in \text{out}(s)} x_e\right)^2 + \lambda_{\text{goal}} \left(1 - \sum_{e \in \text{in}(g)} x_e\right)^2 + \lambda_{\text{flow}} \sum_{v \notin \{s, g\}} \left(\sum_{e \in \text{in}(v)} x_e - \sum_{e \in \text{out}(v)} x_e\right)^2$$
The objective is converted to an Ising spin Hamiltonian via $x_e = \frac{1 - Z_e}{2}$ and solved using QAOA with depth $p=1$.

### 3.3 Multi-Site TSP Formulations

#### 1. One-Hot Permutation Encoding ($N^2$ Qubits)
For $N$ science target sites, let binary variable $x_{i,t} = 1$ denote visiting site $i$ at time step $t \in \{0, \dots, N-1\}$:
$$H_C = \sum_{t=0}^{N-1} \sum_{i=1}^N \sum_{j \ne i} c_{ij} x_{i,t} x_{j,(t+1)\bmod N} + \lambda \sum_{t=0}^{N-1} \left(1 - \sum_{i=1}^N x_{i,t}\right)^2 + \lambda \sum_{i=1}^N \left(1 - \sum_{t=0}^{N-1} x_{i,t}\right)^2$$
- Qubits: $Q = N^2$. At $N=4 \implies 16$ qubits; at $N=8 \implies 64$ qubits.
- Feasible Subspace: Valid tours constitute only $N! / 2^{N^2}$ of all bitstrings (at $N=4$, $24 / 65,536 \approx 0.0366\%$).

#### 2. Windowed QAOA Decomposition
To scale beyond $N=4$ without requiring $>20$ qubits, Windowed QAOA uses a sliding 3-site window ($9$ qubits) seeded by an initial Nearest Neighbour tour. The sub-tour within the window is optimized via QAOA, the best order is committed, and the window slides forward across the mission tour.

#### 3. Compact Permutation Encoding ($\lceil \log_2 N! \rceil$ Qubits)
To eliminate penalty constraints entirely, the compact encoding indexes all $N!$ permutations lexicographically:
$$k \in \{0, 1, \dots, N!-1\} \implies \mathbf{b} \in \{0, 1\}^Q, \quad Q = \left\lceil \log_2(N!) \right\rceil$$
Bitstrings $k \ge N!$ wrap cyclically via $k \bmod N!$.
- Qubits: $N=5 \implies 7$ qubits; $N=8 \implies 16$ qubits (vs. 64 in one-hot).
- Every bitstring is guaranteed to represent a valid tour.
- Standard transverse-field $X$ mixer: $H_M = \sum_i X_i$.

### 3.4 Science Site Orienteering Under Battery Budget
Given $n$ candidate sites, each with scientific reward $R_i \ge 0$, select a subset $\mathbf{x} \in \{0, 1\}^n$ ($1$ qubit per site) to maximize total reward subject to transit energy not exceeding battery capacity $B$:
$$\text{Cost}(\mathbf{x}) = -\sum_{i=1}^n R_i x_i + \lambda \max\left(0, \text{TourEnergy}(\mathbf{x}) - B\right)^2$$
The transit energy $\text{TourEnergy}(\mathbf{x})$ is computed via exact dynamic programming over the selected subset.

### 3.5 Simulated Dürr-Høyer Minimum Finding
To evaluate quantum query complexity without heuristic parameter landscapes, we simulate the Dürr-Høyer quantum minimum-finding algorithm over $M = N!$ permutations using the exact analytical Grover rotation probability:
$$P(\text{improvement} \mid k \text{ better items}) = \sin^2\left((2r + 1) \arcsin \sqrt{k/M}\right), \quad r = \left\lfloor \frac{\pi}{4} \sqrt{M/k} \right\rfloor$$

---

## 4. Comprehensive Experimental Results

All experiments were executed on standardized terrain and synthetic problem instances with exact Held-Karp dynamic programming and exhaustive search providing absolute ground-truth optima.

### 4.1 Point-to-Point Navigation Benchmark

Point-to-point planning was evaluated on the reference terrain graph (`results/graphs/latest.graphml`). Note that individual execution artifacts (`results/paths/latest_classical.json`, `latest_quantum.json`, `results/energy_comparison/latest_comparison.json`) are runtime generated and gitignored in the repository. The performance numbers below document the validated test runs recorded in project documentation:

#### (i) Single-Route Evaluation (Spawn $(0.0, 0.0)$ to Goal $(1.5, 1.5)$)

| Planner | Energy Cost | Distance (m) | Runtime | Energy Ratio to A* |
|---|---|---|---|---|
| **Classical Dijkstra** | 2.828 | 2.83 | 1.0 ms | 1.0000 |
| **Classical A*** | 2.828 | 2.83 | 0.25 ms | 1.0000 |
| **QAOA (p=1, corridor)** | 2.828 | 2.83 | 5,554 ms | 1.0000 |
| **Distance-Only (Shortest)** | 2.828 | 2.83 | 1.1 ms | 1.0000 |

- **Quality:** QAOA matches the classical A* path energy on the single evaluated route (ratio 1.0000, zero fallback required).
- **Runtime:** QAOA on the CPU simulator is $\approx 22,000\times$ slower than classical A* ($5.55\text{ s}$ vs. $0.25\text{ ms}$).
- **Energy Savings on Single Route:** 0.0% saving against the Euclidean shortest-distance path on this route.

#### (ii) 200-Route Monte Carlo Sweep Summary
Over a 200-route sweep of random start-goal pairs on the reference terrain graph:
- **Mean energy saving vs. distance-only:** **0.24%** (maximum observed saving: **8.5%**).
- **Route divergence:** **109 of the 200 routes** selected paths differing from the Euclidean shortest path.
- **Physical Reason for Low Savings:** Because the Gazebo DART physics engine cannot collide heightmap meshes, the simulation terrain uses a flat collision plane; slope derived from the heightmap gradient is the only source of cost variation, keeping energy savings over distance-only paths modest.

### 4.2 Multi-Site TSP: Full QAOA ($N=4$, 16 Qubits)

Results across 15 benchmark runs on asymmetric lunar heightmap cost maps ($p=1$, 4096 shots; data from `results/multi_site/FINDINGS.md` and `benchmark_summary.csv`):

| Method / Configuration | Optimal Runs | Mean Tour Ratio | $P(\text{feasible})$ | $P(\text{opt})$ | Feasible Enrichment | Conditional Amplification | Mean Runtime |
|---|---|---|---|---|---|---|---|
| **Classical NN + 2-opt** | **5 / 5** | **1.0000** | Always valid (1.0) | 1.0000 | — | — | **0.23 ms** |
| **Exact Held-Karp** | **5 / 5** | **1.0000** | Always valid (1.0) | 1.0000 | — | — | **0.50 ms** |
| **Uniform Random Bitstring** | — | 1.142 | 0.0366% | 0.0031% | $1.0\times$ | $1.0\times$ | — |
| **QAOA ($p=1, \alpha=0.2$)** | 5 / 5 | 1.0000 | 2.05% | 0.190% | $56\times$ | **1.08×** | 15.1 s |
| **QAOA ($p=1, \alpha=0.5$)** | 3 / 5 | 1.0338 | 1.09% | 0.093% | $30\times$ | **0.54×** | 15.6 s |
| **QAOA ($p=1, \alpha=1.0$)** | 5 / 5 | 1.0000 | 1.88% | 0.146% | $51\times$ | **0.90×** | 18.8 s |

> [!CRITICAL]
> **Defensible Insight: Conditional Amplification $\approx 1.0$**  
> While QAOA enriches feasible tours by $30\times\text{--}56\times$ over uniform random bitstrings, the **conditional amplification** $P(\text{optimal} \mid \text{feasible}) / P_{\text{uniform}}(\text{optimal} \mid \text{feasible})$ is only **$0.54\text{--}1.08$**. This proves empirically that QAOA at $p=1$ learns only the row/column constraint penalties; once a sample is valid, it chooses the optimal tour no better than random permutation sampling. QAOA never beat NN+2-opt on any run (tied 13/15, worse 2/15) and was $\approx 77,000\times$ slower.

### 4.3 Multi-Site TSP: Windowed QAOA ($N=5 \dots 8$, 9 Qubits)

| Problem Size ($N$) | QAOA Window Ratio | Exact Classical Window Ratio | NN + 2-opt Ratio | QAOA = Exact Window | QAOA Worse than NN+2-opt |
|---|---|---|---|---|---|
| **$N=4$** | 1.0000 | 1.0000 | 1.0000 | 5 / 5 | 0 / 5 |
| **$N=5$** | 1.0129 | 1.0129 | 1.0000 | 5 / 5 | 2 / 5 |
| **$N=6$** | 1.0241 | 1.0241 | 1.0000 | 5 / 5 | 1 / 5 |
| **$N=7$** | 1.0195 | 1.0195 | 1.0000 | 5 / 5 | 1 / 5 |
| **$N=8$** | 1.0293 | 1.0293 | 1.0110 | 5 / 5 | 1 / 5 |

Across all 75 windowed runs, QAOA achieved the exact same sub-tour as classical enumeration on **74 out of 75 runs**. However, because the sliding window decomposition is myopic, windowed optimization was **worse than classical NN + 2-opt on 16 out of 75 runs**.

### 4.4 Compact Permutation Encoding QAOA

Every bitstring is a valid tour; standard $X$ mixer. Costs are evaluated over all $N!$ permutations:

| $N$ | Qubits Compact | Qubits One-Hot | Best Config | $P(\text{opt})$ QAOA | $P(\text{opt})$ Uniform | Amplification | Gap Closed | QAOA Shots ($99\%$) | Brute Force ($N!$) |
|---|---|---|---|---|---|---|---|---|---|
| **5** | 7 | 25 | $p=4, \alpha=0.2$ | 0.1003 | 0.0156 | $6.4\times$ | 0.50 | 44 | 120 |
| **6** | 10 | 36 | $p=4, \alpha=0.2$ | 0.0413 | 0.0024 | $17.0\times$ | 0.58 | 109 | 720 |
| **7** | 13 | 49 | $p=4, \alpha=1.0$ | 0.0088 | 0.00046 | $19.2\times$ | 0.69 | 520 | 5,040 |
| **8** | 16 | 64 | $p=3, \alpha=0.2$ | 0.00025 | 0.00005 | $5.3\times$ | 0.22 | 18,592 | 40,320 |

> [!NOTE]
> The table above reflects optimistic best configurations selected *post hoc*. When averaged across all tested hyperparameter combinations ($p \in \{1, 2, 3, 4\}$, $\alpha \in \{0.2, 1.0\}$), the **median gap closed is 0.25**.
> 
> **Classical Training Cost Disclosure:** The QAOA shot counts in the table exclude classical parameter training. Each run required a few hundred circuit evaluations (`n_evals` column in `results/ordering_hard/compact_*.csv`; for trained runs at $N=8$, evaluations ranged from min: 169, median: 260, max: 320). With a conventional assumption of ~1,000 measurement shots per training evaluation on physical hardware, the total physical execution count ($\approx 260,000$ shots) significantly exceeds the brute-force search space $N! = 40,320$.

#### Depth-Averaged Gap Closed Table
Mean fraction of cost gap closed across all synthetic and terrain families:

| $N$ | $p=1, \alpha=0.2$ | $p=1, \alpha=1.0$ | $p=2, \alpha=0.2$ | $p=2, \alpha=1.0$ | $p=3, \alpha=0.2$ | $p=3, \alpha=1.0$ | $p=4, \alpha=0.2$ | $p=4, \alpha=1.0$ |
|---|---|---|---|---|---|---|---|---|
| **5** | 0.20 | 0.24 | 0.38 | 0.46 | 0.37 | 0.49 | 0.41 | 0.51 |
| **6** | 0.14 | 0.17 | 0.32 | 0.26 | 0.44 | 0.36 | 0.42 | 0.40 |
| **7** | 0.15 | 0.13 | 0.20 | 0.23 | 0.35 | 0.49 | 0.38 | 0.55 |
| **8** | 0.06 | 0.12 | 0.10 | 0.16 | 0.15 | 0.21 | — | — |

#### Parameter Transfer Evaluation
To test whether classical training could be eliminated, optimal angles $(\boldsymbol{\gamma}^*, \boldsymbol{\beta}^*)$ trained on $N=6$ were transferred directly to $N=7$ and $N=8$.
Evaluating across all 228 transferred runs in `results/ordering_hard/compact_*.csv`, parameter transfer is **no better than random sampling**:
- **Transferred Gap Closed Mean:** **$-0.045$** (median: $-0.003$, min: $-0.331$, max: $+0.247$).
- **At $N=7$:** Mean transferred gap closed is **$-0.049$** (compared to $+0.31$ mean when trained).
- **At $N=8$:** Mean transferred gap closed is **$-0.040$** (compared to $+0.13$ mean when trained).
- **Median Degradation:** $\Delta(\text{gap closed}) = -0.23$ relative to trained parameters.
- **Conclusion:** The cost phase landscape does not generalize across problem dimensions; classical training cannot be bypassed.

### 4.5 Science Site Orienteering Under Battery Budget

Results across 80 benchmark instances on synthetic and terrain instances (data from `results/orienteering/FINDINGS.md`, `benchmark_raw.csv`, and `benchmark_summary.csv`):

| Sites ($n$) | Best Config | $P(\text{opt})$ | Uniform $P(\text{opt})$ | Amplification | Conditional Amp. | QAOA Shots ($99\%$) | Training Evals | Exact DP Runtime (Mean / Max) |
|---|---|---|---|---|---|---|---|---|
| **6** | $p=3, \alpha=0.5$ | 0.0912 | 0.0156 | $5.8\times$ | 1.38 | 48 | 378 | 0.001 s / 0.002 s |
| **8** | $p=2, \alpha=1.0$ | 0.0243 | 0.0039 | $6.2\times$ | 1.15 | 187 | 306 | 0.005 s / 0.007 s |
| **10** | $p=3, \alpha=0.5$ | 0.0317 | 0.0010 | $32.5\times$ | 2.64 | 143 | 399 | 0.022 s / 0.052 s |
| **12** | $p=3, \alpha=1.0$ | 0.0075 | 0.00024 | $30.7\times$ | 3.17 | 612 | 398 | 0.086 s / 0.123 s |
| **14** | $p=3, \alpha=1.0$ | 0.0123 | 0.00006 | $201.9\times$ | 14.52 | 371 | 400 | 0.311 s / 0.422 s |

- **Exact Classical Dominance:** Classical exact dynamic programming solves every evaluated instance up to $n=14$ in **$\le 0.42\text{ s}$**.
- **Training Burden:** At $n=14$, QAOA requires $371$ evaluation shots, but training the variational parameters required $\approx 400$ statevector evaluations. Under the assumption of 1,000 measurement shots per training evaluation on physical hardware (an assumption, not a measurement), this equates to $\approx 400,000$ physical circuit executions, exceeding the 16,384 total states of brute-force search and the single pass of exact DP.

### 4.6 Quantum Query Complexity (Dürr-Høyer Algorithm)

| Problem Size ($N$) | State Space ($M = N!$) | Brute Force Queries | Durr-Hoyer Mean Queries | Ratio: Queries / $\sqrt{M}$ | Held-Karp Operations ($N^2 2^N$) |
|---|---|---|---|---|---|
| **4** | 24 | 24 | 11.4 | 2.32 | 256 |
| **6** | 720 | 720 | 79.5 | 2.96 | 2,304 |
| **8** | 40,320 | 40,320 | 579 | 2.88 | 16,384 |
| **10** | 3,628,800 | 3,628,800 | 5,364 | 2.82 | 102,400 |
| **12** | $4.79 \times 10^8$ | $4.79 \times 10^8$ | 56,818 | 2.60 | 589,824 |
| **14** | $8.72 \times 10^{10}$ | $8.72 \times 10^{10}$ | 790,363 | 2.68 | 3,211,264 |
| **16** | $2.09 \times 10^{13}$ | $2.09 \times 10^{13}$ | $1.24 \times 10^7$ | 2.70 | $1.68 \times 10^7$ |
| **17** | $3.56 \times 10^{14}$ | $3.56 \times 10^{14}$ | $4.93 \times 10^7$ | 2.61 | $3.79 \times 10^7$ |
| **20** | $2.43 \times 10^{18}$ | $2.43 \times 10^{18}$ | $4.17 \times 10^9$ | 2.67 | $4.19 \times 10^8$ |

- **Empirical Fit:** The log-log regression of Dürr-Høyer queries against $M$ yields a slope of **0.499**, confirming an exact $\mathcal{O}(\sqrt{N!})$ query scaling.
- **Comparison to Classical Dynamic Programming:**
  The comparison between Dürr-Høyer oracle queries and Held-Karp basic steps counts fundamentally different quantities (quantum oracle evaluations vs. classical arithmetic steps) and is not a statement about wall-clock execution speed.
  Dürr-Høyer provides a quadratic query speedup over *brute force search only*. As recorded in `results/ordering_hard/queries.csv`, classical Held-Karp basic step counts ($N^2 2^N$) remain higher than Dürr-Høyer query counts for $N \le 16$, but fall *below* Dürr-Høyer queries from $N = 17$ onward (at $N=17$: Held-Karp $3.79 \times 10^7$ steps vs. Dürr-Høyer $4.93 \times 10^7$ queries; at $N=20$: Held-Karp $4.19 \times 10^8$ steps vs. Dürr-Høyer $4.17 \times 10^9$ queries).
  No wall-clock claim is made, as physical quantum oracle queries require executing deep, coherent quantum circuits on hardware that does not yet exist.

### 4.7 Heuristic Hardness and Stress Sets
To examine whether classical heuristics fail on deceptive instances, we evaluated 660 instances across clustered and uniform geometries:
- Classical Nearest Neighbour with a single 2-opt pass was suboptimal on **84 out of 660 instances** (gaps up to $10.8\%$).
- Running **20 random restarts** of 2-opt eliminated the gap on **all 84 of the 84 non-optimal instances** (0 non-optimal remaining).
- Classical multi-start heuristics easily escape local minima without quantum intervention.

---

## 5. Quantum Resource Scaling & Simulation Limits

To understand why simulation is capped at small $N$, resource scaling was modeled across all formulations (generated by `scripts/plot_scaling_analysis.py`):

```
+-----------------------------------------------------------------------------------------+
|                                QUANTUM RESOURCE SCALING                                 |
+-----------------------------------------------------------------------------------------+
|  Problem Size (N) | One-Hot Qubits (N^2) | Compact Qubits (ceil log2 N!) | Orienteering |
|-------------------|----------------------|-------------------------------|--------------|
|        N = 4      |          16          |               5               |       4      |
|        N = 5      |          25          |               7               |       5      |
|        N = 6      |          36          |              10               |       6      |
|        N = 8      |          64          |              16               |       8      |
|        N = 10     |         100          |              22               |      10      |
|        N = 14     |         196          |              37               |      14      |
+-----------------------------------------------------------------------------------------+
```

### 5.1 The Statevector Memory Wall
Simulating $Q$ qubits classically requires storing $2^Q$ complex numbers (`np.complex128`, 16 bytes per amplitude):
$$\text{Memory} = 16 \times 2^Q \text{ bytes}$$
- **$Q = 16$ ($N=4$ one-hot, $N=8$ compact):** $1.05\text{ MB}$ (instantaneous on CPU).
- **$Q = 20$:** $16.0\text{ MB}$ (standard interactive simulation threshold).
- **$Q = 25$ ($N=5$ one-hot):** $536.8\text{ MB}$.
- **$Q = 30$:** $17.18\text{ GB}$ (standard laptop RAM exhaustion limit).
- **$Q = 36$ ($N=6$ one-hot):** $1.10\text{ TB}$.
- **$Q = 64$ ($N=8$ one-hot):** $2.95 \times 10^{20}\text{ bytes} \approx 295,000\text{ Petabytes}$ (physically impossible to store or compute).

### 5.2 Disclosure of the Simulation Shortcut
In both `tsp_compact_qaoa.py` and `orienteering_qaoa.py`, the cost operator is applied as a **classical diagonal lookup table** in `qaoa_sim.py`:
$$|\psi_{k}\rangle \leftarrow |\psi_{k}\rangle \cdot e^{-i \gamma C(k)}$$
This is a **simulation shortcut**, not a gate-level quantum circuit. On physical hardware:
1. Lookup tables do not exist in Hilbert space.
2. The cost function $C(k)$ must be synthesized using quantum arithmetic circuits (adders, multipliers, permutation decoders).
3. Compiling compact permutation decoders into physical gates requires thousands of CNOT and Toffoli gates, demanding fault-tolerant error correction. Thus, the compact encoding achieves space compression in classical simulation, but does not provide near-term gate-level execution speedup.

---

## 6. Honest Engineering Conclusions

1. **Absence of Quantum Advantage:** Across all evaluated robotics domains (point-to-point path selection, multi-site TSP, and orienteering), QAOA and simulated Dürr-Høyer methods do not outperform classical algorithms. Classical A*, Held-Karp, orienteering DP, and 2-opt with restarts consistently achieve equal or superior solution quality without classical parameter training overhead.
2. **The Constraint Penalty Trap:** One-hot encodings allocate almost all optimization capacity to constraint satisfaction ($P(\text{feasible}) \approx 1\%$), leaving negligible discrimination for cost minimization (conditional amplification $\approx 1.0$).
3. **Training Cost Exceeds Sampling Gains:** When classical training iterations are accounted for, QAOA requires hundreds of circuit evaluations (each requiring thousands of physical shots on hardware), far exceeding the cost evaluations of classical brute force or exact dynamic programming.
4. **Oracle Queries vs. Classical Operations:** Dürr-Høyer minimum finding offers a quadratic query speedup over *brute force search only* ($\approx 2.7\sqrt{N!}$). It counts oracle evaluations, not clock cycles, and does not beat classical dynamic programming ($O(N^2 2^N)$), which requires fewer operations than Dürr-Høyer queries for $N \ge 17$. No wall-clock speedup is claimed.
5. **What Would Have to Change for Quantum to Help?**
   - Physical fault-tolerant processors with $>1,000$ logical qubits, two-qubit error rates $<10^{-4}$, and circuit depths $p \gg 10$.
   - Native hardware Hamiltonians that encode permutation constraints without penalty terms.
   - Mission scales ($N > 40$) where classical dynamic programming ($O(N^2 2^N)$) becomes strictly intractable, and where classical multi-start heuristics demonstrably fail.

---

## 7. Verification, Reproducibility & Future Work

### 7.1 Verification Status
- **Pure-Python Test Suite:** Verified **301 tests passing**, 5 deselected (ROS/Gazebo markers), 7 warnings in **19.20 s** (`python -m pytest -q`).
- **Package Layout:** Verified across all six packages (`planning_pkg`, `navigation_pkg`, `evaluation_pkg`, `sensor_pkg`, `mapping_pkg`, `rover_simulation`).
- **Reproducibility:** All benchmark scripts (`benchmark_multi_site.py`, `benchmark_orienteering.py`, `benchmark_hard_and_compact.py`, `plot_scaling_analysis.py`) execute independently from the CLI and regenerate exact CSVs and figures.
- **Support Documents Verified:** [docs/VIVA_QA.md](file:///e:/Downloads/lunar-rover-quantum%20(2)%20(1)/lunar-rover-quantum/docs/VIVA_QA.md), [scripts/plot_scaling_analysis.py](file:///e:/Downloads/lunar-rover-quantum%20(2)%20(1)/lunar-rover-quantum/scripts/plot_scaling_analysis.py), and [results/scaling_analysis.png](file:///e:/Downloads/lunar-rover-quantum%20(2)%20(1)/lunar-rover-quantum/results/scaling_analysis.png) exist and have been tested.

### 7.2 Phase 11 & Phase 12 Roadmap
- **Phase 11 (Future Work):** Physical deployment on the **Arjuna AMR robotics kit** with an onboard **NVIDIA Jetson Nano**. The software interfaces developed in Layer 3 (`path_executor`) and Layer 4 (`battery_monitor`) are designed as direct hardware abstractions; Gazebo topic bridges will be replaced by physical micro-ROS serial nodes and hardware current/voltage shunt sensors (INA219/INA226).
- **Phase 12 (Project Documentation & Viva Preparation):** Technical report, defensible results analysis, and viva defense preparation guide ([docs/VIVA_QA.md](file:///e:/Downloads/lunar-rover-quantum%20(2)%20(1)/lunar-rover-quantum/docs/VIVA_QA.md)).

### 7.3 Traceability & Data Verification Register
- **Single-Route Runtime Files:** The individual runtime JSON files (`latest_classical.json`, `latest_quantum.json`, `latest_comparison.json` in `results/paths/` and `results/energy_comparison/`) are runtime generated by live ROS/CLI pipelines and are gitignored in the repository. The single-route numbers (energy 2.828, distance 2.83 m, runtime 5,554 ms vs 0.25 ms) and 200-route sweep metrics (0.24% mean, 8.5% max, 109/200 diverging routes) are documented directly from earlier committed logs and project notes (`docs/PROGRESS.md`).
- **Removed Unverified Numbers:** The previously reported row "Distance-Only 2.835, ratio 1.0024" could not be traced to any committed data file or earlier note (which consistently reported 2.828 / 0% saving on the single route) and has been removed.
- **Physical Shot Assumption:** Total training shot counts ($\approx 400,000$ for orienteering at $n=14$, $\approx 260,000$ for compact TSP at $N=8$) are explicitly designated as calculations based on an **assumption of 1,000 measurement shots per evaluation on physical quantum hardware**, not empirical hardware measurements.

