# Viva Voce Examination & Defense Q&A

This guide prepares the project team for academic defense, external examinations, and technical vivas. It provides honest, scientifically rigorous, and mathematically grounded answers to the most challenging questions.

---

### Q1: Why does QAOA fail to outperform classical algorithms in your project?

**Defensible Answer:**
"QAOA does not beat classical methods in our experiments for three fundamental structural reasons:
1. **The Feasibility Penalty Burden in Constrained Spaces:** In the one-hot formulation ($N^2$ qubits), valid tours occupy an exponentially vanishing fraction of the Hilbert space ($N! / 2^{N^2}$). To penalize illegal tours, large penalty multipliers ($\lambda$) must be introduced. At shallow circuit depths ($p=1$), the optimizer spends almost all its parameter expressive power satisfying the feasibility constraints rather than minimizing tour cost. Our conditional amplification data proves that once a bitstring is valid, QAOA samples the optimal tour no more frequently than uniform random guessing (ratio ~1.0).
2. **Classical Heuristic Efficacy:** For traveling salesperson problems at $N \le 10$, simple classical heuristics like Nearest Neighbour followed by 2-opt local search find the optimal tour on over 87% of instances in less than a millisecond. On our hard stress instances where 2-opt gets stuck, just 20 random restarts resolve all 84 of the 84 suboptimal cases.
3. **Training Overhead:** QAOA requires hundreds of classical optimizer evaluations (e.g., COBYLA) in a non-convex parameter landscape. For orienteering with $n=14$, QAOA requires ~400 circuit evaluations during training, whereas classical dynamic programming finds the exact global optimum in a single pass taking less than 0.5 seconds."

---

### Q2: In Phase 8 (point-to-point planning), why did you build the quantum graph corridor around the A* path?

**Defensible Answer:**
"In Phase 8, running QAOA on the raw terrain grid (which contains hundreds or thousands of nodes) is impossible because simulating a full grid graph would require hundreds of qubits. To make the circuit tractable on a classical CPU simulator (`AerSimulator`), we formulated a corridor sub-graph of 8 to 12 macro-edges surrounding the classical A* path.
The purpose was **not to claim that QAOA found the end-to-end path independently**, but rather:
1. To implement and verify the complete end-to-end software integration: bridging ROS 2 sensor odometry (`/odom`), formulating the QUBO cost matrix, submitting to Qiskit Aer, decoding the sampled bitstring into a `nav_msgs/msg/Path`, and having the physical path executor drive the rover.
2. To test whether QAOA could re-rank alternative macro-edges within a local corridor.
Because the corridor was centered on the A* path, QAOA returned the exact same energy cost as A* (ratio 1.0000), but took ~5.5 seconds compared to 0.25 milliseconds for A*."

---

### Q3: Why did you limit empirical benchmarks to $N \le 10$ (and $N \le 14$ for orienteering)?

**Defensible Answer:**
"There are two reasons: simulation physics and validation rigor.
1. **Classical Statevector Memory Wall:** Simulating a quantum circuit with statevectors requires storing $2^Q$ complex amplitudes in memory ($16 \times 2^Q$ bytes). 
   - For one-hot TSP ($Q = N^2$), $N=4$ requires 16 qubits (1 MB RAM), but $N=5$ requires 25 qubits (536 MB RAM), and $N=6$ requires 36 qubits (1.1 TB RAM). Simulating $N > 4$ with full one-hot QAOA on our development hardware is memory-prohibitive.
   - For compact encoding ($Q = \lceil \log_2 N! \rceil$), $N=8$ requires 16 qubits (1 MB RAM), but $N=9$ jumps to 19 qubits (8.4 MB) and $N=10$ to 22 qubits (67 MB).
2. **Ground Truth Verification:** To rigorously evaluate approximation ratios, gap-closed percentages, and probability amplification, we needed exact global optima as ground truth. We computed exact optima using Held-Karp dynamic programming and exhaustive brute-force search. At $N \le 10$, we can cross-check every single instance against exact classical baselines."

---

### Q4: Is your battery energy model a measured physical model or an assumption?

**Defensible Answer:**
"Our battery model is an **assumed empirical model**, not a physical hardware measurement. We modeled power consumption as:
$$P = P_{\text{idle}} + k_{\text{lin}} |v| + k_{\text{ang}} |\omega|$$
with a nominal 40 Wh, 24 V battery bank, where $P_{\text{idle}} = 5\text{ W}$, $k_{\text{lin}} = 25\text{ W}/(\text{m/s})$, and $k_{\text{ang}} = 10\text{ W}/(\text{rad/s})$.
This model serves as a standardized software interface (`/battery/status` and `/metrics`) in Layer 4 (`evaluation_pkg`). In Phase 11 (the future work deployment on the real Arjuna rover with a Jetson Nano), this software module will be connected directly to hardware current/voltage shunt sensors (e.g., INA219/INA226) to log true empirical power consumption across real terrain."

---

### Q5: What is the 'simulation shortcut' in your compact QAOA and orienteering code, and why does it matter?

**Defensible Answer:**
"In our compact-encoding QAOA (`tsp_compact_qaoa.py`) and orienteering QAOA (`orienteering_qaoa.py`), we use a custom statevector simulator (`qaoa_sim.py`). In this simulator, the cost Hamiltonian phase-shift operator $U(C, \gamma) = e^{-i \gamma C}$ is implemented by multiplying the amplitude vector element-wise by a precomputed classical cost array:
$$\psi[z] \leftarrow \psi[z] \cdot e^{-i \gamma C(z)}$$
This is a **classical simulation shortcut**, not a gate-level circuit.
On real quantum hardware, you cannot look up costs in an external NumPy array. The cost function $C(z)$ must be compiled into physical unitary gates (CNOTs, phase gates, and Toffoli ancilla gates) that compute the cost in a quantum register and apply the phase. 
For the compact encoding, compiling $N!$ arbitrary permutations into physical quantum logic gates would require thousands of gates and substantial ancilla qubit overhead. Therefore, our compact QAOA demonstrates a **state-space representation efficiency**, but it does not demonstrate gate-level quantum speedup."

---

### Q6: Why did parameter transfer fail when reusing angles trained at $N=6$ on $N=7$ and $N=8$?

**Defensible Answer:**
"Parameter transfer—where optimal angles $(\boldsymbol{\gamma}, \boldsymbol{\beta})$ from small instances are transferred to larger instances to bypass classical training—failed because:
1. **Topological Disruption of the Index Mapping:** In our compact encoding, bitstrings represent lexicographical permutation indices. When $N$ increases from 6 to 7, the number of permutations expands by a factor of 7 (from 720 to 5,040), completely rearranging the permutation neighborhood mapped by bitwise Hamming distances.
2. **Phase Landscape Deformation:** The optimal QAOA parameters depend intimately on the eigenspectrum density and energy landscape of the cost Hamiltonian. Because the cost function wraps around $N!$ and the energy scales with $N$, the parameter landscape shifts dramatically.
When we transferred angles from $N=6$ to $N=7$ and $N=8$, the mean gap closed was **$-0.045$** (no better than random sampling; median $-0.003$). This proves that training cannot be avoided in this encoding, meaning training cost must always be accounted for when assessing practical viability."

---

### Q7: Why do you report Dürr-Høyer results in query counts rather than wall-clock time?

**Defensible Answer:**
"We evaluate Dürr-Høyer minimum finding using **oracle query complexity** because quantum algorithms are fundamentally governed by query complexity bounds ($\mathcal{O}(\sqrt{M})$ queries for unstructured search).
Our simulation confirmed the exact theoretical exponent: fitting queries against search space size $M = N!$ yielded a log-log slope of **0.499** (theory: 0.500), achieving an average query count of $\approx 2.7 \sqrt{N!}$.
However, making a wall-clock runtime claim would be academically dishonest:
1. Each oracle query on a quantum computer involves executing an entire coherent circuit of depth proportional to the cost calculation.
2. Classical clock speeds are in gigahertz ($10^9$ cycles/sec), whereas physical qubit gate times are on the order of tens to hundreds of nanoseconds.
3. Dürr-Høyer offers a quadratic query speedup over *brute-force search only*, not over classical dynamic programming. Oracle queries and classical arithmetic steps are fundamentally different units and not directly comparable; Held-Karp basic step counts ($N^2 2^N$) fall below Dürr-Høyer queries from $N = 17$ on. Crucially, each quantum oracle call requires coherent circuit execution on real hardware, and no wall-clock speedup is claimed."

---

### Q8: What is your primary contribution if quantum did not beat classical?

**Defensible Answer:**
"Our project makes four concrete contributions to autonomous systems and applied quantum computing:
1. **A Fully Integrated Hybrid Robotics Pipeline:** We designed and verified an end-to-end ROS 2 Jazzy + Gazebo Harmonic architecture spanning perception, terrain hazard mapping, energy modeling, quantum path selection, motor execution, and battery monitoring.
2. **Rigorous and Defensible Benchmarking:** Rather than presenting cherry-picked quantum toy problems, we built reproducible benchmarks comparing QAOA against exact solvers and heuristics on realistic lunar terrain cost maps.
3. **Encoding Insights:** We demonstrated why one-hot QUBOs fail at shallow depths due to constraint dilution ($P(\text{feasible}) \approx 1\%$), formulated the unconstrained compact permutation encoding, and empirically quantified the limitations of parameter transfer.
4. **Honest Engineering Deliverable:** We established clear boundary conditions on when quantum methods could become viable, warning against false claims of NISQ quantum advantage in autonomous robotics."

