# 🚀 Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation
## PHASE 1 — STEP 1: Project Planning, Architecture & Workflow

---

## 📋 STEP OVERVIEW

| Field | Details |
|---|---|
| **Phase** | 1 — Project Planning |
| **Step** | 1 of 1 in this phase |
| **Estimated Time** | 2–3 days |
| **Difficulty** | ⭐ Beginner |
| **Prerequisites** | None |

---

## 🎯 Objective

Before writing a single line of code, we need a **crystal-clear blueprint** of the entire project. This step defines:
- What we are building (system architecture)
- Why each part exists (purpose)
- How everything connects (data flow)
- What software and hardware we need (stack)
- Where every file lives (folder structure)
- When we do what (timeline/milestones)

Think of this step as drawing the **architect's blueprint** before construction begins. Skipping this is like building a house without a plan — you'll waste weeks tearing down walls you built wrong.

---

## 🧠 Concepts You Must Understand First

### 1. What is ROS2?
ROS2 (Robot Operating System 2) is **not a traditional OS**. It is a **middleware framework** — a communication layer that lets different software components (called **nodes**) talk to each other via **topics**, **services**, and **actions**.

Think of it like WhatsApp groups for your robot:
- Each node is a person in the group
- Topics are group chats where nodes broadcast messages
- Services are private messages with a reply required
- Actions are long tasks with progress updates

### 2. What is Gazebo?
Gazebo is a **physics simulation environment**. It simulates:
- Gravity, friction, inertia
- Sensor data (LiDAR, cameras, IMU)
- Terrain and obstacles
- Motor responses

We use it so we can **test our rover without breaking real hardware**.

### 3. What is QAOA / Quantum Optimization?
QAOA (Quantum Approximate Optimization Algorithm) is a quantum algorithm that finds **near-optimal solutions** to combinatorial problems (problems with many possible choices, like path planning).

For us:
- Classical computers check paths one by one
- Quantum simulators check many paths simultaneously (in superposition)
- Result: a near-optimal energy-efficient path

We use **Qiskit AerSimulator** — this runs quantum circuits on your laptop CPU. No real quantum hardware needed.

### 4. What is QUBO?
QUBO = Quadratic Unconstrained Binary Optimization. It's the **language** quantum optimizers understand. We convert our terrain graph into a QUBO matrix, then let QAOA solve it.

### 5. What is a Terrain Graph?
We represent the lunar surface as a **graph**:
- **Nodes** = positions on the terrain
- **Edges** = paths between positions
- **Weights** = energy cost to traverse that path (based on slope, roughness, distance)

---

## 🏗️ System Architecture

The system has **4 major layers**:

```
┌─────────────────────────────────────────────────────────────┐
│                     PERCEPTION LAYER                        │
│   LiDAR → Point Cloud → Terrain Map → Occupancy Grid       │
│   IMU → Orientation → Slope Estimation                      │
│   Encoders → Odometry → Position Estimation                  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                   PLANNING LAYER                            │
│   Graph Builder (NetworkX)                                  │
│   Classical Planner (A* / Dijkstra)                         │
│   Quantum Optimizer (QAOA via Qiskit)                       │
│   Energy Model (terrain cost weights)                        │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                  EXECUTION LAYER                            │
│   Waypoint Publisher (ROS2 Nav2)                            │
│   Motor Controller (Arduino / ROS2)                         │
│   Battery Monitor                                           │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│               EVALUATION / VISUALIZATION LAYER              │
│   RViz2 (visualize paths, maps, sensors)                    │
│   Matplotlib (energy comparison graphs)                     │
│   ROS2 Bag Files (record and replay sessions)               │
│   Performance Metrics Logger                                │
└─────────────────────────────────────────────────────────────┘
```

---

## 📡 ROS2 Node Architecture

Here is a complete list of **every ROS2 node** we will build across all phases:

| Node Name | Package | Purpose | Topics Published | Topics Subscribed |
|---|---|---|---|---|
| `lidar_processor` | `sensor_pkg` | Processes raw LiDAR data | `/terrain/pointcloud` | `/scan` |
| `imu_processor` | `sensor_pkg` | Processes IMU, estimates slope | `/imu/slope` | `/imu/data` |
| `encoder_processor` | `sensor_pkg` | Wheel odometry | `/odom` | `/encoder/ticks` |
| `terrain_mapper` | `mapping_pkg` | Builds occupancy grid | `/terrain/grid` | `/terrain/pointcloud` |
| `terrain_classifier` | `mapping_pkg` | Labels terrain type | `/terrain/classified` | `/terrain/grid` |
| `graph_builder` | `planning_pkg` | Builds NetworkX graph | `/graph/data` | `/terrain/classified` |
| `energy_model` | `planning_pkg` | Assigns energy weights | `/graph/weighted` | `/graph/data` |
| `classical_planner` | `planning_pkg` | A* / Dijkstra path | `/path/classical` | `/graph/weighted` |
| `quantum_optimizer` | `quantum_pkg` | QAOA optimization | `/path/quantum` | `/graph/weighted` |
| `path_executor` | `navigation_pkg` | Sends waypoints to robot | `/cmd_vel` | `/path/quantum`, `/path/classical` |
| `battery_monitor` | `evaluation_pkg` | Tracks energy use | `/battery/status` | `/cmd_vel`, `/odom` |
| `evaluator` | `evaluation_pkg` | Compares methods | `/metrics` | `/path/classical`, `/path/quantum`, `/battery/status` |

---

## 📦 Complete Software Stack

| Category | Tool | Version | Purpose |
|---|---|---|---|
| OS | Ubuntu | 22.04 LTS | Base operating system |
| ROS | ROS2 Humble | Humble Hawksbill | Robot middleware |
| Simulator | Gazebo | Classic 11 or Ignition | Physics simulation |
| Language | Python | 3.10+ | All logic |
| Quantum | Qiskit | 1.x | QAOA, quantum circuits |
| Quantum Sim | Qiskit Aer | 0.14+ | CPU-based quantum simulation |
| Graph | NetworkX | 3.x | Graph representation |
| Optimization | CVXPY / SciPy | Latest | QUBO formulation aid |
| Visualization | RViz2 | Bundled with ROS2 | Real-time robot viz |
| Visualization | Matplotlib | 3.x | Graphs, plots |
| CV | OpenCV | 4.x | Optional image processing |
| IDE | VS Code | Latest | Code editor |
| Version Control | Git + GitHub | Latest | Code management |
| Arduino | Arduino IDE | 2.x | Motor controller firmware |

---

## 🗂️ Complete Repository Folder Structure

This is the **entire folder structure** for the project. We will create files in it gradually.

```
lunar_rover_ws/                          ← ROS2 Workspace Root
│
├── src/                                 ← All ROS2 packages go here
│   │
│   ├── sensor_pkg/                      ← PHASE 4: Sensor Integration
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── sensor_pkg/
│   │   │   ├── __init__.py
│   │   │   ├── lidar_processor.py       ← Processes LiDAR scans
│   │   │   ├── imu_processor.py         ← Processes IMU, slope
│   │   │   └── encoder_processor.py     ← Wheel odometry
│   │   └── launch/
│   │       └── sensors_launch.py
│   │
│   ├── mapping_pkg/                     ← PHASE 5: Terrain Mapping
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── mapping_pkg/
│   │   │   ├── __init__.py
│   │   │   ├── terrain_mapper.py        ← Occupancy grid builder
│   │   │   └── terrain_classifier.py   ← Labels terrain zones
│   │   └── launch/
│   │       └── mapping_launch.py
│   │
│   ├── planning_pkg/                    ← PHASES 6,7,8: Planning
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── planning_pkg/
│   │   │   ├── __init__.py
│   │   │   ├── graph_builder.py         ← NetworkX graph from terrain
│   │   │   ├── energy_model.py          ← Energy cost assignment
│   │   │   ├── classical_planner.py     ← A* and Dijkstra
│   │   │   └── quantum_optimizer.py     ← QAOA via Qiskit
│   │   ├── config/
│   │   │   └── planner_params.yaml
│   │   └── launch/
│   │       └── planning_launch.py
│   │
│   ├── navigation_pkg/                  ← PHASE 9: ROS2 Integration
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── navigation_pkg/
│   │   │   ├── __init__.py
│   │   │   └── path_executor.py         ← Sends waypoints to motors
│   │   └── launch/
│   │       └── navigation_launch.py
│   │
│   ├── evaluation_pkg/                  ← PHASE 10: Evaluation
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── evaluation_pkg/
│   │   │   ├── __init__.py
│   │   │   ├── battery_monitor.py       ← Tracks energy usage
│   │   │   └── evaluator.py             ← Comparison metrics
│   │   └── launch/
│   │       └── evaluation_launch.py
│   │
│   └── rover_simulation/               ← PHASE 3: Simulation
│       ├── package.xml
│       ├── setup.py
│       ├── urdf/
│       │   └── rover.urdf.xacro         ← Robot description
│       ├── worlds/
│       │   └── lunar_terrain.world      ← Gazebo world file
│       ├── config/
│       │   ├── gazebo_params.yaml
│       │   └── rviz_config.rviz
│       └── launch/
│           └── simulation_launch.py
│
├── quantum/                             ← Standalone quantum scripts
│   ├── qaoa_solver.py                   ← Core QAOA algorithm
│   ├── qubo_formulator.py               ← QUBO matrix builder
│   ├── test_qaoa.py                     ← Unit tests for quantum
│   └── notebooks/
│       ├── qaoa_exploration.ipynb       ← Jupyter experimentation
│       └── energy_analysis.ipynb
│
├── docs/                                ← PHASE 12: Documentation
│   ├── architecture_diagram.png
│   ├── report/
│   │   └── final_report.pdf
│   └── presentation/
│       └── slides.pptx
│
├── scripts/                             ← Helper scripts
│   ├── setup_environment.sh             ← One-shot environment setup
│   ├── run_simulation.sh
│   └── run_full_pipeline.sh
│
├── tests/                               ← All tests
│   ├── test_sensor_pkg.py
│   ├── test_mapping_pkg.py
│   ├── test_planning_pkg.py
│   └── test_quantum.py
│
├── results/                             ← Experiment results
│   ├── classical_paths/
│   ├── quantum_paths/
│   └── energy_comparison/
│       └── comparison_plot.png
│
├── .gitignore
├── README.md
└── requirements.txt                     ← Python dependencies
```

---

## 📅 Complete Project Timeline (12 Weeks)

| Week | Phase | Key Goals |
|---|---|---|
| Week 1 | Phase 1 | Planning, architecture, GitHub setup |
| Week 2 | Phase 2 | Ubuntu, ROS2, Gazebo, Python, Qiskit install |
| Week 3 | Phase 3 | Lunar terrain in Gazebo, rover URDF, movement test |
| Week 4 | Phase 4 | LiDAR, IMU, encoder nodes, topic testing |
| Week 5 | Phase 5 | Occupancy grid, terrain classifier |
| Week 6 | Phase 6 | Energy model, terrain cost weights |
| Week 7 | Phase 7 | A* and Dijkstra classical planners |
| Week 8 | Phase 8 (part 1) | Qiskit setup, QUBO formulation |
| Week 9 | Phase 8 (part 2) | QAOA implementation, testing |
| Week 10 | Phase 9 | Full ROS2 pipeline integration |
| Week 11 | Phase 10 | Evaluation, comparison graphs |
| Week 12 | Phase 11–12 | Real robot + documentation |

---

## 🛠️ Step-by-Step Tasks for Phase 1

### Task 1.1 — Install Git and Create GitHub Repository

**Objective**: Set up version control so your work is saved and trackable.

**Commands**:
```bash
# Install Git
sudo apt update
sudo apt install git -y

# Configure Git identity
git config --global user.name "Your Name"
git config --global user.email "your@email.com"

# Verify installation
git --version
```

**Create GitHub Repository**:
1. Go to https://github.com
2. Click "New Repository"
3. Name: `lunar-rover-quantum`
4. Set to Public or Private
5. Initialize with README
6. Copy the clone URL

```bash
# Clone to your computer
cd ~
git clone https://github.com/YOUR_USERNAME/lunar-rover-quantum.git
cd lunar-rover-quantum
```

---

### Task 1.2 — Create the Full Folder Structure

**Objective**: Create all folders now, even if files come later.

```bash
# Navigate to project root
cd ~/lunar-rover-quantum

# Create ROS2 workspace structure
mkdir -p lunar_rover_ws/src

# Create package folders
mkdir -p lunar_rover_ws/src/sensor_pkg/sensor_pkg
mkdir -p lunar_rover_ws/src/sensor_pkg/launch
mkdir -p lunar_rover_ws/src/mapping_pkg/mapping_pkg
mkdir -p lunar_rover_ws/src/mapping_pkg/launch
mkdir -p lunar_rover_ws/src/planning_pkg/planning_pkg
mkdir -p lunar_rover_ws/src/planning_pkg/config
mkdir -p lunar_rover_ws/src/planning_pkg/launch
mkdir -p lunar_rover_ws/src/navigation_pkg/navigation_pkg
mkdir -p lunar_rover_ws/src/navigation_pkg/launch
mkdir -p lunar_rover_ws/src/evaluation_pkg/evaluation_pkg
mkdir -p lunar_rover_ws/src/evaluation_pkg/launch
mkdir -p lunar_rover_ws/src/rover_simulation/urdf
mkdir -p lunar_rover_ws/src/rover_simulation/worlds
mkdir -p lunar_rover_ws/src/rover_simulation/config
mkdir -p lunar_rover_ws/src/rover_simulation/launch

# Create non-ROS folders
mkdir -p quantum/notebooks
mkdir -p docs/report docs/presentation
mkdir -p scripts tests results/classical_paths results/quantum_paths results/energy_comparison

echo "✅ All folders created!"

# Verify structure
find lunar_rover_ws -type d | head -30
```

---

### Task 1.3 — Create README.md

**Objective**: Document the project right from the start.

Create a file `README.md` at the root of your GitHub repo:

```markdown
# 🌕 Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation

## Project Overview
An autonomous lunar rover navigation system that uses Quantum Approximate
Optimization Algorithm (QAOA) via Qiskit to find energy-efficient paths
across simulated lunar terrain.

## Hardware
- Arjuna AMR Robot Kit
- Jetson Nano / Raspberry Pi
- LiDAR, IMU, Encoders

## Software Stack
- ROS2 Humble
- Gazebo Classic / Ignition
- Qiskit (AerSimulator)
- Python 3.10+
- NetworkX

## Project Phases
1. Project Planning ← YOU ARE HERE
2. Environment Setup
3. Robot Simulation
4. Sensor Integration
5. Terrain Mapping
6. Energy Modeling
7. Classical Path Planning
8. Quantum Optimization
9. ROS2 Integration
10. Comparison & Evaluation
11. Real Robot Deployment
12. Final Documentation

## Team
[Your team names here]

## Institution
[Your institution name here]

## Setup Instructions
See: `docs/setup_guide.md` (coming in Phase 2)
```

---

### Task 1.4 — Create requirements.txt

**Objective**: List all Python dependencies in one place.

Create `requirements.txt` at root:

```txt
# ROS2 Python tools
colcon-common-extensions
rosdep

# Quantum Computing
qiskit>=1.0.0
qiskit-aer>=0.14.0
qiskit-algorithms>=0.3.0

# Graph & Math
networkx>=3.0
numpy>=1.24.0
scipy>=1.10.0

# Visualization
matplotlib>=3.7.0
seaborn>=0.12.0

# Computer Vision (optional)
opencv-python>=4.8.0

# Utilities
pyyaml>=6.0
tqdm>=4.65.0
pandas>=2.0.0
```

---

### Task 1.5 — Create .gitignore

**Objective**: Avoid committing build artifacts and large files.

Create `.gitignore`:

```gitignore
# ROS2 build artifacts
build/
install/
log/
*.pyc
__pycache__/

# Python
*.egg-info/
.venv/
venv/
dist/
.pytest_cache/

# Gazebo
*.bag
*.log

# Jupyter
.ipynb_checkpoints/

# VS Code
.vscode/
*.code-workspace

# OS
.DS_Store
Thumbs.db

# Large simulation assets
*.dae
*.stl
```

---

### Task 1.6 — Draw System Data Flow on Paper

**Objective**: Understand how data flows through the system.

Before continuing, draw this on paper (or use a whiteboard):

```
[LUNAR TERRAIN]
      │
      ▼
[LiDAR Sensor] ──► /scan ──► [lidar_processor] ──► /terrain/pointcloud
[IMU Sensor]   ──► /imu  ──► [imu_processor]   ──► /imu/slope
[Encoders]     ──► ticks ──► [encoder_proc]     ──► /odom
                                                        │
                                     ┌──────────────────┘
                                     ▼
                              [terrain_mapper]   ──► /terrain/grid
                              [terrain_classifier] ► /terrain/classified
                                                        │
                                     ┌──────────────────┘
                                     ▼
                              [graph_builder]    ──► /graph/data
                              [energy_model]     ──► /graph/weighted
                                                        │
                              ┌─────────────────────────┤
                              ▼                         ▼
                    [classical_planner]       [quantum_optimizer]
                              │                         │
                              ▼                         ▼
                        /path/classical           /path/quantum
                              │                         │
                              └────────────┬────────────┘
                                           ▼
                                   [path_executor]
                                           │
                                           ▼
                                     [ROVER MOTORS]
```

---

### Task 1.7 — Define Energy Cost Model (On Paper)

**Objective**: Understand what makes one path better than another.

Our energy cost for any terrain cell depends on:

```
Energy Cost = Base_Distance × Slope_Factor × Terrain_Factor × Roughness_Factor

Where:
  Base_Distance    = actual distance (meters)
  Slope_Factor     = 1.0 + 2.0 × |sin(slope_angle)|
  Terrain_Factor:
    flat rock      = 1.0
    loose soil     = 1.5
    crater edge    = 2.0
    steep slope    = 3.0
    obstacle       = ∞ (blocked)
  Roughness_Factor = 1.0 to 2.0 based on LiDAR variance
```

Write this in your notebook. We implement it in Phase 6.

---

## ✅ Expected Output for Phase 1

After completing all tasks:

1. GitHub repository created with proper README
2. Full folder structure exists on disk
3. `.gitignore` and `requirements.txt` created
4. System architecture understood and drawn
5. Data flow diagram drawn (on paper or digital)
6. Energy cost model defined conceptually

**Run this to verify:**
```bash
# List all created folders
find ~/lunar-rover-quantum/lunar_rover_ws -type d

# Check git status
cd ~/lunar-rover-quantum
git status
```

---

## ⚠️ Common Errors in Phase 1

| Error | Cause | Fix |
|---|---|---|
| `git: command not found` | Git not installed | Run `sudo apt install git -y` |
| `Permission denied` on mkdir | Wrong directory | Use `sudo` or check current path with `pwd` |
| Can't push to GitHub | No SSH key or wrong URL | Use HTTPS URL or setup SSH key |
| "Repository not found" | Wrong clone URL | Double-check GitHub URL |

---

## 🧪 Testing Phase 1

```bash
# Test 1: Git is working
git --version
# Expected: git version 2.x.x

# Test 2: Folder structure exists
ls lunar_rover_ws/src/
# Expected: sensor_pkg  mapping_pkg  planning_pkg  navigation_pkg  evaluation_pkg  rover_simulation

# Test 3: Commit and push to GitHub
cd ~/lunar-rover-quantum
git add .
git commit -m "Phase 1: Initial project structure and planning"
git push origin main
# Expected: No errors, files appear on GitHub
```

---

## 📦 Deliverables for Phase 1

- [ ] GitHub repository created and accessible
- [ ] `README.md` written
- [ ] `requirements.txt` created
- [ ] `.gitignore` created
- [ ] Full folder structure created
- [ ] System architecture diagram (hand-drawn or digital)
- [ ] Data flow diagram (hand-drawn or digital)
- [ ] Energy cost model defined in notes
- [ ] First git commit pushed to GitHub

---

## 📝 Phase 1 Summary

**What we completed:**

1. Understood the complete project structure and purpose
2. Defined the 4-layer system architecture (Perception → Planning → Execution → Evaluation)
3. Listed all 12 ROS2 nodes we will build
4. Defined the complete software stack
5. Created the full folder structure (empty, but ready)
6. Set up GitHub version control
7. Defined the energy cost model conceptually
8. Planned 12-week timeline

**Key Insight**: Everything in this project connects together. LiDAR → terrain graph → quantum optimizer → motor commands. Understanding this flow early will prevent confusion in later phases.

---

## 👀 Next Step Preview — Phase 2: Environment Setup

In Phase 2, we will:
1. Install Ubuntu 22.04 (if not done)
2. Install ROS2 Humble (the full desktop version)
3. Install Gazebo simulation environment
4. Install Python dependencies (Qiskit, NetworkX, etc.)
5. Set up VS Code with ROS2 extensions
6. Test that everything is installed correctly

This takes approximately **1–2 days** and requires a stable internet connection.

---

## 🔁 Continuation Prompt

Copy and paste this prompt to continue to Phase 2:

---

**CONTINUATION PROMPT:**

```
We are building a project titled "Quantum-Assisted Energy Optimization for 
Autonomous Lunar Rover Navigation" using the Arjuna AMR robot kit.

PROJECT STATUS:
✅ Phase 1 (Project Planning) — COMPLETE

WHAT WAS DONE IN PHASE 1:
- Defined 4-layer system architecture: Perception → Planning → Execution → Evaluation
- Listed all 12 ROS2 nodes with their topics
- Defined complete software stack: Ubuntu 22.04, ROS2 Humble, Gazebo, 
  Python 3.10, Qiskit AerSimulator, NetworkX
- Created full folder structure in ~/lunar-rover-quantum/lunar_rover_ws/src/
  Packages: sensor_pkg, mapping_pkg, planning_pkg, navigation_pkg, 
  evaluation_pkg, rover_simulation
- Created README.md, requirements.txt, .gitignore
- Defined energy cost formula: Energy = Distance × Slope_Factor × Terrain_Factor
- Set up GitHub repository

HARDWARE: Arjuna AMR kit, Jetson Nano/RPi, LiDAR, IMU, Encoders, Arduino
QUANTUM: Using Qiskit AerSimulator (NOT real quantum hardware)
ROS VERSION: ROS2 Humble
OS: Ubuntu 22.04

CURRENT STATE:
- GitHub repo exists with initial folder structure
- No software installed yet
- No code written yet

NEXT NEEDED:
Please give me PHASE 2 — ENVIRONMENT SETUP with full beginner-friendly steps for:
1. Ubuntu 22.04 installation check/setup
2. ROS2 Humble complete installation
3. Gazebo installation and first test
4. Python environment setup with all dependencies
5. Qiskit installation and basic test
6. VS Code installation with ROS2 extensions
7. Testing all installations work together

Format each step with: Objective, Concepts, Requirements, Commands, 
Expected Output, Common Errors, Debugging Tips, Testing, Deliverables,
Step Summary, Next Step Preview, and Continuation Prompt.
```

---

*Document Version: Phase 1 Complete | Project: Lunar Rover Quantum Navigation*
