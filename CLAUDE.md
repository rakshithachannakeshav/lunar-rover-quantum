# CLAUDE.md

Persistent context for this repo. Read this before doing anything — it captures where the project actually stands, not where the docs claim it stands.

## What this project is

"Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation" — a final-year major project. Simulates a lunar rover planning energy-efficient paths using both classical algorithms (A*/Dijkstra) and QAOA (Qiskit AerSimulator, CPU-simulated, no real quantum hardware). Full architecture, setup instructions, and the complete phase-by-phase implementation blueprint are in **`docs/PROJECT_GUIDE.md`** — that's the one canonical doc, read it for depth. This file is just the "pick up where we left off" summary.

## Actual phase status (verified against files, not assumed)

| Phase | Status |
|---|---|
| 1 — Planning | Done |
| 2 — Environment Setup | Scripts exist (`scripts/install_*.sh`), but **no environment has successfully completed them yet** — see Environment Notes below |
| 3 — Robot Simulation (`rover_simulation`) | Done, verified working (user confirmed the sim runs) |
| 4 — Sensor Integration (`sensor_pkg`) | Mostly done. LiDAR + IMU processors verified correct by direct testing. **Encoder/odometry has a confirmed gap: tracks forward/backward distance but never tracks heading/turning** (`self.theta` never updates in `encoder_processor.py`) |
| 5 — Terrain Mapping (`mapping_pkg`) | Not started — scaffolding only (`package.xml`/`setup.py`/empty `__init__.py`) |
| 6 — Energy Modeling, 7 — Classical Planning (`planning_pkg`) | Not started — scaffolding only |
| 8 — Quantum Optimization (QAOA) | Not started — `quantum/notebooks/` is empty |
| 9 — ROS2 Integration (`navigation_pkg`) | Not started — scaffolding only |
| 10 — Evaluation (`evaluation_pkg`) | Not started — scaffolding only |
| 11 — Real Robot Deployment | Not started |
| 12 — Final Documentation | Partial — `README.md` + `docs/PROJECT_GUIDE.md` are current; no final report/slides yet |

**Next logical work:** `mapping_pkg` (Phase 5) — full reference implementation (terrain_mapper, terrain_classifier) is in `docs/PROJECT_GUIDE.md` Part 5, Step 5.1 onward.

## Repo structure conventions

- **Repo root IS the ROS2 colcon workspace** — `src/` sits directly at the top, there's no separate `lunar_rover_ws/` folder. `colcon build` runs from the repo root.
- **`src/`** — the actual ROS2 packages (buildable, each with `package.xml`+`setup.py`). `mapping_pkg`/`planning_pkg`/`navigation_pkg`/`evaluation_pkg` use `ament_python` scaffolding (package.xml + setup.py + resource marker + empty `__init__.py`, no logic yet). `rover_simulation` and `sensor_pkg` use `ament_cmake` (real, working code).
- **`scripts/`** — standalone one-off shell/Python scripts, not part of any ROS2 package: environment install scripts, terrain asset generators, diagnostics, manual test helpers.
- **`docs/`** — just `PROJECT_GUIDE.md` (the one consolidated doc) and `reference/teammate_simulation_files/` (an alternate, unused simulation setup from a teammate's zip, kept for reference, not wired into the build).
- **`web/`** — a standalone browser-based (Three.js) rover viewer, separate from the ROS2/Gazebo simulation. Not part of the original 4-layer architecture, kept as a top-level folder per an explicit decision to keep it.

## Git/branch setup

- History was completely rewritten partway through this project's life (old messy history wiped, fresh start) — don't be surprised the history looks short.
- Branches: `main`, `develop`, `feature/rover_simulation` (simulation-only snapshot), `feature/sensor_pkg` (simulation + sensors). Workflow is PR-based: `develop` → `main` via GitHub PR, not direct push (manual GitHub branch-protection setup was given but confirm it's actually been applied before assuming direct pushes are blocked).
- **`gh` CLI is not installed** on the primary dev (Windows) machine — branch protection / PR operations need the GitHub web UI, not `gh`.
- **The user runs `git commit` themselves** — don't run `git commit` on their behalf; give them the exact commands instead. (Other git operations like `checkout`/`merge`/`push` have been fine to run directly.)

## Environment / VM notes

- Primary dev machine is Windows; ROS2 Jazzy + Gazebo Harmonic need Ubuntu 24.04 ("Noble") — doesn't run natively on Windows, and WSL2 isn't usable on this machine (needs "Virtual Machine Platform" + BIOS virtualization, neither enabled).
- There's an existing VirtualBox VM named **"Ubuntu-ROS"** — but it's actually **Ubuntu 22.04 ("Jammy")**, previously set up for ROS2 Humble + Gazebo Classic 11. This is incompatible with Jazzy/Harmonic (confirmed: apt fails with a wall of "not installable" dependency errors — Jazzy needs `libstdc++6 >= 13.1`, `libc6 >= 2.38`, etc. that don't exist in 22.04's repos).
- Decision on how to fix this (fresh Ubuntu 24.04 VM vs. `do-release-upgrade` the existing one) was **deferred** — check with the user before assuming either path.
- Don't trust VirtualBox's stored "OS type" label — confirm actual OS with `cat /etc/os-release` inside the VM.

## Known deferred work (explicitly asked to wait, not forgotten)

- `tests/sensor_pkg` test file — user wants it eventually, explicitly said to defer so the sensor/mapping teammate isn't blocked.
- GitHub Actions CI (`.github/workflows/`) to run tests on every PR — also explicitly deferred, nothing currently exists on disk or in git for this.
- `D:\lunar-rover-quantum-pending-upload\rover_video.tar` — a demo screen recording, deliberately kept out of git (too large for a repo). Needs to be uploaded somewhere (Drive/YouTube/etc.) and linked from the README once that's done.

## Other loose ends

- `D:\lunar-rover-quantum-BACKUP-2026-07-29\` — a full backup taken before the initial reorg, still there, not needed unless something needs recovering.
- `D:\lunar-rover-quantum\.extract_tmp\` (outside the repo) — an empty, harmless folder stuck with a locked file handle from an earlier zip extraction. Not part of the repo, safe to ignore; will clear on its own eventually (reboot/closing whatever has it open).
