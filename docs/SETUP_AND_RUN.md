# Setup & Run Guide

Complete step-by-step instructions to get this project running from a fresh clone — environment setup, build, launch, and what's actually verified working right now. Read this before starting new work so you know what state things are in.

## Current status (as of this doc)

| Package | Status |
|---|---|
| `rover_simulation` | **Working.** Gazebo Harmonic sim with terrain, rover model, physics — verified end-to-end. |
| `sensor_pkg` | **Mostly working.** LiDAR and IMU processors verified correct (unit-tested). Encoder/odometry tracks forward/backward motion correctly, but **does not track turning** — heading (`theta`) never updates, so `/odom` is wrong whenever the rover actually turns. See [Known issues](#known-issues). |
| `mapping_pkg`, `planning_pkg`, `navigation_pkg`, `evaluation_pkg` | Scaffolding only (`package.xml`/`setup.py`/empty `__init__.py`) — no implementation yet. |
| `web/` (browser 3D viewer) | **Working.** A CSS bug that hid the working scene behind a false error message has been fixed. |
| `quantum/notebooks` | Not started. |

## 1. Environment requirements

ROS2 Jazzy + Gazebo Harmonic need **Ubuntu 24.04 "Noble"**. This does not run natively on Windows — you need a Linux environment: a VM (VirtualBox recommended, see below), WSL2, or dual-boot.

**Important:** if you're using an existing VM from earlier project work, check its actual OS version first:
```bash
cat /etc/os-release
```
Ubuntu 22.04 ("Jammy") **cannot** install ROS2 Jazzy — the required system libraries (`libstdc++6 >= 13.1`, `libc6 >= 2.38`, `libpython3.12t64`) don't exist in 22.04's repos, and the package manager will fail with a wall of "not installable" dependency errors. If your VM is 22.04, you need a fresh Ubuntu 24.04 VM instead of trying to force Jazzy onto it.

### Setting up a VirtualBox VM (if you don't have one already)

1. Install [VirtualBox](https://www.virtualbox.org/wiki/Downloads) + Extension Pack.
2. Download the **Ubuntu 24.04 LTS "Noble Numbat"** desktop ISO.
3. Create a VM: Type Linux, Version Ubuntu (64-bit). Give it real resources — Gazebo isn't lightweight:

   | Resource | Minimum | Recommended |
   |---|---|---|
   | RAM | 4 GB | 8 GB+ |
   | CPU cores | 2 | 4+ |
   | Disk | 25 GB | 40 GB+ |

4. In VM Display settings: max out Video Memory (128 MB) and enable **3D Acceleration** if you want the real Gazebo GUI window (optional — RViz works without this, see Step 5).
5. Install Ubuntu, then install Guest Additions (VirtualBox menu → *Devices → Insert Guest Additions CD*).

## 2. Get the code

```bash
git clone https://github.com/rakshithachannakeshav/lunar-rover-quantum.git ~/lunar-rover-quantum
cd ~/lunar-rover-quantum
```

The repo root **is** the ROS2 workspace — `src/` sits directly at the top, there's no separate `lunar_rover_ws/` folder to `cd` into. Every command below assumes you're standing in `~/lunar-rover-quantum`.

## 3. Install dependencies

These scripts already exist in `scripts/` — run them once, in this order (each does real `apt-get` work and takes a few minutes):

```bash
chmod +x scripts/*.sh

bash scripts/setup_ros2_repo.sh        # ROS2 apt repo + locale setup
bash scripts/install_ros2_jazzy.sh     # ROS2 Jazzy desktop + colcon + rosdep
bash scripts/install_gazebo_harmonic.sh # Gazebo Harmonic + ros_gz bridge
bash scripts/install_sim_dependencies.sh # xacro, rviz2, nav2, teleop, tf tools...
bash scripts/install_python_quantum.sh  # Python + Qiskit/quantum stack (creates a venv)
```

**Note:** `install_ros2_jazzy.sh` and `install_python_quantum.sh` hardcode `/home/monis/.bashrc` (a specific person's username from before this repo was reorganized). If your username isn't `monis`, those specific "add sourcing to `.bashrc`" lines silently do nothing — harmless, but it means new terminals won't auto-source ROS2. Just run `source /opt/ros/jazzy/setup.bash` manually each new terminal.

## 4. Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/lunar-rover-quantum
colcon build --symlink-install
```

Expect `evaluation_pkg`, `mapping_pkg`, `navigation_pkg`, `planning_pkg`, `rover_simulation`, `sensor_pkg` to all report **Finished**, none **Failed**. The four scaffolding packages build near-instantly since there's no real code in them yet.

## 5. Launch the simulation

```bash
source install/setup.bash
ros2 launch rover_simulation simulation_launch.py
```

Gazebo starts **headless** by default (no window) — this is intentional (see the comment at the top of `src/rover_simulation/launch/simulation_launch.py`). To see something visually:

- **Gazebo GUI** (needs working 3D acceleration): `bash scripts/open_gazebo_gui.sh` in a second terminal
- **RViz instead** (more VM-friendly, usually the better default):
  ```bash
  ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true
  ```

If the Gazebo GUI window is black or crashes, that's a 3D-acceleration problem in the VM, not a code bug — use RViz instead.

## 6. Sanity-check it's running

```bash
ros2 topic list
ros2 topic echo /odom --field pose.pose.position
```
Expect `/odom`, `/cmd_vel`, `/scan`, `/imu/data`, `/joint_states`, `/tf` among the topics. Drive it manually:
```bash
ros2 run rover_simulation rover_keyboard.py
```

## 7. Run the sensor nodes

```bash
ros2 launch sensor_pkg sensors_launch.py
```
Publishes `/terrain/pointcloud` (from `/scan`) and `/imu/slope` (from `/imu/data`) — both verified correct. Also republishes `/odom` from `/joint_states` — see [Known issues](#known-issues) below before relying on this for anything involving turning.

## 8. Test the web viewer (no Linux/VM needed — runs on Windows too)

```bash
cd web
python -m http.server 8080
```
Open `http://localhost:8080/index.html` in a browser (not a `file://` URL directly — it needs to be served over HTTP). You should see the 3D rover scene immediately: rover model, rocks, terrain, telemetry HUD, drive controls. Drive with WASD after clicking the canvas; try the **Patrol AI** tab and **Orbit**/**Top** camera modes.

If you still see a "3D engine failed to load" message, hard-refresh (`Ctrl+Shift+R`) to bust a cached pre-fix copy of `style.css`.

## Known issues

- **`encoder_processor.py` doesn't track heading.** It correctly computes forward/backward distance from wheel joint deltas, but `self.theta` is initialized to `0.0` and never updated — there's no `(dr - dl) / wheel_separation` angular term. The published orientation quaternion is always identity. Confirmed by direct testing: even a sharp left/right wheel differential leaves `theta` at exactly `0.0` and `y` at ~0. This matches the encoder's own documented status ("🟡 Mostly Completed... final odometry verification may require relaunch") — it's a known gap, not a new regression. Needs real differential-drive kinematics (wheel separation + radius) added before `/odom` can be trusted for anything involving turning.
- **VirtualBox's stored "OS type" label isn't authoritative.** Always confirm the actual guest OS with `cat /etc/os-release` rather than trusting VirtualBox's VM settings label — they can silently disagree.

## Troubleshooting reference

| Symptom | Likely cause |
|---|---|
| `gz sim --version` fails | Gazebo install step didn't complete — re-run `install_gazebo_harmonic.sh` |
| Wall of "Depends: ... not installable" errors during `install_ros2_jazzy.sh` | Wrong Ubuntu version — you need 24.04, not 22.04 (see Section 1) |
| colcon build fails on a specific package | Missing rosdep — try `rosdep install --from-paths src --ignore-src -r -y` from the repo root |
| Gazebo GUI window is black/crashes | 3D acceleration not working in the VM — use RViz instead (Step 5) |
| `ros2: command not found` | Forgot to `source /opt/ros/jazzy/setup.bash` in this terminal |
| Package not found after build | Forgot to `source install/setup.bash` (separate from the ROS2 system source) |
