# Port rover_simulation to Gazebo Classic 11 / ROS2 Humble

## Context

The project's documented stack (`README.md`) and all Phase 2 install scripts
target ROS2 Jazzy + Gazebo Harmonic, which require Ubuntu 24.04. The only VM
currently available for testing (VirtualBox "Ubuntu-ROS") is Ubuntu 22.04
("Jammy"), previously set up for ROS2 Humble + Gazebo Classic 11. Jazzy/Harmonic
cannot install on 22.04 (confirmed: apt dependency errors — Jazzy needs
`libstdc++6 >= 13.1`, `libc6 >= 2.38`, unavailable in 22.04's repos).

Decision: rather than build a new 24.04 VM or upgrade this one, port the
simulation backward to target Gazebo Classic 11 / ROS2 Humble, matching what
this VM already has. `docs/PROJECT_GUIDE.md` Phase 2/3 (lines 447-1130)
documents an original Humble/Classic-targeted design from before the project
pivoted to Jazzy/Harmonic — this port reuses that design's plugin syntax
(diff-drive, LiDAR, IMU) rather than inventing a new Classic port from
scratch.

## Goals

- `src/rover_simulation` runs on Gazebo Classic 11 / ROS2 Humble on the
  existing "Ubuntu-ROS" VM, with the same terrain, rover geometry, and
  drive behavior it currently has under Harmonic.
- Add working LiDAR (`/scan`) and IMU (`/imu/data`) Gazebo sensor plugins to
  the simulated rover — currently neither exists under Harmonic even though
  `sensor_pkg` (`lidar_processor.py`, `imu_processor.py`) already expects
  those topics. This lets `sensor_pkg` be tested end-to-end against live sim
  data in the same session.
- Supporting install/test scripts on the VM work against Humble/Classic
  instead of Jazzy/Harmonic.

## Non-goals

- No new Ubuntu 24.04 VM, no `do-release-upgrade`.
- No reconciling the existing mismatch between the 2-wheel+casters rover
  actually spawned into Gazebo (`models/lunar_rover/model.sdf`, via the
  world's `<include>`) and the separate 6-wheel `rover.urdf.xacro` used only
  for `robot_state_publisher`/RViz. That mismatch predates this port and is
  left as-is; `model.sdf` is the one being ported since it's what's actually
  simulated.
- No changes to `rover.urdf.xacro` itself — its own Gazebo plugin block is
  already dead code (never spawned into Gazebo today) and stays that way.
- No dual Harmonic/Classic support — this replaces the Harmonic-specific
  files in place. Recoverable from git history if a 24.04 environment shows
  up later.
- `setup_workspace.sh` / `setup_ws_wsl.sh` are WSL-only (copy files from a
  Windows mount) and don't apply to a VirtualBox VM tested over `git clone`.
  Left untouched; no VM-equivalent script added — cloning the repo directly
  in the VM is simple enough to be a plain instruction, not a script.

## Design

### Core simulation files (`src/rover_simulation/`)

**`worlds/lunar_terrain.world`**
- Remove the five Harmonic-only system `<plugin>` declarations
  (`gz-sim-physics-system`, `gz-sim-user-commands-system`,
  `gz-sim-scene-broadcaster-system`, `gz-sim-contact-system`,
  `gz-sim-sensors-system`) — Classic loads its systems automatically, no
  explicit world-level declaration needed.
- Convert `<physics name="default_physics" type="ignored">` (DART, Harmonic
  syntax) to `<physics type="ode">` with equivalent `max_step_size` /
  `real_time_update_rate` (derived from the existing `real_time_factor`).
- Everything else unchanged: heightmap terrain, static rocks, trail decals,
  goal marker, and the `<include><uri>model://lunar_rover</uri></include>`
  that spawns the rover. Gazebo Classic's SDF heightmap support
  (`<heightmap><uri>/<size>/<pos>/<texture>`) is compatible with what's
  already there.

**`models/lunar_rover/model.sdf`**
- Replace the `gz::sim::systems::DiffDrive` plugin
  (`gz-sim-diff-drive-system`) with Classic's `libgazebo_ros_diff_drive.so`,
  keeping the same `left_wheel_joint`/`right_wheel_joint`, wheel separation
  (0.84 m), and wheel radius (0.18 m). Classic's plugin publishes odometry
  and TF directly — set `publish_odom`, `publish_odom_tf`, `publish_wheel_tf`
  true, `odometry_frame: odom`, `robot_base_frame: base_link` (matching the
  existing `frame_id`/`child_frame_id`).
- Add two new links/joints that don't exist in `model.sdf` today —
  `lidar_link` and `imu_link` — fixed-mounted on `base_link`, carrying sensor
  + plugin blocks adapted from `docs/PROJECT_GUIDE.md` lines 931-992:
  - LiDAR: `<sensor type="ray">`, 360 samples, ±π range, 0.12–15.0 m,
    `libgazebo_ros_ray_sensor.so` remapped to `/scan`.
  - IMU: `<sensor type="imu">`, `libgazebo_ros_imu_sensor.so` remapped to
    `/imu/data`.
  - These topic names match what `sensor_pkg` already subscribes to — no
    changes needed in `sensor_pkg` itself.

**`config/bridge.yaml`**
- Deleted. Classic's `gazebo_ros` plugins publish ROS topics natively; no
  `ros_gz_bridge` equivalent is needed.

**`package.xml` / `CMakeLists.txt`**
- `package.xml`: drop `ros_gz_sim`, `ros_gz_bridge`; add `gazebo_ros`,
  `gazebo_plugins`, `gazebo_ros_pkgs`. Update the `<description>` from
  "Gazebo Harmonic" to "Gazebo Classic 11".
- `CMakeLists.txt`: no structural changes needed (it only installs
  directories/files, doesn't `find_package` the sim-specific deps).

**`launch/simulation_launch.py`**
- Replace the `gz sim -s`/`gz sim -g` `ExecuteProcess` pair and the
  `ros_gz_bridge` parameter_bridge `Node` with `gazebo_ros`'s standard
  `gazebo.launch.py` (`IncludeLaunchDescription`), passing `world` as a
  launch argument. Model resolution moves from `GZ_SIM_RESOURCE_PATH` to
  `GAZEBO_MODEL_PATH` (set to the package's `models` dir).
- `robot_state_publisher` and `rviz2` nodes are unchanged.
- `headless` argument keeps the same meaning (Classic: server always runs;
  `gzclient` only launches when `headless:=false`).

**`launch/demo.launch.py`**
- No changes — it only includes `simulation_launch.py` and adds the
  patrol/creep mode nodes, none of which are Harmonic-specific.

**`rover.urdf.xacro`**
- No changes (see Non-goals).

### Supporting scripts (`scripts/`)

- `setup_ros2_repo.sh`: apt line `noble main` → `jammy main`.
- `install_ros2_jazzy.sh` → rename to `install_ros2_humble.sh`:
  `ros-jazzy-desktop` → `ros-humble-desktop`; fix hardcoded
  `/home/monis/.bashrc` → `$HOME/.bashrc`.
- `install_gazebo_harmonic.sh` → rename to `install_gazebo_classic.sh`: drop
  the OSRF repo/key addition (Gazebo 11 Classic ships via
  `ros-humble-gazebo-ros-pkgs` in ROS Humble's own apt repo, no separate OSRF
  repo needed); verify with `gazebo --version` instead of `gz sim --version`.
- `install_sim_dependencies.sh`: `ros-jazzy-*` → `ros-humble-*` for every
  package in the list.
- `open_gazebo_gui.sh`: `/opt/ros/jazzy` → `/opt/ros/humble`; drop
  `GZ_SIM_RESOURCE_PATH`/`GZ_SIM_SYSTEM_PLUGIN_PATH`; use `gzclient` instead
  of `gz sim -g`.
- `test_physics.sh`: `/opt/ros/jazzy` → `/opt/ros/humble`; fix hardcoded
  `/home/monisha/...` paths → `$HOME/...`; replace the manual `gz sim -s` +
  `ros_gz_bridge parameter_bridge` + `ros_gz_sim create` sequence with
  `ros2 launch gazebo_ros gazebo.launch.py` + `spawn_entity.py` (no bridge
  step — Classic plugins publish directly).
- `drive_test.sh`: `/opt/ros/jazzy` → `/opt/ros/humble`.
- `setup_workspace.sh` / `setup_ws_wsl.sh`: untouched (see Non-goals).

### Docs

- `README.md`: "Stack" line — "ROS2 (Jazzy), Gazebo Harmonic" → "ROS2
  (Humble), Gazebo Classic 11".
- `src/rover_simulation/package.xml`: description field updated (see above).
- `CLAUDE.md`: "Environment / VM notes" section updated to record that the
  22.04-vs-24.04 mismatch was resolved by porting the simulation to
  Humble/Classic instead of upgrading the VM or building a new one.

## Validation plan

No ROS2/Gazebo toolchain exists on the Windows dev machine — all validation
happens on the VM, after `colcon build --symlink-install` in the repo root:

1. `ros2 launch rover_simulation demo.launch.py mode:=creep` — Gazebo starts
   headless with no plugin-load errors in the log.
2. `ros2 topic list` includes `/odom`, `/tf`, `/scan`, `/imu/data`,
   `/cmd_vel`.
3. `ros2 topic echo /odom --field pose.pose.position` shows changing values
   while `rover_creep.py` drives the rover.
4. `ros2 topic echo /scan --once` and `ros2 topic echo /imu/data --once`
   both return non-empty, plausible data — confirms the new sensor plugins
   actually work (this is new coverage; neither topic has ever been live
   before, even under Harmonic).
5. `ros2 run sensor_pkg lidar_processor` / `imu_processor` /
   `encoder_processor` all run against the live topics without errors.

## Open questions

None outstanding — all scope decisions were confirmed during brainstorming.
