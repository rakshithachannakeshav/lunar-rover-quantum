# Complete Phases 1–5 — Design

**Date:** 2026-09-09
**Status:** Implemented and **verified end-to-end on Ubuntu 24.04 + ROS2 Jazzy +
Gazebo Harmonic 8.15** (2026-09-10). Sim launches, rover drives, `/scan` 10 Hz +
`/imu/data` 50 Hz, `/map` + `/terrain_map` build, RViz renders, `.npz` persisted,
13 unit tests pass. Open follow-up: terrain-classifier parameter tuning.
**Scope:** Close the gaps that stop Phases 1–5 from being complete and runnable end-to-end.

## Problem

The mapping-phase code (`mapping_pkg`) is written and its pure-Python logic is unit-tested,
but the pipeline has never run against the simulation because **nothing publishes `/scan`**:

- `lunar_terrain.world`'s inlined `lunar_rover` model has no `<sensor>` blocks.
- `rover.urdf.xacro` has a `lidar_link` "placeholder" only — no sensor, no `imu_link`.
- `bridge.yaml` bridges only `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`.

Secondary gaps:

- `encoder_processor.py` never updates heading (`theta` stays `0.0`) and integrates wheel
  joint-angle deltas (radians) as if they were metres — `/odom` is wrong on turns and
  ~5.5× short on distance.
- `README.md`, `docs/PROJECT_GUIDE.md`, `src/sensor_pkg/README.md`, `tests/README.md` all
  still describe the pre-mapping state; `sensor_pkg/README.md` documents a different
  mapping design than what was built.

## Constraints

- No ROS2 / Gazebo on the dev machine (Windows). Sim changes are written and reviewed
  but not executed here; end-to-end verification happens on Ubuntu 24.04 + ROS2 Jazzy
  via a delivered checklist.
- The rover is **inlined** in `lunar_terrain.world`; `gz sim` never reads the Xacro at
  runtime (Xacro feeds `robot_state_publisher` for TF only). Sensor blocks must go in the
  world file to take effect; Xacro gets mirrored additions for consistency + TF frames.

## Decisions

- **LiDAR sensor type: `type='gpu_lidar'`.** Gazebo Harmonic's sensor set only ships
  `gpu_lidar` (the Classic CPU `ray`/`lidar` sensor was not ported), so this is the only
  option. It renders offscreen via the already-loaded `gz-sim-sensors-system` (ogre2).
  On a VM without 3D acceleration the checklist covers the fallbacks:
  `LIBGL_ALWAYS_SOFTWARE=1` (llvmpipe) or switching the render engine to `ogre`.
- **LiDAR profile: balanced** — 360 samples over ±π, 0.12–12 m range, 10 Hz, gaussian
  noise stddev 0.01. Matches values already in `sensor_pkg/lidar_processor.py`.
- **IMU:** CPU sensor, 50 Hz; requires adding `gz-sim-imu-system` to the world plugins.
- **No changes to `mapping_pkg` logic** — it is already correct, just starved of input.
- **DiffDrive `mid_*_joint` config in the Xacro left as-is** — out of scope.

## Changes

### 1. `src/rover_simulation/worlds/lunar_terrain.world`
- Add `<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>` to the world
  plugin list (after the existing `gz-sim-sensors-system` line).
- Inside the inlined model's `<link name='base_footprint'>`, add:
  - `<sensor name='lidar' type='gpu_lidar'>` — pose `0.2 0 0.555 0 0 0` (the existing
    `lidar_link` collision-lump location), `<topic>scan</topic>`,
    `<gz_frame_id>lidar_link</gz_frame_id>`, `<update_rate>10</update_rate>`,
    `<always_on>1</always_on>`, `<horizontal>` 360 samples `-3.14159`..`3.14159`,
    `<vertical>` 1 sample `0..0`, range `0.12`..`12.0` res `0.01`,
    gaussian noise mean `0` stddev `0.01`.
  - `<sensor name='imu' type='imu'>` — pose `0 0 0.4 0 0 0`, `<topic>imu/data</topic>`,
    `<gz_frame_id>imu_link</gz_frame_id>`, `<update_rate>50</update_rate>`,
    `<always_on>1</always_on>`, small gaussian noise on angular_velocity (stddev 0.002)
    and linear_acceleration (stddev 0.017).

### 2. `src/rover_simulation/urdf/rover.urdf.xacro`
- Add `imu_link` + fixed `imu_joint` (parent `base_link`, origin `0 0 0.07`), mirroring
  the existing `lidar_link` block.
- Add `<gazebo reference="lidar_link">` and `<gazebo reference="imu_link">` `<sensor>`
  blocks equivalent to the world file's, so a future `gz sdf` regeneration stays in sync.
  Documented as not-used-at-runtime.

### 3. `src/rover_simulation/config/bridge.yaml`
Append two `GZ_TO_ROS` entries (no TF entries touched):
- `/scan` — `sensor_msgs/msg/LaserScan` ↔ `gz.msgs.LaserScan`
- `/imu/data` — `sensor_msgs/msg/Imu` ↔ `gz.msgs.IMU`

### 4. `src/sensor_pkg/sensor_pkg/` — encoder odometry
- `sensor_pkg` is `ament_cmake` and installs its nodes as **flat scripts** into
  `lib/sensor_pkg/`, not as an importable package. So the new module is imported
  by bare name (`from diff_drive_odometry import ...`), added to the
  `install(PROGRAMS ...)` list in `CMakeLists.txt`, and the unit test puts
  `src/sensor_pkg/sensor_pkg` on `sys.path` and imports it flat.
- New pure module `diff_drive_odometry.py`:
  - `integrate_pose(x, y, theta, d_left_m, d_right_m, wheel_separation) -> (x, y, theta)`
    — exact/midpoint differential-drive kinematics; θ wrapped to (−π, π].
  - `yaw_to_quaternion(theta) -> (qx, qy, qz, qw)`.
- Rewire `encoder_processor.py`:
  - params: `wheel_radius=0.18`, `wheel_separation=0.84`,
    `left_joint_names=['front_left_joint','mid_left_joint','rear_left_joint']`,
    `right_joint_names=['front_right_joint','mid_right_joint','rear_right_joint']`
    (average whichever are present in the incoming `JointState`).
  - wheel-angle delta → arc length via `× wheel_radius`; update θ; publish a real yaw
    quaternion; set `child_frame_id='base_footprint'`; fill `twist.twist` from per-callback
    deltas / dt.
- `tests/test_diff_drive_odometry.py` (runs under existing `pytest.ini`, `sys.path` insert
  of `src/sensor_pkg` like `tests/test_grid_utils.py`):
  - straight line (`d_left == d_right`) advances x only, θ unchanged;
  - in-place spin (`d_left == -d_right`) changes θ only, x/y ≈ 0;
  - known quarter-turn arc lands at the expected pose within tolerance;
  - θ wraps past ±π correctly.

### 5. Docs
- `README.md` — Current status: Phase 3 & 4 done, Phase 5 implemented + unit-tested;
  note `/scan` and `/imu/data` now produced by the sim; add `occupancy_grid_node` /
  `terrain_classifier_node`; add encoder-heading fix to "Resolved issues"; fix folder tree
  (`docs/reference/`, no `docs/report|presentation/`).
- `docs/PROJECT_GUIDE.md` — status table + lines ~92 / ~309; add a "Launch the mapping
  pipeline" step to Part 1 (§9); add `/map`, `/terrain_map` to expected topics; mark the
  Part 5 Phase-5 blueprint superseded by `docs/reference/terrain_map_encoding.md`.
- `src/sensor_pkg/README.md` — rewrite "Phase 5 — Mapping" to the real nodes/topics/labels
  (`/scan`→`/map`→`/terrain_map`, `-1/10/50/90/100`); encoder status → complete; correct
  the sensor-topic component list.
- `tests/README.md` — "Current state": mapping + odometry unit tests exist and run in CI.

### 6. `docs/VERIFICATION_up_to_phase5.md` (new)
Terminal-by-terminal checklist for Ubuntu / ROS2 Jazzy:
`colcon build --symlink-install` → launch sim → `ros2 topic list` shows `/scan` +
`/imu/data` → `ros2 topic echo /scan --field header.frame_id` = `lidar_link` →
`ros2 run tf2_tools view_frames` shows `map → odom → base_footprint → … → lidar_link` →
`ros2 launch mapping_pkg mapping.launch.py` → drive with `rover_keyboard.py` → `/map`
fills in → `/terrain_map` gets non-`-1` cells → `results/terrain_maps/latest.npz` written.
Fallbacks: `LIBGL_ALWAYS_SOFTWARE=1`; switch the lidar `<render_engine>` / sim render
engine `ogre2`→`ogre`; RViz via `demo.launch.py ... use_rviz:=true`.

## Addendum (2026-09-09, during verification)

End-to-end sim run surfaced two pre-existing issues, fixed here:

1. **No terrain collision.** `lunar_terrain.world` gave the terrain a `<heightmap>`
   collision, which DART (Gazebo Harmonic's physics engine) silently ignores — the
   rover's wheels spun with no ground to grip. First tried a triangle-mesh collision
   from `heightmap.png` (`scripts/heightmap_to_collision_obj.py`), but DART+ODE
   trimesh collision **segfaults** on it (`OdeMesh::fillArrays`). Final fix: a large
   flat `<box>` collision (300×300×0.5, top at z 0.375 m = the visual terrain height
   along the flat spawn/goal corridor), μ 0.9. Terrain relief off the corridor is
   visual-only; the gpu_lidar still sees it and rock models keep their own collision.
   Rover spawn nudged `0.38 → 0.42` m. The generator script + regenerated
   `terrain_collision.obj` are kept for a future bullet-physics contour collision.
   The terrain **visual** was also flattened (`<heightmap>` → flat `<plane>` at
   z 0.375) so it matches the flat collision — otherwise the rover appears to fly
   where the visual surface dips below the box. Consequence: the 2D LiDAR now only
   returns off the rock models, not the ground; `/map` becomes free space + rock
   obstacles. Restoring contour terrain = switch physics engine to Bullet.
2. **Terrain classifier over-triggers** — Hough crater detection floods ~87% of the
   map as `crater_interior`, roughness/rocky never fires. NOT fixed here (needs
   iterative tuning of `config/mapping_params.yaml` against the sim); logged as a
   follow-up.

## Out of scope

- Phase 6+ (`planning_pkg`, `navigation_pkg`, `evaluation_pkg`, `quantum/`).
- Terrain classifier parameter tuning (issue 2 above).
- DiffDrive joint-set mismatch between Xacro (`mid_*`) and world (all six).
- `model.sdf` staleness (`left_wheel_joint` names) — unused; note only.
- `web/` viewer.

## Verification

- Here: `python -m pytest` (existing 6 + new odometry tests), `python -m compileall src`.
- On ROS2 box: the §6 checklist.
