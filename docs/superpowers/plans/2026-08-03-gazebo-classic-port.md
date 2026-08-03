# Gazebo Classic Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port `src/rover_simulation` from Gazebo Harmonic/ROS2 Jazzy to Gazebo Classic 11/ROS2 Humble so it runs on the existing "Ubuntu-ROS" VM (Ubuntu 22.04), and add real LiDAR/IMU sensor plugins so `sensor_pkg` can be tested against live sim data.

**Architecture:** Straight in-place replacement of Harmonic-specific SDF plugins, ROS bridge config, and launch wiring with their Gazebo Classic / `gazebo_ros_pkgs` equivalents, following the reference plugin syntax already vetted in `docs/PROJECT_GUIDE.md` (lines 931-992). No new abstractions — same terrain, same rover geometry, same topic names, different plugin backend.

**Tech Stack:** ROS2 Humble, Gazebo Classic 11, `gazebo_ros_pkgs` (`libgazebo_ros_diff_drive.so`, `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_imu_sensor.so`), SDF 1.6, Python 3 (launch files, `xacro`), bash.

## Global Constraints

- Target OS: Ubuntu 22.04 "Jammy" (the existing "Ubuntu-ROS" VM) — not 24.04.
- Target ROS2 distro: Humble. Target Gazebo: Classic 11 (not Harmonic/Garden).
- Topic names must stay: `/cmd_vel`, `/odom`, `/tf`, `/scan`, `/imu/data`, `/joint_states` — `sensor_pkg` and `rover_simulation`'s own nodes (`rover_patrol.py`, `rover_creep.py`, `odom_to_tf.py`) already depend on these exact names.
- The rover actually simulated in Gazebo is `models/lunar_rover/model.sdf` (spawned via `worlds/lunar_terrain.world`'s `<include>`) — not `rover.urdf.xacro`, which stays RViz/`robot_state_publisher`-only and is not touched by this plan.
- No dual Harmonic/Classic support — this replaces the Harmonic files in place (recoverable via git history).
- No local ROS2/Gazebo toolchain exists on the Windows dev machine — every task's local verification is limited to syntax/well-formedness checks (`python -c "...ElementTree..."` for XML/SDF, `python -m py_compile` for launch files, `bash -n` for shell scripts). Runtime behavior is verified only on the VM, in the final task.
- Python on this dev machine is invoked as `python`, not `python3` (confirmed: `python3` is not on PATH here; `python --version` → 3.10.0).

---

### Task 1: Update `package.xml` dependencies

**Files:**
- Modify: `src/rover_simulation/package.xml`

**Interfaces:**
- Produces: `gazebo_ros`, `gazebo_plugins` deps available to later tasks' plugins (`libgazebo_ros_diff_drive.so` etc. ship in these packages).

- [ ] **Step 1: Edit `package.xml`**

Change the `<description>` and the dependency block:

```xml
  <description>Lunar rover simulation package for ROS2 Humble + Gazebo Classic 11.</description>
```

Replace:
```xml
  <depend>ros_gz_sim</depend>
  <depend>ros_gz_bridge</depend>
  <depend>gazebo_ros_pkgs</depend>
```
with:
```xml
  <depend>gazebo_ros</depend>
  <depend>gazebo_plugins</depend>
  <depend>gazebo_ros_pkgs</depend>
```

- [ ] **Step 2: Verify the file is still well-formed XML**

Run: `python -c "import xml.etree.ElementTree as ET; ET.parse('src/rover_simulation/package.xml'); print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add src/rover_simulation/package.xml
git commit -m "rover_simulation: swap ros_gz deps for gazebo_ros/gazebo_plugins"
```

---

### Task 2: Port the world file's plugins and physics tag

**Files:**
- Modify: `src/rover_simulation/worlds/lunar_terrain.world`

**Interfaces:**
- Consumes: nothing new.
- Produces: a Classic-loadable world (still `<include>`s `model://lunar_rover`, unchanged).

- [ ] **Step 1: Edit the file header, `<sdf>` version, and physics block**

Replace lines 1-30 (everything from the opening comment through the five Harmonic system `<plugin>` tags) with:

```xml
<?xml version="1.0" ?>
<!--
  ╔══════════════════════════════════════════════════════════════════════════╗
  ║  LUNAR TERRAIN WORLD — Gazebo Classic 11 (SDF 1.6)                     ║
  ║  Terrain: 1025×1025 16-bit heightmap, 150×150 m, 25 m height range    ║
  ║  ONE continuous surface — no strips, no gaps, no floating geometry     ║
  ╚══════════════════════════════════════════════════════════════════════════╝
-->
<sdf version="1.6">
<world name="lunar_terrain">

  <!-- ═══════════════════════════════════════════════════════════════════════
       PHYSICS — Moon gravity, ODE engine (Gazebo Classic default)
  ═══════════════════════════════════════════════════════════════════════════ -->
  <physics name="default_physics" type="ode">
    <max_step_size>0.002</max_step_size>
    <real_time_update_rate>500</real_time_update_rate>
    <max_contacts>80</max_contacts>
  </physics>

  <gravity>0 0 -1.62</gravity>
```

(`real_time_update_rate` of 500 = `1 / max_step_size * real_time_factor` = `1/0.002 * 1.0`, preserving the original 1.0 real-time factor.)

Everything from `<!-- SCENE ... -->` onward (scene, lighting, heightmap terrain, rocks, trail decals, rover pad, the `lunar_rover` include, and the goal marker) is unchanged — do not touch it.

- [ ] **Step 2: Verify the file is still well-formed XML**

Run: `python -c "import xml.etree.ElementTree as ET; ET.parse('src/rover_simulation/worlds/lunar_terrain.world'); print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add src/rover_simulation/worlds/lunar_terrain.world
git commit -m "rover_simulation: port lunar_terrain.world to Gazebo Classic"
```

---

### Task 3: Port `model.sdf`'s diff-drive plugin to Classic

**Files:**
- Modify: `src/rover_simulation/models/lunar_rover/model.sdf`

**Interfaces:**
- Consumes: `left_wheel_joint`, `right_wheel_joint` (already defined earlier in this same file — unchanged).
- Produces: `/cmd_vel` (subscribed), `/odom` (published), `/tf` (published, `odom`→`base_link`) — same topic names the rest of the stack already expects.

- [ ] **Step 1: Change the SDF version**

Change line 4 from `<sdf version="1.9">` to `<sdf version="1.6">`, and update the header comment on line 2 from "Gazebo Harmonic diff-drive rover" to "Gazebo Classic 11 diff-drive rover".

- [ ] **Step 2: Replace the diff-drive plugin block**

Replace:
```xml
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.84</wheel_separation>
      <wheel_radius>0.18</wheel_radius>
      <topic>cmd_vel</topic>
      <odom_topic>odometry</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>20</odom_publish_frequency>
      <max_linear_velocity>0.6</max_linear_velocity>
      <min_linear_velocity>-0.6</min_linear_velocity>
      <max_angular_velocity>1.2</max_angular_velocity>
      <min_angular_velocity>-1.2</min_angular_velocity>
      <max_linear_acceleration>0.35</max_linear_acceleration>
      <max_angular_acceleration>0.8</max_angular_acceleration>
    </plugin>
```
with:
```xml
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace></namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.84</wheel_separation>
      <wheel_diameter>0.36</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <update_rate>20</update_rate>
    </plugin>
```

Note `odometry_topic` is `odom` (not `odometry`) — under Harmonic, `bridge.yaml` remapped the gz-side `odometry` topic to ROS `/odom`; Classic's plugin publishes ROS topics directly, so it must publish under the final name itself.

- [ ] **Step 3: Verify the file is still well-formed XML**

Run: `python -c "import xml.etree.ElementTree as ET; ET.parse('src/rover_simulation/models/lunar_rover/model.sdf'); print('OK')"`
Expected: `OK`

- [ ] **Step 4: Commit**

```bash
git add src/rover_simulation/models/lunar_rover/model.sdf
git commit -m "rover_simulation: port model.sdf diff-drive plugin to Gazebo Classic"
```

---

### Task 4: Add LiDAR + IMU sensor links to `model.sdf`

**Files:**
- Modify: `src/rover_simulation/models/lunar_rover/model.sdf`

**Interfaces:**
- Consumes: `base_link` (already defined earlier in this file — unchanged).
- Produces: `/scan` (`sensor_msgs/LaserScan`), `/imu/data` (`sensor_msgs/Imu`) — the exact topics `sensor_pkg`'s `lidar_processor.py` and `imu_processor.py` already subscribe to.

- [ ] **Step 1: Add `lidar_link` + `lidar_joint` and `imu_link` + `imu_joint`**

Insert the following new links/joints immediately after the `rear_caster_joint` block (i.e., right before the `<plugin name="diff_drive" ...>` block from Task 3):

```xml
    <link name="lidar_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.2</mass>
        <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
      </inertial>
      <visual name="vis">
        <geometry><cylinder><radius>0.04</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <sensor name="lidar" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>15.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <remapping>~/out:=/scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>lidar_link</frame_name>
        </plugin>
      </sensor>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
      <pose>0.30 0 0.15 0 0 0</pose>
    </joint>

    <link name="imu_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
      </inertial>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>50</update_rate>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <remapping>~/out:=/imu/data</remapping>
          </ros>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
        </plugin>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.017</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.017</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.017</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
```

This follows the same placement convention already used by `front_caster`/`rear_caster` in this file: the link's own `<pose>` is identity, and the actual mount offset lives on the fixed joint's `<pose>` (relative to `base_link`). `lidar_link` sits 0.30 m forward and 0.15 m above the chassis center (roughly on top of the body); `imu_link` is co-located with the chassis center, which is standard IMU placement.

- [ ] **Step 2: Verify the file is still well-formed XML**

Run: `python -c "import xml.etree.ElementTree as ET; ET.parse('src/rover_simulation/models/lunar_rover/model.sdf'); print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add src/rover_simulation/models/lunar_rover/model.sdf
git commit -m "rover_simulation: add LiDAR + IMU sensor plugins to model.sdf"
```

---

### Task 5: Delete `bridge.yaml` and rewrite `simulation_launch.py`

**Files:**
- Delete: `src/rover_simulation/config/bridge.yaml`
- Modify: `src/rover_simulation/launch/simulation_launch.py`

**Interfaces:**
- Consumes: `rover.urdf.xacro` (for `robot_state_publisher`/RViz only — unchanged), `lunar_terrain.world` (from Task 2).
- Produces: same launch args as before (`headless`, `use_rviz`, `use_sim_time`) — `demo.launch.py` (which includes this file) needs no changes.

- [ ] **Step 1: Delete `bridge.yaml`**

```bash
git rm src/rover_simulation/config/bridge.yaml
```

- [ ] **Step 2: Replace `simulation_launch.py` in full**

```python
#!/usr/bin/env python3
"""
Gazebo Classic 11 + ROS 2 Humble — lunar rover simulation.

- gazebo_ros's gazebo.launch.py starts gzserver and (optionally) gzclient.
- Default headless=true: no gzclient window (recommended on VMs without
  reliable 3D acceleration — use RViz instead).
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('rover_simulation').find('rover_simulation')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'lunar_terrain.world')
    rviz_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    models_dir = os.path.join(pkg_share, 'models')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='true = no Gazebo GUI (recommended on VMs without 3D accel). false = attach gzclient.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 (optional)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock',
    )

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_dir)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('gazebo_ros').find('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': world_file,
            'gui': PythonExpression(
                ['"true" if "', headless, '" == "false" else "false"']
            ),
            'verbose': 'true',
        }.items(),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    headless_notice = LogInfo(
        msg=(
            '\n'
            '=== Gazebo HEADLESS (no window) — sim is running in the background ===\n'
            '  See the rover:  bash ~/lunar-rover-quantum/scripts/open_gazebo_gui.sh\n'
            '  Or use RViz:    ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true\n'
            '  Or check odom:  ros2 topic echo /odom --field pose.pose.position\n'
            '====================================================================\n'
        ),
        condition=IfCondition(PythonExpression(["'", headless, "' == 'true'"])),
    )

    return LaunchDescription([
        headless_arg,
        use_rviz_arg,
        use_sim_time_arg,
        headless_notice,
        set_model_path,
        robot_state_publisher,
        gazebo,
        rviz2,
    ])
```

Note the rover itself is spawned by `lunar_terrain.world`'s own `<include>` (baked into the world file, from Task 2) — Classic supports `<include><uri>model://...</uri></include>` in world files the same way Harmonic did, so no separate `spawn_entity.py` node is needed here, matching the original design.

- [ ] **Step 3: Verify the launch file is syntactically valid Python**

Run: `python -m py_compile src/rover_simulation/launch/simulation_launch.py`
Expected: no output, exit code 0. Clean up the generated `.pyc`: `rm -rf src/rover_simulation/launch/__pycache__`

- [ ] **Step 4: Commit**

```bash
git add src/rover_simulation/launch/simulation_launch.py
git commit -m "rover_simulation: rewrite simulation_launch.py for Gazebo Classic, drop ros_gz_bridge"
```

---

### Task 6: Port the ROS2/Gazebo install scripts to Humble/Classic

**Files:**
- Modify: `scripts/setup_ros2_repo.sh`
- Rename + modify: `scripts/install_ros2_jazzy.sh` → `scripts/install_ros2_humble.sh`
- Rename + modify: `scripts/install_gazebo_harmonic.sh` → `scripts/install_gazebo_classic.sh`
- Modify: `scripts/install_sim_dependencies.sh`

**Interfaces:**
- Consumes: nothing (standalone apt scripts).
- Produces: nothing consumed by other tasks — these only need to be internally correct and runnable on the VM.

- [ ] **Step 1: Fix the ROS2 apt repo codename in `setup_ros2_repo.sh`**

Change:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
to:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

- [ ] **Step 2: Rename and rewrite `install_ros2_jazzy.sh` → `install_ros2_humble.sh`**

```bash
git mv scripts/install_ros2_jazzy.sh scripts/install_ros2_humble.sh
```

Replace its contents in full:

```bash
#!/bin/bash
set -e

echo "=== STEP 1: INSTALLING ROS2 HUMBLE DESKTOP ==="
# Note: This is a large package, apt-get install may take several minutes.
sudo apt-get update
sudo apt-get install -y ros-humble-desktop

echo "=== STEP 2: INSTALLING COLCON BUILD TOOLS AND ROSDEP ==="
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep

echo "=== STEP 3: INITIALIZING ROSDEP ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
else
    echo "rosdep already initialized, skipping init."
fi

echo "=== STEP 4: UPDATING ROSDEP ==="
# Run rosdep update as the current user, not sudo/root
rosdep update

echo "=== STEP 5: CONFIGURING BASHRC SOURCE ==="
BASHRC="$HOME/.bashrc"
SOURCE_LINE="source /opt/ros/humble/setup.bash"
if ! grep -Fxq "$SOURCE_LINE" "$BASHRC"; then
    echo "$SOURCE_LINE" >> "$BASHRC"
    echo "Added ROS2 sourcing to $BASHRC"
else
    echo "ROS2 sourcing already in $BASHRC"
fi

echo "=== ROS2 INSTALLATION COMPLETE ==="
```

(This also fixes a pre-existing bug: the old script hardcoded `/home/monis/.bashrc`, which silently did nothing on any VM where the user isn't literally named "monis" — this VM's user is `vboxuser`.)

- [ ] **Step 3: Rename and rewrite `install_gazebo_harmonic.sh` → `install_gazebo_classic.sh`**

```bash
git mv scripts/install_gazebo_harmonic.sh scripts/install_gazebo_classic.sh
```

Replace its contents in full:

```bash
#!/bin/bash
set -e

echo "=== STEP 1: INSTALLING GAZEBO CLASSIC 11 + ROS INTEGRATION ==="
# Gazebo 11 Classic ships in ROS Humble's own apt repo — no separate
# OSRF repository is needed (that's only required for gz-harmonic/gz-garden).
sudo apt-get update
sudo apt-get install -y ros-humble-gazebo-ros-pkgs

echo "=== STEP 2: VERIFYING GAZEBO INSTALLATION ==="
gazebo --version

echo "=== GAZEBO CLASSIC 11 INSTALLATION COMPLETE ==="
```

- [ ] **Step 4: Update package names in `install_sim_dependencies.sh`**

Replace the `PACKAGES` array:
```bash
PACKAGES=(
    "ros-jazzy-xacro"
    "ros-jazzy-joint-state-publisher"
    "ros-jazzy-joint-state-publisher-gui"
    "ros-jazzy-robot-state-publisher"
    "ros-jazzy-rviz2"
    "ros-jazzy-teleop-twist-keyboard"
    "ros-jazzy-navigation2"
    "ros-jazzy-nav2-bringup"
    "ros-jazzy-tf-transformations"
    "ros-jazzy-tf2-tools"
    "ros-jazzy-urdf"
)
```
with:
```bash
PACKAGES=(
    "ros-humble-xacro"
    "ros-humble-joint-state-publisher"
    "ros-humble-joint-state-publisher-gui"
    "ros-humble-robot-state-publisher"
    "ros-humble-rviz2"
    "ros-humble-teleop-twist-keyboard"
    "ros-humble-navigation2"
    "ros-humble-nav2-bringup"
    "ros-humble-tf-transformations"
    "ros-humble-tf2-tools"
    "ros-humble-urdf"
)
```

- [ ] **Step 5: Verify all four scripts are syntactically valid bash**

Run: `bash -n scripts/setup_ros2_repo.sh && bash -n scripts/install_ros2_humble.sh && bash -n scripts/install_gazebo_classic.sh && bash -n scripts/install_sim_dependencies.sh && echo OK`
Expected: `OK`

- [ ] **Step 6: Commit**

```bash
git add scripts/setup_ros2_repo.sh scripts/install_ros2_humble.sh scripts/install_gazebo_classic.sh scripts/install_sim_dependencies.sh
git status --short scripts/install_ros2_jazzy.sh scripts/install_gazebo_harmonic.sh
git commit -m "scripts: port install scripts from Jazzy/Harmonic to Humble/Gazebo Classic"
```

---

### Task 7: Port the runtime/test scripts to Humble/Classic

**Files:**
- Modify: `scripts/open_gazebo_gui.sh`
- Modify: `scripts/test_physics.sh`
- Modify: `scripts/drive_test.sh`

**Interfaces:**
- Consumes: the `rover_simulation` package built by Tasks 1-5 (installed under `~/lunar-rover-quantum/install/`).

- [ ] **Step 1: Rewrite `open_gazebo_gui.sh`**

```bash
#!/usr/bin/env bash
# Attach Gazebo GUI to an already-running headless server.
# Start the sim first:  ros2 launch rover_simulation demo.launch.py mode:=creep

set -e
source /opt/ros/humble/setup.bash
source ~/lunar-rover-quantum/install/setup.bash

echo "Opening Gazebo GUI (connects to running server)..."
echo "If this crashes on a VM without 3D acceleration, use RViz instead:"
echo "  ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true"
exec gzclient
```

- [ ] **Step 2: Rewrite `test_physics.sh`**

This script predates the repo's current layout (it referenced a separate `lunar_rover_ws/` subfolder that no longer exists — the repo root is the workspace) and manually re-created a `ros_gz_bridge` pipeline that no longer applies under Classic. Rewrite it to test the rover that's actually spawned (`model.sdf`, via the world's own `<include>`), matching the current workspace layout:

```bash
#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/lunar-rover-quantum/install/setup.bash"

WORLD="$HOME/lunar-rover-quantum/install/rover_simulation/share/rover_simulation/worlds/lunar_terrain.world"
export GAZEBO_MODEL_PATH="$HOME/lunar-rover-quantum/install/rover_simulation/share/rover_simulation/models"

echo "1. Starting Gazebo server (headless, rover spawns via the world's own <include>) ..."
gzserver --verbose -s libgazebo_ros_factory.so -s libgazebo_ros_init.so "$WORLD" &
GZ_PID=$!
sleep 5

echo "2. Checking active topics ..."
ros2 topic list

echo "3. Capturing Odometry ..."
ros2 topic echo --once /odom

echo "4. Capturing TF ..."
timeout 2 ros2 topic echo --once /tf

echo "5. Capturing LiDAR scan ..."
ros2 topic echo --once /scan

echo "6. Capturing IMU data ..."
ros2 topic echo --once /imu/data

echo "7. Cleaning up ..."
kill $GZ_PID
echo "Done!"
```

- [ ] **Step 3: Fix `drive_test.sh`**

Change:
```bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
```
to:
```bash
source /opt/ros/humble/setup.bash 2>/dev/null || true
```
Nothing else in this file changes.

- [ ] **Step 4: Verify all three scripts are syntactically valid bash**

Run: `bash -n scripts/open_gazebo_gui.sh && bash -n scripts/test_physics.sh && bash -n scripts/drive_test.sh && echo OK`
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add scripts/open_gazebo_gui.sh scripts/test_physics.sh scripts/drive_test.sh
git commit -m "scripts: port runtime/test scripts to Gazebo Classic, fix stale workspace paths"
```

---

### Task 8: Update docs (`README.md`, `CLAUDE.md`)

**Files:**
- Modify: `README.md`
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update `README.md`'s Stack line**

Change:
```
Ubuntu, ROS2 (Jazzy), Gazebo Harmonic for simulation, Python, Qiskit +
```
to:
```
Ubuntu, ROS2 (Humble), Gazebo Classic 11 for simulation, Python, Qiskit +
```

- [ ] **Step 2: Update `CLAUDE.md`'s "Environment / VM notes" section**

Replace:
```
- There's an existing VirtualBox VM named **"Ubuntu-ROS"** — but it's actually **Ubuntu 22.04 ("Jammy")**, previously set up for ROS2 Humble + Gazebo Classic 11. This is incompatible with Jazzy/Harmonic (confirmed: apt fails with a wall of "not installable" dependency errors — Jazzy needs `libstdc++6 >= 13.1`, `libc6 >= 2.38`, etc. that don't exist in 22.04's repos).
- Decision on how to fix this (fresh Ubuntu 24.04 VM vs. `do-release-upgrade` the existing one) was **deferred** — check with the user before assuming either path.
```
with:
```
- There's an existing VirtualBox VM named **"Ubuntu-ROS"** — it's **Ubuntu 22.04 ("Jammy")**, previously set up for ROS2 Humble + Gazebo Classic 11. This was incompatible with Jazzy/Harmonic (confirmed: apt fails with a wall of "not installable" dependency errors — Jazzy needs `libstdc++6 >= 13.1`, `libc6 >= 2.38`, etc. that don't exist in 22.04's repos).
- **Decision (2026-08-03): ported `rover_simulation` backward to target ROS2 Humble + Gazebo Classic 11** instead of building a new 24.04 VM or upgrading this one — see `docs/superpowers/specs/2026-08-03-gazebo-classic-port-design.md` for the full design. The project's documented "Stack" (README) now reflects Humble/Classic, not Jazzy/Harmonic.
```

- [ ] **Step 3: Commit**

```bash
git add README.md CLAUDE.md
git commit -m "docs: update Stack/environment notes for Gazebo Classic port"
```

---

### Task 9: Manual VM validation (user-run)

**Files:** none — this is a verification pass on the VM, not a code change.

This task can't be run by an agent — it requires the VirtualBox VM. Run these on the "Ubuntu-ROS" VM after pulling this branch and running the install scripts from Task 6 (`setup_ros2_repo.sh`, `install_ros2_humble.sh`, `install_gazebo_classic.sh`, `install_sim_dependencies.sh`, in that order) if Humble/Gazebo Classic aren't already installed:

- [ ] **Step 1: Build the workspace**

```bash
cd ~/lunar-rover-quantum   # git clone this repo here first if not already present
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
Expected: build succeeds, no errors for `rover_simulation`.

- [ ] **Step 2: Launch the sim and check for plugin-load errors**

```bash
ros2 launch rover_simulation demo.launch.py mode:=creep
```
Expected: Gazebo starts headless; no "plugin not found" or "failed to load" errors in the log for `libgazebo_ros_diff_drive.so`, `libgazebo_ros_ray_sensor.so`, or `libgazebo_ros_imu_sensor.so`.

- [ ] **Step 3: Check active topics (in a second terminal)**

```bash
source ~/lunar-rover-quantum/install/setup.bash
ros2 topic list
```
Expected: list includes `/odom`, `/tf`, `/scan`, `/imu/data`, `/cmd_vel`, `/joint_states`.

- [ ] **Step 4: Confirm the rover is moving**

```bash
ros2 topic echo /odom --field pose.pose.position
```
Expected: `x`/`y` values change over time (rover_creep.py is driving it, per `mode:=creep`).

- [ ] **Step 5: Confirm the new sensor plugins are producing real data**

```bash
ros2 topic echo /scan --once
ros2 topic echo /imu/data --once
```
Expected: both return populated messages (non-empty `ranges` array for `/scan`; non-zero `linear_acceleration.z` around 1.62 m/s² lunar gravity for `/imu/data`) — this is new coverage, neither topic has ever been live before, even under Harmonic.

- [ ] **Step 6: Run `sensor_pkg` against the live topics**

```bash
ros2 run sensor_pkg lidar_processor
ros2 run sensor_pkg imu_processor
ros2 run sensor_pkg encoder_processor
```
Expected: each starts cleanly and logs processed output (no exceptions) while the sim is running.

No commit for this task — it's a verification pass. If any step fails, that's a bug in one of Tasks 1-8 to fix and re-commit, not something to work around here.
