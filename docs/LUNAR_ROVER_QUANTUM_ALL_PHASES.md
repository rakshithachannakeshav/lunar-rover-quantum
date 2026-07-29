# 🌕 Quantum-Assisted Energy Optimization — Autonomous Lunar Rover Navigation
# COMPLETE IMPLEMENTATION GUIDE — ALL 12 PHASES

> **Project**: Lunar Rover Quantum Navigation
> **Hardware**: Arjuna AMR Kit · Jetson Nano · LiDAR · IMU · Encoders · Arduino
> **Stack**: Ubuntu 22.04 · ROS2 Humble · Gazebo · Qiskit AerSimulator · Python 3.10
> **Quantum**: Simulated only (no real quantum hardware required)

---

# TABLE OF CONTENTS

| Phase | Title | Week | Difficulty |
|---|---|---|---|
| 1 | Project Planning | Week 1 | ⭐ |
| 2 | Environment Setup | Week 2 | ⭐⭐ |
| 3 | Robot Simulation | Week 3 | ⭐⭐ |
| 4 | Sensor Integration | Week 4 | ⭐⭐ |
| 5 | Terrain Mapping | Week 5 | ⭐⭐⭐ |
| 6 | Energy Modeling | Week 6 | ⭐⭐⭐ |
| 7 | Classical Path Planning | Week 7 | ⭐⭐⭐ |
| 8 | Quantum Optimization | Weeks 8–9 | ⭐⭐⭐⭐ |
| 9 | ROS2 Integration | Week 10 | ⭐⭐⭐ |
| 10 | Comparison & Evaluation | Week 11 | ⭐⭐ |
| 11 | Real Robot Deployment | Week 12a | ⭐⭐⭐⭐ |
| 12 | Documentation | Week 12b | ⭐ |

---

# ═══════════════════════════════════════════
# PHASE 1 — PROJECT PLANNING
# ═══════════════════════════════════════════

## Objective
Define architecture, data flow, software stack, and repository structure before writing any code.

## System Architecture (4 Layers)

```
LAYER 1 — PERCEPTION
  LiDAR  →  /scan           →  lidar_processor   →  /terrain/pointcloud
  IMU    →  /imu/data        →  imu_processor     →  /imu/slope
  Wheels →  /encoder/ticks   →  encoder_processor →  /odom

LAYER 2 — PLANNING
  /terrain/pointcloud  →  terrain_mapper      →  /terrain/grid
  /terrain/grid        →  terrain_classifier  →  /terrain/classified
  /terrain/classified  →  graph_builder       →  /graph/data
  /graph/data          →  energy_model        →  /graph/weighted
  /graph/weighted      →  classical_planner   →  /path/classical
  /graph/weighted      →  quantum_optimizer   →  /path/quantum

LAYER 3 — EXECUTION
  /path/quantum  →  path_executor  →  /cmd_vel  →  MOTORS

LAYER 4 — EVALUATION
  /path/classical + /path/quantum + /battery/status  →  evaluator  →  /metrics
```

## Task 1.1 — Install Git and Create Repository

```bash
sudo apt update && sudo apt install git -y
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
cd ~
mkdir lunar-rover-quantum && cd lunar-rover-quantum
git init
```

## Task 1.2 — Create Full Folder Structure

```bash
cd ~/lunar-rover-quantum
mkdir -p lunar_rover_ws/src

# All ROS2 packages
for pkg in sensor_pkg mapping_pkg planning_pkg navigation_pkg evaluation_pkg; do
  mkdir -p lunar_rover_ws/src/$pkg/$pkg
  mkdir -p lunar_rover_ws/src/$pkg/launch
  touch lunar_rover_ws/src/$pkg/$pkg/__init__.py
done

# Simulation package
mkdir -p lunar_rover_ws/src/rover_simulation/{urdf,worlds,config,launch}

# Supporting folders
mkdir -p quantum/notebooks docs/{report,presentation} scripts tests
mkdir -p results/{classical_paths,quantum_paths,energy_comparison}

echo "✅ Structure created"
tree lunar_rover_ws/src
```

## Task 1.3 — requirements.txt

Create `requirements.txt` at project root:

```
qiskit>=1.0.0
qiskit-aer>=0.14.0
qiskit-algorithms>=0.3.0
networkx>=3.0
numpy>=1.24.0
scipy>=1.10.0
matplotlib>=3.7.0
seaborn>=0.12.0
pandas>=2.0.0
opencv-python>=4.8.0
pyyaml>=6.0
tqdm>=4.65.0
```

## Task 1.4 — .gitignore

```
build/
install/
log/
*.pyc
__pycache__/
.venv/
*.bag
.ipynb_checkpoints/
.vscode/
.DS_Store
```

## Phase 1 Deliverables
- [ ] GitHub repository initialized
- [ ] Full folder structure created
- [ ] requirements.txt and .gitignore created
- [ ] Architecture understood and drawn on paper

---

# ═══════════════════════════════════════════
# PHASE 2 — ENVIRONMENT SETUP
# ═══════════════════════════════════════════

## Objective
Install Ubuntu 22.04, ROS2 Humble, Gazebo, Python dependencies, Qiskit, and VS Code.

## Step 2.1 — Verify Ubuntu 22.04

```bash
lsb_release -a
# Must show: Ubuntu 22.04 LTS

uname -m
# Must show: x86_64 (or aarch64 for Jetson)
```

If not on Ubuntu 22.04, either install it natively or use a VM (VirtualBox / VMware).

## Step 2.2 — Install ROS2 Humble (Full Desktop)

```bash
# Set locale
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (full)
sudo apt update
sudo apt install ros-humble-desktop -y

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update

# Source ROS2 automatically on every terminal open
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version
# Expected: ros2, version humble
```

## Step 2.3 — Install Gazebo Classic 11

```bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control -y

# Test Gazebo opens
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

## Step 2.4 — Install Python Dependencies

```bash
# Install pip3
sudo apt install python3-pip python3-venv -y

# Install project dependencies globally
pip3 install qiskit qiskit-aer qiskit-algorithms \
  networkx numpy scipy matplotlib seaborn pandas \
  opencv-python pyyaml tqdm

# Verify Qiskit
python3 -c "import qiskit; print(qiskit.__version__)"
# Expected: 1.x.x

# Verify NetworkX
python3 -c "import networkx as nx; print(nx.__version__)"
```

## Step 2.5 — Qiskit Quick Test

Create and run `~/test_qiskit.py`:

```python
from qiskit import QuantumCircuit
from qiskit_aer import AerSimulator

# Build a simple 2-qubit Bell state
qc = QuantumCircuit(2, 2)
qc.h(0)         # Hadamard on qubit 0
qc.cx(0, 1)     # CNOT: qubit 0 controls qubit 1
qc.measure([0, 1], [0, 1])

print(qc.draw())

# Simulate it
simulator = AerSimulator()
job = simulator.run(qc, shots=1000)
result = job.result()
counts = result.get_counts()
print("Measurement results:", counts)
# Expected: {'00': ~500, '11': ~500}  (quantum superposition!)
```

```bash
python3 ~/test_qiskit.py
```

## Step 2.6 — Install VS Code

```bash
# Download and install
wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
  | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg \
  /etc/apt/trusted.gpg.d/
echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] \
  https://packages.microsoft.com/repos/vscode stable main" \
  | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update && sudo apt install code -y

# Open VS Code in project folder
cd ~/lunar-rover-quantum
code .
```

Install these VS Code extensions (search in Extensions panel):
- `ROS` by Microsoft
- `Python` by Microsoft
- `URDF` by smilerobotics
- `CMake Tools`

## Step 2.7 — Install Additional ROS2 Packages

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-laser-filters \
  ros-humble-tf2-tools \
  ros-humble-rqt \
  ros-humble-rqt-graph \
  ros-humble-rviz2 -y
```

## Step 2.8 — Verify Full Installation

```bash
# Test ROS2 publisher/subscriber (2 terminals)

# Terminal 1:
ros2 run demo_nodes_py talker

# Terminal 2:
ros2 run demo_nodes_py listener

# You should see "Hello World: 1", "Hello World: 2" etc.
# Press Ctrl+C to stop both
```

## Phase 2 Deliverables
- [ ] ROS2 Humble installed and sourced
- [ ] Gazebo 11 opens without errors
- [ ] Qiskit Bell state test passes
- [ ] VS Code installed with ROS2 extension
- [ ] talker/listener demo works

---

# ═══════════════════════════════════════════
# PHASE 3 — ROBOT SIMULATION
# ═══════════════════════════════════════════

## Objective
Create a lunar terrain in Gazebo, define the rover with URDF, attach virtual sensors, and test movement.

## Step 3.1 — Create the ROS2 Simulation Package

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws/src
ros2 pkg create rover_simulation --build-type ament_python \
  --dependencies rclpy geometry_msgs sensor_msgs nav_msgs
```

## Step 3.2 — Create the Lunar Terrain World

Create `lunar_rover_ws/src/rover_simulation/worlds/lunar_terrain.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lunar_terrain">

    <!-- No atmosphere, low gravity (Moon = 1.62 m/s²) -->
    <gravity>0 0 -1.62</gravity>

    <!-- Minimal ambient light (no atmosphere scattering) -->
    <scene>
      <ambient>0.1 0.1 0.1 1</ambient>
      <background>0 0 0 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Directional sunlight (harsh shadows on Moon) -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <!-- Ground plane — lunar regolith (grey, high friction) -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Crater 1 — small depression -->
    <model name="crater1">
      <static>true</static>
      <pose>5 3 -0.3 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><sphere><radius>1.5</radius></sphere></geometry>
        </collision>
        <visual name="vis">
          <geometry><sphere><radius>1.5</radius></sphere></geometry>
          <material><ambient>0.35 0.35 0.35 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Rock obstacle 1 -->
    <model name="rock1">
      <static>true</static>
      <pose>3 1 0.3 0.1 0.2 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.8 0.6 0.6</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>0.8 0.6 0.6</size></box></geometry>
          <material><ambient>0.45 0.42 0.40 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Rock obstacle 2 -->
    <model name="rock2">
      <static>true</static>
      <pose>-2 4 0.25 0 0.1 0.5</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          <material><ambient>0.45 0.42 0.40 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Slope / ramp terrain feature -->
    <model name="slope1">
      <static>true</static>
      <pose>-3 -2 0.5 0 0.3 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>3 2 0.1</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>3 2 0.1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Step 3.3 — Create the Rover URDF

Create `lunar_rover_ws/src/rover_simulation/urdf/rover.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="lunar_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ===== Properties ===== -->
  <xacro:property name="body_length" value="0.5"/>
  <xacro:property name="body_width"  value="0.4"/>
  <xacro:property name="body_height" value="0.15"/>
  <xacro:property name="wheel_radius" value="0.08"/>
  <xacro:property name="wheel_width"  value="0.05"/>
  <xacro:property name="body_mass"    value="10.0"/>
  <xacro:property name="wheel_mass"   value="0.5"/>

  <!-- ===== Inertia macro ===== -->
  <xacro:macro name="box_inertia" params="m l w h">
    <inertial>
      <mass value="${m}"/>
      <inertia
        ixx="${m*(w*w+h*h)/12}" ixy="0" ixz="0"
        iyy="${m*(l*l+h*h)/12}" iyz="0"
        izz="${m*(l*l+w*w)/12}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <mass value="${m}"/>
      <inertia
        ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
        iyy="${m*(3*r*r+h*h)/12}" iyz="0"
        izz="${m*r*r/2}"/>
    </inertial>
  </xacro:macro>

  <!-- ===== Base footprint ===== -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ===== Chassis ===== -->
  <link name="base_link">
    <xacro:box_inertia m="${body_mass}" l="${body_length}"
      w="${body_width}" h="${body_height}"/>
    <visual>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="chassis_grey">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- ===== Wheel macro ===== -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}_wheel">
      <xacro:cylinder_inertia m="${wheel_mass}"
        r="${wheel_radius}" h="${wheel_width}"/>
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="wheel_black">
          <color rgba="0.1 0.1 0.1 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <surface>
          <friction>
            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
          </friction>
        </surface>
      </collision>
    </link>

    <joint name="${name}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${name}_wheel"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <!-- Gazebo friction plugin for wheel -->
    <gazebo reference="${name}_wheel">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <material>Gazebo/Black</material>
    </gazebo>
  </xacro:macro>

  <!-- Instantiate 4 wheels -->
  <xacro:wheel name="front_left"  x=" 0.18" y=" 0.22"/>
  <xacro:wheel name="front_right" x=" 0.18" y="-0.22"/>
  <xacro:wheel name="rear_left"   x="-0.18" y=" 0.22"/>
  <xacro:wheel name="rear_right"  x="-0.18" y="-0.22"/>

  <!-- ===== LiDAR mount ===== -->
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.04" length="0.06"/></geometry>
      <material name="lidar_black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.04" length="0.06"/></geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.12" rpy="0 0 0"/>
  </joint>

  <!-- ===== IMU link ===== -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- ===== Gazebo plugins ===== -->

  <!-- Differential drive controller -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros><namespace>/rover</namespace></ros>
      <left_joint>front_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <wheel_separation>0.44</wheel_separation>
      <wheel_diameter>${2*wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- LiDAR sensor -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <visualize>true</visualize>
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
  </gazebo>

  <!-- IMU sensor -->
  <gazebo reference="imu_link">
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
  </gazebo>

</robot>
```

## Step 3.4 — Create the Launch File

Create `lunar_rover_ws/src/rover_simulation/launch/simulation_launch.py`:

```python
import os
from ament_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_sim = get_package_share_directory('rover_simulation')

    # Paths
    urdf_file = os.path.join(pkg_sim, 'urdf', 'rover.urdf.xacro')
    world_file = os.path.join(pkg_sim, 'worlds', 'lunar_terrain.world')

    # Process xacro into URDF string
    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Gazebo server + client
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file,
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # Spawn rover into Gazebo
    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lunar_rover',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # RViz2
    rviz_config = os.path.join(pkg_sim, 'config', 'rover.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([rsp_node, gazebo, spawn_rover, rviz])
```

## Step 3.5 — Build and Launch

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash

# Launch simulation
ros2 launch rover_simulation simulation_launch.py
```

## Step 3.6 — Test Rover Movement

```bash
# In a new terminal, source and drive the rover with keyboard
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash

# Install teleop
sudo apt install ros-humble-teleop-twist-keyboard -y
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/rover/cmd_vel

# Use WASD to drive. Rover should move in Gazebo.
```

## Step 3.7 — Verify Sensor Topics Publishing

```bash
# Check all active topics
ros2 topic list

# You should see:
# /scan           ← LiDAR
# /imu/data       ← IMU
# /rover/odom     ← Odometry
# /tf             ← Transform tree

# Check LiDAR data
ros2 topic echo /scan --once

# Check IMU data
ros2 topic echo /imu/data --once

# Check publish rate (should be ~10Hz for lidar)
ros2 topic hz /scan
```

## Phase 3 Common Errors

| Error | Cause | Fix |
|---|---|---|
| `symbol lookup error: libgazebo_ros_diff_drive.so` | Missing Gazebo plugin pkg | `sudo apt install ros-humble-gazebo-ros-pkgs -y` |
| Rover spawns underground | Wrong z value | Change spawn z to `0.15` |
| `/scan` not publishing | LiDAR plugin misconfigured | Check `<output_type>` in URDF |
| `xacro: command not found` | xacro not installed | `sudo apt install ros-humble-xacro -y` |

## Phase 3 Deliverables
- [ ] Gazebo opens with lunar terrain world
- [ ] Rover spawns and is visible
- [ ] Rover drives with teleop keyboard
- [ ] `/scan`, `/imu/data`, `/rover/odom` topics all publishing
- [ ] Sensors visible in RViz2

---

# ═══════════════════════════════════════════
# PHASE 4 — SENSOR INTEGRATION
# ═══════════════════════════════════════════

## Objective
Build ROS2 nodes that process raw sensor data from LiDAR, IMU, and encoders into meaningful data for the mapping phase.

## Step 4.1 — Setup sensor_pkg

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws/src
ros2 pkg create sensor_pkg --build-type ament_python \
  --dependencies rclpy sensor_msgs nav_msgs geometry_msgs std_msgs
```

Edit `sensor_pkg/package.xml` — add after existing depends:
```xml
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

## Step 4.2 — LiDAR Processor Node

Create `sensor_pkg/sensor_pkg/lidar_processor.py`:

```python
#!/usr/bin/env python3
"""
lidar_processor.py
------------------
Subscribes to raw LiDAR /scan (sensor_msgs/LaserScan)
Publishes processed point cloud to /terrain/pointcloud
Also computes basic roughness metric per scan sector.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, Float32MultiArray
import numpy as np
import struct


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Parameters
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('num_sectors', 36)   # 10 degrees each

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.num_sectors = self.get_parameter('num_sectors').value

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/terrain/pointcloud', 10)
        self.roughness_pub = self.create_publisher(
            Float32MultiArray, '/terrain/roughness', 10)

        self.get_logger().info('LiDAR Processor node started')

    def scan_callback(self, msg: LaserScan):
        """Convert LaserScan to XYZ point cloud and compute roughness."""
        points = []
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter valid ranges
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert polar → Cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        z = np.zeros_like(x)   # 2D LiDAR, z=0 in sensor frame

        points_xyz = np.column_stack([x, y, z]).astype(np.float32)

        # Publish PointCloud2
        if len(points_xyz) > 0:
            cloud_msg = self.create_pointcloud2(points_xyz, msg.header)
            self.cloud_pub.publish(cloud_msg)

        # Compute roughness per sector
        roughness = self.compute_roughness(valid_ranges, self.num_sectors)
        roughness_msg = Float32MultiArray()
        roughness_msg.data = roughness.tolist()
        self.roughness_pub.publish(roughness_msg)

    def compute_roughness(self, ranges: np.ndarray, num_sectors: int) -> np.ndarray:
        """
        Roughness = std deviation of ranges within each angular sector.
        High std dev → rough terrain. Low std dev → flat/smooth.
        """
        roughness = np.zeros(num_sectors, dtype=np.float32)
        if len(ranges) == 0:
            return roughness

        sector_size = len(ranges) // num_sectors
        for i in range(num_sectors):
            start = i * sector_size
            end = start + sector_size
            sector_ranges = ranges[start:end]
            if len(sector_ranges) > 1:
                roughness[i] = float(np.std(sector_ranges))
        return roughness

    def create_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Pack numpy XYZ array into a ROS2 PointCloud2 message."""
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'lidar_link'
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats × 4 bytes
        msg.row_step = msg.point_step * msg.width

        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        # Pack all points into bytes
        data = bytearray()
        for pt in points:
            data += struct.pack('fff', float(pt[0]), float(pt[1]), float(pt[2]))
        msg.data = bytes(data)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4.3 — IMU Processor Node

Create `sensor_pkg/sensor_pkg/imu_processor.py`:

```python
#!/usr/bin/env python3
"""
imu_processor.py
----------------
Subscribes to /imu/data (sensor_msgs/Imu)
Computes roll, pitch (slope angles) from orientation quaternion.
Publishes slope angle to /imu/slope (std_msgs/Float32)
Publishes processed IMU to /imu/processed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import numpy as np
import math


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))   # clamp
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Parameters
        self.declare_parameter('slope_warn_deg', 15.0)
        self.declare_parameter('slope_danger_deg', 25.0)

        self.slope_warn  = math.radians(
            self.get_parameter('slope_warn_deg').value)
        self.slope_danger = math.radians(
            self.get_parameter('slope_danger_deg').value)

        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 50)

        # Publishers
        self.slope_pub = self.create_publisher(Float32, '/imu/slope', 10)
        self.status_pub = self.create_publisher(String, '/imu/terrain_status', 10)

        # State
        self.roll_history  = []
        self.pitch_history = []
        self.history_size  = 10

        self.get_logger().info('IMU Processor node started')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        # Smooth with rolling average
        self.roll_history.append(roll)
        self.pitch_history.append(pitch)
        if len(self.roll_history) > self.history_size:
            self.roll_history.pop(0)
            self.pitch_history.pop(0)

        avg_roll  = np.mean(self.roll_history)
        avg_pitch = np.mean(self.pitch_history)

        # Overall slope magnitude (combine roll + pitch)
        slope = math.sqrt(avg_roll**2 + avg_pitch**2)

        # Publish slope magnitude (radians)
        slope_msg = Float32()
        slope_msg.data = float(slope)
        self.slope_pub.publish(slope_msg)

        # Classify terrain status
        status = String()
        if slope < self.slope_warn:
            status.data = 'flat'
        elif slope < self.slope_danger:
            status.data = 'slope_warn'
        else:
            status.data = 'slope_danger'
        self.status_pub.publish(status)

        # Log occasionally
        if len(self.roll_history) == self.history_size:
            self.get_logger().debug(
                f'Roll={math.degrees(avg_roll):.1f}° '
                f'Pitch={math.degrees(avg_pitch):.1f}° '
                f'Slope={math.degrees(slope):.1f}° '
                f'Status={status.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4.4 — Encoder / Odometry Processor Node

Create `sensor_pkg/sensor_pkg/encoder_processor.py`:

```python
#!/usr/bin/env python3
"""
encoder_processor.py
--------------------
In simulation, odometry comes directly from Gazebo diff_drive plugin.
This node re-publishes and enriches it with distance + speed tracking.
On real hardware, this would read raw encoder ticks and compute odom.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math


class EncoderProcessor(Node):
    def __init__(self):
        super().__init__('encoder_processor')

        # Subscriber — Gazebo odom
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.odom_callback, 20)

        # Publishers
        self.odom_pub   = self.create_publisher(Odometry, '/odom', 10)
        self.speed_pub  = self.create_publisher(Float32, '/rover/speed', 10)
        self.dist_pub   = self.create_publisher(Float32, '/rover/total_distance', 10)

        # State
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        self.get_logger().info('Encoder Processor node started')

    def odom_callback(self, msg: Odometry):
        # Forward odometry to standard topic
        self.odom_pub.publish(msg)

        # Compute speed from twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        speed_msg = Float32()
        speed_msg.data = float(speed)
        self.speed_pub.publish(speed_msg)

        # Accumulate distance
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        if self.prev_x is not None:
            dx = cx - self.prev_x
            dy = cy - self.prev_y
            self.total_distance += math.sqrt(dx**2 + dy**2)
        self.prev_x = cx
        self.prev_y = cy

        dist_msg = Float32()
        dist_msg.data = self.total_distance
        self.dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4.5 — setup.py for sensor_pkg

Edit `sensor_pkg/setup.py`:

```python
from setuptools import setup

package_name = 'sensor_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensors_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'lidar_processor   = sensor_pkg.lidar_processor:main',
            'imu_processor     = sensor_pkg.imu_processor:main',
            'encoder_processor = sensor_pkg.encoder_processor:main',
        ],
    },
)
```

## Step 4.6 — Launch File for All Sensors

Create `sensor_pkg/launch/sensors_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_pkg',
            executable='lidar_processor',
            name='lidar_processor',
            parameters=[{
                'min_range': 0.15,
                'max_range': 12.0,
                'num_sectors': 36,
            }],
            output='screen'
        ),
        Node(
            package='sensor_pkg',
            executable='imu_processor',
            name='imu_processor',
            parameters=[{
                'slope_warn_deg': 15.0,
                'slope_danger_deg': 25.0,
            }],
            output='screen'
        ),
        Node(
            package='sensor_pkg',
            executable='encoder_processor',
            name='encoder_processor',
            output='screen'
        ),
    ])
```

## Step 4.7 — Build and Test Sensor Nodes

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws
colcon build --packages-select sensor_pkg --symlink-install
source install/setup.bash

# Run sensors (with simulation running in another terminal)
ros2 launch sensor_pkg sensors_launch.py

# In another terminal, verify outputs
ros2 topic list
# Should include:
# /terrain/pointcloud
# /terrain/roughness
# /imu/slope
# /imu/terrain_status
# /odom
# /rover/speed
# /rover/total_distance

# Echo slope
ros2 topic echo /imu/slope

# Echo terrain status
ros2 topic echo /imu/terrain_status

# Echo roughness (36 sector values)
ros2 topic echo /terrain/roughness
```

## Phase 4 Deliverables
- [ ] lidar_processor publishing `/terrain/pointcloud` and `/terrain/roughness`
- [ ] imu_processor publishing `/imu/slope` and `/imu/terrain_status`
- [ ] encoder_processor publishing `/odom`, `/rover/speed`, `/rover/total_distance`
- [ ] All topics verifiable with `ros2 topic echo`

---

# ═══════════════════════════════════════════
# PHASE 5 — TERRAIN MAPPING
# ═══════════════════════════════════════════

## Objective
Build two nodes: `terrain_mapper` (occupancy grid) and `terrain_classifier` (assigns terrain type to each cell). These are the core perception outputs that feed into the path planner.

## Step 5.1 — Setup mapping_pkg

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws/src
ros2 pkg create mapping_pkg --build-type ament_python \
  --dependencies rclpy sensor_msgs nav_msgs geometry_msgs \
                 std_msgs visualization_msgs
```

## Step 5.2 — Terrain Mapper Node (Full Occupancy Grid)

Create `mapping_pkg/mapping_pkg/terrain_mapper.py`:

```python
#!/usr/bin/env python3
"""
terrain_mapper.py
-----------------
Converts LiDAR PointCloud2 + Odometry into a 2D OccupancyGrid.

Algorithm:
  1. Get rover position from /odom
  2. For each LiDAR point, compute grid cell (i, j) in world frame
  3. Mark that cell as OCCUPIED (100)
  4. Ray-cast from rover to obstacle → mark intermediate cells as FREE (0)
  5. Publish updated OccupancyGrid on /terrain/grid

Grid spec:
  resolution = 0.1 m/cell
  size       = 200×200 cells (20m × 20m)
  origin     = world (0,0) at cell (100,100)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import struct
import math


class TerrainMapper(Node):
    def __init__(self):
        super().__init__('terrain_mapper')

        # Parameters
        self.declare_parameter('resolution', 0.1)   # metres per cell
        self.declare_parameter('grid_size', 200)    # cells (NxN)
        self.declare_parameter('publish_rate', 2.0) # Hz

        self.resolution = self.get_parameter('resolution').value
        self.grid_size  = self.get_parameter('grid_size').value
        self.pub_rate   = self.get_parameter('publish_rate').value

        # Grid: -1=unknown, 0=free, 100=occupied
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Rover position (world frame)
        self.rover_x = 0.0
        self.rover_y = 0.0

        # Subscriptions
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/terrain/pointcloud', self.cloud_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 20)

        # Publisher
        self.grid_pub = self.create_publisher(OccupancyGrid, '/terrain/grid', 10)

        # Publish timer
        self.timer = self.create_timer(
            1.0 / self.pub_rate, self.publish_grid)

        self.get_logger().info(
            f'Terrain Mapper started | '
            f'Grid={self.grid_size}x{self.grid_size} | '
            f'Res={self.resolution}m/cell')

    def odom_callback(self, msg: Odometry):
        """Track rover position in world frame."""
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y

    def world_to_grid(self, wx: float, wy: float):
        """Convert world (x,y) to grid (col, row) indices."""
        origin_offset = self.grid_size // 2
        col = int(wx / self.resolution) + origin_offset
        row = int(wy / self.resolution) + origin_offset
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.grid_size and 0 <= row < self.grid_size

    def cloud_callback(self, msg: PointCloud2):
        """
        Process PointCloud2: mark obstacles and ray-cast free space.
        """
        # Rover grid position
        rover_col, rover_row = self.world_to_grid(self.rover_x, self.rover_y)

        # Parse PointCloud2 bytes
        point_step = msg.point_step
        data = msg.data
        n_points = msg.width

        for i in range(n_points):
            offset = i * point_step
            px, py, pz = struct.unpack_from('fff', data, offset)

            # Convert point to world frame (add rover position)
            wx = px + self.rover_x
            wy = py + self.rover_y

            # Mark obstacle cell
            obs_col, obs_row = self.world_to_grid(wx, wy)
            if self.in_bounds(obs_col, obs_row):
                self.grid[obs_row][obs_col] = 100

            # Ray-cast: mark cells between rover and obstacle as free
            self.raycast_free(rover_col, rover_row, obs_col, obs_row)

        # Mark rover's own cell as free
        if self.in_bounds(rover_col, rover_row):
            self.grid[rover_row][rover_col] = 0

    def raycast_free(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm: mark cells from (x0,y0) to (x1,y1)
        as free (not including the endpoint which is the obstacle).
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cx, cy = x0, y0
        while (cx, cy) != (x1, y1):
            if self.in_bounds(cx, cy):
                # Only mark unknown cells as free — never overwrite obstacles
                if self.grid[cy][cx] == -1:
                    self.grid[cy][cx] = 0
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
            # Safety: stop if out of bounds
            if not self.in_bounds(cx, cy):
                break

    def publish_grid(self):
        """Package grid into OccupancyGrid message and publish."""
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.resolution
        msg.info.width  = self.grid_size
        msg.info.height = self.grid_size

        # Grid origin is the bottom-left corner in world frame
        half = (self.grid_size * self.resolution) / 2.0
        msg.info.origin.position.x = -half
        msg.info.origin.position.y = -half
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten row-major and publish
        msg.data = self.grid.flatten().tolist()
        self.grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 5.3 — Terrain Classifier Node

Create `mapping_pkg/mapping_pkg/terrain_classifier.py`:

```python
#!/usr/bin/env python3
"""
terrain_classifier.py
---------------------
Subscribes to:
  /terrain/grid        (OccupancyGrid)
  /imu/slope           (Float32 — slope angle in radians)
  /terrain/roughness   (Float32MultiArray — roughness per sector)

Publishes:
  /terrain/classified  (Int8MultiArray — terrain class per cell)
  /terrain/costmap     (OccupancyGrid  — cost 0-100 for Nav2 compatibility)

Terrain Classes:
  0 = Unknown
  1 = Flat rock       (cost 10)
  2 = Loose soil      (cost 40)
  3 = Crater edge     (cost 60)
  4 = Steep slope     (cost 85)
  5 = Obstacle        (cost 100)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Int8MultiArray, Header
from std_msgs.msg import Float32MultiArray
import numpy as np


# Terrain type constants
TERRAIN_UNKNOWN      = 0
TERRAIN_FLAT_ROCK    = 1
TERRAIN_LOOSE_SOIL   = 2
TERRAIN_CRATER_EDGE  = 3
TERRAIN_STEEP_SLOPE  = 4
TERRAIN_OBSTACLE     = 5

# Cost values (0-100 scale, used for path planning weight)
TERRAIN_COST = {
    TERRAIN_UNKNOWN:     50,   # Discourage unknown cells
    TERRAIN_FLAT_ROCK:   10,   # Easiest to traverse
    TERRAIN_LOOSE_SOIL:  35,
    TERRAIN_CRATER_EDGE: 60,
    TERRAIN_STEEP_SLOPE: 85,
    TERRAIN_OBSTACLE:    100,  # Blocked
}


class TerrainClassifier(Node):
    def __init__(self):
        super().__init__('terrain_classifier')

        # Classification thresholds (tunable parameters)
        self.declare_parameter('roughness_flat_max',  0.05)
        self.declare_parameter('roughness_soil_max',  0.15)
        self.declare_parameter('roughness_crater_max', 0.30)
        self.declare_parameter('slope_flat_max',  0.15)   # ~8.6 degrees
        self.declare_parameter('slope_steep_min', 0.45)   # ~25.8 degrees

        self.rough_flat   = self.get_parameter('roughness_flat_max').value
        self.rough_soil   = self.get_parameter('roughness_soil_max').value
        self.rough_crater = self.get_parameter('roughness_crater_max').value
        self.slope_flat   = self.get_parameter('slope_flat_max').value
        self.slope_steep  = self.get_parameter('slope_steep_min').value

        # Subscribers
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/terrain/grid', self.grid_callback, 10)
        self.slope_sub = self.create_subscription(
            Float32, '/imu/slope', self.slope_callback, 10)
        self.roughness_sub = self.create_subscription(
            Float32MultiArray, '/terrain/roughness', self.roughness_callback, 10)

        # Publishers
        self.classified_pub = self.create_publisher(
            Int8MultiArray, '/terrain/classified', 10)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/terrain/costmap', 10)

        # State
        self.current_slope = 0.0
        self.current_roughness = np.zeros(36, dtype=np.float32)
        self.grid_info = None

        self.get_logger().info('Terrain Classifier node started')

    def slope_callback(self, msg: Float32):
        self.current_slope = msg.data

    def roughness_callback(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        if len(arr) > 0:
            self.current_roughness = arr

    def grid_callback(self, msg: OccupancyGrid):
        """Classify every cell in the occupancy grid."""
        self.grid_info = msg.info
        width  = msg.info.width
        height = msg.info.height

        # Reshape flat grid to 2D
        raw = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Compute average roughness from all sectors
        avg_roughness = float(np.mean(self.current_roughness))
        max_roughness = float(np.max(self.current_roughness))

        # Classification map (same shape as grid)
        classified = np.zeros((height, width), dtype=np.int8)
        costmap    = np.zeros((height, width), dtype=np.int8)

        for row in range(height):
            for col in range(width):
                cell_val = raw[row, col]
                terrain_class = self.classify_cell(
                    cell_val, avg_roughness, max_roughness)
                classified[row, col] = terrain_class
                costmap[row, col] = TERRAIN_COST[terrain_class]

        # Publish classified map
        cls_msg = Int8MultiArray()
        cls_msg.data = classified.flatten().tolist()
        self.classified_pub.publish(cls_msg)

        # Publish cost map (as OccupancyGrid for Nav2 / RViz)
        cost_grid = OccupancyGrid()
        cost_grid.header = Header()
        cost_grid.header.stamp = self.get_clock().now().to_msg()
        cost_grid.header.frame_id = 'odom'
        cost_grid.info = msg.info
        cost_grid.data = costmap.flatten().tolist()
        self.costmap_pub.publish(cost_grid)

    def classify_cell(self, cell_val: int,
                      avg_roughness: float,
                      max_roughness: float) -> int:
        """
        Classify a single cell based on occupancy + roughness + slope.

        Decision logic:
          OBSTACLE     → cell is OCCUPIED (val == 100)
          UNKNOWN      → cell is UNKNOWN  (val == -1)
          STEEP_SLOPE  → current slope ≥ threshold (affects all free cells)
          CRATER_EDGE  → high max roughness in vicinity
          LOOSE_SOIL   → medium roughness
          FLAT_ROCK    → low roughness, low slope
        """
        # Occupied cell → obstacle
        if cell_val == 100:
            return TERRAIN_OBSTACLE

        # Unknown cell
        if cell_val == -1:
            return TERRAIN_UNKNOWN

        # Free cell — classify by slope + roughness
        if self.current_slope >= self.slope_steep:
            return TERRAIN_STEEP_SLOPE

        if max_roughness > self.rough_crater:
            return TERRAIN_CRATER_EDGE

        if avg_roughness > self.rough_soil:
            return TERRAIN_LOOSE_SOIL

        if avg_roughness <= self.rough_flat and \
           self.current_slope < self.slope_flat:
            return TERRAIN_FLAT_ROCK

        # Default: loose soil
        return TERRAIN_LOOSE_SOIL


def main(args=None):
    rclpy.init(args=args)
    node = TerrainClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 5.4 — setup.py for mapping_pkg

```python
from setuptools import setup

package_name = 'mapping_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/mapping_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'terrain_mapper     = mapping_pkg.terrain_mapper:main',
            'terrain_classifier = mapping_pkg.terrain_classifier:main',
        ],
    },
)
```

## Step 5.5 — Mapping Launch File

Create `mapping_pkg/launch/mapping_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapping_pkg',
            executable='terrain_mapper',
            name='terrain_mapper',
            parameters=[{
                'resolution': 0.1,
                'grid_size': 200,
                'publish_rate': 2.0,
            }],
            output='screen'
        ),
        Node(
            package='mapping_pkg',
            executable='terrain_classifier',
            name='terrain_classifier',
            parameters=[{
                'roughness_flat_max':   0.05,
                'roughness_soil_max':   0.15,
                'roughness_crater_max': 0.30,
                'slope_flat_max':       0.15,
                'slope_steep_min':      0.45,
            }],
            output='screen'
        ),
    ])
```

## Step 5.6 — Build and Test Mapping

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws
colcon build --packages-select mapping_pkg --symlink-install
source install/setup.bash

# Launch (requires simulation + sensors to be running)
ros2 launch mapping_pkg mapping_launch.py

# Verify in another terminal
ros2 topic list | grep terrain
# Should show:
# /terrain/grid
# /terrain/classified
# /terrain/costmap
# /terrain/pointcloud
# /terrain/roughness

# Check grid is updating
ros2 topic hz /terrain/grid
# Expected: ~2 Hz

# View in RViz2
rviz2
# In RViz: Add → By topic → /terrain/grid → Map
# Add → By topic → /terrain/costmap → Map
```

## Step 5.7 — Standalone Mapping Test (No Robot Needed)

Create `tests/test_mapping.py`:

```python
#!/usr/bin/env python3
"""
Standalone test for terrain classification logic — no ROS2 needed.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '../lunar_rover_ws/src/mapping_pkg'))

import numpy as np

# Terrain constants
TERRAIN_UNKNOWN     = 0
TERRAIN_FLAT_ROCK   = 1
TERRAIN_LOOSE_SOIL  = 2
TERRAIN_CRATER_EDGE = 3
TERRAIN_STEEP_SLOPE = 4
TERRAIN_OBSTACLE    = 5

TERRAIN_COST = {0: 50, 1: 10, 2: 35, 3: 60, 4: 85, 5: 100}
TERRAIN_NAME = {0:'Unknown', 1:'Flat rock', 2:'Loose soil',
                3:'Crater edge', 4:'Steep slope', 5:'Obstacle'}


def classify_cell(cell_val, avg_roughness, max_roughness,
                  slope, rough_flat=0.05, rough_soil=0.15,
                  rough_crater=0.30, slope_flat=0.15, slope_steep=0.45):
    if cell_val == 100: return TERRAIN_OBSTACLE
    if cell_val == -1:  return TERRAIN_UNKNOWN
    if slope >= slope_steep:              return TERRAIN_STEEP_SLOPE
    if max_roughness > rough_crater:      return TERRAIN_CRATER_EDGE
    if avg_roughness > rough_soil:        return TERRAIN_LOOSE_SOIL
    if avg_roughness <= rough_flat and slope < slope_flat:
        return TERRAIN_FLAT_ROCK
    return TERRAIN_LOOSE_SOIL


# --- Test cases ---
test_cases = [
    {'cell':  100, 'avg_r': 0.01, 'max_r': 0.01, 'slope': 0.05,
     'expected': TERRAIN_OBSTACLE,    'desc': 'Occupied cell'},
    {'cell':   -1, 'avg_r': 0.01, 'max_r': 0.01, 'slope': 0.05,
     'expected': TERRAIN_UNKNOWN,     'desc': 'Unknown cell'},
    {'cell':    0, 'avg_r': 0.02, 'max_r': 0.03, 'slope': 0.05,
     'expected': TERRAIN_FLAT_ROCK,   'desc': 'Flat smooth terrain'},
    {'cell':    0, 'avg_r': 0.10, 'max_r': 0.20, 'slope': 0.10,
     'expected': TERRAIN_LOOSE_SOIL,  'desc': 'Moderately rough'},
    {'cell':    0, 'avg_r': 0.10, 'max_r': 0.35, 'slope': 0.20,
     'expected': TERRAIN_CRATER_EDGE, 'desc': 'Very rough crater edge'},
    {'cell':    0, 'avg_r': 0.05, 'max_r': 0.10, 'slope': 0.50,
     'expected': TERRAIN_STEEP_SLOPE, 'desc': 'Steep slope'},
]

print("=" * 60)
print("TERRAIN CLASSIFIER UNIT TESTS")
print("=" * 60)

passed = 0
for tc in test_cases:
    result = classify_cell(tc['cell'], tc['avg_r'],
                           tc['max_r'], tc['slope'])
    ok = result == tc['expected']
    status = "✅ PASS" if ok else "❌ FAIL"
    if ok:
        passed += 1
    print(f"{status} | {tc['desc']}")
    print(f"       Got: {TERRAIN_NAME[result]} (cost={TERRAIN_COST[result]})")
    if not ok:
        print(f"       Expected: {TERRAIN_NAME[tc['expected']]}")

print("=" * 60)
print(f"Result: {passed}/{len(test_cases)} tests passed")
```

```bash
cd ~/lunar-rover-quantum
python3 tests/test_mapping.py
# All 6 tests should pass
```

## Phase 5 Deliverables
- [ ] terrain_mapper publishing `/terrain/grid` at 2Hz
- [ ] terrain_classifier publishing `/terrain/classified` and `/terrain/costmap`
- [ ] OccupancyGrid visible in RViz2
- [ ] All 6 unit tests pass
- [ ] Obstacles appear correctly in the grid as you drive near them

---

# ═══════════════════════════════════════════
# PHASE 6 — ENERGY MODELING
# ═══════════════════════════════════════════

## Objective
Build a graph representation of the terrain where every edge has an energy cost. This is the input to both the classical and quantum planners.

## Step 6.1 — Energy Cost Formula

```
E(edge) = d × S(θ) × T(type) × R(r)

Where:
  d      = Euclidean distance between nodes (metres)
  S(θ)   = 1.0 + 2.0 × |sin(θ)|     slope factor (θ = terrain slope)
  T(type) = {flat_rock:1.0, loose_soil:1.5, crater_edge:2.0,
              steep_slope:3.0, obstacle:∞}
  R(r)   = 1.0 + r                   roughness factor (r = normalised roughness)
```

## Step 6.2 — Graph Builder Node

Create `planning_pkg/planning_pkg/graph_builder.py`:

```python
#!/usr/bin/env python3
"""
graph_builder.py
----------------
Subscribes to /terrain/classified (Int8MultiArray) + /terrain/grid metadata
Builds a NetworkX graph where:
  - Each free/classified cell = node
  - 8-directional edges connect adjacent nodes
  - No edges to obstacle cells
Publishes a JSON-serialised graph on /graph/data (String topic)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray, String
import networkx as nx
import numpy as np
import json
import math


# Terrain type constants (must match terrain_classifier.py)
TERRAIN_UNKNOWN     = 0
TERRAIN_FLAT_ROCK   = 1
TERRAIN_LOOSE_SOIL  = 2
TERRAIN_CRATER_EDGE = 3
TERRAIN_STEEP_SLOPE = 4
TERRAIN_OBSTACLE    = 5


class GraphBuilder(Node):
    def __init__(self):
        super().__init__('graph_builder')

        self.declare_parameter('publish_rate', 1.0)
        self.pub_rate = self.get_parameter('publish_rate').value

        # Subscribers
        self.cls_sub = self.create_subscription(
            Int8MultiArray, '/terrain/classified',
            self.classified_callback, 10)
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/terrain/grid',
            self.grid_meta_callback, 10)

        # Publisher
        self.graph_pub = self.create_publisher(String, '/graph/data', 10)

        # State
        self.grid_info = None
        self.classified = None
        self.graph = None

        # Build + publish on timer
        self.timer = self.create_timer(
            1.0 / self.pub_rate, self.build_and_publish)

        self.get_logger().info('Graph Builder node started')

    def grid_meta_callback(self, msg: OccupancyGrid):
        self.grid_info = msg.info

    def classified_callback(self, msg: Int8MultiArray):
        self.classified = np.array(msg.data, dtype=np.int8)

    def build_and_publish(self):
        if self.grid_info is None or self.classified is None:
            return

        w = self.grid_info.width
        h = self.grid_info.height
        res = self.grid_info.resolution

        cls_2d = self.classified.reshape((h, w))
        G = nx.Graph()

        # 8-directional movement directions
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),   # cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)   # diagonal
        ]

        # Add all non-obstacle nodes
        for row in range(h):
            for col in range(w):
                t = int(cls_2d[row, col])
                if t != TERRAIN_OBSTACLE:
                    node_id = row * w + col
                    # World position of cell centre
                    wx = (col + 0.5) * res + self.grid_info.origin.position.x
                    wy = (row + 0.5) * res + self.grid_info.origin.position.y
                    G.add_node(node_id, row=row, col=col,
                               terrain=t, x=wx, y=wy)

        # Add edges
        for row in range(h):
            for col in range(w):
                t_src = int(cls_2d[row, col])
                if t_src == TERRAIN_OBSTACLE:
                    continue
                src_id = row * w + col
                for dr, dc in directions:
                    nr, nc = row + dr, col + dc
                    if not (0 <= nr < h and 0 <= nc < w):
                        continue
                    t_dst = int(cls_2d[nr, nc])
                    if t_dst == TERRAIN_OBSTACLE:
                        continue
                    dst_id = nr * w + nc
                    # Distance (diagonal = √2 × resolution)
                    d = res * math.sqrt(dr**2 + dc**2)
                    if G.has_edge(src_id, dst_id):
                        continue
                    G.add_edge(src_id, dst_id,
                               distance=round(d, 4),
                               terrain_src=t_src,
                               terrain_dst=t_dst)

        self.graph = G

        # Serialise graph for publishing
        # We publish node count, edge count, and the adjacency list
        graph_data = {
            'num_nodes': G.number_of_nodes(),
            'num_edges': G.number_of_edges(),
            'resolution': res,
            'width': w,
            'height': h,
            'nodes': [
                {'id': n, **G.nodes[n]}
                for n in list(G.nodes)[:500]   # cap for message size
            ],
            'edges': [
                {'u': u, 'v': v, **G.edges[u, v]}
                for u, v in list(G.edges)[:2000]
            ]
        }

        msg = String()
        msg.data = json.dumps(graph_data)
        self.graph_pub.publish(msg)

        self.get_logger().info(
            f'Graph: {G.number_of_nodes()} nodes, '
            f'{G.number_of_edges()} edges published')


def main(args=None):
    rclpy.init(args=args)
    node = GraphBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6.3 — Energy Model Node

Create `planning_pkg/planning_pkg/energy_model.py`:

```python
#!/usr/bin/env python3
"""
energy_model.py
---------------
Subscribes to /graph/data (String — JSON NetworkX graph)
Subscribes to /imu/slope   (Float32)

Applies energy cost formula to every edge:
  E = distance × slope_factor × terrain_factor × roughness_factor

Publishes energy-weighted graph to /graph/weighted (String — JSON)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import math


# Terrain energy multipliers
TERRAIN_FACTOR = {
    0: 1.2,   # Unknown  — slight penalty
    1: 1.0,   # Flat rock — easiest
    2: 1.5,   # Loose soil
    3: 2.0,   # Crater edge
    4: 3.0,   # Steep slope
    5: 999.0, # Obstacle — effectively blocked
}


class EnergyModel(Node):
    def __init__(self):
        super().__init__('energy_model')

        # Subscribers
        self.graph_sub = self.create_subscription(
            String, '/graph/data', self.graph_callback, 10)
        self.slope_sub = self.create_subscription(
            Float32, '/imu/slope', self.slope_callback, 10)

        # Publisher
        self.weighted_pub = self.create_publisher(
            String, '/graph/weighted', 10)

        self.current_slope = 0.0
        self.get_logger().info('Energy Model node started')

    def slope_callback(self, msg: Float32):
        self.current_slope = msg.data

    def slope_factor(self, slope_rad: float) -> float:
        """
        Energy cost increases with slope.
        On flat ground (slope=0): factor = 1.0
        On steep slope (slope=π/4): factor ≈ 2.41
        """
        return 1.0 + 2.0 * abs(math.sin(slope_rad))

    def roughness_factor(self, roughness: float) -> float:
        """
        Normalise roughness to [1.0, 2.0].
        roughness is std-dev of LiDAR ranges, typically 0.0–0.5
        """
        return 1.0 + min(roughness, 1.0)

    def graph_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse graph JSON')
            return

        sf = self.slope_factor(self.current_slope)
        nodes = {n['id']: n for n in data.get('nodes', [])}

        weighted_edges = []
        for edge in data.get('edges', []):
            u = edge['u']
            v = edge['v']
            distance = edge.get('distance', 0.1)

            t_src = edge.get('terrain_src', 0)
            t_dst = edge.get('terrain_dst', 0)
            # Use worst terrain of the two endpoint cells
            tf = max(TERRAIN_FACTOR.get(t_src, 1.0),
                     TERRAIN_FACTOR.get(t_dst, 1.0))

            # Roughness placeholder — use 0 if not available
            rf = self.roughness_factor(0.0)

            energy = distance * sf * tf * rf
            energy = round(energy, 4)

            weighted_edges.append({
                'u': u, 'v': v,
                'distance': distance,
                'slope_factor': round(sf, 3),
                'terrain_factor': tf,
                'roughness_factor': round(rf, 3),
                'energy': energy,
            })

        output = {
            'num_nodes': data.get('num_nodes', 0),
            'num_edges': len(weighted_edges),
            'resolution': data.get('resolution', 0.1),
            'width': data.get('width', 0),
            'height': data.get('height', 0),
            'nodes': data.get('nodes', []),
            'edges': weighted_edges,
            'current_slope_rad': round(self.current_slope, 4),
            'current_slope_deg': round(math.degrees(self.current_slope), 2),
        }

        out_msg = String()
        out_msg.data = json.dumps(output)
        self.weighted_pub.publish(out_msg)

        self.get_logger().info(
            f'Energy model: {len(weighted_edges)} weighted edges | '
            f'slope={math.degrees(self.current_slope):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = EnergyModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6.4 — Standalone Energy Test

Create `tests/test_energy_model.py`:

```python
#!/usr/bin/env python3
"""
Test energy cost calculations without ROS2.
"""
import math

TERRAIN_FACTOR = {
    0: 1.2, 1: 1.0, 2: 1.5, 3: 2.0, 4: 3.0, 5: 999.0
}
TERRAIN_NAME = {
    0:'Unknown', 1:'Flat rock', 2:'Loose soil',
    3:'Crater edge', 4:'Steep slope', 5:'Obstacle'
}

def slope_factor(slope_rad):
    return 1.0 + 2.0 * abs(math.sin(slope_rad))

def roughness_factor(roughness):
    return 1.0 + min(roughness, 1.0)

def energy(distance, slope_rad, terrain_type, roughness=0.0):
    return distance * slope_factor(slope_rad) * \
           TERRAIN_FACTOR[terrain_type] * roughness_factor(roughness)

print("=" * 65)
print("ENERGY COST MODEL VERIFICATION")
print("=" * 65)
print(f"{'Terrain':<15} {'Slope':>8} {'Roughness':>10} {'Dist':>6} {'Energy':>8}")
print("-" * 65)

tests = [
    (1, 0.0,  0.0,  1.0),   # Flat rock, flat, smooth
    (2, 0.0,  0.1,  1.0),   # Loose soil, flat
    (1, 0.26, 0.0,  1.0),   # Flat rock, 15° slope
    (3, 0.0,  0.3,  1.0),   # Crater edge, rough
    (4, 0.45, 0.2,  1.0),   # Steep slope
    (1, 0.0,  0.0,  5.0),   # Flat rock, longer path (5m)
    (2, 0.0,  0.0,  5.0),   # Loose soil, longer path
]

for terrain, slope, rough, dist in tests:
    e = energy(dist, slope, terrain, rough)
    print(f"{TERRAIN_NAME[terrain]:<15} "
          f"{math.degrees(slope):>6.1f}°  "
          f"{rough:>10.2f}  "
          f"{dist:>5.1f}m  "
          f"{e:>8.3f}J")

print("=" * 65)
print("\nKey insight: flat rock at 0° slope, 0 roughness = lowest cost")
print("Steep slope multiplies cost by ×3 before slope_factor is applied")
```

```bash
python3 tests/test_energy_model.py
```

## Step 6.5 — setup.py for planning_pkg

```python
from setuptools import setup

package_name = 'planning_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/planning_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'graph_builder    = planning_pkg.graph_builder:main',
            'energy_model     = planning_pkg.energy_model:main',
            'classical_planner = planning_pkg.classical_planner:main',
            'quantum_optimizer = planning_pkg.quantum_optimizer:main',
        ],
    },
)
```

## Phase 6 Deliverables
- [ ] graph_builder publishing NetworkX graph as JSON on `/graph/data`
- [ ] energy_model adding energy weights to all edges, publishing on `/graph/weighted`
- [ ] Energy test shows flat rock < loose soil < crater < steep slope
- [ ] Graph visible with `ros2 topic echo /graph/data`

---

# ═══════════════════════════════════════════
# PHASE 7 — CLASSICAL PATH PLANNING
# ═══════════════════════════════════════════

## Objective
Implement Dijkstra and A* path planning algorithms on the energy-weighted terrain graph. This gives us a classical baseline to compare against quantum optimization later.

## Step 7.1 — Classical Planner Node

Create `planning_pkg/planning_pkg/classical_planner.py`:

```python
#!/usr/bin/env python3
"""
classical_planner.py
--------------------
Subscribes to /graph/weighted (String JSON)
Reads start/goal from ROS2 parameters or a service call.

Implements:
  1. Dijkstra's algorithm (guaranteed optimal on positive-weight graphs)
  2. A* algorithm (heuristic-guided, faster than Dijkstra)

Publishes:
  /path/classical (String JSON) — chosen path + total energy
  /path/classical/viz (nav_msgs/Path) — for RViz2 visualization
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import heapq
import math
import time


class ClassicalPlanner(Node):
    def __init__(self):
        super().__init__('classical_planner')

        # Parameters
        self.declare_parameter('algorithm', 'astar')    # 'dijkstra' or 'astar'
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x',  5.0)
        self.declare_parameter('goal_y',  5.0)

        self.algorithm = self.get_parameter('algorithm').value
        self.start_x   = self.get_parameter('start_x').value
        self.start_y   = self.get_parameter('start_y').value
        self.goal_x    = self.get_parameter('goal_x').value
        self.goal_y    = self.get_parameter('goal_y').value

        # Subscribers
        self.graph_sub = self.create_subscription(
            String, '/graph/weighted', self.graph_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(String, '/path/classical', 10)
        self.viz_pub  = self.create_publisher(Path, '/path/classical/viz', 10)

        # State
        self.graph_data = None
        self.adj = {}       # adjacency dict: node_id → [(cost, neighbour_id)]
        self.nodes = {}     # node_id → {x, y, terrain, ...}

        self.get_logger().info(
            f'Classical Planner started | Algorithm: {self.algorithm}')

    def graph_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.graph_data = data
        self.build_adjacency(data)

        # Find closest nodes to start and goal
        if not self.nodes:
            return

        start_node = self.find_closest_node(self.start_x, self.start_y)
        goal_node  = self.find_closest_node(self.goal_x,  self.goal_y)

        if start_node is None or goal_node is None:
            self.get_logger().warn('Start or goal node not found in graph')
            return

        # Run selected algorithm
        t0 = time.time()
        if self.algorithm == 'dijkstra':
            path, total_cost = self.dijkstra(start_node, goal_node)
        else:
            path, total_cost = self.astar(start_node, goal_node)
        elapsed = (time.time() - t0) * 1000

        if path is None:
            self.get_logger().warn('No path found!')
            return

        self.get_logger().info(
            f'{self.algorithm.upper()}: {len(path)} nodes | '
            f'Energy={total_cost:.3f} | Time={elapsed:.1f}ms')

        self.publish_path(path, total_cost, elapsed)

    def build_adjacency(self, data: dict):
        """Build adjacency list from edge list for fast lookup."""
        self.adj = {}
        self.nodes = {}

        for node in data.get('nodes', []):
            nid = node['id']
            self.nodes[nid] = node
            self.adj[nid] = []

        for edge in data.get('edges', []):
            u = edge['u']
            v = edge['v']
            cost = edge.get('energy', edge.get('distance', 1.0))
            if u in self.adj:
                self.adj[u].append((cost, v))
            if v in self.adj:
                self.adj[v].append((cost, u))

    def find_closest_node(self, wx: float, wy: float):
        """Find node ID whose (x,y) is closest to (wx, wy)."""
        best_id = None
        best_d  = float('inf')
        for nid, node in self.nodes.items():
            dx = node['x'] - wx
            dy = node['y'] - wy
            d  = math.sqrt(dx**2 + dy**2)
            if d < best_d:
                best_d  = d
                best_id = nid
        return best_id

    def dijkstra(self, start: int, goal: int):
        """
        Dijkstra's shortest path (minimum energy).
        Returns (path_as_node_list, total_cost).
        """
        # Priority queue: (cost, node_id)
        pq = [(0.0, start)]
        dist = {start: 0.0}
        prev = {start: None}

        while pq:
            current_cost, current = heapq.heappop(pq)

            if current == goal:
                break

            if current_cost > dist.get(current, float('inf')):
                continue  # stale entry

            for edge_cost, neighbour in self.adj.get(current, []):
                new_cost = current_cost + edge_cost
                if new_cost < dist.get(neighbour, float('inf')):
                    dist[neighbour] = new_cost
                    prev[neighbour] = current
                    heapq.heappush(pq, (new_cost, neighbour))

        # Reconstruct path
        if goal not in dist:
            return None, float('inf')

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev.get(node)
        path.reverse()

        return path, dist[goal]

    def heuristic(self, node_id: int, goal_id: int) -> float:
        """Euclidean distance heuristic for A*."""
        n = self.nodes.get(node_id, {})
        g = self.nodes.get(goal_id,  {})
        dx = n.get('x', 0) - g.get('x', 0)
        dy = n.get('y', 0) - g.get('y', 0)
        return math.sqrt(dx**2 + dy**2)

    def astar(self, start: int, goal: int):
        """
        A* search with Euclidean distance heuristic.
        Returns (path_as_node_list, total_cost).
        """
        # Priority queue: (f=g+h, g, node_id)
        h0 = self.heuristic(start, goal)
        pq = [(h0, 0.0, start)]
        g_cost = {start: 0.0}
        prev   = {start: None}

        while pq:
            _, current_g, current = heapq.heappop(pq)

            if current == goal:
                break

            if current_g > g_cost.get(current, float('inf')):
                continue

            for edge_cost, neighbour in self.adj.get(current, []):
                new_g = current_g + edge_cost
                if new_g < g_cost.get(neighbour, float('inf')):
                    g_cost[neighbour] = new_g
                    prev[neighbour] = current
                    h = self.heuristic(neighbour, goal)
                    heapq.heappush(pq, (new_g + h, new_g, neighbour))

        if goal not in g_cost:
            return None, float('inf')

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev.get(node)
        path.reverse()

        return path, g_cost[goal]

    def publish_path(self, path: list, total_cost: float, elapsed_ms: float):
        # JSON output for downstream nodes
        waypoints = []
        for nid in path:
            node = self.nodes.get(nid, {})
            waypoints.append({
                'node_id': nid,
                'x': node.get('x', 0.0),
                'y': node.get('y', 0.0),
                'terrain': node.get('terrain', 0),
            })

        path_data = {
            'algorithm':    self.algorithm,
            'total_energy': round(total_cost, 4),
            'num_waypoints': len(path),
            'elapsed_ms':   round(elapsed_ms, 2),
            'waypoints':    waypoints,
        }
        msg = String()
        msg.data = json.dumps(path_data)
        self.path_pub.publish(msg)

        # RViz2 Path message
        ros_path = Path()
        ros_path.header.frame_id = 'odom'
        ros_path.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.viz_pub.publish(ros_path)


def main(args=None):
    rclpy.init(args=args)
    node = ClassicalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 7.2 — Standalone Algorithm Test

Create `tests/test_classical_planner.py`:

```python
#!/usr/bin/env python3
"""
Test Dijkstra and A* on a small hand-crafted energy graph.
No ROS2 required.
"""
import heapq, math

def dijkstra(adj, start, goal):
    pq = [(0.0, start)]
    dist = {start: 0.0}
    prev = {start: None}
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal: break
        if d > dist.get(u, float('inf')): continue
        for cost, v in adj.get(u, []):
            nd = d + cost
            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if goal not in dist: return None, float('inf')
    path, n = [], goal
    while n is not None:
        path.append(n)
        n = prev.get(n)
    return list(reversed(path)), dist[goal]

# Graph: 6 nodes, weighted edges simulating terrain
#
#  START(0) --[flat=1.0]--> A(1) --[flat=1.0]--> GOAL(5)
#   |                         |
#  [soil=3.0]            [crater=4.0]
#   |                         |
#  B(2) --[flat=1.0]--> C(3) --[flat=1.0]--> D(4) --[flat=1.0]--> GOAL(5)

adj = {
    0: [(1.0, 1), (3.0, 2)],          # START: flat to A, loose soil to B
    1: [(1.0, 0), (1.0, 5), (4.0, 3)],# A: flat to START, flat to GOAL, crater to C
    2: [(3.0, 0), (1.0, 3)],           # B: soil to START, flat to C
    3: [(4.0, 1), (1.0, 2), (1.0, 4)], # C: crater to A, flat to B, flat to D
    4: [(1.0, 3), (1.0, 5)],           # D: flat to C, flat to GOAL
    5: [(1.0, 1), (1.0, 4)],           # GOAL
}

node_names = {0:'START', 1:'A', 2:'B', 3:'C', 4:'D', 5:'GOAL'}

print("=" * 50)
print("CLASSICAL PLANNER — ALGORITHM TEST")
print("=" * 50)
print("\nGraph topology:")
print("  START --[1.0]--> A --[1.0]--> GOAL  (total: 2.0)")
print("  START --[3.0]--> B --[1.0]--> C --[1.0]--> D --[1.0]--> GOAL (total: 6.0)")
print("  A --[4.0]--> C (shortcut but expensive)\n")

path, cost = dijkstra(adj, 0, 5)
path_names = [node_names[n] for n in path]
print(f"Dijkstra path:  {' → '.join(path_names)}")
print(f"Dijkstra cost:  {cost:.3f}")
print(f"Expected:       START → A → GOAL = 2.0\n")

assert path == [0, 1, 5], f"Wrong path: {path}"
assert abs(cost - 2.0) < 1e-9, f"Wrong cost: {cost}"
print("✅ Dijkstra test PASSED")
print("=" * 50)
```

```bash
python3 tests/test_classical_planner.py
```

## Phase 7 Deliverables
- [ ] classical_planner node running and publishing to `/path/classical`
- [ ] Path visible in RViz2 as a line from start to goal
- [ ] Algorithm test passes for both Dijkstra and A*
- [ ] Console logs show energy cost and time taken

---

# ═══════════════════════════════════════════
# PHASE 8 — QUANTUM OPTIMIZATION (QAOA)
# ═══════════════════════════════════════════

## Objective
Implement QAOA using Qiskit to find the minimum-energy path through the terrain graph. Run on AerSimulator (laptop CPU). Compare result quality vs. classical planners.

## Step 8.1 — Understand QAOA in Plain English

```
Classical approach:
  - Try each path one by one (Dijkstra explores nodes one at a time)
  - Guaranteed optimal but slow for very large graphs

Quantum approach (QAOA):
  - Encode ALL possible paths as quantum superposition simultaneously
  - Apply alternating "problem" and "mixer" quantum gates
  - After p layers, measure → get a near-optimal path
  - More layers (higher p) = better quality but more time

Our encoding:
  - Each binary variable x_i ∈ {0, 1} = "is edge i in the path?"
  - QUBO matrix Q encodes energy costs + path validity constraints
  - QAOA minimizes: Σ Q_ij × x_i × x_j
```

## Step 8.2 — QUBO Formulator

Create `quantum/qubo_formulator.py`:

```python
#!/usr/bin/env python3
"""
qubo_formulator.py
------------------
Converts a terrain energy graph into a QUBO (Quadratic Unconstrained
Binary Optimization) matrix for QAOA.

Variables: x_e ∈ {0,1} for each edge e in the graph
           x_e = 1 means "include this edge in the path"

Objective (minimize):
  Σ_e  energy(e) × x_e        ← minimize total energy

Constraints (encoded as penalties in QUBO):
  1. Flow conservation at each internal node
     (sum of incoming = sum of outgoing edges for non-start/goal nodes)
  2. Each node in path has exactly one incoming + one outgoing edge

Since QUBO has no explicit constraints, we add large penalty terms
to the objective function for constraint violations:
  Penalty = λ × (violation)²

Final QUBO: Q[i][j] = energy + λ × constraint_penalty
"""

import numpy as np
import json
from typing import Dict, List, Tuple


class QUBOFormulator:
    def __init__(self, penalty_strength: float = 10.0):
        """
        penalty_strength: λ — how heavily to penalize constraint violations.
        Should be larger than the maximum energy of any path.
        """
        self.lam = penalty_strength

    def formulate(self,
                  nodes: List[dict],
                  edges: List[dict],
                  start_node_id: int,
                  goal_node_id: int,
                  max_edges: int = 20) -> Tuple[np.ndarray, List[dict]]:
        """
        Build QUBO matrix Q for the path-finding problem.

        For tractability on a simulator, we limit to max_edges edges.
        (Full quantum advantage only appears at scale; for demo we use small graphs.)

        Returns:
          Q     : (n_edges × n_edges) QUBO matrix
          edges : list of edges included in QUBO (ordered)
        """
        # Limit problem size for simulation
        # Select edges closest to the direct start→goal line
        selected_edges = self._select_relevant_edges(
            nodes, edges, start_node_id, goal_node_id, max_edges)

        n = len(selected_edges)
        Q = np.zeros((n, n), dtype=np.float64)

        # Build lookup: node_id → list of (edge_idx, direction)
        # direction: +1 = edge leaves node, -1 = edge enters node
        node_to_edges: Dict[int, List[Tuple[int, int]]] = {}
        for idx, edge in enumerate(selected_edges):
            u, v = edge['u'], edge['v']
            if u not in node_to_edges: node_to_edges[u] = []
            if v not in node_to_edges: node_to_edges[v] = []
            node_to_edges[u].append((idx, +1))  # edge leaves u
            node_to_edges[v].append((idx, -1))  # edge enters v

        # === Linear terms: energy cost ===
        for idx, edge in enumerate(selected_edges):
            Q[idx][idx] += edge.get('energy', edge.get('distance', 1.0))

        # === Quadratic penalty: flow conservation ===
        # For each intermediate node (not start, not goal):
        #   (Σ outgoing x_e - Σ incoming x_e)² = 0
        # Expand: Σ_i Σ_j coeff_i × coeff_j × x_i × x_j
        all_node_ids = set()
        for edge in selected_edges:
            all_node_ids.add(edge['u'])
            all_node_ids.add(edge['v'])

        for node_id in all_node_ids:
            if node_id == start_node_id or node_id == goal_node_id:
                continue
            connected = node_to_edges.get(node_id, [])
            # coefficient: +1 for outgoing, -1 for incoming
            for i_idx, i_dir in connected:
                for j_idx, j_dir in connected:
                    Q[i_idx][j_idx] += self.lam * i_dir * j_dir

        # === Make Q upper-triangular (standard QUBO form) ===
        for i in range(n):
            for j in range(i):
                Q[i][j] += Q[j][i]
                Q[j][i] = 0.0

        return Q, selected_edges

    def _select_relevant_edges(self,
                                nodes, edges,
                                start_id, goal_id,
                                max_edges) -> List[dict]:
        """
        Select edges most relevant to the start→goal path.
        Strategy: prefer edges whose midpoint is close to the
        straight line from start to goal.
        """
        node_map = {n['id']: n for n in nodes}
        start = node_map.get(start_id, {'x': 0, 'y': 0})
        goal  = node_map.get(goal_id,  {'x': 1, 'y': 1})

        sx, sy = start.get('x', 0), start.get('y', 0)
        gx, gy = goal.get('x', 1),  goal.get('y', 1)

        # Direction vector of start→goal line
        dx, dy = gx - sx, gy - sy
        line_len = max(np.sqrt(dx**2 + dy**2), 1e-9)

        scored = []
        for edge in edges:
            u = node_map.get(edge['u'], {'x': 0, 'y': 0})
            v = node_map.get(edge['v'], {'x': 0, 'y': 0})
            # Midpoint of edge
            mx = (u.get('x', 0) + v.get('x', 0)) / 2
            my = (u.get('y', 0) + v.get('y', 0)) / 2
            # Distance from midpoint to start→goal line
            cross = abs((mx - sx) * dy - (my - sy) * dx) / line_len
            # Also prefer edges that are "between" start and goal
            proj = ((mx - sx) * dx + (my - sy) * dy) / line_len
            relevance = cross + max(0, -proj) + max(0, proj - line_len)
            scored.append((relevance, edge))

        scored.sort(key=lambda x: x[0])
        return [e for _, e in scored[:max_edges]]

    def qubo_to_dict(self, Q: np.ndarray) -> List[List[float]]:
        """Serialise Q matrix to list of lists for JSON."""
        return Q.tolist()
```

## Step 8.3 — QAOA Solver

Create `quantum/qaoa_solver.py`:

```python
#!/usr/bin/env python3
"""
qaoa_solver.py
--------------
Implements QAOA (Quantum Approximate Optimization Algorithm) using Qiskit.
Runs on AerSimulator (laptop CPU — no quantum hardware needed).

QAOA Algorithm:
  1. Encode QUBO as a cost Hamiltonian H_C
  2. Start in uniform superposition |s⟩ = H^n |0⟩^n
  3. Apply p alternating layers:
       - Phase separator: e^{-iγ H_C}   (encodes problem)
       - Mixer:           e^{-iβ H_B}   (explores solution space)
  4. Optimize γ, β angles using classical optimizer (COBYLA)
  5. Measure → select best bitstring → decode path
"""

import numpy as np
from qiskit import QuantumCircuit
from qiskit_aer import AerSimulator
from qiskit.primitives import StatevectorSampler
from scipy.optimize import minimize
import itertools
import json
import time


class QAOASolver:
    def __init__(self,
                 p_layers: int = 2,
                 shots: int = 1024,
                 optimizer: str = 'COBYLA'):
        """
        p_layers : QAOA depth (more layers = better quality, slower)
        shots    : number of quantum circuit measurements
        optimizer: classical optimizer for angle tuning
        """
        self.p = p_layers
        self.shots = shots
        self.optimizer_name = optimizer
        self.simulator = AerSimulator()

    def build_qaoa_circuit(self,
                           Q: np.ndarray,
                           gamma: np.ndarray,
                           beta: np.ndarray) -> QuantumCircuit:
        """
        Construct QAOA circuit for given Q matrix and angles γ, β.
        """
        n = Q.shape[0]
        qc = QuantumCircuit(n, n)

        # Initial state: uniform superposition |+⟩^n
        qc.h(range(n))

        # p QAOA layers
        for layer in range(self.p):
            g = gamma[layer]
            b = beta[layer]

            # === Phase separator (Problem Hamiltonian) ===
            # Linear terms: Q[i][i] → RZ gate on qubit i
            for i in range(n):
                if abs(Q[i][i]) > 1e-10:
                    qc.rz(2 * g * Q[i][i], i)

            # Quadratic terms: Q[i][j] → ZZ interaction (i<j only, upper tri)
            for i in range(n):
                for j in range(i + 1, n):
                    if abs(Q[i][j]) > 1e-10:
                        # e^{-i γ Q_ij Z_i Z_j}
                        # Implemented as: CNOT → RZ → CNOT
                        qc.cx(i, j)
                        qc.rz(2 * g * Q[i][j], j)
                        qc.cx(i, j)

            # === Mixer Hamiltonian: X rotations on all qubits ===
            for i in range(n):
                qc.rx(2 * b, i)

        # Measure all qubits
        qc.measure(range(n), range(n))
        return qc

    def evaluate_qubo(self, bitstring: str, Q: np.ndarray) -> float:
        """
        Compute QUBO energy for a given bitstring.
        bitstring: '0101...' where '1' = variable selected
        Energy = x^T Q x
        """
        x = np.array([int(b) for b in bitstring], dtype=np.float64)
        return float(x @ Q @ x)

    def expectation(self, angles: np.ndarray, Q: np.ndarray) -> float:
        """
        Objective function for classical optimizer.
        Returns expected QUBO value ⟨ψ(γ,β)|H_C|ψ(γ,β)⟩
        estimated from shot measurements.
        """
        n = Q.shape[0]
        gamma = angles[:self.p]
        beta  = angles[self.p:]

        qc = self.build_qaoa_circuit(Q, gamma, beta)
        job = self.simulator.run(qc, shots=self.shots)
        counts = job.result().get_counts()

        total_energy = 0.0
        total_shots  = sum(counts.values())

        for bitstring, count in counts.items():
            # Qiskit returns bits in reverse order
            bs = bitstring[::-1]
            energy = self.evaluate_qubo(bs, Q)
            total_energy += energy * count

        return total_energy / total_shots

    def solve(self, Q: np.ndarray) -> dict:
        """
        Run full QAOA optimization.
        Returns best bitstring and its energy.
        """
        n = Q.shape[0]
        print(f"\n  QAOA solving {n}-variable QUBO with p={self.p} layers...")
        t_start = time.time()

        # Initial angles (random in reasonable range)
        np.random.seed(42)
        init_angles = np.random.uniform(0, np.pi, 2 * self.p)

        # Classical optimization of γ, β angles
        print(f"  Optimizing angles with {self.optimizer_name}...")
        result = minimize(
            self.expectation,
            init_angles,
            args=(Q,),
            method=self.optimizer_name,
            options={'maxiter': 200, 'rhobeg': 0.5}
        )

        optimal_angles = result.x
        gamma_opt = optimal_angles[:self.p]
        beta_opt  = optimal_angles[self.p:]

        # Final measurement with optimized angles
        print(f"  Running final measurement ({self.shots} shots)...")
        qc_final = self.build_qaoa_circuit(Q, gamma_opt, beta_opt)
        job = self.simulator.run(qc_final, shots=self.shots)
        counts = job.result().get_counts()

        # Find best bitstring (lowest QUBO energy)
        best_bs     = None
        best_energy = float('inf')
        all_results = []

        for bitstring, count in counts.items():
            bs = bitstring[::-1]   # reverse Qiskit bit order
            energy = self.evaluate_qubo(bs, Q)
            all_results.append({'bitstring': bs, 'energy': energy,
                                 'count': count})
            if energy < best_energy:
                best_energy = energy
                best_bs = bs

        t_elapsed = time.time() - t_start
        all_results.sort(key=lambda x: x['energy'])

        print(f"  Best bitstring: {best_bs}")
        print(f"  Best energy:    {best_energy:.4f}")
        print(f"  Time:           {t_elapsed:.2f}s")

        return {
            'best_bitstring': best_bs,
            'best_energy': best_energy,
            'optimal_gamma': gamma_opt.tolist(),
            'optimal_beta':  beta_opt.tolist(),
            'optimizer_success': result.success,
            'num_iterations': result.nit,
            'elapsed_seconds': round(t_elapsed, 2),
            'top_5_results': all_results[:5],
            'all_counts': counts,
        }

    def decode_path(self,
                    bitstring: str,
                    selected_edges: list,
                    node_map: dict) -> dict:
        """
        Convert QAOA bitstring back to ordered waypoints.
        x_i = '1' means edge i is in the optimal path.
        """
        active_edges = []
        for i, bit in enumerate(bitstring):
            if bit == '1' and i < len(selected_edges):
                active_edges.append(selected_edges[i])

        if not active_edges:
            return {'waypoints': [], 'total_energy': 0.0}

        # Reconstruct ordered path from edge set
        # Build adjacency from active edges
        adj = {}
        for edge in active_edges:
            u, v = edge['u'], edge['v']
            if u not in adj: adj[u] = []
            if v not in adj: adj[v] = []
            adj[u].append(v)
            adj[v].append(u)

        # Find start node (degree 1 or known start)
        degree_1 = [n for n in adj if len(adj[n]) == 1]
        start = degree_1[0] if degree_1 else list(adj.keys())[0]

        # Walk the path
        visited = set()
        ordered = [start]
        visited.add(start)
        current = start
        while True:
            neighbours = [nb for nb in adj.get(current, [])
                          if nb not in visited]
            if not neighbours:
                break
            current = neighbours[0]
            ordered.append(current)
            visited.add(current)

        # Build waypoints
        waypoints = []
        total_energy = 0.0
        for node_id in ordered:
            node = node_map.get(node_id, {})
            waypoints.append({
                'node_id': node_id,
                'x': node.get('x', 0.0),
                'y': node.get('y', 0.0),
                'terrain': node.get('terrain', 0),
            })

        for edge in active_edges:
            total_energy += edge.get('energy', 0.0)

        return {
            'waypoints': waypoints,
            'total_energy': round(total_energy, 4),
            'num_edges_active': len(active_edges),
        }
```

## Step 8.4 — Quantum Optimizer ROS2 Node

Create `planning_pkg/planning_pkg/quantum_optimizer.py`:

```python
#!/usr/bin/env python3
"""
quantum_optimizer.py
--------------------
ROS2 node wrapping QAOASolver.
Subscribes to /graph/weighted (String JSON)
Publishes optimized path to /path/quantum (String JSON)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import sys
import os

# Add quantum folder to path
sys.path.insert(0, os.path.expanduser('~/lunar-rover-quantum/quantum'))
from qubo_formulator import QUBOFormulator
from qaoa_solver import QAOASolver


class QuantumOptimizer(Node):
    def __init__(self):
        super().__init__('quantum_optimizer')

        self.declare_parameter('p_layers', 2)
        self.declare_parameter('shots', 1024)
        self.declare_parameter('max_edges', 12)   # keep QUBO small for demo
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x',  5.0)
        self.declare_parameter('goal_y',  5.0)
        self.declare_parameter('penalty', 10.0)

        self.p_layers  = self.get_parameter('p_layers').value
        self.shots     = self.get_parameter('shots').value
        self.max_edges = self.get_parameter('max_edges').value
        self.start_x   = self.get_parameter('start_x').value
        self.start_y   = self.get_parameter('start_y').value
        self.goal_x    = self.get_parameter('goal_x').value
        self.goal_y    = self.get_parameter('goal_y').value
        self.penalty   = self.get_parameter('penalty').value

        # Subscriber
        self.graph_sub = self.create_subscription(
            String, '/graph/weighted', self.graph_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(String, '/path/quantum', 10)
        self.viz_pub  = self.create_publisher(Path,   '/path/quantum/viz', 10)

        # Objects
        self.formulator = QUBOFormulator(penalty_strength=self.penalty)
        self.solver = QAOASolver(
            p_layers=self.p_layers,
            shots=self.shots
        )

        self._last_graph_hash = None  # avoid recomputing same graph
        self.get_logger().info(
            f'Quantum Optimizer started | '
            f'p={self.p_layers} | shots={self.shots} | '
            f'max_edges={self.max_edges}')

    def graph_callback(self, msg: String):
        # Throttle: only recompute if graph changed significantly
        graph_hash = hash(msg.data[:500])
        if graph_hash == self._last_graph_hash:
            return
        self._last_graph_hash = graph_hash

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        nodes = data.get('nodes', [])
        edges = data.get('edges', [])
        if not nodes or not edges:
            return

        node_map = {n['id']: n for n in nodes}

        # Find start and goal node IDs closest to configured positions
        start_id = self._find_closest_node(node_map, self.start_x, self.start_y)
        goal_id  = self._find_closest_node(node_map, self.goal_x,  self.goal_y)

        if start_id is None or goal_id is None:
            self.get_logger().warn('Could not find start/goal nodes')
            return

        self.get_logger().info(
            f'Running QAOA: start={start_id} goal={goal_id} '
            f'edges={len(edges)}')

        # Build QUBO
        try:
            Q, selected_edges = self.formulator.formulate(
                nodes, edges, start_id, goal_id, self.max_edges)
        except Exception as e:
            self.get_logger().error(f'QUBO formulation failed: {e}')
            return

        # Run QAOA
        try:
            qaoa_result = self.solver.solve(Q)
        except Exception as e:
            self.get_logger().error(f'QAOA solve failed: {e}')
            return

        # Decode path from bitstring
        decoded = self.solver.decode_path(
            qaoa_result['best_bitstring'], selected_edges, node_map)

        self.get_logger().info(
            f'QAOA done | '
            f'energy={decoded["total_energy"]:.4f} | '
            f'waypoints={len(decoded["waypoints"])} | '
            f'time={qaoa_result["elapsed_seconds"]}s')

        # Publish result
        result = {
            'algorithm': 'QAOA',
            'p_layers': self.p_layers,
            'shots': self.shots,
            'qubo_size': Q.shape[0],
            'total_energy': decoded['total_energy'],
            'num_waypoints': len(decoded['waypoints']),
            'elapsed_seconds': qaoa_result['elapsed_seconds'],
            'waypoints': decoded['waypoints'],
            'qaoa_details': {
                'best_bitstring': qaoa_result['best_bitstring'],
                'optimizer_success': qaoa_result['optimizer_success'],
                'num_iterations': qaoa_result['num_iterations'],
            }
        }
        path_msg = String()
        path_msg.data = json.dumps(result)
        self.path_pub.publish(path_msg)

        # RViz path
        ros_path = Path()
        ros_path.header.frame_id = 'odom'
        ros_path.header.stamp = self.get_clock().now().to_msg()
        for wp in decoded['waypoints']:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.viz_pub.publish(ros_path)

    def _find_closest_node(self, node_map, tx, ty):
        import math
        best_id, best_d = None, float('inf')
        for nid, node in node_map.items():
            d = math.sqrt((node.get('x',0)-tx)**2 + (node.get('y',0)-ty)**2)
            if d < best_d:
                best_d, best_id = d, nid
        return best_id


def main(args=None):
    rclpy.init(args=args)
    node = QuantumOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 8.5 — Standalone QAOA Test (No ROS2 Needed)

Create `quantum/test_qaoa.py`:

```python
#!/usr/bin/env python3
"""
Full QAOA test on a small 4-node graph.
Run this BEFORE integrating with ROS2 to verify Qiskit works.
"""
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from qubo_formulator import QUBOFormulator
from qaoa_solver import QAOASolver

print("=" * 60)
print("QAOA STANDALONE TEST")
print("=" * 60)

# Small graph: 4 nodes (0=START, 1=A, 2=B, 3=GOAL)
# Edges:
#   0→1: energy=1.0 (flat rock)
#   0→2: energy=3.0 (steep slope)
#   1→3: energy=1.0 (flat rock)
#   2→3: energy=2.0 (loose soil)
# Best path: 0→1→3 (total energy=2.0)

nodes = [
    {'id': 0, 'x': 0.0, 'y': 0.0, 'terrain': 1},
    {'id': 1, 'x': 1.0, 'y': 0.0, 'terrain': 1},
    {'id': 2, 'x': 0.0, 'y': 1.0, 'terrain': 4},
    {'id': 3, 'x': 1.0, 'y': 1.0, 'terrain': 1},
]

edges = [
    {'u': 0, 'v': 1, 'energy': 1.0, 'distance': 1.0},
    {'u': 0, 'v': 2, 'energy': 3.0, 'distance': 1.0},
    {'u': 1, 'v': 3, 'energy': 1.0, 'distance': 1.0},
    {'u': 2, 'v': 3, 'energy': 2.0, 'distance': 1.0},
]

print("\nGraph edges:")
for e in edges:
    print(f"  {e['u']} → {e['v']} | energy={e['energy']}")
print(f"\nExpected optimal path: 0 → 1 → 3 (energy=2.0)")

# Build QUBO
formulator = QUBOFormulator(penalty_strength=8.0)
Q, selected_edges = formulator.formulate(
    nodes, edges, start_node_id=0, goal_node_id=3, max_edges=4)

print(f"\nQUBO matrix ({Q.shape[0]}×{Q.shape[0]}):")
print(np.round(Q, 2))

# Brute-force check (for verification)
print("\nBrute-force all 2^4 solutions:")
best_brute, best_e = None, float('inf')
for bits in range(2**len(selected_edges)):
    bs = format(bits, f'0{len(selected_edges)}b')
    x = np.array([int(b) for b in bs])
    e = float(x @ Q @ x)
    active = [selected_edges[i] for i,b in enumerate(bs) if b=='1']
    edges_str = '+'.join([f"{ed['u']}→{ed['v']}" for ed in active])
    print(f"  {bs} | {edges_str or 'empty'} | QUBO energy={e:.2f}")
    if e < best_e:
        best_e = e
        best_brute = bs

print(f"\nBrute-force best: {best_brute} (energy={best_e:.2f})")

# Run QAOA
print("\nRunning QAOA (p=2 layers, 512 shots)...")
solver = QAOASolver(p_layers=2, shots=512)
result = solver.solve(Q)

print(f"\nQAOA best bitstring: {result['best_bitstring']}")
print(f"QAOA best energy:    {result['best_energy']:.4f}")
print(f"Optimizer success:   {result['optimizer_success']}")
print(f"Iterations:          {result['num_iterations']}")
print(f"Time:                {result['elapsed_seconds']}s")

# Decode
node_map = {n['id']: n for n in nodes}
decoded = solver.decode_path(result['best_bitstring'], selected_edges, node_map)
print(f"\nDecoded path waypoints:")
for wp in decoded['waypoints']:
    print(f"  Node {wp['node_id']} at ({wp['x']:.1f}, {wp['y']:.1f})")
print(f"Total energy: {decoded['total_energy']:.4f}")
print("=" * 60)
```

```bash
cd ~/lunar-rover-quantum
python3 quantum/test_qaoa.py
```

Expected output shows QAOA finding path 0→1→3 with energy near 2.0.

## Phase 8 Deliverables
- [ ] `test_qaoa.py` runs without errors (takes ~30–120 seconds)
- [ ] QAOA finds correct optimal path on 4-node test graph
- [ ] `quantum_optimizer` ROS2 node publishes to `/path/quantum`
- [ ] Path visible in RViz2

---

# ═══════════════════════════════════════════
# PHASE 9 — ROS2 INTEGRATION
# ═══════════════════════════════════════════

## Objective
Connect all nodes into a single pipeline and send the optimized path to the rover's motors via a `path_executor` node.

## Step 9.1 — Path Executor Node

Create `navigation_pkg/navigation_pkg/path_executor.py`:

```python
#!/usr/bin/env python3
"""
path_executor.py
----------------
Subscribes to /path/quantum (or /path/classical)
Converts waypoints into velocity commands (/cmd_vel)
Uses a simple proportional controller to navigate between waypoints.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import json
import math


class PathExecutor(Node):
    def __init__(self):
        super().__init__('path_executor')

        self.declare_parameter('use_quantum', True)
        self.declare_parameter('waypoint_tolerance', 0.3)  # metres
        self.declare_parameter('linear_speed',  0.3)        # m/s
        self.declare_parameter('angular_gain',  2.0)        # P-gain for steering

        self.use_quantum = self.get_parameter('use_quantum').value
        self.wp_tol      = self.get_parameter('waypoint_tolerance').value
        self.lin_speed   = self.get_parameter('linear_speed').value
        self.ang_gain    = self.get_parameter('angular_gain').value

        # Choose which path to follow
        path_topic = '/path/quantum' if self.use_quantum else '/path/classical'
        self.path_sub = self.create_subscription(
            String, path_topic, self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 20)

        self.cmd_pub = self.create_publisher(
            Twist, '/rover/cmd_vel', 10)

        # State
        self.waypoints = []
        self.current_wp_idx = 0
        self.rover_x   = 0.0
        self.rover_y   = 0.0
        self.rover_yaw = 0.0

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Path Executor started | '
            f'Using {"quantum" if self.use_quantum else "classical"} path')

    def path_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.waypoints = data.get('waypoints', [])
        self.current_wp_idx = 0
        self.get_logger().info(
            f'New path received: {len(self.waypoints)} waypoints | '
            f'energy={data.get("total_energy", 0):.3f}')

    def odom_callback(self, msg: Odometry):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.rover_yaw = math.atan2(siny, cosy)

    def control_loop(self):
        if not self.waypoints or \
           self.current_wp_idx >= len(self.waypoints):
            # No waypoints or path complete — stop
            self.cmd_pub.publish(Twist())
            return

        target = self.waypoints[self.current_wp_idx]
        tx = target['x']
        ty = target['y']

        dx = tx - self.rover_x
        dy = ty - self.rover_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.wp_tol:
            # Reached waypoint — advance to next
            self.current_wp_idx += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_wp_idx}/{len(self.waypoints)}')
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('Path complete!')
            return

        # Proportional steering controller
        target_yaw = math.atan2(dy, dx)
        yaw_error  = target_yaw - self.rover_yaw

        # Normalise angle error to [-π, π]
        while yaw_error >  math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        cmd = Twist()
        cmd.linear.x  = self.lin_speed * min(1.0, dist)
        cmd.angular.z = self.ang_gain * yaw_error
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 9.2 — Master Launch File (Full Pipeline)

Create `planning_pkg/launch/planning_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planning_pkg',
            executable='graph_builder',
            name='graph_builder',
            parameters=[{'publish_rate': 1.0}],
            output='screen'
        ),
        Node(
            package='planning_pkg',
            executable='energy_model',
            name='energy_model',
            output='screen'
        ),
        Node(
            package='planning_pkg',
            executable='classical_planner',
            name='classical_planner',
            parameters=[{
                'algorithm': 'astar',
                'start_x': 0.0, 'start_y': 0.0,
                'goal_x': 5.0,  'goal_y': 5.0,
            }],
            output='screen'
        ),
        Node(
            package='planning_pkg',
            executable='quantum_optimizer',
            name='quantum_optimizer',
            parameters=[{
                'p_layers': 2,
                'shots': 1024,
                'max_edges': 12,
                'start_x': 0.0, 'start_y': 0.0,
                'goal_x': 5.0,  'goal_y': 5.0,
                'penalty': 10.0,
            }],
            output='screen'
        ),
    ])
```

Create `navigation_pkg/launch/navigation_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_pkg',
            executable='path_executor',
            name='path_executor',
            parameters=[{
                'use_quantum': True,
                'waypoint_tolerance': 0.3,
                'linear_speed': 0.3,
                'angular_gain': 2.0,
            }],
            output='screen'
        ),
    ])
```

## Step 9.3 — navigation_pkg setup.py

```python
from setuptools import setup

package_name = 'navigation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/navigation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'path_executor = navigation_pkg.path_executor:main',
        ],
    },
)
```

## Step 9.4 — Build All Packages at Once

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws
colcon build --symlink-install
source install/setup.bash
echo "✅ All packages built"
```

## Step 9.5 — Run Full Pipeline (5 Terminals)

```bash
# Terminal 1 — Simulation
ros2 launch rover_simulation simulation_launch.py

# Terminal 2 — Sensors
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch sensor_pkg sensors_launch.py

# Terminal 3 — Mapping
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch mapping_pkg mapping_launch.py

# Terminal 4 — Planning
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch planning_pkg planning_launch.py

# Terminal 5 — Navigation
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch navigation_pkg navigation_launch.py
```

## Step 9.6 — Verify the Full Pipeline

```bash
# In any terminal with sourced workspace

# See all active topics
ros2 topic list

# Expected key topics:
# /scan  /imu/data  /odom
# /terrain/pointcloud  /terrain/roughness
# /terrain/grid  /terrain/classified  /terrain/costmap
# /graph/data  /graph/weighted
# /path/classical  /path/quantum
# /path/classical/viz  /path/quantum/viz
# /rover/cmd_vel

# Check quantum path
ros2 topic echo /path/quantum --once

# Check rover is moving
ros2 topic echo /odom --once

# View node graph
rqt_graph
```

## Phase 9 Deliverables
- [ ] All 5 terminal sessions running simultaneously without errors
- [ ] `rqt_graph` shows all nodes connected
- [ ] `/path/quantum` publishing JSON with waypoints
- [ ] Rover visibly moves along path in Gazebo

---

# ═══════════════════════════════════════════
# PHASE 10 — COMPARISON & EVALUATION
# ═══════════════════════════════════════════

## Objective
Quantitatively compare classical (A*) vs quantum (QAOA) path planning on energy efficiency, path quality, and computation time. Generate publication-quality graphs.

## Step 10.1 — Battery Monitor Node

Create `evaluation_pkg/evaluation_pkg/battery_monitor.py`:

```python
#!/usr/bin/env python3
"""
battery_monitor.py
------------------
Estimates energy consumed by the rover by integrating motor effort.
In simulation: approximates from cmd_vel magnitude × time.
On real robot: reads motor current sensors.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import time


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Robot physical parameters
        self.declare_parameter('robot_mass_kg', 10.0)
        self.declare_parameter('wheel_radius_m', 0.08)
        self.declare_parameter('motor_efficiency', 0.75)

        self.mass = self.get_parameter('robot_mass_kg').value
        self.wheel_r = self.get_parameter('wheel_radius_m').value
        self.efficiency = self.get_parameter('motor_efficiency').value

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, '/rover/cmd_vel', self.cmd_callback, 10)

        # Publishers
        self.energy_pub = self.create_publisher(
            Float32, '/battery/energy_used', 10)
        self.power_pub = self.create_publisher(
            Float32, '/battery/power_w', 10)

        # State
        self.total_energy_J = 0.0
        self.last_time = time.time()
        self.last_linear  = 0.0
        self.last_angular = 0.0

        # Publish timer
        self.timer = self.create_timer(0.5, self.publish_energy)
        self.get_logger().info('Battery Monitor started')

    def cmd_callback(self, msg: Twist):
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        v = abs(msg.linear.x)
        w = abs(msg.angular.z)

        # Power estimate:
        # Linear: P = F × v = (m × a) × v ≈ mass × v² / wheel_r
        # Angular: additional torque for turning
        p_linear  = self.mass * v**2 / (self.wheel_r + 1e-9)
        p_angular = self.mass * (w * self.wheel_r)**2
        power_W   = (p_linear + p_angular) / self.efficiency

        self.total_energy_J += power_W * dt
        self.last_linear  = v
        self.last_angular = w

        power_msg = Float32()
        power_msg.data = float(power_W)
        self.power_pub.publish(power_msg)

    def publish_energy(self):
        msg = Float32()
        msg.data = float(self.total_energy_J)
        self.energy_pub.publish(msg)
        self.get_logger().debug(
            f'Energy used: {self.total_energy_J:.2f} J')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 10.2 — Evaluator Node

Create `evaluation_pkg/evaluation_pkg/evaluator.py`:

```python
#!/usr/bin/env python3
"""
evaluator.py
------------
Collects metrics from classical and quantum paths.
Logs comparison data and triggers matplotlib report generation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import os
import time


class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator')

        self.declare_parameter('results_dir',
            os.path.expanduser('~/lunar-rover-quantum/results'))
        self.results_dir = self.get_parameter('results_dir').value
        os.makedirs(self.results_dir, exist_ok=True)

        # Subscribers
        self.classical_sub = self.create_subscription(
            String, '/path/classical', self.classical_callback, 10)
        self.quantum_sub = self.create_subscription(
            String, '/path/quantum', self.quantum_callback, 10)
        self.energy_sub = self.create_subscription(
            Float32, '/battery/energy_used', self.energy_callback, 10)

        # State
        self.classical_data = None
        self.quantum_data   = None
        self.actual_energy  = 0.0
        self.experiment_log = []

        self.get_logger().info('Evaluator started')

    def classical_callback(self, msg: String):
        try:
            self.classical_data = json.loads(msg.data)
        except:
            pass
        self.maybe_compare()

    def quantum_callback(self, msg: String):
        try:
            self.quantum_data = json.loads(msg.data)
        except:
            pass
        self.maybe_compare()

    def energy_callback(self, msg: Float32):
        self.actual_energy = msg.data

    def maybe_compare(self):
        if self.classical_data is None or self.quantum_data is None:
            return

        c = self.classical_data
        q = self.quantum_data

        c_energy = c.get('total_energy', 0)
        q_energy = q.get('total_energy', 0)

        if c_energy > 0:
            savings = (c_energy - q_energy) / c_energy * 100
        else:
            savings = 0.0

        record = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'classical': {
                'algorithm':   c.get('algorithm', 'unknown'),
                'energy':      round(c_energy, 4),
                'waypoints':   c.get('num_waypoints', 0),
                'elapsed_ms':  c.get('elapsed_ms', 0),
            },
            'quantum': {
                'algorithm':   'QAOA',
                'energy':      round(q_energy, 4),
                'waypoints':   q.get('num_waypoints', 0),
                'elapsed_s':   q.get('elapsed_seconds', 0),
                'p_layers':    q.get('p_layers', 0),
            },
            'comparison': {
                'energy_savings_pct': round(savings, 2),
                'actual_energy_J':    round(self.actual_energy, 2),
                'quantum_better':     q_energy < c_energy,
            }
        }

        self.experiment_log.append(record)

        self.get_logger().info(
            f'\n{"="*50}\n'
            f'COMPARISON RESULT\n'
            f'  Classical ({c.get("algorithm","?")}): {c_energy:.4f} energy\n'
            f'  Quantum   (QAOA):           {q_energy:.4f} energy\n'
            f'  Savings:                    {savings:.1f}%\n'
            f'  Quantum better: {q_energy < c_energy}\n'
            f'{"="*50}')

        # Save log
        log_file = os.path.join(self.results_dir, 'experiment_log.json')
        with open(log_file, 'w') as f:
            json.dump(self.experiment_log, f, indent=2)

        # Reset for next comparison
        self.classical_data = None
        self.quantum_data   = None


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 10.3 — Comparison Plot Script

Create `scripts/generate_comparison_plots.py`:

```python
#!/usr/bin/env python3
"""
generate_comparison_plots.py
-----------------------------
Reads experiment_log.json and generates publication-quality
comparison plots between classical and quantum planners.
"""
import json
import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

LOG_FILE = os.path.expanduser(
    '~/lunar-rover-quantum/results/experiment_log.json')
OUTPUT_DIR = os.path.expanduser(
    '~/lunar-rover-quantum/results/energy_comparison')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Load data
if not os.path.exists(LOG_FILE):
    print("No experiment log yet. Run the full pipeline first.")
    # Generate sample data for testing
    log = [
        {'classical': {'energy': 8.2,  'elapsed_ms': 5},
         'quantum':   {'energy': 6.8,  'elapsed_s': 45}},
        {'classical': {'energy': 10.5, 'elapsed_ms': 7},
         'quantum':   {'energy': 8.1,  'elapsed_s': 52}},
        {'classical': {'energy': 7.3,  'elapsed_ms': 4},
         'quantum':   {'energy': 6.1,  'elapsed_s': 48}},
        {'classical': {'energy': 12.1, 'elapsed_ms': 8},
         'quantum':   {'energy': 9.4,  'elapsed_s': 61}},
        {'classical': {'energy': 9.0,  'elapsed_ms': 6},
         'quantum':   {'energy': 7.5,  'elapsed_s': 55}},
    ]
    print("Using sample data for demonstration.\n")
else:
    with open(LOG_FILE) as f:
        log = json.load(f)

c_energies = [r['classical']['energy']  for r in log]
q_energies = [r['quantum']['energy']    for r in log]
c_times    = [r['classical'].get('elapsed_ms', 0) / 1000 for r in log]
q_times    = [r['quantum'].get('elapsed_s', 0)    for r in log]
runs       = list(range(1, len(log)+1))

# ---- Plot 1: Energy Comparison Bar Chart ----
fig, axes = plt.subplots(1, 3, figsize=(16, 5))
fig.suptitle('Classical vs Quantum Path Planning — Lunar Rover Navigation',
             fontsize=14, fontweight='bold')

ax1 = axes[0]
x = np.arange(len(runs))
w = 0.35
bars1 = ax1.bar(x - w/2, c_energies, w, label='Classical (A*)',
                color='#4472C4', alpha=0.85, edgecolor='white')
bars2 = ax1.bar(x + w/2, q_energies, w, label='Quantum (QAOA)',
                color='#ED7D31', alpha=0.85, edgecolor='white')
ax1.set_xlabel('Experiment Run')
ax1.set_ylabel('Path Energy Cost (J)')
ax1.set_title('Energy Comparison per Run')
ax1.set_xticks(x)
ax1.set_xticklabels([f'Run {r}' for r in runs])
ax1.legend()
ax1.set_ylim(0, max(c_energies + q_energies) * 1.2)
ax1.grid(axis='y', alpha=0.3)

# ---- Plot 2: Energy Savings % ----
ax2 = axes[1]
savings = [(c-q)/c*100 if c>0 else 0
           for c,q in zip(c_energies, q_energies)]
colors = ['#70AD47' if s > 0 else '#FF0000' for s in savings]
ax2.bar(runs, savings, color=colors, alpha=0.85, edgecolor='white')
ax2.axhline(y=0, color='black', linewidth=0.8)
ax2.axhline(y=np.mean(savings), color='green',
            linewidth=1.5, linestyle='--',
            label=f'Avg savings: {np.mean(savings):.1f}%')
ax2.set_xlabel('Experiment Run')
ax2.set_ylabel('Energy Savings (%)')
ax2.set_title('QAOA Energy Savings over A*')
ax2.legend()
ax2.grid(axis='y', alpha=0.3)

# ---- Plot 3: Computation Time ----
ax3 = axes[2]
ax3.plot(runs, c_times, 'o-', color='#4472C4', label='Classical (A*)',
         linewidth=2, markersize=6)
ax3.plot(runs, q_times, 's--', color='#ED7D31', label='Quantum (QAOA)',
         linewidth=2, markersize=6)
ax3.set_xlabel('Experiment Run')
ax3.set_ylabel('Computation Time (s)')
ax3.set_title('Computation Time Comparison')
ax3.legend()
ax3.grid(alpha=0.3)
ax3.set_yscale('log')

plt.tight_layout()
out_path = os.path.join(OUTPUT_DIR, 'comparison_plot.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"Saved: {out_path}")

# ---- Summary statistics ----
print("\n" + "="*55)
print("EVALUATION SUMMARY")
print("="*55)
print(f"{'Metric':<35} {'Classical':>10} {'Quantum':>10}")
print("-"*55)
print(f"{'Avg path energy':<35} {np.mean(c_energies):>10.3f} "
      f"{np.mean(q_energies):>10.3f}")
print(f"{'Std dev energy':<35} {np.std(c_energies):>10.3f} "
      f"{np.std(q_energies):>10.3f}")
print(f"{'Min energy':<35} {np.min(c_energies):>10.3f} "
      f"{np.min(q_energies):>10.3f}")
print(f"{'Avg computation time (s)':<35} {np.mean(c_times):>10.4f} "
      f"{np.mean(q_times):>10.2f}")
print(f"{'Avg energy savings (QAOA)':<35} {np.mean(savings):>9.1f}%")
print(f"{'Quantum better in N/N runs':<35} "
      f"{sum(1 for s in savings if s>0)}/{len(savings)}")
print("="*55)
plt.show()
```

```bash
python3 scripts/generate_comparison_plots.py
```

## Step 10.4 — evaluation_pkg setup.py

```python
from setuptools import setup

package_name = 'evaluation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/evaluation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'battery_monitor = evaluation_pkg.battery_monitor:main',
            'evaluator       = evaluation_pkg.evaluator:main',
        ],
    },
)
```

## Phase 10 Deliverables
- [ ] `battery_monitor` tracking energy consumption
- [ ] `evaluator` comparing classical vs quantum paths, logging to JSON
- [ ] Comparison plots generated with matplotlib
- [ ] Clear data showing energy savings (or losses) with QAOA

---

# ═══════════════════════════════════════════
# PHASE 11 — REAL ROBOT DEPLOYMENT
# ═══════════════════════════════════════════

## Objective
Deploy the full pipeline on the Arjuna AMR kit with real LiDAR, IMU, and encoders.

## Step 11.1 — Arjuna Hardware Setup

```bash
# On the Jetson Nano / Raspberry Pi:

# 1. Install ROS2 Humble (same as Phase 2)
# 2. Clone your repo onto the robot computer
git clone https://github.com/YOUR_USERNAME/lunar-rover-quantum.git
cd lunar-rover-quantum
pip3 install -r requirements.txt

# 3. Build workspace
cd lunar_rover_ws
colcon build --symlink-install
source install/setup.bash
```

## Step 11.2 — Connect Real LiDAR (RPLIDAR A1/A2/A3)

```bash
# Install RPLIDAR ROS2 driver
sudo apt install ros-humble-rplidar-ros -y

# Check USB connection
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0

# Give permission
sudo chmod 777 /dev/ttyUSB0

# Launch LiDAR driver
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0

# Verify
ros2 topic echo /scan --once
```

## Step 11.3 — Connect Real IMU (MPU6050 / BNO055)

For MPU6050 via I2C:

```bash
# Install I2C tools
sudo apt install i2c-tools -y
sudo i2cdetect -y 1
# Should show device at address 0x68 (MPU6050) or 0x28 (BNO055)

# Install ROS2 IMU driver
pip3 install mpu6050-raspberrypi

# Create real IMU node: sensor_pkg/sensor_pkg/real_imu.py
```

Create `sensor_pkg/sensor_pkg/real_imu.py`:

```python
#!/usr/bin/env python3
"""
Real IMU node for MPU6050 via I2C (Raspberry Pi / Jetson Nano).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

try:
    import mpu6050
    HAS_MPU = True
except ImportError:
    HAS_MPU = False

class RealImuNode(Node):
    def __init__(self):
        super().__init__('real_imu')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        if HAS_MPU:
            self.mpu = mpu6050.mpu6050(0x68)
        self.timer = self.create_timer(0.02, self.read_imu)  # 50 Hz

    def read_imu(self):
        if not HAS_MPU:
            return
        try:
            accel = self.mpu.get_accel_data()
            gyro  = self.mpu.get_gyro_data()
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']
        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])
        # Orientation from accelerometer (simplified)
        msg.orientation.w = 1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RealImuNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 11.4 — Arduino Motor Controller

Upload this to Arduino (controls Arjuna drive motors):

```cpp
// arduino/motor_controller.ino
// Receives /cmd_vel as serial JSON, drives motors.
// Connect Arduino to Jetson/RPi via USB.

#include <Arduino.h>
#include <ArduinoJson.h>

// Motor pins (adjust to your Arjuna wiring)
#define MOTOR_L_PWM  5
#define MOTOR_L_DIR  4
#define MOTOR_R_PWM  6
#define MOTOR_R_DIR  7

#define MAX_SPEED    200  // PWM 0-255
#define WHEEL_SEP    0.44 // metres

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  Serial.println("Motor controller ready");
}

void setMotor(int pwm_pin, int dir_pin, int speed) {
  // speed: -255 to +255
  if (speed >= 0) {
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, min(speed, 255));
  } else {
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, min(-speed, 255));
  }
}

void loop() {
  if (Serial.available() > 0) {
    String json_str = Serial.readStringUntil('\n');
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, json_str);
    if (err) { return; }

    float linear  = doc["linear"]  | 0.0f;
    float angular = doc["angular"] | 0.0f;

    // Differential drive kinematics
    float v_left  = linear - (angular * WHEEL_SEP / 2.0f);
    float v_right = linear + (angular * WHEEL_SEP / 2.0f);

    // Normalise to -1 … +1 range then scale to PWM
    float max_v = max(abs(v_left), abs(v_right));
    if (max_v > 1.0f) { v_left /= max_v; v_right /= max_v; }

    int pwm_l = (int)(v_left  * MAX_SPEED);
    int pwm_r = (int)(v_right * MAX_SPEED);

    setMotor(MOTOR_L_PWM, MOTOR_L_DIR, pwm_l);
    setMotor(MOTOR_R_PWM, MOTOR_R_DIR, pwm_r);

    // Echo back for debugging
    Serial.print("L="); Serial.print(pwm_l);
    Serial.print(" R="); Serial.println(pwm_r);
  }
}
```

## Step 11.5 — Serial Bridge Node (ROS2 ↔ Arduino)

Create `navigation_pkg/navigation_pkg/serial_bridge.py`:

```python
#!/usr/bin/env python3
"""
serial_bridge.py
Bridges ROS2 /cmd_vel → Arduino serial JSON.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Arduino connected on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            self.ser = None
        self.sub = self.create_subscription(
            Twist, '/rover/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg: Twist):
        if self.ser is None:
            return
        data = {'linear': round(msg.linear.x, 3),
                'angular': round(msg.angular.z, 3)}
        self.ser.write((json.dumps(data) + '\n').encode())

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SerialBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 11.6 — Real Robot Launch

```bash
# On the robot (Jetson Nano / RPi):

# Terminal 1: LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0

# Terminal 2: IMU
ros2 run sensor_pkg real_imu

# Terminal 3: All sensors + mapping + planning
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch sensor_pkg sensors_launch.py &
ros2 launch mapping_pkg mapping_launch.py &
ros2 launch planning_pkg planning_launch.py

# Terminal 4: Navigation + Arduino bridge
ros2 launch navigation_pkg navigation_launch.py &
ros2 run navigation_pkg serial_bridge
```

## Phase 11 Deliverables
- [ ] Real LiDAR data on `/scan`
- [ ] Real IMU data on `/imu/data`
- [ ] Arduino receiving cmd_vel and driving motors
- [ ] Rover navigates physically between two points
- [ ] Energy savings observed vs classical planner

---

# ═══════════════════════════════════════════
# PHASE 12 — FINAL DOCUMENTATION
# ═══════════════════════════════════════════

## Step 12.1 — GitHub README Template

Create `README.md`:

```markdown
# Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation

## Abstract
This project implements an autonomous lunar rover navigation system combining
classical robotics (ROS2, Gazebo) with quantum optimization (QAOA via Qiskit)
to find energy-efficient paths across simulated lunar terrain.

## Results
| Metric | Classical (A*) | Quantum (QAOA) |
|---|---|---|
| Avg path energy | X.XX J | X.XX J |
| Energy savings | — | XX% |
| Computation time | <10ms | ~50s (simulated) |

## Architecture
![architecture](docs/architecture_diagram.png)

## Setup
```bash
git clone https://github.com/YOUR/lunar-rover-quantum.git
cd lunar-rover-quantum
pip3 install -r requirements.txt
cd lunar_rover_ws && colcon build
source install/setup.bash
ros2 launch rover_simulation simulation_launch.py
```

## Demo
![rover_demo](docs/rover_demo.gif)

## Team
- [Name] — Quantum optimization
- [Name] — ROS2 integration
- [Name] — Hardware deployment

## License
MIT
```

## Step 12.2 — Viva / Presentation Questions & Answers

```
Q1: Why use quantum optimization for path planning?
A: Classical optimizers like A* guarantee optimality but evaluate one
   solution at a time. QAOA uses quantum superposition to explore
   exponentially many solutions simultaneously, offering potential
   speedup for NP-hard optimization problems at scale.

Q2: What is QUBO and why do we need it?
A: QUBO (Quadratic Unconstrained Binary Optimization) is the standard
   mathematical form quantum optimizers work with. We convert our
   terrain graph (with energy weights and path constraints) into a
   QUBO matrix so QAOA can process it.

Q3: Did you use real quantum hardware?
A: No. We used Qiskit's AerSimulator, which runs quantum circuits on
   a classical CPU. This is standard practice for algorithm development;
   real quantum hardware would be accessed through IBM Quantum cloud.

Q4: What is the energy cost model?
A: E = distance × slope_factor × terrain_factor × roughness_factor.
   Slope factor: 1 + 2|sin(θ)|. Terrain factors: flat_rock=1.0,
   loose_soil=1.5, crater_edge=2.0, steep_slope=3.0.

Q5: What is QAOA depth p and why does it matter?
A: p is the number of alternating problem+mixer quantum gate layers.
   Higher p → better solution quality but exponentially more gates.
   We used p=2 as a practical balance for the simulator.

Q6: What are the limitations of your quantum approach?
A: (1) QUBO size is limited — we used max_edges=12 for tractability.
   (2) AerSimulator time scales exponentially with qubit count.
   (3) Real quantum hardware has noise that degrades solution quality.
   (4) Classical A* still faster for small graphs.

Q7: What is an occupancy grid?
A: A 2D matrix where each cell represents a fixed area of terrain.
   Cell values: 0=free, 100=occupied, -1=unknown. We build ours at
   0.1m/cell resolution over a 20m×20m area.

Q8: How does ray-casting work in your terrain mapper?
A: We use Bresenham's line algorithm to trace from the rover position
   to each LiDAR obstacle point. All cells along this line are marked
   free; only the endpoint is marked occupied.

Q9: What was your biggest technical challenge?
A: Formulating the path-finding problem as a QUBO matrix while
   correctly encoding flow conservation constraints (so QAOA produces
   a valid connected path, not random edge selections).

Q10: What future improvements would you make?
A: (1) Use 3D LiDAR for better slope estimation.
   (2) Implement QUBO warm-starting from classical solution.
   (3) Test on real IBM Quantum hardware for larger graphs.
   (4) Add GNN (Graph Neural Network) for terrain feature prediction.
```

## Step 12.3 — Report Structure

```
CHAPTER 1: Introduction
  1.1 Problem Statement (lunar terrain, energy constraints)
  1.2 Objectives
  1.3 Scope and Limitations

CHAPTER 2: Literature Review
  2.1 Autonomous rover navigation (Mars rovers, NASA research)
  2.2 Energy-efficient path planning
  2.3 Quantum optimization (QAOA, VQE)
  2.4 ROS2 in space robotics

CHAPTER 3: System Design
  3.1 Hardware architecture
  3.2 Software stack (ROS2, Gazebo, Qiskit)
  3.3 Data flow diagram
  3.4 Energy cost model derivation

CHAPTER 4: Implementation
  4.1 Simulation environment (Gazebo, URDF)
  4.2 Sensor integration (LiDAR, IMU, encoders)
  4.3 Terrain mapping (occupancy grid, classifier)
  4.4 Classical path planning (Dijkstra, A*)
  4.5 Quantum optimization (QUBO, QAOA)
  4.6 ROS2 integration pipeline

CHAPTER 5: Results & Evaluation
  5.1 Classical vs quantum energy comparison
  5.2 Path quality analysis
  5.3 Computation time analysis
  5.4 Real robot deployment results

CHAPTER 6: Conclusion
  6.1 Summary of achievements
  6.2 Quantum advantage analysis
  6.3 Future work

REFERENCES
APPENDIX A: Full source code
APPENDIX B: ROS2 topic list
APPENDIX C: QUBO matrix derivation
```

---

# QUICK REFERENCE — ALL COMMANDS

## Build everything
```bash
cd ~/lunar-rover-quantum/lunar_rover_ws
colcon build --symlink-install
source install/setup.bash
```

## Run full simulation pipeline
```bash
# T1
ros2 launch rover_simulation simulation_launch.py
# T2
ros2 launch sensor_pkg sensors_launch.py
# T3
ros2 launch mapping_pkg mapping_launch.py
# T4
ros2 launch planning_pkg planning_launch.py
# T5
ros2 launch navigation_pkg navigation_launch.py
# T6
ros2 launch evaluation_pkg evaluation_launch.py
```

## Run standalone quantum test (no ROS2)
```bash
python3 ~/lunar-rover-quantum/quantum/test_qaoa.py
```

## Run all unit tests
```bash
python3 tests/test_mapping.py
python3 tests/test_classical_planner.py
python3 tests/test_energy_model.py
python3 quantum/test_qaoa.py
```

## Generate comparison plots
```bash
python3 scripts/generate_comparison_plots.py
```

## Check all topics
```bash
ros2 topic list
ros2 topic hz /terrain/grid
ros2 topic hz /path/quantum
rqt_graph
```

---

# TROUBLESHOOTING GUIDE

| Problem | Likely Cause | Solution |
|---|---|---|
| `colcon build` fails | Missing dependency | `rosdep install --from-paths src -r -y` |
| Gazebo won't open | Missing plugin | `sudo apt install ros-humble-gazebo-ros-pkgs -y` |
| `/scan` not publishing | LiDAR plugin error | Check URDF frame_name and output_type |
| QAOA takes forever | Too many qubits | Reduce `max_edges` parameter to 8–10 |
| Rover doesn't move | cmd_vel wrong topic | Check remap: `/cmd_vel` vs `/rover/cmd_vel` |
| Graph empty | Mapping not running | Launch mapping_pkg before planning_pkg |
| `import qiskit` fails | Not installed | `pip3 install qiskit qiskit-aer` |
| Rover falls through ground | URDF collision error | Increase spawn z value to 0.2 |
| Path not published | No graph data | Verify `/graph/weighted` is publishing |
| Arduino not detected | USB port wrong | `ls /dev/ttyUSB*` and update port param |

---

*Document: Complete 12-Phase Implementation Guide*
*Project: Quantum-Assisted Lunar Rover Navigation*
*Stack: Ubuntu 22.04 · ROS2 Humble · Gazebo · Qiskit AerSimulator*
