# PHASE 3 — Robot Simulation
## ROS2 Jazzy + Gazebo Harmonic Edition

> Ubuntu 24.04 · ROS2 Jazzy · Gazebo Harmonic (gz sim 8.x)

---

## KEY DIFFERENCE FROM HUMBLE

| Thing | Humble | Jazzy (this guide) |
|---|---|---|
| Gazebo command | `gazebo` | `gz sim` |
| Plugin prefix | `libgazebo_ros_*` | `gz-sim-*` |
| ROS-Gazebo bridge | built into plugins | separate `ros_gz_bridge` node |
| Sensor topics | plugins publish directly | bridge.yaml translates gz→ROS |
| Launch package | `gazebo_ros` | `ros_gz_sim` |

---

## STEP 1 — Install packages

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-tools \
  ros-jazzy-rqt \
  ros-jazzy-rqt-graph

# Verify Gazebo Harmonic
gz sim --version
# Expected: Gazebo Harmonic, version 8.x.x
```

---

## STEP 2 — Create package + folders

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws/src

ros2 pkg create rover_simulation \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs sensor_msgs nav_msgs

cd rover_simulation
mkdir -p urdf worlds config launch

# Create empty __init__.py
touch rover_simulation/__init__.py
```

---

## STEP 3 — World file

**File:** `worlds/lunar_terrain.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="lunar_terrain">

    <!-- Moon gravity: 1.62 m/s^2 -->
    <gravity>0 0 -1.62</gravity>

    <!-- Physics plugin (required in Harmonic) -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <!-- Black sky, minimal ambient (no Moon atmosphere) -->
    <scene>
      <ambient>0.1 0.1 0.1 1</ambient>
      <background>0.01 0.01 0.01 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Harsh directional sun -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <!-- Ground: lunar regolith, grey, high friction -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Rock 1 — in front of rover -->
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

    <!-- Rock 2 — to the left -->
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

    <!-- Rock 3 — large boulder -->
    <model name="rock3">
      <static>true</static>
      <pose>6 -2 0.4 0.05 0.15 1.2</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>1.0 0.8 0.8</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>1.0 0.8 0.8</size></box></geometry>
          <material><ambient>0.40 0.38 0.36 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Slope/ramp -->
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

    <!-- Crater depression -->
    <model name="crater1">
      <static>true</static>
      <pose>5 5 -0.4 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry>
            <cylinder><radius>2.0</radius><length>0.5</length></cylinder>
          </geometry>
        </collision>
        <visual name="vis">
          <geometry>
            <cylinder><radius>2.0</radius><length>0.5</length></cylinder>
          </geometry>
          <material><ambient>0.30 0.30 0.30 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## STEP 4 — Rover URDF (Gazebo Harmonic plugins)

**File:** `urdf/rover.urdf.xacro`

> IMPORTANT: Plugin filenames are completely different from Humble.
> Use `gz-sim-*` not `libgazebo_ros_*`

```xml
<?xml version="1.0"?>
<robot name="lunar_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ========== PROPERTIES ========== -->
  <xacro:property name="body_length"  value="0.5"/>
  <xacro:property name="body_width"   value="0.4"/>
  <xacro:property name="body_height"  value="0.15"/>
  <xacro:property name="wheel_radius" value="0.08"/>
  <xacro:property name="wheel_width"  value="0.05"/>
  <xacro:property name="body_mass"    value="10.0"/>
  <xacro:property name="wheel_mass"   value="0.5"/>
  <xacro:property name="pi"           value="3.14159265359"/>

  <!-- ========== INERTIA MACROS ========== -->
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

  <!-- ========== BASE ========== -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ========== CHASSIS ========== -->
  <link name="base_link">
    <xacro:box_inertia m="${body_mass}" l="${body_length}"
                       w="${body_width}" h="${body_height}"/>
    <visual>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="grey"><color rgba="0.6 0.6 0.6 1"/></material>
    </visual>
    <collision>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- ========== WHEEL MACRO ========== -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}_wheel">
      <xacro:cylinder_inertia m="${wheel_mass}"
                              r="${wheel_radius}" h="${wheel_width}"/>
      <visual>
        <origin rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
      </visual>
      <collision>
        <origin rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${name}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${name}_wheel"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.1" friction="0.1"/>
    </joint>
  </xacro:macro>

  <!-- 4 wheels -->
  <xacro:wheel name="front_left"  x=" 0.18" y=" 0.22"/>
  <xacro:wheel name="front_right" x=" 0.18" y="-0.22"/>
  <xacro:wheel name="rear_left"   x="-0.18" y=" 0.22"/>
  <xacro:wheel name="rear_right"  x="-0.18" y="-0.22"/>

  <!-- ========== LIDAR MOUNT ========== -->
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.04" length="0.06"/></geometry>
      <material name="sensor_black"><color rgba="0.1 0.1 0.1 1"/></material>
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

  <!-- ========== IMU LINK ========== -->
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

  <!-- =====================================================
       GAZEBO HARMONIC PLUGINS
       NOTE: These are gz-sim-* plugins, NOT libgazebo_ros_*
       ===================================================== -->

  <!-- Differential drive (moves rover via /model/lunar_rover/cmd_vel) -->
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>front_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <wheel_separation>0.44</wheel_separation>
      <wheel_radius>${wheel_radius}</wheel_radius>
      <odom_publish_frequency>20</odom_publish_frequency>
      <!-- These gz topics get bridged to ROS2 via bridge.yaml -->
      <topic>/model/lunar_rover/cmd_vel</topic>
      <odom_topic>/model/lunar_rover/odometry</odom_topic>
      <tf_topic>/model/lunar_rover/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
    </plugin>
  </gazebo>

  <!-- Sensors system (required to activate all sensor plugins) -->
  <gazebo>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>

  <!-- IMU sensor (publishes to gz topic /imu, bridged to /imu/data) -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <gz_frame_id>imu_link</gz_frame_id>
      <topic>/imu</topic>
      <update_rate>50</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
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

  <!-- LiDAR sensor (publishes to gz topic /lidar, bridged to /scan) -->
  <gazebo reference="lidar_link">
    <sensor name="lidar_sensor" type="gpu_lidar">
      <gz_frame_id>lidar_link</gz_frame_id>
      <topic>/lidar</topic>
      <update_rate>10</update_rate>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <lidar>
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
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>
    </sensor>
  </gazebo>

</robot>
```

---

## STEP 5 — Bridge config (NEW in Jazzy)

**File:** `config/bridge.yaml`

This file maps Gazebo internal topics → ROS2 topics. Without it, nothing appears in `ros2 topic list`.

```yaml
- ros_topic_name: /scan
  gz_topic_name: /lidar
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS

- ros_topic_name: /imu/data
  gz_topic_name: /imu
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS

- ros_topic_name: /rover/cmd_vel
  gz_topic_name: /model/lunar_rover/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ

- ros_topic_name: /rover/odom
  gz_topic_name: /model/lunar_rover/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS

- ros_topic_name: /tf
  gz_topic_name: /model/lunar_rover/tf
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS

- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
```

---

## STEP 6 — Launch file

**File:** `launch/simulation_launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('rover_simulation')

    urdf_file    = os.path.join(pkg, 'urdf',   'rover.urdf.xacro')
    world_file   = os.path.join(pkg, 'worlds', 'lunar_terrain.world')
    bridge_file  = os.path.join(pkg, 'config', 'bridge.yaml')

    robot_desc = {'robot_description': Command(['xacro ', urdf_file])}

    # 1. Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_desc, {'use_sim_time': True}]
    )

    # 2. Gazebo Harmonic — gz sim (NOT 'gazebo')
    #    -r flag = run simulation immediately (don't pause on start)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 3. Spawn rover (wait 4s for Gazebo to fully load)
    spawn = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',  'lunar_rover',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.15',
                ],
                output='screen'
            )
        ]
    )

    # 4. ROS-Gazebo bridge (translates gz topics → ROS2 topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_file,
                     'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    # 5. RViz2
    rviz_config = os.path.join(pkg, 'config', 'rover.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([rsp, gz_sim, spawn, bridge, rviz])
```

---

## STEP 7 — setup.py

**File:** `setup.py` — replace the full file:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'rover_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': []},
)
```

---

## STEP 8 — package.xml

**File:** `package.xml` — replace full file:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_simulation</name>
  <version>0.1.0</version>
  <description>Lunar rover Gazebo Harmonic simulation</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_msgs</depend>
  <depend>ros_gz_sim</depend>
  <depend>ros_gz_bridge</depend>
  <depend>ros_gz_interfaces</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>
  <depend>rviz2</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## STEP 9 — Build

```bash
cd ~/lunar-rover-quantum/lunar_rover_ws

# Auto-install missing deps
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
echo "Build complete"
```

---

## STEP 10 — Launch

```bash
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch rover_simulation simulation_launch.py
```

Expected sequence:
1. Terminal prints "gz sim" starting
2. Gazebo Harmonic window opens with grey lunar terrain
3. After ~4 seconds: "Spawning entity [lunar_rover]"
4. Rover appears in Gazebo
5. RViz2 opens

---

## STEP 11 — Drive with keyboard (new terminal)

```bash
# Open a NEW terminal
source /opt/ros/jazzy/setup.bash
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/rover/cmd_vel
```

Keys: `i` = forward, `,` = backward, `j`/`l` = turn, `k` = stop

---

## STEP 12 — Verify sensors (another new terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash

# List all ROS2 topics
ros2 topic list

# Check rates
ros2 topic hz /scan          # expect ~10 Hz
ros2 topic hz /imu/data      # expect ~50 Hz
ros2 topic hz /rover/odom    # expect ~20 Hz

# Debug: see what Gazebo is publishing internally
gz topic -l
```

Expected `ros2 topic list` output:
```
/clock
/imu/data
/parameter_events
/rosout
/rover/cmd_vel
/rover/odom
/scan
/tf
/tf_static
```

---

## STEP 13 — Configure RViz2

In the RViz2 window:
1. Global Options → Fixed Frame → change to `odom`
2. Add → By Topic → `/scan` → LaserScan → OK
3. Add → By display type → RobotModel → OK
4. Add → By display type → TF → OK
5. File → Save Config As → `config/rover.rviz`

You should see red dots forming a circle around the rover — that is the LiDAR scan.

---

## TROUBLESHOOTING

| Problem | Fix |
|---|---|
| `gz sim: command not found` | `source /opt/ros/jazzy/setup.bash` |
| `/scan` not in `ros2 topic list` | Bridge not running or wrong `gz_topic_name` in bridge.yaml |
| Rover spawns but won't move | Check bridge has `/rover/cmd_vel` with `ROS_TO_GZ` direction |
| `gz topic -l` shows `/lidar` but `/scan` missing | Bridge crashed — check config_file path in launch |
| Rover sinks underground | Change spawn `-z 0.15` to `-z 0.25` in launch file |
| `ogre2 render engine not found` | `sudo apt install libogre-next-2.3-dev` or switch to `ogre` |
| RViz2 fixed frame error | Set Fixed Frame to `odom` in Global Options |
| `create: command not found` | `sudo apt install ros-jazzy-ros-gz-sim -y` |
| IMU data all zeros | Sensors system plugin missing from URDF — check gz-sim-sensors-system block |
| xacro property error | Run `xacro rover.urdf.xacro` to find the exact error line |

---

## COMMIT

```bash
cd ~/lunar-rover-quantum
git add .
git commit -m "Phase 3: Lunar terrain + rover URDF + Gazebo Harmonic simulation (Jazzy)"
git push origin main
```

---

## PHASE 3 CHECKLIST

- [ ] `gz sim --version` shows Harmonic 8.x
- [ ] `rover_simulation` package built without errors
- [ ] `lunar_terrain.world` opens in `gz sim` alone
- [ ] URDF uses `gz-sim-diff-drive-system` (not libgazebo_ros)
- [ ] `bridge.yaml` has all 6 topic mappings
- [ ] Launch file starts: gz sim + spawn + bridge + RViz2
- [ ] Rover visible in Gazebo after spawn
- [ ] Teleop keyboard moves rover
- [ ] `/scan` at ~10 Hz
- [ ] `/imu/data` at ~50 Hz
- [ ] `/rover/odom` at ~20 Hz
- [ ] LiDAR dots visible in RViz2
- [ ] Changes committed to GitHub

---

## CONTINUATION PROMPT FOR PHASE 4

```
Building "Quantum-Assisted Energy Optimization for Autonomous Lunar Rover Navigation"

STACK: ROS2 Jazzy + Gazebo Harmonic (gz sim 8.x) + Ubuntu 24.04

COMPLETED:
✅ Phase 1 — Planning, GitHub repo, folder structure
✅ Phase 2 — Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic, Qiskit installed
✅ Phase 3 — Simulation COMPLETE (Jazzy edition):
   - Package: rover_simulation (built and working)
   - lunar_terrain.world: moon gravity, rocks, crater, slope
   - rover.urdf.xacro: gz-sim-diff-drive-system, gz-sim-sensors-system,
     gpu_lidar sensor, imu sensor (all Gazebo Harmonic plugin names)
   - bridge.yaml: maps gz topics → ROS2 (/lidar→/scan, /imu→/imu/data, etc.)
   - simulation_launch.py: gz sim + ros_gz_sim spawn + ros_gz_bridge + rviz2
   - Rover drives with teleop
   - /scan (10Hz), /imu/data (50Hz), /rover/odom (20Hz) confirmed

Please give me Phase 4 — Sensor Integration for ROS2 Jazzy.
Need complete code for:
1. lidar_processor.py
2. imu_processor.py
3. encoder_processor.py
4. sensors_launch.py
5. setup.py + package.xml for sensor_pkg
All updated for Jazzy (sourcing from /opt/ros/jazzy/, correct package deps).
```
