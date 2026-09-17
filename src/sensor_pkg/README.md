# Lunar Rover Navigation Project
**ROS2 Jazzy + Gazebo Harmonic**

## Project Overview

This project simulates a **Lunar Rover Navigation System** using **ROS2 Jazzy** and **Gazebo Harmonic**.

The rover is designed for autonomous terrain navigation using:

- LiDAR terrain sensing
- IMU-based slope estimation
- Differential drive rover movement
- Terrain point cloud generation
- ROS ↔ Gazebo integration

---

# ✅ Phase 4 — Sensors (Week 4)

## Objective

Build the three sensor nodes (`lidar_processor`, `imu_processor`, `encoder_processor`) inside `sensor_pkg` and publish clean processed data to:

- `/terrain/pointcloud`
- `/imu/slope`
- `/odom`

---

## Completed Work

### 1. LiDAR Integration

**Purpose:** LiDAR is used for terrain sensing and obstacle detection.

| | |
|---|---|
| **Input Topic** | `/scan` |
| **Input Type** | `sensor_msgs/msg/LaserScan` |
| **Processor Node** | `lidar_processor.py` |
| **Output Topic** | `/terrain/pointcloud` |
| **Output Type** | `sensor_msgs/msg/PointCloud2` |

**Functionality:**
- Reads LiDAR scan data
- Converts scan data into terrain representation
- Generates terrain point cloud

**Verification:**
```bash
ros2 topic echo /terrain/pointcloud --once
```

**Status: ✅ Fully Working**

---

### 2. IMU Integration

**Purpose:** IMU is used to estimate terrain slope and rover orientation.

| | |
|---|---|
| **Input Topic** | `/imu/data` |
| **Input Type** | `sensor_msgs/msg/Imu` |
| **Processor Node** | `imu_processor.py` |
| **Output Topic** | `/imu/slope` |
| **Output Type** | `std_msgs/msg/Float32` |

**Functionality:**
- Reads IMU orientation data
- Estimates terrain slope
- Publishes simplified slope information

**Verification:**
```bash
ros2 topic echo /imu/slope --once
```

Example output:
```
data: 0.005428861360996962
```

**Status: ✅ Fully Working**

---

### 3. Encoder / Odometry Integration

**Purpose:** Generate rover odometry using wheel movement.

**Note:** Gazebo Harmonic does not publish wheel encoder ticks directly. The original `/encoder/ticks` topic was unavailable, so the implementation was updated to use `/joint_states` for wheel movement estimation.

| | |
|---|---|
| **Input Topic** | `/joint_states` |
| **Processor Node** | `encoder_processor.py` |
| **Output Topic** | `/odom` |
| **Output Type** | `nav_msgs/msg/Odometry` |

**Status: 🟡 Mostly Completed**
- Rover movement verified
- Joint state bridge added
- Encoder processor updated
- Final odometry verification may require relaunch

---

## Rover Simulation Components

The rover includes:

- **LiDAR Sensor** — publishes `/scan` and `/scan/points`
- **IMU Sensor** — publishes `/imu/data`
- **Differential Drive Plugin** — allows rover movement via `/cmd_vel`
- **Joint State Publisher** — provides wheel movement info via `/joint_states`

---

## Available ROS Topics

**Raw Topics:**
- `/scan`
- `/imu/data`
- `/joint_states`
- `/cmd_vel`

**Processed Topics:**

| Topic | Status |
|---|---|
| `/terrain/pointcloud` | ✅ Working |
| `/imu/slope` | ✅ Working |
| `/odom` | 🟡 Partially Verified |

---

## How To Run This Project

### Step 1 — Build Workspace
```bash
cd ros2_ws
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Step 2 — Start Gazebo Simulation
```bash
gz sim simulation/moon.world
```

### Step 3 — Spawn Rover

Open a new terminal:
```bash
gz service -s /world/moon_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf_filename: "simulation/rover.sdf", name: "lunar_rover", pose: {position: {x: 0, y: 0, z: 5}}'
```

### Step 4 — Start ROS ↔ Gazebo Bridges

**IMU + LiDAR Bridge** — open a new terminal:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

**Rover Movement Bridge** — open another terminal:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

**Joint State Bridge** — open another terminal:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /joint_states@sensor_msgs/msg/JointState@gz.msgs.Model
```

### Step 5 — Launch Sensor Nodes
```bash
ros2 launch sensor_pkg sensors_launch.py
```

### Step 6 — Move Rover (Optional)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10
```

---

## Phase 4 Completion Summary

| Component | Status |
|---|---|
| LiDAR Integration | ✅ Complete |
| IMU Integration | ✅ Complete |
| Rover Movement | ✅ Complete |
| ROS ↔ Gazebo Bridge | ✅ Complete |
| `lidar_processor.py` | ✅ Complete |
| `imu_processor.py` | ✅ Complete |
| `encoder_processor.py` | 🟡 Mostly Complete |
| `/terrain/pointcloud` | ✅ Working |
| `/imu/slope` | ✅ Working |
| `/odom` | 🟡 Pending Final Verification |

---

# Phase 5 — Mapping (Week 5)

## Objective

Build `mapping_pkg` containing:
- `terrain_mapper`
- `terrain_classifier`

> **Important for Phase 5:** Do NOT rebuild sensors. Phase 4 sensor integration is already complete. Use the outputs from Phase 4 directly.

Use these as inputs:
- `/terrain/pointcloud`
- `/imu/slope`
- `/joint_states`

---

## 1. terrain_mapper

**Purpose:** Convert terrain point cloud into an occupancy grid map.

| | |
|---|---|
| **Input Topic** | `/terrain/pointcloud` (`sensor_msgs/msg/PointCloud2`) |
| **Output Topic** | `/terrain/grid_map` (`nav_msgs/msg/OccupancyGrid`) |

**Expected Functionality:**
- Convert LiDAR terrain point cloud into traversable and blocked areas
- Generate occupancy map using PointCloud2 → 2D grid conversion
- Mark occupied vs free regions

---

## 2. terrain_classifier

**Purpose:** Classify terrain regions into navigational categories.

| | |
|---|---|
| **Inputs** | `/terrain/grid_map`, `/imu/slope` |
| **Output Topic** | `/terrain/classification` |

**Required Terrain Labels:**

| Label | Condition |
|---|---|
| Flat Rock | Low slope, stable terrain |
| Loose Soil | Uneven but traversable |
| Crater Edge | Sudden terrain drop |
| Steep Slope | High IMU slope value |
| Obstacle | Dense occupancy cluster |
