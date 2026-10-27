# arjuna_quantum_nav — run the planners on the real Arjuna robot

ROS **Melodic** package for the **Arjuna Edu Kit** (Jetson Nano, Ubuntu 18.04, Python 3.6).
It runs the same Dijkstra / A* / QAOA planners as the web sim, then drives the robot along the
chosen path.

```
RViz "2D Nav Goal" ─► quantum_nav ─► /cmd_vel ─► Arjuna_ticks_pub.py ─► STS servos
                         ▲  ▲  ▲  ▲
   /robot_pose_ekf/odom_combined  (servo ticks + BNO055, from the kit's EKF)
   /map (map_server)   /scan (RPLidar C1)   /imu/data (BNO055)   /distance (HC-SR04)
```

It does **not** replace any kit driver. The kit's existing nodes already turn `/cmd_vel` into
wheel motion and estimate the robot's pose. This package only plans the path and follows it.

| Algorithm | What it optimises |
|---|---|
| `dijkstra` | Shortest **distance**. Slope-blind baseline. |
| `astar` | Lowest **energy** (uphill costs more, gentle downhill less). Optimal, and the fastest search. |
| `qaoa` | Energy, chosen by QAOA-style sampling (most-measured result, like a real QAOA run). |
| `auto` | Runs all three and drives the best one: lowest energy, ties broken by fastest planning. |

---

## 1. Install on the Jetson

Do this once. Copy the package into the kit workspace and build it:

```bash
cp -r ~/Documents/"Lunar Rover"/lunar-rover-quantum/robot/arjuna_quantum_nav ~/arjuna_ws/src/
chmod +x ~/arjuna_ws/src/arjuna_quantum_nav/scripts/*.py
cd ~/arjuna_ws && catkin_make
source ~/arjuna_ws/devel/setup.bash
```

It needs nothing beyond `rospy` and the standard message packages. There is no pip install.

## 2. Start the kit as usual

In terminal 1, run the kit's normal bringup from the curriculum. That's the launch with
`rplidarNode`, `imu_bno055`, `Arjuna_ticks_pub.py`, `EKF_data_pub`, `robot_pose_ekf`,
`map_server` and RViz.

> ⚠️ **Do not start the kit's own auto-nav node at the same time.** Two nodes publishing
> `/cmd_vel` will fight over the motors.

The sonar is optional. If you want it, start rosserial in another terminal:
`rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB? _baud:=9600`.

## 3. First-run checks (wheels OFF the ground first)

```bash
rosrun arjuna_quantum_nav drive_test.py _mode:=sensors
```
- Put a box about 30 cm **in front** of the robot. `lidar front` should read about `0.30 m`.
  If not, re-run with `_laser_yaw_offset_deg:=180` and use the same value in step 4.
- Lift the **nose up**. `pitch` should go **positive**. If it goes negative, use `pitch_sign:=-1`.

Then put the robot on the floor, with 1 m of clear space ahead:
```bash
rosrun arjuna_quantum_nav drive_test.py _mode:=straight _distance:=0.5
rosrun arjuna_quantum_nav drive_test.py _mode:=turn _angle_deg:=90
```
Measure the real distance and angle. If they're more than about 10% off, fix the kit's
tick→metre calibration first. The path follower trusts the pose.

## 4. Drive autonomously

In terminal 2:
```bash
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=auto max_linear:=0.10
```
1. In RViz, click **2D Pose Estimate** and set where the robot really is on the map.
2. Click **2D Nav Goal** and pick the target. The robot plans and drives.
3. Add RViz **Path** displays for `/quantum_nav/path/dijkstra`, `/quantum_nav/path/astar`,
   `/quantum_nav/path/qaoa` (all candidates) and `/quantum_nav/path` (the one being driven).

Other ways to run it:
```bash
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=astar goal_x:=1.5 goal_y:=0.0
roslaunch arjuna_quantum_nav quantum_nav.launch use_map:=false   # no map: open ground + live LiDAR
rostopic echo /quantum_nav/status   # what it's doing right now
rostopic echo /quantum_nav/report   # planned energy/distance/climb/nodes/runtime per algorithm
```

**Safety built in.** The robot stops when:
- the pose stops updating;
- the LiDAR sees something within 25 cm ahead, or the sonar reads under 20 cm. If it stays
  blocked for 3 s, it replans around the obstacle;
- it tilts more than 20°. The run is aborted.

Ctrl+C always stops the motors.

## 5. Compare the algorithms on hardware

Every run is appended to `~/.ros/quantum_nav_runs.csv`. Each row records the planned energy,
distance, climb, nodes and runtime, plus what was **measured** while driving: energy (from the
same power model as the web sim, using IMU pitch), driven distance, climb, descent and time.

Use the same start pose and the same goal for each algorithm:
```bash
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=dijkstra   # drive, then reset robot
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=astar
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=qaoa
```
Repeat each one 3–5 times and average the CSV rows.

### Uphill / downhill on the real robot

On a flat lab floor every algorithm sees the same flat cost, so their paths only differ
around obstacles. To test slopes, build a ramp or hill, keeping it under `max_slope_deg`
(15° by default). Then give the planner its shape as a height map: a CSV of heights in metres,
one row per `elevation_resolution` step in y, with row 0 at `elevation_origin_y`, in map
coordinates.

```bash
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=auto \
    elevation_csv:=~/ramp.csv elevation_resolution:=0.1 elevation_origin_x:=0.0 elevation_origin_y:=0.0
```
Now Dijkstra drives straight over the ramp, while A* and QAOA go around it if that's cheaper.
The CSV shows the measured energy difference.

## Notes
- The Python is 3.6-compatible: no dataclasses, no `from __future__ import annotations`.
  The repo's ROS 2 planners need Python 3.7+ and ROS 2, which is why this package exists.
- The planning core (`scripts/nav_core.py`) is pure Python. It's unit-tested in
  `tests/test_arjuna_nav_core.py`, which also passes under Python 3.6.
- A 20 m × 20 m map at 0.1 m resolution plans in about 0.4–0.7 s per algorithm on a laptop.
  Expect several times that on the Jetson Nano, so `auto` takes a few seconds before moving.
- `robot/COLCON_IGNORE` keeps the repo's ROS 2 `colcon build` from picking this ROS 1 package up.
