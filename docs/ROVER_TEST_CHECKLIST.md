# Rover tests: commands

Every terminal: `source ~/arjuna_ws/devel/setup.bash`
T1 = kit bringup, T2 = test, T3 = helper.
Replace anything in `<...>`, and the pose and goal numbers, with your real values (metres from the arena's origin corner).

**Before EVERY launch in T2** (ROS remembers the last run's goal, and a launch with "no goal" would drive off to it):
```bash
rosparam delete /quantum_nav 2>/dev/null
```

## 0. Setup

Laptop:
```bash
# REPLACE <user> and <jetson-ip>. Copies the updated package to the rover.
scp -r robot/arjuna_quantum_nav <user>@<jetson-ip>:~/arjuna_ws/src/
```
Jetson:
```bash
# Strips Windows line endings (a Windows copy breaks rosrun: most scripts are saved with CRLF), then makes the scripts runnable.
# Run this after EVERY copy from the laptop.
sed -i 's/\r$//' ~/arjuna_ws/src/arjuna_quantum_nav/scripts/*.py
chmod +x ~/arjuna_ws/src/arjuna_quantum_nav/scripts/*.py
# Builds the package so ROS finds the new launch files.
cd ~/arjuna_ws && catkin_make && source devel/setup.bash
# Installs rosbridge (web page), map_server (known map) and numpy (camera code).
sudo apt install -y ros-melodic-rosbridge-server ros-melodic-map-server python3-numpy
# Confirms numpy and the camera library are installed.
python3 -c "import numpy, depthai; print('ok')"
# Frees the OAK-D: only one program can open it.
pkill -f Arjuna_camera_pub.py
```

T1: start the kit bringup (LiDAR, IMU, wheels, EKF, RViz). Not the kit's auto-nav node.
```bash
# Sections A-H: the kit's default map is loaded but NOT used (every T2 command below has use_map:=false).
roslaunch arjuna Arjuna.launch
```
(For sections I-K, T1 is restarted with the arena map: see section I.)

T2, every session:
```bash
# Do a slow figure-8 in the air, put the rover down still, then check: the calibration value must be ODD (e.g. 55).
# Even (e.g. 48) = magnetometer not calibrated: orientation freezes. Repeat the figure-8.
rostopic echo -n 1 /imu/status
```

T2, before EVERY run (put the rover on its start spot first, facing the arena's +x direction):
```bash
# REPLACE x, y with the rover's real start spot. Odometry is silent until this is sent.
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.25, y: 0.25, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}"
# Must print a pose near (x, y). If it hangs, the pose is not live yet: send /initialpose again.
rostopic echo -n 1 /robot_pose_ekf/odom_combined
```

## A. Straight and turn distance
```bash
# Drives 1 m (needs ~1.2 m clear ahead) so you can compare the tape with the pose and check the wheel calibration.
rosrun arjuna_quantum_nav drive_test.py _mode:=straight _distance:=1.0 _laser_yaw_offset_deg:=180
# Turns 90 degrees to check the turn calibration.
rosrun arjuna_quantum_nav drive_test.py _mode:=turn _angle_deg:=90 _laser_yaw_offset_deg:=180
```
Pass: within 10% of the tape. (Turning is expected to pass: heading comes from the IMU. Wheel odometry alone undercounts turns ~3x.)

## B. LiDAR and the table edge (rover still, on the table, ~30 cm from the edge, facing it)
```bash
# Prints LiDAR, IMU and sonar readings. "lidar front" ~0.30 m = the edge/border is visible; much more or "—" = it is not.
rosrun arjuna_quantum_nav drive_test.py _mode:=sensors _laser_yaw_offset_deg:=180
```
If the edge is invisible, stand at the edge on every run and keep goals 30 cm inside the border.

## C. Camera calibration and rim visibility (run each, read the result, Ctrl+C before the next)
```bash
# Flat clear floor, 1 m ahead. Fits the floor plane; compare the printed camera height with a tape (within 1 cm).
roslaunch arjuna_quantum_nav crater_detector.launch calibrate:=true
# Plain floor in view: measures the stereo noise.
roslaunch arjuna_quantum_nav crater_detector.launch measure:=true
# Crater 0.4-0.8 m ahead, whole rim in view: measures the rim height and says whether a 2 cm rim is visible.
roslaunch arjuna_quantum_nav crater_detector.launch measure:=true
# Border wall in view: measures the wall height for the tall_min setting.
roslaunch arjuna_quantum_nav crater_detector.launch measure:=true
# A rock in view: measures the rock height.
roslaunch arjuna_quantum_nav crater_detector.launch measure:=true
```
Result is saved in `~/.ros/terrain_measure.json`. If the verdict suggests a threshold, add it to later runs, e.g. `rim_min:=0.012`.
Re-run the `calibrate:=true` step whenever the camera mount is moved.

## D. Terrain detection, standing still
```bash
# T2: clears any old goal, then starts the detector and navigator with no goal, so the rover stays put.
rosparam delete /quantum_nav 2>/dev/null
roslaunch arjuna_quantum_nav arena.launch use_map:=false
# T3: shows what the camera sees right now.
rostopic echo /terrain/status
```
RViz: add MarkerArray `/quantum_nav/terrain_markers`.
Web page: open `web/rover.html`, REPLACE `<jetson-ip>` in `ws://<jetson-ip>:9090`, then check **Topics and settings → Laser yaw offset = 180** (if you change it: Disconnect, Connect).
Check: empty floor, crater, half-visible crater, rock, border wall, table edge, click a goal on a crater (should be rejected).

## D2. Full arena run with live crater and rock detection (the main test)
Do this only after C passes (rim visible in the measurement) and D shows craters and rocks correctly.
Start spot in one corner, goal in the opposite corner, with craters and rocks in between.
```bash
# T2: clears any old goal. Camera ON, no map: the rover only knows what the camera and LiDAR see while driving.
rosparam delete /quantum_nav 2>/dev/null
# REPLACE the goal (metres from the origin corner, at least 30 cm inside the border).
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=true algorithm:=astar max_linear:=0.06 goal_x:=1.9 goal_y:=1.9
# T3: what it is doing ("crater on the path ahead; replanning", "goal reached", ...).
rostopic echo /quantum_nav/status
# T4 (optional): what the camera sees at the same time.
rostopic echo /terrain/status
```
Watch in RViz (MarkerArray `/quantum_nav/terrain_markers`, Path `/quantum_nav/path`) or on the web page.
Pass: reaches the goal, never drives onto a crater rim, never touches a rock or the border. Note how far ahead each crater was first marked.
Then repeat with `algorithm:=dijkstra`, `algorithm:=qaoa` and `algorithm:=auto` (rover back on the start spot and `/initialpose` before each run).

Live detection + known map (planner knows the layout from the start, the camera adds anything new):
```bash
# Needs T1 started with the arena map (see section I).
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=true algorithm:=astar max_linear:=0.06 goal_x:=1.9 goal_y:=1.9
```

## E. IMU bump
```bash
# REPLACE goal so a crater lies between the rover and it. Camera off, slow: tests only the bump backup.
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=false max_linear:=0.06 goal_x:=1.5 goal_y:=0.7
# Use instead if it reports bumps on flat floor: less sensitive.
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=false max_linear:=0.06 bump_pitch_deg:=3.5 goal_x:=1.5 goal_y:=0.7
# Use instead if it climbs a rim without reporting a bump: more sensitive.
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=false max_linear:=0.06 bump_pitch_deg:=1.8 goal_x:=1.5 goal_y:=0.7
```
Keep a hand ready: this test drives at a crater on purpose.

## F. Reversing
```bash
# Box 15 cm ahead: it should reverse. Then add a wall behind: it should not reverse, only turn. Avoider is on here only for this test.
roslaunch arjuna_quantum_nav quantum_nav.launch use_map:=false avoid_obstacles:=true use_craters:=false laser_yaw_offset_deg:=180 robot_radius:=0.15 max_linear:=0.08 goal_x:=1.5 goal_y:=0.0
```

## G. avoid_obstacles:=false
```bash
# T2: REPLACE goal. Put a box in the path: it should stop and wait, then replan.
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=false goal_x:=1.8 goal_y:=0.4
# T3: shows "paused: obstacle ..." and the stop distance.
rostopic echo /quantum_nav/status
```

## H. Web control "no path" logs
```bash
# T2: runs with the web server and saves the log. Send two goals from the web page (zoom in first: goals far away give "no path").
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=false web:=true 2>&1 | tee ~/nopath_log.txt
# T3: saves the status messages around the failure.
rostopic echo /quantum_nav/status | tee ~/status_log.txt
```

## I. Layout map
```bash
# Copies the example layout to edit.
cp $(rospack find arjuna_quantum_nav)/config/arena_layout_example.json ~/my_layout.json
# REPLACE the example positions and sizes with your measured ones (arena 2.21 x 2.21 m, origin = the corner you start from).
nano ~/my_layout.json
# Makes the map_server map (arena_map.pgm and .yaml) with the border blocked.
rosrun arjuna_quantum_nav make_arena_map.py ~/my_layout.json --out ~/arena_map --preview
# Checks the layout is passable for your robot size. REPLACE 0.12 with half the rover's diagonal.
rosrun arjuna_quantum_nav compare_plans.py ~/my_layout.json --start 0.25,0.25 --goal 1.95,1.95 --robot-radius 0.12 --margin 0.02
```
T1: restart the kit with the arena map (Ctrl+C the old one first). Only ONE map_server may run:
```bash
roslaunch arjuna Arjuna.launch map_file:=$HOME/arena_map.yaml
```
Then IMU check and `/initialpose` again (section 0), with the start spot measured from the same origin corner as the layout.
```bash
# Drives on the known map. REPLACE the goal.
# Do NOT pass map_file to arena.launch: it would start a second map_server and both would publish /map.
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false algorithm:=astar goal_x:=1.95 goal_y:=1.95
```
RViz should show the arena map with the craters and rocks blocked, lined up with the LiDAR dots. If the dots and the map are offset or rotated, the start pose does not match the layout's origin: fix the rover's start spot/heading, not the map.

## J. Algorithm comparison (3-5 runs each; rover back on the start spot and /initialpose before each run)
```bash
# T2, one per run: same start and goal for every algorithm so the CSV rows are comparable. REPLACE the goal.
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false algorithm:=dijkstra goal_x:=1.95 goal_y:=1.95
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false algorithm:=astar goal_x:=1.95 goal_y:=1.95
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false algorithm:=qaoa goal_x:=1.95 goal_y:=1.95
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false algorithm:=auto goal_x:=1.95 goal_y:=1.95
# T3, after each run: waits for the goal, then Ctrl+C in T2 so the next run starts clean.
rostopic echo /quantum_nav/status | grep -m1 "goal reached"
# After all runs: averages ~/.ros/quantum_nav_runs.csv per algorithm.
rosrun arjuna_quantum_nav compare_runs.py
# Keeps a dated copy of the results.
cp ~/.ros/quantum_nav_runs.csv ~/runs_$(date +%F).csv
```
On flat ground the planners often pick the same route; that is a valid result. Section K is where they should differ.

## K. Hill / ramp
```bash
# Offline: shows A* going round the example hill where Dijkstra climbs it.
rosrun arjuna_quantum_nav compare_plans.py ~/my_layout.json --start 0.3,0.3 --goal 1.9,1.9 --elevation $(rospack find arjuna_quantum_nav)/config/hill_example.csv
# On the rover, after writing ~/my_hill.csv (heights in metres, one row per 0.1 m in y). Ramp under 15 deg.
# bump_pitch_deg:=0 stops the ramp being read as a crater rim.
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false bump_pitch_deg:=0 algorithm:=dijkstra goal_x:=1.9 goal_y:=1.9 elevation_csv:=$HOME/my_hill.csv
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false bump_pitch_deg:=0 algorithm:=astar goal_x:=1.9 goal_y:=1.9 elevation_csv:=$HOME/my_hill.csv
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false bump_pitch_deg:=0 algorithm:=qaoa goal_x:=1.9 goal_y:=1.9 elevation_csv:=$HOME/my_hill.csv
# Compares measured energy of the three planners on the hill.
rosrun arjuna_quantum_nav compare_runs.py
```

## Stop at any time
Ctrl+C in T2, the STOP button on the web page, or from any terminal:
```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

## Bring back
`~/.ros/terrain_measure.json`, `~/.ros/crater_camera.json`, `~/runs_*.csv`, `~/nopath_log.txt`, `~/status_log.txt`