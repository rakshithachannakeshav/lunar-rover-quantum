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

The navigator needs nothing beyond `rospy` and the standard message packages. There is no pip
install. The depth-camera terrain detector (section 6) also needs `python3-numpy` and `depthai`;
without them (or without the camera) everything else still works, with the camera features off.

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

### Drive from a web page (goal or your own route)

`web/rover.html` controls the robot from any browser on the same network. It shows the
map, the robot, live LiDAR hits and the path being driven.

One-time install on the Jetson: `sudo apt install ros-melodic-rosbridge-server`.

```bash
roslaunch arjuna_quantum_nav quantum_nav.launch algorithm:=astar max_linear:=0.10 web:=true
hostname -I        # the Jetson's IP address
```
On the laptop, open `web/rover.html` (double-click it; no server needed). Enter
`ws://<jetson-ip>:9090` and press **Connect**.

| Mode | You click | The robot |
|---|---|---|
| **Goal** | one point | plans its own route there (`/move_base_simple/goal`, same as RViz) |
| **Route** | points in order | drives through each point in turn, planning each leg round mapped obstacles (`/quantum_nav/route`, `nav_msgs/Path`) |

Press **Send** to start. Drag pans, the mouse wheel zooms, and Backspace undoes a point.
Both modes keep LiDAR obstacle avoidance on. **STOP** (or the Space key) publishes
`/quantum_nav/cancel`, which stops the robot and drops the goal or route. It also sends a zero
`/cmd_vel`. If the pose topic or laser offset differs from the defaults, set them under
**Topics and settings**.

From the command line, the same route looks like this:
```bash
rostopic pub -1 /quantum_nav/route nav_msgs/Path "{header: {frame_id: map}, poses: [
  {pose: {position: {x: 1.0, y: 0.0}}}, {pose: {position: {x: 1.0, y: 1.0}}}]}"
rostopic pub -1 /quantum_nav/cancel std_msgs/Empty "{}"
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

## 6. The lunar arena: craters, rocks and the table edge

The arena is a raised table, 2.21 m x 2.21 m (87 in), dark grey, with a low black border wall.
Driving off the edge drops the rover. It holds 3 craters (octagonal **raised rims**, about 2 cm
high, 25-45 cm across, flat floor inside, **not holes**) and 4 rocks (crumpled shiny plastic,
20-30 cm tall). The rover must never drive over a rim or into a crater, so each whole crater
(rim and inside) is blocked, together with the rocks, the border and anything with no floor.

```
                       OAK-D Lite depth ──► crater_detector_node ──► /terrain/detections (JSON)
                                                                              │
RPLidar /scan (rocks, 18 cm up) ─────────────────────────────┐                ▼
BNO055 /imu/data ──► bump detector (wheel on a rim) ────────►  quantum_nav_node ──► /cmd_vel
/robot_pose_ekf/odom_combined ───────────────────────────────┘     │  world-frame terrain map
                                                                   ├─► /quantum_nav/terrain (web page)
                                                                   └─► /quantum_nav/terrain_markers (RViz)
```

| Sensor | What it sees | What it cannot see |
|---|---|---|
| RPLidar, 18 cm up, mounted backwards (`laser_yaw_offset_deg:=180`, now the default) | most rocks | crater rims (2 cm), the border wall, shiny plastic that scatters the beam |
| OAK-D Lite stereo depth | crater rims, rocks, border, the table edge | anything outside its view (about 0.3-1.2 m ahead, set by your mount) |
| IMU | a short pitch/roll **bump** when a wheel climbs a rim | craters before the wheel is on them |

### What the detector does (`terrain_core.py`)
1. Depth pixels become 3-D points; their height above the calibrated floor plane is measured.
   Speckle spikes (a pixel far from its neighbours' median) are dropped.
2. Points of the last 3 frames are moved into the current robot frame with the odometry pose
   and binned into a 2 cm bird's-eye grid. Each cell keeps the median (and upper quartile)
   height. Heights are taken relative to the median floor level nearby, so a slightly wrong
   plane or a pitching rover does not shift them.
3. **Crater rim:** a thin ridge `rim_min` (default 1 cm) to `tall_min` (5 cm) above the floor.
   Gaps in the ring are bridged, a circle is fitted, and the **whole circle is filled**
   (rim + inside), also when only part of the ring is in view. A short arc (under about 60
   degrees) cannot give a radius, so just the arc is blocked until more of it is seen.
4. **Rock / border wall:** cells more than `tall_min` above the floor. A long thin cluster is
   the border wall. From a low camera only the front face of a rock is visible, so 15 cm of
   hidden floor behind it is blocked as well (the LiDAR gives the rest of the outline).
5. **Invalid depth** is handled along each camera ray: a gap in or right in front of a rock is
   part of the rock; a gap behind a rock is its shadow (ignored); a long gap between floor
   cells is a rock that returned no depth (blocked, first 30 cm); a gap that runs on to the end
   of the view after solid floor is **no floor** = the table edge (blocked); a few scattered
   missing pixels are noise (ignored).
6. In the navigator, detections are moved into the **world frame** with the pose of the frame
   (corrected for the camera delay), merged and kept for the whole run. A crater seen again
   updates one circle (the view with the most rim wins). A cell must be seen twice (three times
   for no-floor) before it blocks. A new object lying on the remaining path triggers a replan.
   Craters and rocks are added to the grid together with the LiDAR obstacles at every plan.

### Object sizes and ids (`terrain_objects.py`)
Every detection is also reported to you as an **object** with its size, on `/terrain/objects`
(JSON), as one line per object in `/terrain/status`, as RViz labels on `/terrain/object_markers`,
and in the web page's **Objects** list:

```
rock #2: 31 x 27 cm, 24 cm tall, 0.82 m ahead
crater #1: 34 cm across (inner 29 cm), rim 2.0 cm high, 0.55 m ahead-left
rock #3: 28 x 6 cm (back not seen), >=16 cm tall, 0.95 m ahead
```

- **Sizes come from the 3-D points**, not the 2 cm cells. Rock or wall: the smallest rectangle
  round the points above the floor (length = long side, width = short side), and the top of
  the points above the local floor. Crater: rim height, and outer / inner rim diameter
  measured round the fitted centre.
- **What the camera cannot see is flagged, not guessed.** A camera that is lower than a rock
  never sees the rock's back, and one whose view ends below the rock's top cannot see the top.
  Then the line says `(back not seen)` or `>=` and the JSON has `footprint_complete: false` /
  `height_is_min: true`. The arena's rocks are 20-30 cm tall, so expect `>=` heights for them
  unless the camera is mounted higher than the rock. Craters and low rocks are measured fully.
- **Ids stay the same** while the rover drives. Objects are kept in the map frame and matched
  by class and position. Sizes are the median of recent views, preferring complete ones. An
  object the camera is looking at but no longer finds is dropped after
  `object_miss_frames` (6) frames. One that is out of view is kept, shown greyed out.
- In simulation (tests/test_arjuna_terrain_objects.py) the sizes are within 3 cm (footprint)
  and 2 cm (height) of the truth whenever the whole object was in view. Rocks 4 cm apart side
  by side, or 8 cm one behind the other, are two objects.
- `measure_objects:=false` switches the sizes off. `/terrain/detections`, which the navigator
  plans with, is identical either way. On a desktop the sizes add about 5-8 ms per frame. The
  detector logs its frame rate and time per frame every 30 s.
- `use_preview:=true` (off by default) also publishes a 2 fps JPEG from the colour camera with
  each object outlined and labelled on `/terrain/preview/compressed`, shown on the web page. It
  needs `python3-opencv`. It runs in its own thread and drops frames rather than slowing the
  detector. If the colour camera cannot be opened, the detector runs without it. The overlay
  uses the factory calibration between the colour and depth cameras, so it can be off by a few
  pixels.

**Every command for the rover tests, in order, is in
[`docs/ROVER_TEST_CHECKLIST.md`](../../docs/ROVER_TEST_CHECKLIST.md). The steps below explain
what each one is for.**

### Pre-flight, every session
1. Copy the package to the Jetson. Files saved on Windows have CRLF line endings, which break
   `rosrun`: run `sed -i 's/\r$//' ~/arjuna_ws/src/arjuna_quantum_nav/scripts/*.py` and `chmod +x`
   on them after **every** copy, then `catkin_make`.
2. `sudo apt install ros-melodic-rosbridge-server ros-melodic-map-server python3-numpy`, and
   `depthai` for the camera (already there if Lab 17 works).
3. Kit bringup as in section 2 (`roslaunch arjuna Arjuna.launch`). **Do not** start the kit's
   auto-nav node. **Stop `Arjuna_camera_pub.py`**: only one program can open the OAK-D.
4. **IMU calibration:** slow figure 8 in the air, then put the rover down still and check
   `rostopic echo -n 1 /imu/status`. The calibration value must be **odd** (e.g. 55). An even
   value (e.g. 48) means the magnetometer is not calibrated and the orientation freezes.
5. Put the rover on its start spot, facing the arena's +x, and send `/initialpose` (odometry is
   silent until it is set; repeat before **every** run), then check
   `rostopic echo -n 1 /robot_pose_ekf/odom_combined` prints a pose near it. The map frame is
   the arena frame: origin at the corner you measured the layout from.
6. Before every `roslaunch` of ours: `rosparam delete /quantum_nav 2>/dev/null`. ROS remembers the
   last run's goal, and a launch meant to stay still would drive off to it.

### Step 1: floor calibration (once, and after the camera is moved)
Rover on **flat, clear** floor with about 1 m of floor in front of the camera (not the border,
not an object):
```bash
roslaunch arjuna_quantum_nav crater_detector.launch calibrate:=true
```
It prints `camera X m above the floor, tilted Y deg down` and saves `~/.ros/crater_camera.json`.
Check X with a tape measure (within about 1 cm). A wrong height means the camera saw
something that is not floor. Press Ctrl+C when it says it is running.

### Step 2: threshold check (measure mode)
The rim is only 2 cm high, near the stereo noise floor, so measure before trusting it. Rover
standing still on the arena, then
```bash
roslaunch arjuna_quantum_nav crater_detector.launch measure:=true
```
- First with the camera looking at **plain floor**: note `floor_noise_mm_fused_cell`.
- Then looking at a **crater from 0.4-0.8 m** (rim and the floor inside in view): it prints
  `rim_height_mm_median`, `rim_snr`, a verdict, and suggested thresholds (saved to
  `~/.ros/terrain_measure.json`). The verdict is one of:
  - *reliable* (rim >= 4x noise): use `rim_min:=<suggested>`.
  - *marginal* (2.5-4x): use the suggestion, keep craters within about 0.7 m (`rim_range:=0.7`),
    and rely on the IMU bump as the backup.
  - *cannot see the rim reliably*: the camera will not find the craters; the **bump detector
    is then your only crater sensor** (the rover will touch each rim once, back out, and mark
    the crater). Try mounting the camera lower or tilting it down.
- Point it at the **border wall** and at a **rock** to see their heights; `tall_min` (default
  5 cm) must be below the wall and above the noise.
Pass what you learn to the real run: `arena.launch rim_min:=0.012 tall_min:=0.06`.
Re-run the calibration (step 1) whenever the camera mount is moved.

### Step 3: first test, standing still
Wheels do not need to move for any of this.
```bash
rosparam delete /quantum_nav 2>/dev/null           # forget the last run's goal
roslaunch arjuna_quantum_nav arena.launch use_map:=false   # no goal: the rover stays put
rostopic echo /terrain/status                      # what the camera sees right now
```
Open RViz (fixed frame `map`), add a `MarkerArray` on `/quantum_nav/terrain_markers`, and open
`web/rover.html` (default laser offset is now 180). Then:
1. Empty floor in view: `clear`, no markers.
2. Put a **crater** 0.5-0.8 m ahead: an orange disc appears (web page: dashed ring) at the
   right place, about the crater's size. Measure its position with a tape; it should be within
   a few cm. Turn the rover so the crater is only half in view: the disc stays whole.
3. Put a **rock** in view: red cells on its front face (and a bit behind). Try the shiny side.
4. Look at the **border wall** and the **table edge** (rover on the table, wheels held or
   blocked): purple / black cells. If the edge is not seen, lower `tall_min` or check the
   camera's `floor X-Y m ahead` range in the detector's log line.
5. Click a goal ON a detected crater or rock on the web page: it must refuse with a message.
6. **Bump test:** lift the front wheels onto a crater rim by hand while sending a goal that
   needs forward motion, or drive over a rim at `max_linear:=0.05` with an operator ready on
   STOP. The log should say `Bump (nose_up ...)`, the rover backs out 15 cm and marks a
   crater-sized disc. Adjust `bump_pitch_deg` if it triggers on nothing (raise) or misses the rim
   (lower, minimum useful is about 1.5 deg).

### Step 4: a full run with live detection (the main test)
Do it only after step 2 shows the rim is visible and step 3 shows craters and rocks correctly.
Start in one corner, goal in the opposite corner, craters and rocks in between, no map: the
rover only knows what the camera and LiDAR show while it drives.
```bash
rosparam delete /quantum_nav 2>/dev/null
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=true algorithm:=astar \
        max_linear:=0.06 goal_x:=1.9 goal_y:=1.9
rostopic echo /quantum_nav/status        # replanning / goal reached
```
Pass: reaches the goal, never drives onto a rim, never touches a rock or the border. Repeat with
`dijkstra`, `qaoa`, `auto` (rover back on the start spot and `/initialpose` each time). You can
also send goals and routes from the web page (it starts rosbridge). Switches:
`use_camera:=false` (no detector at all), `use_craters:=false` (the navigator ignores the camera
terrain and the bump), `calibrate:=true`, `use_map:=true` (known map, see step 5).
Start slow: `max_linear:=0.06`. **Keep a hand on STOP** (Space on the web page) for the first
runs near the table edge. The navigator also aborts with `ABORTED: at the table edge` if the
pose gets within 6 cm of the arena boundary.

What it does on the way: plans on the grid (arena rectangle blocked outside, border strip
blocked, LiDAR rocks, camera terrain), drives, and replans when a new crater/rock appears on
the remaining path or after a bump. If no path fits, it retries with down to 80% of the
clearance and says so in the log.

**Clearances.** Every blocked object is grown by `robot_radius + crater_margin` (default
0.12 + 0.02 m in `arena.launch`). The arena is small, so set `robot_radius` to half the rover's
**diagonal** and no more. If the planner reports `no path`, run the offline check below with a
smaller radius, or measure the rover. `soft_margin` (0.10) adds a cost band so paths keep away
from objects where there is room, without closing gaps.

**Why `avoid_obstacles:=false` in the arena.** The kit's reactive swerve turns "to the open
side" and knows nothing about the table edge or a crater, so in simulation it drove off the
table. With it off, the rover stops if the LiDAR sees something within 18 cm ahead and replans
after 3 s. Set `avoid_obstacles:=true` only for experiments.

### Step 5: known layout and planner comparison
Measure the position of each crater and rock from the arena's origin corner, put them in a
layout file (see `config/arena_layout_example.json`: inches or metres, sizes across the widest
point), and make the map:
```bash
rosrun arjuna_quantum_nav make_arena_map.py ~/my_layout.json --out ~/arena_map --preview
rosrun arjuna_quantum_nav compare_plans.py ~/my_layout.json \
        --start 0.25,0.25 --goal 1.95,1.95 --robot-radius 0.12 --margin 0.02   # offline, no robot
```
The map includes the border strip and everything outside the table as blocked. `compare_plans`
shows planned energy, length, search nodes and planning time per algorithm, and tells you if a
gap is too narrow for the robot radius.

The map is served by the **kit's** launch: restart the kit with it (Ctrl+C the old one first;
only ONE map_server may run) and do **not** pass a map to `arena.launch`, or two map servers
would both publish `/map`:
```bash
roslaunch arjuna Arjuna.launch map_file:=$HOME/arena_map.yaml
```
Then the IMU check and `/initialpose` again, with the start spot measured from the same origin
corner as the layout. In RViz the map must line up with the LiDAR dots; if it is offset or
rotated, fix the rover's start spot and heading, not the map. Drive the same start and goal with
each planner on the full known layout:
```bash
rosparam delete /quantum_nav 2>/dev/null
roslaunch arjuna_quantum_nav arena.launch use_map:=true use_camera:=false algorithm:=dijkstra \
        goal_x:=1.95 goal_y:=1.95            # reset the rover to the same start pose each run
roslaunch ... algorithm:=astar  ...
roslaunch ... algorithm:=qaoa   ...
roslaunch ... algorithm:=auto   ...
rosrun arjuna_quantum_nav compare_runs.py        # averages ~/.ros/quantum_nav_runs.csv
```
Repeat each 3-5 times. `use_camera:=false` is a fair way to compare planners when the map is
known; to also test detection with the known map, use `use_camera:=true` (craters then appear
twice, which is fine). On flat ground the planners often pick the same route, which is a valid
result; the hill (below) is where they should differ.
Every run appends planned and **measured** energy, distance and time to
`~/.ros/quantum_nav_runs.csv`; `compare_runs.py` averages them per algorithm and outcome
(compare only `reached` rows).

### Open checks: what the code covers and what needs the rover
Checklist section letters (A-K) refer to `docs/ROVER_TEST_CHECKLIST.md`.

| Item | Status |
|---|---|
| A. Straight-test distance (tape vs pose) | **Rover only.** `drive_test.py _mode:=straight _distance:=1.0`, measure with a tape; fix the kit's tick calibration if more than ~10% off. Turns should pass: heading comes from the IMU. |
| Reversing when a box is ~15 cm ahead, none when the rear is blocked | Logic unit-tested and run in the simulator (`test_avoider_on_reverses_...`). **Rover:** put a box 15 cm ahead, then also a wall behind. |
| `avoid_obstacles:=false` (old stop-and-wait) | Simulated run passes (`test_old_stop_and_wait_...`); it is the `arena.launch` default. **Rover:** check it pauses within 18 cm and replans after 3 s. |
| Algorithm comparison, 3-5 runs each | Offline: `compare_plans.py`; simulated for all three. **Rover only:** the real runs into `~/.ros/quantum_nav_runs.csv`, then `compare_runs.py` (step 5). |
| Map the terrain, then `use_map:=true` | Map from `make_arena_map.py` (the real map you saved by driving is not needed); use it as in step 5. |
| Ramp + elevation file | `config/hill_example.csv` (14 cm hill): `compare_plans.py layout.json --start 0.3,0.3 --goal 1.9,1.9 --elevation config/hill_example.csv` shows A* going round it where Dijkstra climbs (tested). **Rover:** build the hill and pass `elevation_csv:=`. |
| Web control: frequent `no path` after the first goal | **Needs your Terminal-2 output** (the lines around `No path found` and `status:`). Likely causes, in order: a stale pose after the first goal, the rover standing inside an inflated zone, or too large `robot_radius`. The navigator now retries with 80% clearance and logs it. |
| B. Does the LiDAR see the clear table edge? | **Rover only.** Rover still on the table, ~30 cm from the edge, facing it: `drive_test.py _mode:=sensors _laser_yaw_offset_deg:=180`. `lidar front` about 0.30 m means the edge/border is visible; much more or no value means it is not, and the camera's no-floor detection is what covers the edge (keep goals 30 cm inside the border). |
| C-D2. Camera calibration, rim visibility, live detection run | **Rover only.** Steps 1, 2 and 4 above. |

### Parameters worth knowing
| Where | Parameter | Default | Meaning |
|---|---|---|---|
| detector | `rim_min` | 0.010 m | lowest ridge counted as a crater rim (set from measure mode) |
| detector | `tall_min` | 0.05 m | above this = rock / wall |
| detector | `rim_range` | 0.90 m | rims are searched only this far ahead |
| detector | `x_max`, `half_width` | 1.2, 0.6 m | watched floor |
| detector | `camera_forward` | 0.10 m | camera ahead of the rover's centre |
| detector | `frames`, `pixel_step` | 3, 3 | frames fused / sampling; raise `pixel_step` if the Nano is slow |
| navigator | `robot_radius`, `crater_margin` | 0.12, 0.02 m | clearance round every blocked object |
| navigator | `bump_pitch_deg`, `bump_roll_deg` | 2.5, 3.0 | IMU jolt that counts as a rim (0 = bump detector off) |
| navigator | `bump_crater_radius` | 0.20 m | crater size assumed when only a bump marks it |
| navigator | `confirm_crater/rock/border/nofloor` | 2/2/2/3 | sightings before it blocks |
| navigator | `soft_margin` | 0.10 m | extra-cost band keeping paths off objects |
| navigator | `edge_stop` | 0.06 m | abort this close to the arena boundary |

### What was only tested in simulation, and what you must still verify
Everything below ran without a rover, a camera or ROS: unit tests on synthetic depth images, and
the real `quantum_nav_node.py` driven against stub `rospy` and a simulated rover and arena
(`tests/arena_sim.py`, `tests/test_arjuna_arena_sim.py`). In the simulated arena (3 craters, 4
rocks) each of Dijkstra, A* and QAOA reached the goal without crossing a rim or touching a rock.

**Not verified on hardware - please check:**
- **Whether the 2 cm rim is visible to your OAK-D at all.** The simulated stereo noise (about
  1.5 mm on the floor) is my assumption. Real noise is probably several times larger. Step 2
  decides this, and the bump detector is the fallback.
- The camera geometry. Tests assume 25 cm up, tilted 30 degrees down; the floor range really
  watched depends on your mount (the detector logs it). The calibration code is unchanged but
  never ran on a real floor here.
- `crater_detector_node.py` and the depthai pipeline were not run (no camera, no depthai). The
  launch files are only checked as valid XML and for matching argument names.
- Shiny plastic: simulated as 35% dropouts plus spikes. Real plastic will differ; watch the rocks
  in step 3.
- IMU bump size and threshold (depends on wheel size and wheelbase), BNO055 noise, and the
  back-out distance on the real floor.
- Rover size, speed, turning accuracy and heading drift. In the tests the pose is exact (the
  simulator can add drift, but the committed tests do not) and the rover is 9 cm in radius with
  the planner told 10 + 1.5 cm. Wheel odometry that undercounts turns about 3x makes real tracking
  worse; keep the IMU heading calibrated.
- Run time on the Jetson Nano: the detector takes about 50 ms per frame on a development PC and will be
  several times slower on the Nano. Lower `rate:=`, raise `pixel_step:=`, or set `frames:=2` (arena.launch and crater_detector.launch take all three).
- Python 3.6: all robot scripts are parsed with the 3.6 grammar in the tests, but nothing ran under 3.6.
  numpy calls were kept to ones that exist in numpy 1.13.
- The web page logic runs under Node with a fake DOM in the tests, not in a real browser.
- Whether a layout is passable depends on your `robot_radius`: the 87 x 87 in arena with 7
  objects is tight. The simulated layout had corridors found by search; yours may not.

### Troubleshooting
| Symptom | Likely cause |
|---|---|
| Status `waiting for pose` | set the start pose with 2D Pose Estimate; check the EKF topic |
| Detector log `no camera: ...` | Arjuna_camera_pub.py still running, cable, or depthai missing. The navigator carries on without the camera |
| Craters never appear, floor noise high | run measure mode; rely on the bump |
| Phantom rocks/edges | `confirm_*` too low, or the floor calibration is off: recalibrate on flat clear floor |
| Rover refuses a goal | the point is on a detected object or outside the arena (the message says which) |
| `no path` | clearances too big for the gaps: lower `robot_radius` / `crater_margin`, or `compare_plans.py` |
| Aborts with `kept hitting crater rims` | more than 4 bumps for one goal; check the layout and `bump_pitch_deg` |
| Turns wrongly / LiDAR shows walls behind | `laser_yaw_offset_deg` must be 180 on this rover |
| `rosrun` says bad interpreter / `^M` | scripts copied from Windows: `sed -i 's/\r$//'` on `scripts/*.py`, then `chmod +x` |
| A launch with no goal drives off by itself | ROS remembered the last goal: `rosparam delete /quantum_nav` before every launch |
| Two `/map` publishers, map jumps | only one map_server: start the kit with `map_file:=`, never pass a map to `arena.launch` |
| `/imu/status` calibration value even | magnetometer not calibrated: figure 8 until it is odd |


## Notes
- The Python is 3.6-compatible: no dataclasses, no `from __future__ import annotations`.
  The repo's ROS 2 planners need Python 3.7+ and ROS 2, which is why this package exists.
- The planning core (`scripts/nav_core.py`) is pure Python. It's unit-tested in
  `tests/test_arjuna_nav_core.py`, which also passes under Python 3.6.
- `scripts/crater_core.py` (floor calibration) and `scripts/terrain_core.py` (terrain detection) need
  numpy; `nav_core.py` stays pure Python. Tests: `tests/test_arjuna_terrain_core.py`,
  `tests/test_arjuna_arena_tools.py`, `tests/test_arjuna_arena_sim.py`, `tests/test_rover_web.py`.
- A 20 m × 20 m map at 0.1 m resolution plans in about 0.4–0.7 s per algorithm on a laptop.
  Expect several times that on the Jetson Nano, so `auto` takes a few seconds before moving.
- `robot/COLCON_IGNORE` keeps the repo's ROS 2 `colcon build` from picking this ROS 1 package up.
