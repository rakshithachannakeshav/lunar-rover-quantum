# Arjuna rover: run it and drive it from the laptop web page

Rover = Jetson Nano (ROS Melodic). Laptop = any browser on the **same Wi-Fi** as the rover.
In every Jetson terminal, run first:
```bash
source ~/arjuna_ws/devel/setup.bash
```
T1 / T2 / T3 = Jetson terminals.

---

## 1. One-time setup

**Jetson: get the latest code and build it** (repeat whenever the code changes):
```bash
cd ~/lunar-rover-quantum && git pull
rm -rf ~/arjuna_ws/src/arjuna_quantum_nav
cp -r ~/lunar-rover-quantum/robot/arjuna_quantum_nav ~/arjuna_ws/src/
sed -i 's/\r$//' ~/arjuna_ws/src/arjuna_quantum_nav/scripts/*.py     # removes Windows line endings
chmod +x ~/arjuna_ws/src/arjuna_quantum_nav/scripts/*.py
cd ~/arjuna_ws && catkin_make && source devel/setup.bash
```

**Jetson: install the web bridge and helpers** (once):
```bash
sudo apt install -y ros-melodic-rosbridge-server ros-melodic-map-server python3-numpy
```

**Laptop:** have the repo (`git pull`), or at least the `web/` folder (`web/rover.html` + `web/js/`).

---

## 2. Every session (Jetson)

**T1: start the kit** (LiDAR, IMU, wheels, position estimate, RViz):
```bash
roslaunch arjuna Arjuna.launch
```
Wait for `RPLIDAR health status : OK`. Do not start the kit's own auto-nav node.

**T2: calibrate the IMU.** Lift the rover, trace a slow figure-8 in the air for ~30 s, put it down still, then:
```bash
rostopic echo -n 1 /imu/status
```
"Calibration status" must be an **odd** number (e.g. 55). Even = repeat the figure-8 (otherwise the heading freezes).

**T2: set the start position.** Put the rover on its start spot facing +x, then (replace x, y; 0, 0 if you don't use arena coordinates):
```bash
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.25, y: 0.25, z: 0}, orientation: {w: 1}}}}"
rostopic echo -n 1 /robot_pose_ekf/odom_combined
```
The second command must print a pose. If it hangs, send `/initialpose` again.

**T2: start the navigator with the web bridge.** Clear old goals first (ROS remembers them):
```bash
rosparam delete /quantum_nav 2>/dev/null
roslaunch arjuna_quantum_nav arena.launch use_map:=false use_camera:=false algorithm:=astar
```
- Web bridge (rosbridge, port 9090) starts automatically.
- `use_camera:=true` turns on crater/rock detection with the OAK-D (stop `Arjuna_camera_pub.py` first: `pkill -f Arjuna_camera_pub.py`).
- `algorithm:=` `astar` | `dijkstra` | `qaoa` | `auto`.
- Without the arena package, the plain navigator works too:
  ```bash
  roslaunch arjuna_quantum_nav quantum_nav.launch use_map:=false laser_yaw_offset_deg:=180 robot_radius:=0.15 max_linear:=0.08 web:=true
  ```

**T3: find the rover's IP address** for the web page:
```bash
hostname -I
```
Use the first address, e.g. `10.26.117.143`.

---

## 3. Laptop: control it from the web page

1. Double-click **`web/rover.html`** (opens in the browser; no server or internet needed).
2. **Connection** box: type `ws://<rover-ip>:9090`, e.g. `ws://10.26.117.143:9090`, click **Connect**. The dot turns green.
3. **Topics and settings**: check **Laser yaw offset (deg) = 180** (this rover's LiDAR faces backwards). If you change it: **Disconnect**, then **Connect**.
4. Zoom in on the rover: **◎** (centre on rover), then **+** a few times. The white arrow is the rover; red dots are what the LiDAR sees.
5. Drive:
   - **Goal** mode: click one point → **Send**. The rover plans its own route there.
   - **Route** mode: click several points in order → **Send**. It drives through each one.
   - **Undo** removes the last point, **Clear** removes all.
6. Watch **Rover status** (e.g. `driving astar (waypoint 3/7)`, `goal reached`) and **Last plan** (energy, distance, time per algorithm).
7. **Objects (camera)** lists every crater, rock, wall and table edge the OAK-D found, with its id, length × width × height and distance. Each one is outlined and labelled on the map, and clicking a row highlights it there. `≥` and "back not seen" mean the camera could not see all of the object yet.
8. Optional colour view: start the navigator with `use_preview:=true` (needs `python3-opencv`). The **Camera preview** box then shows the camera image with the objects drawn on it.

**STOP:** the red **STOP** button or the **Space** key.

Rules for goals: open floor, at least 30 cm from red dots, within ~10 m of the rover, and inside the arena.

---

## 4. Drive by hand (no planner)

Stop the navigator first (Ctrl+C in T2), then either:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py      # i forward, , back, j/l turn, k stop, z slower
```
or single commands (Ctrl+C stops):
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.08}}'      # forward
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{angular: {z: 0.4}}'      # turn left
```

---

## 5. Stop, results and restart

**Emergency stop from any terminal:**
```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

**Run log** (every run: planned + measured energy, distance, time):
```bash
rosrun arjuna_quantum_nav compare_runs.py          # average per algorithm
cat ~/.ros/quantum_nav_runs.csv                    # raw rows
```

**Restart ROS completely** (stuck nodes, LiDAR timeout):
```bash
rosnode kill -a
killall -9 roslaunch rosmaster roscore rplidarNode 2>/dev/null
```
Then start again from section 2.

---

## 6. Quick fixes

| Problem | Fix |
| --- | --- |
| Status `waiting for pose` | Send `/initialpose` again; check `rostopic hz /odom_data_quat` shows ~30 Hz |
| Status `idle (last: no path)` | Goal too far / on or behind an obstacle / outside the arena. Zoom in and pick open floor |
| Rover drives off as soon as the navigator starts | An old goal was remembered: `rosparam delete /quantum_nav`, relaunch |
| Obstacles shown behind the rover on the page | Laser yaw offset = 180 in Topics and settings, then Disconnect / Connect |
| **Connect** fails | Same Wi-Fi? T2 still running? Address = `ws://` + `hostname -I` + `:9090` |
| `RESULT_OPERATION_TIMEOUT` (LiDAR) | Another launch is using it: restart ROS (section 5); replug the LiDAR USB |
| Heading wrong / turns wrong | IMU not calibrated: figure-8 until `/imu/status` is odd |
| `rosrun` says "No such file" for a script | Windows line endings: run the `sed` line from section 1 |