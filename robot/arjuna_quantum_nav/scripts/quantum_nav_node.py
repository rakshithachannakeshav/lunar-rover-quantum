#!/usr/bin/env python3
"""ROS Melodic node: plan with Dijkstra / A* / QAOA and drive the Arjuna Edu Kit.

Runs ON the Jetson Nano alongside the kit's own bringup, which already provides:
    /scan                              RPLidar C1 (rplidar_ros)
    /imu/data                          BNO055 (imu_bno055)
    /robot_pose_ekf/odom_combined      servo ticks + IMU fused by robot_pose_ekf
    /map                               map_server (optional)
    /cmd_vel  -> Arjuna_ticks_pub.py   turns Twist into STS servo wheel speeds

This node:  goal (RViz "2D Nav Goal" or ~goal_x/~goal_y, optionally from ~start_x/~start_y)
            -> plan on the map with the chosen algorithm ("auto" runs all three
               and drives the best: lowest energy, ties -> fastest)
            -> follow the path on the fused pose, publishing /cmd_vel
            -> obstacle ahead (LiDAR/sonar): turn towards the open side, drive past,
               then replan from there with the obstacle marked on the grid
            -> stops for: stale pose, excessive tilt, stuck avoiding too long
            -> logs each run (planned + measured energy, climb, descent) to CSV.

Planning/control logic lives in nav_core.py (pure Python 3.6, unit-tested).
"""

import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String

import nav_core


class QuantumNavNode(object):
    def __init__(self):
        p = rospy.get_param
        self.algorithm = p("~algorithm", "auto")
        self.max_linear = float(p("~max_linear", 0.12))
        self.max_angular = float(p("~max_angular", 0.6))
        self.goal_tolerance = float(p("~goal_tolerance", 0.15))
        self.waypoint_tolerance = float(p("~waypoint_tolerance", 0.2))
        self.plan_resolution = float(p("~plan_resolution", 0.1))
        self.robot_radius = float(p("~robot_radius", 0.2))
        self.unknown_is_blocked = bool(p("~unknown_is_blocked", True))
        self.use_map = bool(p("~use_map", True))
        self.map_wait = float(p("~map_wait", 3.0))
        self.max_slope_deg = float(p("~max_slope_deg", 15.0))
        self.pose_timeout = float(p("~pose_timeout", 0.6))
        self.lidar_stop = float(p("~lidar_stop_distance", 0.25))
        self.lidar_half_angle = math.radians(float(p("~lidar_front_half_angle_deg", 30.0)))
        self.laser_yaw_offset = math.radians(float(p("~laser_yaw_offset_deg", 0.0)))
        self.lidar_obstacle_range = float(p("~lidar_obstacle_range", 2.5))
        self.sonar_stop_cm = float(p("~sonar_stop_cm", 20.0))
        self.max_tilt = math.radians(float(p("~max_tilt_deg", 20.0)))
        self.pitch_sign = float(p("~pitch_sign", 1.0))
        self.replan_after = float(p("~replan_after", 3.0))
        # obstacle avoidance (see nav_core.ObstacleAvoider); off -> old stop-and-wait
        self.avoid_obstacles = bool(p("~avoid_obstacles", True))
        self.avoid_params = dict(
            stop_distance=float(p("~avoid_stop_distance", 0.40)),
            clear_distance=float(p("~avoid_clear_distance", 0.60)),
            backup_distance=float(p("~avoid_backup_distance", 0.22)),
            commit_distance=float(p("~avoid_commit_distance", 0.40)),
            rear_clearance=float(p("~avoid_rear_clearance", 0.25)),
        )
        self.replan_after_detour = bool(p("~replan_after_detour", True))
        self.avoid_timeout = float(p("~avoid_timeout", 25.0))
        self.side_clearance = float(p("~avoid_side_clearance", 0.32))
        self.avoid_params["side_clearance"] = self.side_clearance
        self.watchdog = nav_core.ProgressWatchdog(
            travel=float(p("~no_progress_travel", 3.0)))
        self.max_stuck_replans = int(p("~max_stuck_replans", 3))
        self.stuck_replans = 0
        self.rate_hz = float(p("~rate", 10.0))
        self.log_csv = os.path.expanduser(p("~log_csv", "~/.ros/quantum_nav_runs.csv"))
        self.frame_id = p("~frame_id", "map")

        elevation_csv = p("~elevation_csv", "")
        self.elevation = None
        if elevation_csv:
            self.elevation = nav_core.load_elevation_csv(
                os.path.expanduser(elevation_csv),
                float(p("~elevation_resolution", 0.1)),
                float(p("~elevation_origin_x", 0.0)),
                float(p("~elevation_origin_y", 0.0)))
            rospy.loginfo("Loaded elevation model %s", elevation_csv)

        self.meter = nav_core.EnergyMeter(mass=float(p("~mass", 5.0)))

        self.lock = threading.Lock()
        self.pose = None          # (x, y, yaw)
        self.pose_stamp = 0.0
        self.map_msg = None
        self.scan = None
        self.roll = self.pitch = 0.0
        self.sonar_cm = None
        self.goal = None
        self.new_goal = False
        self.follower = None
        self.active = None        # PlanResult being driven
        self.all_results = {}
        self.blocked_since = None
        self.avoider = None
        self.avoiding_since = None
        self._last_index = 0
        self.legs = []            # endpoints still to visit after the current goal
        self.last_cmd = (0.0, 0.0)
        self.status = "idle"

        self.cmd_pub = rospy.Publisher(p("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)
        self.path_pub = rospy.Publisher("~path", Path, queue_size=1, latch=True)
        self.algo_path_pubs = dict((a, rospy.Publisher("~path/" + a, Path, queue_size=1, latch=True))
                                   for a in nav_core.ALGORITHMS)
        self.report_pub = rospy.Publisher("~report", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)

        pose_topic = p("~pose_topic", "/robot_pose_ekf/odom_combined")
        if p("~pose_type", "pose_cov") == "odometry":
            rospy.Subscriber(pose_topic, Odometry, self._on_odometry, queue_size=1)
        else:
            rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, self._on_pose_cov, queue_size=1)
        if self.use_map:
            rospy.Subscriber(p("~map_topic", "/map"), OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber(p("~scan_topic", "/scan"), LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber(p("~imu_topic", "/imu/data"), Imu, self._on_imu, queue_size=1)
        rospy.Subscriber(p("~sonar_topic", "/distance"), Float64, self._on_sonar, queue_size=1)
        rospy.Subscriber(p("~goal_topic", "/move_base_simple/goal"), PoseStamped, self._on_goal, queue_size=1)

        if rospy.has_param("~goal_x") and rospy.has_param("~goal_y"):
            goal = (float(p("~goal_x")), float(p("~goal_y")))
            if rospy.has_param("~start_x") and rospy.has_param("~start_y"):
                # drive to the start endpoint first, then on to the goal
                start = (float(p("~start_x")), float(p("~start_y")))
                self.goal, self.legs = start, [goal]
                rospy.loginfo("Mission: start (%.2f, %.2f) -> goal (%.2f, %.2f)",
                              start[0], start[1], goal[0], goal[1])
            else:
                self.goal = goal
            self.new_goal = True

        rospy.on_shutdown(self._stop_hard)
        rospy.loginfo("quantum_nav ready: algorithm=%s max_linear=%.2f m/s. "
                      "Send a goal with RViz '2D Nav Goal'.", self.algorithm, self.max_linear)

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _set_pose(self, pose):
        q = pose.orientation
        with self.lock:
            self.pose = (pose.position.x, pose.position.y,
                         nav_core.yaw_from_quaternion(q.x, q.y, q.z, q.w))
            self.pose_stamp = time.time()

    def _on_pose_cov(self, msg):
        self._set_pose(msg.pose.pose)

    def _on_odometry(self, msg):
        self._set_pose(msg.pose.pose)

    def _on_map(self, msg):
        with self.lock:
            self.map_msg = msg

    def _on_scan(self, msg):
        with self.lock:
            self.scan = msg

    def _on_imu(self, msg):
        q = msg.orientation
        roll, pitch = nav_core.roll_pitch_from_quaternion(q.x, q.y, q.z, q.w)
        with self.lock:
            self.roll, self.pitch = roll, self.pitch_sign * pitch

    def _on_sonar(self, msg):
        with self.lock:
            self.sonar_cm = msg.data

    def _on_goal(self, msg):
        with self.lock:
            self.goal = (msg.pose.position.x, msg.pose.position.y)
            self.legs = []  # an RViz goal replaces any start->goal mission
            self.new_goal = True
        rospy.loginfo("New goal (%.2f, %.2f)", self.goal[0], self.goal[1])

    # ── helpers ───────────────────────────────────────────────────────────────
    def _publish_cmd(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)
        self.last_cmd = (linear, angular)

    def _stop_hard(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def _scan_points(self, pose, scan, max_range):
        """LiDAR hits as world (x, y) points, for marking unmapped obstacles."""
        x, y, yaw = pose
        pts = []
        a = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(max_range, scan.range_max):
                ang = yaw + self.laser_yaw_offset + a
                pts.append((x + r * math.cos(ang), y + r * math.sin(ang)))
            a += scan.angle_increment
        return pts

    def _front_clearance(self, scan):
        """Closest LiDAR return within ±lidar_front_half_angle of straight ahead."""
        best = float("inf")
        a = scan.angle_min
        for r in scan.ranges:
            rel = nav_core.wrap_angle(a + self.laser_yaw_offset)
            if abs(rel) <= self.lidar_half_angle and scan.range_min < r < scan.range_max:
                best = min(best, r)
            a += scan.angle_increment
        return best

    def _build_grid(self, pose):
        with self.lock:
            m = self.map_msg
            scan = self.scan
        if m is not None:
            info = m.info
            grid = nav_core.PlanningGrid.from_occupancy(
                info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, m.data,
                plan_resolution=self.plan_resolution, robot_radius=self.robot_radius,
                unknown_is_blocked=self.unknown_is_blocked, elevation=self.elevation,
                max_slope_deg=self.max_slope_deg)
        else:
            rospy.logwarn("No /map received — planning on open ground + live LiDAR obstacles.")
            grid = nav_core.PlanningGrid.empty(pose[0], pose[1], size=20.0,
                                               resolution=self.plan_resolution,
                                               elevation=self.elevation,
                                               max_slope_deg=self.max_slope_deg)
        if scan is not None:
            grid.add_obstacle_points(self._scan_points(pose, scan, self.lidar_obstacle_range),
                                     self.robot_radius)
        return grid

    def _to_path_msg(self, points):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        for x, y in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    # ── planning ──────────────────────────────────────────────────────────────
    def _plan(self, pose, goal):
        self._publish_cmd(0.0, 0.0)
        grid = self._build_grid(pose)
        algos = nav_core.ALGORITHMS if self.algorithm == "auto" else (self.algorithm,)
        results = dict((a, nav_core.plan(grid, a, pose[:2], goal)) for a in algos)
        for a, r in results.items():
            if r is not None:
                self.algo_path_pubs[a].publish(self._to_path_msg(r.points))
        ranked = nav_core.rank(results.values())
        if not ranked:
            rospy.logerr("No path found to (%.2f, %.2f) by %s", goal[0], goal[1], ", ".join(algos))
            return False
        best = ranked[0]
        self.active, self.all_results = best, results
        waypoints = nav_core.thin_path(best.points)
        self.follower = nav_core.WaypointFollower(
            waypoints, self.goal_tolerance, self.waypoint_tolerance,
            self.max_linear, self.max_angular)
        self.path_pub.publish(self._to_path_msg(waypoints))
        report = {"chosen": best.algorithm, "goal": list(goal),
                  "results": dict((a, r.summary() if r else None) for a, r in results.items())}
        self.report_pub.publish(String(data=json.dumps(report)))
        for a in algos:
            r = results[a]
            rospy.loginfo("  %-8s %s", a, "no path" if r is None else
                          "energy=%.2f dist=%.2fm climb=%.2fm nodes=%d %.0fms" %
                          (r.energy, r.distance, r.climb, r.expanded, r.runtime_s * 1000))
        rospy.loginfo("Driving %s path (%d waypoints)", best.algorithm, len(waypoints))
        self.meter.reset()
        self.blocked_since = None
        self.avoider = nav_core.ObstacleAvoider(
            max_linear=self.max_linear, max_angular=self.max_angular, **self.avoid_params)
        self.avoiding_since = None
        self._last_index = 0
        self.watchdog.reset()
        return True

    # ── run log ───────────────────────────────────────────────────────────────
    def _log_run(self, outcome):
        r = self.active
        if r is None:
            return
        new = not os.path.exists(self.log_csv)
        try:
            d = os.path.dirname(self.log_csv)
            if d and not os.path.isdir(d):
                os.makedirs(d)
            with open(self.log_csv, "a") as f:
                if new:
                    f.write("time,algorithm,outcome,planned_energy,planned_distance_m,"
                            "planned_climb_m,plan_runtime_ms,nodes,measured_energy_J,"
                            "driven_distance_m,measured_climb_m,measured_descent_m,drive_time_s\n")
                f.write("%s,%s,%s,%.3f,%.3f,%.3f,%.2f,%d,%.1f,%.3f,%.3f,%.3f,%.1f\n" % (
                    time.strftime("%Y-%m-%d %H:%M:%S"), r.algorithm, outcome, r.energy,
                    r.distance, r.climb, r.runtime_s * 1000, r.expanded, self.meter.joules,
                    self.meter.distance, self.meter.climb, self.meter.descent, self.meter.elapsed))
            rospy.loginfo("Run logged to %s", self.log_csv)
        except (IOError, OSError) as e:
            rospy.logwarn("Could not write run log: %s", e)

    # ── main loop ─────────────────────────────────────────────────────────────
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            with self.lock:
                pose = self.pose
                pose_age = time.time() - self.pose_stamp
                goal, new_goal = self.goal, self.new_goal
                self.new_goal = False
                scan, sonar = self.scan, self.sonar_cm
                roll, pitch = self.roll, self.pitch

            status = self._tick(pose, pose_age, goal, new_goal, scan, sonar, roll, pitch, dt, start)
            if status != self.status:
                rospy.loginfo("status: %s", status)
                self.status = status
            self.status_pub.publish(String(data=status))
            rate.sleep()

    def _tick(self, pose, pose_age, goal, new_goal, scan, sonar, roll, pitch, dt, start):
        if pose is None or pose_age > self.pose_timeout:
            self._publish_cmd(0.0, 0.0)
            return "waiting for pose (%s)" % ("none yet" if pose is None else "stale %.1fs" % pose_age)

        if new_goal and goal is not None:
            if self.use_map and self.map_msg is None and time.time() - start < self.map_wait:
                with self.lock:
                    self.new_goal = True  # retry next tick once the map arrives
                return "waiting for /map"
            if self.follower is not None and not self.follower.done:
                self._log_run("preempted")
            self.stuck_replans = 0
            if not self._plan(pose, goal):
                self.follower = None
                return "no path"

        if self.follower is None or self.follower.done:
            self._publish_cmd(0.0, 0.0)
            return "idle"

        # safety: tipping
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self._publish_cmd(0.0, 0.0)
            self._log_run("aborted_tilt")
            self.follower = None
            return "ABORTED: tilt %.0f deg exceeds limit" % math.degrees(max(abs(roll), abs(pitch)))

        # obstacle ahead (LiDAR front sector, sonar)
        sonar_m = sonar / 100.0 if sonar is not None and sonar > 0 else float("inf")
        if self.avoid_obstacles:
            status = self._avoid(pose, goal, scan, sonar_m, pitch, dt)
        else:
            status = self._stop_and_wait(pose, goal, scan, sonar_m)
        if status is not None:
            return status

        # progress watchdog: going nowhere (circling / sliding along an obstacle)
        if self.watchdog.update(pose[0], pose[1], self.follower.waypoints[self.follower.index]):
            self.stuck_replans += 1
            if self.stuck_replans > self.max_stuck_replans:
                self._publish_cmd(0.0, 0.0)
                self._log_run("aborted_no_progress")
                self.follower = None
                rospy.logerr("No progress after %d replans; stopping", self.max_stuck_replans)
                return "ABORTED: no progress towards the goal"
            rospy.logwarn("No progress towards waypoint %d; replanning (%d/%d)",
                          self.follower.index + 1, self.stuck_replans, self.max_stuck_replans)
            if not self._plan(pose, goal):
                self.follower = None
                return "no path"
            return "replanned: no progress"

        cap = nav_core.uphill_speed_cap(self.max_linear, pitch)
        linear, angular, finished = self.follower.step(pose[0], pose[1], pose[2], max_linear=cap)
        if self.follower.index != self._last_index:
            self._last_index = self.follower.index
            self.avoider.reset_side()
        if finished:
            self._publish_cmd(0.0, 0.0)
            self._log_run("reached")
            rospy.loginfo("Reached (%.2f, %.2f): measured %.1f J over %.2f m (climb %.2f m, descent %.2f m)",
                          goal[0], goal[1], self.meter.joules, self.meter.distance,
                          self.meter.climb, self.meter.descent)
            with self.lock:
                if self.legs and not self.new_goal:
                    self.goal = self.legs.pop(0)
                    self.new_goal = True
                    rospy.loginfo("Start endpoint reached; heading for goal (%.2f, %.2f)",
                                  self.goal[0], self.goal[1])
                    return "start reached"
            return "goal reached"
        self._publish_cmd(linear, angular)
        self.meter.update(linear, angular, pitch, dt)
        return "driving %s (waypoint %d/%d)" % (self.active.algorithm, self.follower.index + 1,
                                                len(self.follower.waypoints))


    # ── obstacles ─────────────────────────────────────────────────────────────
    def _avoid(self, pose, goal, scan, sonar_m, pitch, dt):
        """Steer round an obstacle. Returns a status while avoiding, None to follow the path."""
        if scan is not None:
            sectors = nav_core.scan_sectors(scan.ranges, scan.angle_min, scan.angle_increment,
                                            scan.range_max, scan.range_min, self.laser_yaw_offset)
        else:
            sectors = nav_core.ScanSectors(float("inf"), float("inf"), float("inf"))
        sectors.front = min(sectors.front, sonar_m)  # the sonar looks straight ahead

        was_detouring = self.avoider.active
        command, detour_done = self.avoider.update(sectors, pose[0], pose[1])
        if command is not None:
            now = time.time()
            if self.avoiding_since is None:
                self.avoiding_since = now
            if self.avoider.active and not was_detouring:
                rospy.logwarn("Obstacle %.2f m ahead at (%.2f, %.2f): turning %s to go round it",
                              sectors.front, pose[0], pose[1],
                              "left" if self.avoider.side == 1 else "right")
            if now - self.avoiding_since > self.avoid_timeout:
                self._publish_cmd(0.0, 0.0)
                self._log_run("aborted_stuck")
                self.follower = None
                rospy.logerr("Could not get round the obstacle in %.0fs; stopping", self.avoid_timeout)
                return "ABORTED: stuck at obstacle"
            self._publish_cmd(command[0], command[1])
            self.meter.update(command[0], command[1], pitch, dt)
            if not self.avoider.active:
                return "veering away from a close side"
            return "avoiding obstacle (%s)" % self.avoider.state

        self.avoiding_since = None
        if detour_done:
            if self.replan_after_detour:
                rospy.loginfo("Past the obstacle; replanning from (%.2f, %.2f)", pose[0], pose[1])
                if self._plan(pose, goal):
                    return "replanned after obstacle"
                rospy.logwarn("Replan failed; rejoining the old path")
            self.follower.retarget_nearest(pose[0], pose[1])
        return None

    def _stop_and_wait(self, pose, goal, scan, sonar_m):
        """Old behaviour (avoid_obstacles:=false): stop, and replan if blocked too long."""
        front = self._front_clearance(scan) if scan is not None else float("inf")
        sonar_hit = sonar_m * 100.0 < self.sonar_stop_cm
        if not (front < self.lidar_stop or sonar_hit):
            self.blocked_since = None
            return None
        self._publish_cmd(0.0, 0.0)
        now = time.time()
        if self.blocked_since is None:
            self.blocked_since = now
        elif now - self.blocked_since > self.replan_after:
            rospy.logwarn("Blocked for %.0fs — replanning around the obstacle", self.replan_after)
            self.blocked_since = None
            if not self._plan(pose, goal):
                self.follower = None
                return "no path around obstacle"
        return "paused: obstacle %s" % ("sonar %.0f cm" % (sonar_m * 100) if sonar_hit else "lidar %.2f m" % front)


if __name__ == "__main__":
    rospy.init_node("quantum_nav")
    QuantumNavNode().spin()
