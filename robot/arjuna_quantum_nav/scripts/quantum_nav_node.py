#!/usr/bin/env python3
"""ROS Melodic node: plan with Dijkstra / A* / QAOA and drive the Arjuna Edu Kit.

Runs ON the Jetson Nano alongside the kit's own bringup, which already provides:
    /scan                              RPLidar C1 (rplidar_ros)
    /imu/data                          BNO055 (imu_bno055)
    /robot_pose_ekf/odom_combined      servo ticks + IMU fused by robot_pose_ekf
    /map                               map_server (optional)
    /cmd_vel  -> Arjuna_ticks_pub.py   turns Twist into STS servo wheel speeds

This node:  goal (RViz "2D Nav Goal" or ~goal_x/~goal_y, optionally from ~start_x/~start_y),
            or a route (nav_msgs/Path on ~route, e.g. clicked on web/rover.html):
            the rover drives through the route points in order, one planned leg each
            -> plan on the map with the chosen algorithm ("auto" runs all three
               and drives the best: lowest energy, ties -> fastest)
            -> follow the path on the fused pose, publishing /cmd_vel
            -> obstacle ahead (LiDAR/sonar): turn towards the open side, drive past,
               then replan from there with the obstacle marked on the grid
            -> terrain (OAK-D depth camera via crater_detector_node.py: crater rims, rocks, the
               border wall, no-floor): kept in the world frame for the whole run, merged with
               the LiDAR obstacles on the grid; a new one on the remaining path -> replan.
               A wheel climbing a crater rim shows as a short pitch bump on the IMU: stop,
               back out, mark the crater edge, replan.
            -> stops for: stale pose, excessive tilt, stuck avoiding too long
            -> logs each run (planned + measured energy, climb, descent) to CSV.

Planning/control logic lives in nav_core.py (pure Python 3.6, unit-tested).
"""

import collections
import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

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
        # terrain: crater rims / rocks / border / no-floor from crater_detector_node.py, plus an
        # IMU backup that feels a wheel climbing a crater rim
        self.use_craters = bool(p("~use_craters", True))
        self.crater_margin = float(p("~crater_margin", 0.03))        # m beyond robot_radius
        self.terrain_topic = p("~terrain_topic", "/terrain/detections")
        self.crater_topic = p("~crater_topic", "")   # legacy PoseArray of crater cells, "" = off
        self.detection_delay = float(p("~detection_delay", 0.15))    # s from image to message
        self.bump_pitch_deg = float(p("~bump_pitch_deg", 2.5))       # 0 = bump detector off
        self.bump_roll_deg = float(p("~bump_roll_deg", 3.0))
        self.crater_backup = float(p("~crater_backup_distance", 0.15))
        # a bump marks a crater-sized disc: its near edge is where the wheel touched the rim
        self.bump_crater_radius = float(p("~bump_crater_radius", 0.20))  # assumed crater radius, m
        self.bump_contact_ahead = float(p("~bump_contact_ahead", 0.16))  # front wheel contact, m
        self.max_bumps = int(p("~max_bumps", 4))                     # per goal, then give up
        self.terrain = nav_core.TerrainMap(confirm=dict(
            crater=int(p("~confirm_crater", 2)), rock=int(p("~confirm_rock", 2)),
            border=int(p("~confirm_border", 2)), nofloor=int(p("~confirm_nofloor", 3))))
        self.legacy_craters = nav_core.CraterMap()
        self.pending_changes = []   # new terrain not yet checked against the path
        self.bumps = nav_core.BumpDetector(self.bump_pitch_deg, self.bump_roll_deg)
        self.bump_event = None
        self.bump_count = 0
        self.backing = None         # (start_x, start_y, t0) while reversing out of a bump
        self.pose_history = collections.deque(maxlen=200)   # (time, pose) for the camera delay
        self.terrain_published = -1
        self.path_spacing = float(p("~path_spacing", 0.3))   # waypoint spacing along the path, m
        self.grid_size = float(p("~grid_size", 20.0))        # side of the open-ground grid, m
        # the arena (used when there is no /map): size and the corner at (arena_x0, arena_y0);
        # outside it, and a wall_margin strip inside it, is never planned
        self.arena = None
        if rospy.has_param("~arena_width") and rospy.has_param("~arena_height"):
            self.arena = (float(p("~arena_x0", 0.0)), float(p("~arena_y0", 0.0)),
                          float(p("~arena_width")), float(p("~arena_height")))
        self.wall_margin = float(p("~wall_margin", 0.04))
        if self.arena is not None:
            x0, y0, w, h = self.arena
            self.terrain.bounds = (x0, y0, x0 + w, y0 + h)    # drop detections off the table
        self.edge_stop = float(p("~edge_stop", 0.06))          # inside the arena edge, m: stop here
        self.soft_margin = float(p("~soft_margin", 0.0))       # extra-cost band beyond the clearance, m
        self.soft_weight = float(p("~soft_weight", 1.5))
        # no path at full clearance: retry squeezing closer, down to this fraction of it
        self.min_clearance_scale = float(p("~min_clearance_scale", 0.8))
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
        self.cancel = False       # set by ~cancel: stop and forget the goal/route
        self.last_outcome = None  # how the last run ended, kept in the idle status
        self.last_cmd = (0.0, 0.0)
        self.status = "idle"

        self.cmd_pub = rospy.Publisher(p("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)
        self.path_pub = rospy.Publisher("~path", Path, queue_size=1, latch=True)
        self.algo_path_pubs = dict((a, rospy.Publisher("~path/" + a, Path, queue_size=1, latch=True))
                                   for a in nav_core.ALGORITHMS)
        self.report_pub = rospy.Publisher("~report", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.terrain_pub = rospy.Publisher("~terrain", String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("~terrain_markers", MarkerArray, queue_size=1, latch=True)

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
        rospy.Subscriber(p("~route_topic", "~route"), Path, self._on_route, queue_size=1)
        rospy.Subscriber(p("~cancel_topic", "~cancel"), Empty, self._on_cancel, queue_size=1)
        if self.use_craters and self.terrain_topic:
            rospy.Subscriber(self.terrain_topic, String, self._on_terrain, queue_size=2)
        if self.use_craters and self.crater_topic:
            rospy.Subscriber(self.crater_topic, PoseArray, self._on_craters, queue_size=1)

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
            why = self._reject_reason([self.goal] + self.legs)
            if why:
                rospy.logerr("Launch goal rejected: %s", why)
                self.goal, self.legs, self.new_goal = None, [], False
                self.last_outcome = "goal rejected: " + why

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
            self.pose_history.append((self.pose_stamp, self.pose))

    def _pose_at(self, t):
        """The pose closest in time to `t` (lock held by the caller)."""
        if not self.pose_history:
            return self.pose
        return min(self.pose_history, key=lambda tp: abs(tp[0] - t))[1]

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
            driving = self.last_cmd[0] > 0.02 and self.backing is None and                 self.follower is not None and not self.follower.done
            if driving:
                ev = self.bumps.update(time.time(), self.pitch, roll, True)
                if ev is not None:
                    self.bump_event = ev
            else:
                self.bumps.update(time.time(), self.pitch, roll, False)

    def _on_sonar(self, msg):
        with self.lock:
            self.sonar_cm = msg.data

    def _on_terrain(self, msg):
        """Detections from crater_detector_node.py (JSON, robot frame) -> world frame."""
        try:
            dets = json.loads(msg.data).get("detections", [])
        except (ValueError, AttributeError):
            rospy.logwarn_throttle(5.0, "Bad /terrain/detections message")
            return
        with self.lock:
            pose = self._pose_at(time.time() - self.detection_delay)
            if pose is None or not dets:
                return
            changes = self.terrain.add(pose, dets)
            self.pending_changes.extend(changes)
        for c in changes:
            if c[0] == "crater":
                rospy.loginfo("Crater at (%.2f, %.2f), radius %.2f m", c[1], c[2], c[3])
            else:
                rospy.loginfo("%s: %d new cell(s) confirmed", c[1], len(c[2]))

    def _on_craters(self, msg):
        """Legacy: crater cells (PoseArray, robot frame) -> world, de-duplicated."""
        with self.lock:
            pose = self._pose_at(time.time() - self.detection_delay)
            if pose is None or not msg.poses:
                return
            world = nav_core.robot_to_world(pose, [(q.position.x, q.position.y) for q in msg.poses])
            new = self.legacy_craters.add(world)
            if new:
                self.pending_changes.append(("cells", "rim", new))

    def _reject_reason(self, points):
        """Why these goal/route points cannot be driven to, or None. A point on a detected
        crater / rock / the border / the table edge, or outside the arena, is refused."""
        for i, (x, y) in enumerate(points):
            label = "(%.2f, %.2f)" % (x, y) if len(points) == 1 else                 "point %d (%.2f, %.2f)" % (i + 1, x, y)
            if self.arena is not None:
                x0, y0, w, h = self.arena
                m = self.wall_margin
                if not (x0 + m <= x <= x0 + w - m and y0 + m <= y <= y0 + h - m):
                    return "%s is outside the arena" % label
            with self.lock:
                what = self.terrain.blocking(x, y)
            if what:
                return "%s is on a detected %s" % (label, what)
        return None

    def _on_goal(self, msg):
        why = self._reject_reason([(msg.pose.position.x, msg.pose.position.y)])
        if why:
            rospy.logwarn("Goal rejected: %s", why)
            with self.lock:
                self.last_outcome = "goal rejected: " + why
            return
        with self.lock:
            self.goal = (msg.pose.position.x, msg.pose.position.y)
            self.legs = []  # an RViz goal replaces any start->goal mission
            self.new_goal = True
            self.cancel = False
        rospy.loginfo("New goal (%.2f, %.2f)", self.goal[0], self.goal[1])

    def _on_route(self, msg):
        """A user-chosen route: visit each point in order (each leg is planned)."""
        points = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if not points:
            rospy.logwarn("Ignoring an empty route")
            return
        why = self._reject_reason(points)
        if why:
            rospy.logwarn("Route rejected: %s", why)
            with self.lock:
                self.last_outcome = "route rejected: " + why
            return
        with self.lock:
            self.goal, self.legs = points[0], points[1:]
            self.new_goal = True
            self.cancel = False
        rospy.loginfo("New route with %d points: %s", len(points),
                      " -> ".join("(%.2f, %.2f)" % pt for pt in points))

    def _on_cancel(self, msg):
        with self.lock:
            self.cancel = True
            self.new_goal = False
        rospy.logwarn("Cancel received: stopping")

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

    def _build_grid(self, pose, scale=1.0):
        """Planning grid: map or arena + LiDAR + camera terrain. scale < 1 shrinks the
        clearance round everything (the retry when no path fits at full clearance)."""
        radius = self.robot_radius * scale
        with self.lock:
            m = self.map_msg
            scan = self.scan
        if m is not None:
            info = m.info
            grid = nav_core.PlanningGrid.from_occupancy(
                info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, m.data,
                plan_resolution=self.plan_resolution, robot_radius=radius,
                unknown_is_blocked=self.unknown_is_blocked, elevation=self.elevation,
                max_slope_deg=self.max_slope_deg)
        elif self.arena is not None:
            x0, y0, w, h = self.arena
            grid = nav_core.PlanningGrid.arena(
                x0, y0, w, h, resolution=self.plan_resolution, wall_margin=self.wall_margin,
                robot_radius=radius, elevation=self.elevation,
                max_slope_deg=self.max_slope_deg)
        else:
            rospy.logwarn_throttle(10.0, "No /map received: planning on open ground + live "
                                   "LiDAR obstacles.")
            grid = nav_core.PlanningGrid.empty(pose[0], pose[1], size=self.grid_size,
                                               resolution=self.plan_resolution,
                                               elevation=self.elevation,
                                               max_slope_deg=self.max_slope_deg)
        if scan is not None:
            grid.add_obstacle_points(self._scan_points(pose, scan, self.lidar_obstacle_range),
                                     radius)
        self._add_terrain(grid, scale)
        if self.soft_margin > 0:
            grid.add_soft_margin(self.soft_margin, self.soft_weight)
        return grid

    def _add_terrain(self, grid, scale=1.0):
        """Camera terrain (crater discs, rock / border / no-floor cells), world frame."""
        clearance = (self.robot_radius + self.crater_margin) * scale
        with self.lock:
            discs = self.terrain.discs()
            cells = self.terrain.points() + list(self.legacy_craters.points)
        for x, y, r in discs:
            grid.add_disc(x, y, r + clearance)
        if cells:
            grid.add_obstacle_points(cells, clearance)

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
        with self.lock:
            self.pending_changes = []  # everything known goes into this plan's grid
        algos = nav_core.ALGORITHMS if self.algorithm == "auto" else (self.algorithm,)
        scales = [1.0]
        if self.min_clearance_scale < 1.0:
            scales += [1.0 - (1.0 - self.min_clearance_scale) * f for f in (0.5, 1.0)]
        for scale in scales:
            grid = self._build_grid(pose, scale)
            results = dict((a, nav_core.plan(grid, a, pose[:2], goal)) for a in algos)
            ranked = nav_core.rank(results.values())
            if ranked:
                if scale < 1.0:
                    rospy.logwarn("No path at full clearance; planned with %.0f%% of it "
                                  "(%.2f m round obstacles)", 100 * scale,
                                  (self.robot_radius + self.crater_margin) * scale)
                break
        for a, r in results.items():
            if r is not None:
                self.algo_path_pubs[a].publish(self._to_path_msg(r.points))
        if not ranked:
            rospy.logerr("No path found to (%.2f, %.2f) by %s", goal[0], goal[1], ", ".join(algos))
            return False
        best = ranked[0]
        self.active, self.all_results = best, results
        waypoints = nav_core.thin_path(best.points, spacing=self.path_spacing)
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
        self._publish_terrain(force=True)
        while not rospy.is_shutdown():
            self.step(dt, start)
            rate.sleep()

    def step(self, dt, start):
        """One control cycle (the body of spin; the simulator calls it directly)."""
        with self.lock:
            pose = self.pose
            pose_age = time.time() - self.pose_stamp
            goal, new_goal = self.goal, self.new_goal
            self.new_goal = False
            scan, sonar = self.scan, self.sonar_cm
            roll, pitch = self.roll, self.pitch
            cancel, self.cancel = self.cancel, False
            if cancel:
                self.goal, self.legs = None, []

        if cancel:
            self._publish_cmd(0.0, 0.0)
            if self.follower is not None and not self.follower.done:
                self._log_run("cancelled")
            self.follower = None
            self.backing = None  # also stop reversing out of a crater
            self.path_pub.publish(self._to_path_msg([]))
            self.last_outcome = "stopped by user"

        if new_goal and goal is not None:
            self.last_outcome = None
        status = self._tick(pose, pose_age, goal, new_goal, scan, sonar, roll, pitch, dt, start)
        # One-tick outcomes ("goal reached", "ABORTED: ...", "no path") would vanish on the
        # next tick; keep the latest in the idle status so slow subscribers still see it.
        if status == "idle":
            if self.last_outcome:
                status = "idle (last: %s)" % self.last_outcome
        elif self.follower is None or self.follower.done:
            if not status.startswith("waiting"):
                self.last_outcome = status
        if status != self.status:
            rospy.loginfo("status: %s", status)
            self.status = status
        self.status_pub.publish(String(data=status))
        self._publish_terrain()

    def _tick(self, pose, pose_age, goal, new_goal, scan, sonar, roll, pitch, dt, start):
        if pose is None or pose_age > self.pose_timeout:
            self._publish_cmd(0.0, 0.0)
            if new_goal:
                with self.lock:
                    self.new_goal = True  # keep the goal until the pose arrives
            return "waiting for pose (%s)" % ("none yet" if pose is None else "stale %.1fs" % pose_age)

        if new_goal and goal is not None:
            if self.use_map and self.map_msg is None and time.time() - start < self.map_wait:
                with self.lock:
                    self.new_goal = True  # retry next tick once the map arrives
                return "waiting for /map"
            if self.follower is not None and not self.follower.done:
                self._log_run("preempted")
            self.stuck_replans = 0
            self.bump_count = 0
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

        # the table edge: never roll off the arena
        if self.arena is not None:
            x0, y0, w, h = self.arena
            m = self.edge_stop
            if not (x0 + m <= pose[0] <= x0 + w - m and y0 + m <= pose[1] <= y0 + h - m):
                self._publish_cmd(0.0, 0.0)
                self._log_run("aborted_edge")
                self.follower = None
                rospy.logerr("At the table edge (%.2f, %.2f); stopping", pose[0], pose[1])
                return "ABORTED: at the table edge"

        # terrain (camera detections, IMU bump)
        status = self._craters_tick(pose, goal, scan, pitch)
        if status is not None:
            return status

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
                    rospy.loginfo("Waypoint reached; heading for (%.2f, %.2f), %d more after it",
                                  self.goal[0], self.goal[1], len(self.legs))
                    return "waypoint reached (%d left)" % (len(self.legs) + 1)
            return "goal reached"
        self._publish_cmd(linear, angular)
        self.meter.update(linear, angular, pitch, dt)
        return "driving %s (waypoint %d/%d)" % (self.active.algorithm, self.follower.index + 1,
                                                len(self.follower.waypoints))


    # ── terrain (craters, rocks, border) ──────────────────────────────────────
    def _replan_for_crater(self, pose, goal, why="terrain"):
        if not self._plan(pose, goal):
            self.follower = None
            return "no path around %s" % why
        return "replanned: %s" % why

    def _changes_hit_path(self, changes, pose):
        clearance = self.robot_radius + self.crater_margin
        discs, pts = [], []
        for c in changes:
            if c[0] == "crater":
                discs.append((c[1], c[2], c[3]))
            else:
                pts.extend(c[2])
        wps, idx = self.follower.waypoints, self.follower.index
        return (bool(discs) and nav_core.discs_near_path(discs, pose[:2], wps, idx, clearance)) or \
            (bool(pts) and nav_core.points_near_path(pts, pose[:2], wps, idx, clearance))

    def _craters_tick(self, pose, goal, scan, pitch):
        """Returns a status while handling terrain, None to carry on driving."""
        if self.backing is not None:
            # reversing out of a crater rim the IMU felt, then replan with it marked
            bx, by, t0 = self.backing
            rear = float("inf")
            if scan is not None:
                rear = nav_core.scan_sectors(scan.ranges, scan.angle_min, scan.angle_increment,
                                             scan.range_max, scan.range_min,
                                             self.laser_yaw_offset).rear
            moved = math.hypot(pose[0] - bx, pose[1] - by)
            if (moved < self.crater_backup and time.time() - t0 < 4.0 and
                    rear > self.avoid_params["rear_clearance"]):
                self._publish_cmd(-0.06, 0.0)
                return "crater: backing out"
            self._publish_cmd(0.0, 0.0)
            self.backing = None
            rospy.loginfo("Backed out of the crater edge; replanning round it")
            return self._replan_for_crater(pose, goal, "crater edge")

        if not self.use_craters:
            return None

        # IMU: a wheel climbed a crater rim (short pitch / roll bump)
        with self.lock:
            ev, self.bump_event = self.bump_event, None
        if ev is not None:
            return self._on_bump(pose, goal, ev)

        # camera: new terrain on the path still to drive -> replan
        with self.lock:
            pending, self.pending_changes = self.pending_changes, []
        if pending and self._changes_hit_path(pending, pose):
            rospy.logwarn("New terrain on the path ahead; replanning round it")
            return self._replan_for_crater(pose, goal)
        return None

    def _on_bump(self, pose, goal, ev):
        self._publish_cmd(0.0, 0.0)
        self.bump_count += 1
        if self.bump_count > self.max_bumps:
            self._log_run("aborted_bumps")
            self.follower = None
            rospy.logerr("%d bumps on the way to this goal; stopping", self.bump_count - 1)
            return "ABORTED: kept hitting crater rims"
        # nose up / roll: a front wheel is on the rim, just ahead of us. Nose down: the rear
        # wheels are. Either way the crater lies on ahead of that rim point, so mark a
        # crater-sized disc starting there.
        contact = -self.bump_contact_ahead if ev["kind"] == "nose_down" else self.bump_contact_ahead
        r = self.bump_crater_radius
        (mx, my), = nav_core.robot_to_world(pose, [(contact + r - 0.02, 0.0)])
        with self.lock:
            self.terrain.add_mark(mx, my, r)
        rospy.logwarn("Bump (%s %.1f deg) at (%.2f, %.2f): crater edge, marking a %.0f cm crater "
                      "at (%.2f, %.2f)", ev["kind"], ev["deg"], pose[0], pose[1], 200 * r, mx, my)
        self._publish_terrain(force=True)
        if ev["kind"] == "nose_down":
            return self._replan_for_crater(pose, goal, "crater edge")
        self.backing = (pose[0], pose[1], time.time())
        return "crater: bump, backing out"

    def _publish_terrain(self, force=False):
        now = time.time()
        if not force and (self.terrain.version == self.terrain_published or
                          now - getattr(self, "_terrain_pub_t", 0.0) < 0.5):
            return
        self._terrain_pub_t = now
        clearance = self.robot_radius + self.crater_margin
        with self.lock:
            snap = self.terrain.snapshot(clearance)
            legacy = [[round(x, 3), round(y, 3)] for x, y in self.legacy_craters.points]
            self.terrain_published = self.terrain.version
        snap["rim"] = snap["rim"] + legacy
        self.terrain_pub.publish(String(data=json.dumps(snap)))
        self.marker_pub.publish(self._markers(snap))

    def _markers(self, snap):
        """RViz view of the terrain map: orange discs = craters, red = rocks, purple = border,
        black = no floor (table edge). Fixed frame: map."""
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        stamp = rospy.Time.now()
        for i, c in enumerate(snap["craters"]):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame_id, stamp
            m.ns, m.id, m.type, m.action = "craters", i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = c["x"], c["y"], 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * c["r"]
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.0, 0.6
            arr.markers.append(m)
        styles = (("rock", (0.9, 0.1, 0.1), 0.25), ("border", (0.6, 0.2, 0.9), 0.06),
                  ("nofloor", (0.05, 0.05, 0.05), 0.02), ("rim", (1.0, 0.55, 0.0), 0.03))
        for j, (kind, (r, g, b), height) in enumerate(styles):
            pts = snap.get(kind) or []
            if not pts:
                continue
            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame_id, stamp
            m.ns, m.id, m.type, m.action = kind, 100 + j, Marker.CUBE_LIST, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.terrain.cell
            m.scale.z = height
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
            for x, y in pts:
                m.points.append(Point(x=x, y=y, z=height / 2.0))
            arr.markers.append(m)
        return arr

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
