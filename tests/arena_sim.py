"""Closed-loop simulation of the Arjuna rover in the lunar arena.

The REAL robot/arjuna_quantum_nav/scripts/quantum_nav_node.py runs here against stub rospy
(tests/ros_stubs.py). Around it, a simulated rover and arena provide what the hardware would:

  /robot_pose_ekf/odom_combined   the rover pose (optionally with drift / noise)
  /scan                           RPLidar at 18 cm, mounted backwards (yaw offset 180), sees
                                  rocks only (not the 2 cm rims, not the low border); a share
                                  of beams is lost on shiny plastic
  /imu/data                       pitch / roll from wheels climbing a crater rim (nose up when
                                  the front wheels are on it)
  /terrain/detections             the REAL terrain_core detector fed with depth images rendered
                                  from the simulated arena (tests/arena_world.py)

Rim crossings, rock contacts and leaving the table are counted from the true rover position.

    python tests/arena_sim.py            # the three planners on the standard arena
"""

import json
import math
import os
import sys
import time as real_time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.abspath(os.path.join(HERE, '..', 'robot', 'arjuna_quantum_nav', 'scripts'))
for p in (SCRIPTS, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import ros_stubs  # noqa: E402

rospy = ros_stubs.install()

import arena_world as aw  # noqa: E402
import crater_core as cc  # noqa: E402
import nav_core  # noqa: E402
import quantum_nav_node as qn  # noqa: E402
import terrain_core as tc  # noqa: E402

# Arjuna curriculum PDF: wheel radius 0.04 m, wheel centres 0.28-0.30 m apart (odometry code).
BODY_R = 0.16            # physical radius of the rover: half the track (0.15) + wheel half width
WHEEL_FWD, WHEEL_SIDE = 0.07, 0.15
RIM_BUMP = 0.02
WHEEL_REACH = 0.04       # a wheel (r = 4 cm) feels a rim while its centre is this close to it
CAM_LATENCY = 0.1        # s between taking a depth image and the detections reaching the navigator
SUBSTEPS = 5             # physics + IMU at 50 Hz, the navigator at 10 Hz
INTR = cc.Intrinsics(fx=450.0, fy=450.0, cx=320.0, cy=200.0, width=640, height=400)


# What launch/arena.launch sets (tests/test_arjuna_arena_sim.py checks the two agree), plus the
# sim's own choices for tolerances. The sim rover is 16 cm in radius; the planner is told 17 + 5.
ARENA_PARAMS = dict(
    use_map=False, laser_yaw_offset_deg=180.0, robot_radius=0.17, crater_margin=0.05,
    plan_resolution=0.05, path_spacing=0.15, waypoint_tolerance=0.06, goal_tolerance=0.08,
    max_linear=0.12, max_angular=0.8, avoid_obstacles=False, lidar_stop_distance=0.18,
    soft_margin=0.10, bump_pitch_deg=2.5, wall_margin=0.04, lidar_obstacle_range=3.0,
    no_progress_travel=2.0, confirm_nofloor=3, detection_delay=0.1)


def layout_from_arena(arena):
    """The make_arena_map.py layout dict for a simulated arena (metres, origin corner)."""
    return {"units": "m", "size": arena.size, "border_width": arena.wall_t,
            "craters": [{"x": c.x, "y": c.y, "diameter": 2 * c.R,
                         "rotation_deg": math.degrees(c.rot)} for c in arena.craters],
            "rocks": [{"x": r.x, "y": r.y, "width": 2 * r.hw, "depth": 2 * r.hd,
                       "rotation_deg": math.degrees(r.rot)} for r in arena.rocks]}


def make_node(arena, algorithm, params=None, log_csv=None):
    """The real navigator under stub rospy, with a fake clock. Returns (node, clock)."""
    clock = FakeTime()
    qn.time = clock
    ros_stubs.set_clock(clock.t)
    rospy.PARAMS.clear()
    rospy.PUBLISHED.clear()
    rospy.SUBSCRIBERS.clear()
    del rospy.LOG[:]
    rospy.PARAMS.update(ARENA_PARAMS)
    rospy.PARAMS.update(algorithm=algorithm, arena_width=arena.size, arena_height=arena.size,
                        log_csv=log_csv or os.path.join(HERE, "_sim_runs.csv"))
    rospy.PARAMS.update(params or {})
    node = qn.QuantumNavNode()
    if log_csv is None:
        node._log_run = lambda outcome: None      # keep the user's CSV out of the tests
    return node, clock


def standard_arena(size=2.21):
    """3 craters (25-30 cm across), 2 rocks (30 cm wide, 22-25 cm tall) in a 2.21 m arena.
    The rover of the curriculum PDF is 30+ cm wide (wheel centres 0.28-0.30 m apart), so the
    old 4-rock layout no longer leaves a route at the planner's clearance (17 + 5 cm). These
    positions were found by a search so every test route exists with padded obstacles at a
    planner radius of about 0.30 m, while the straight line start->goal crosses obstacles."""
    return aw.Arena(size, craters=[aw.Crater(1.16, 1.36, 0.28, rot_deg=10),
                                   aw.Crater(1.24, 0.74, 0.25, rot_deg=0),
                                   aw.Crater(0.80, 1.05, 0.30, rot_deg=22)],
                    rocks=[aw.Rock(1.90, 1.43, 0.30, 0.30, 0.25, rot_deg=0),
                           aw.Rock(1.67, 0.88, 0.30, 0.30, 0.22, rot_deg=15)])


class FakeTime(object):
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def sleep(self, s):
        pass

    strftime = staticmethod(real_time.strftime)


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Rover(object):
    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.path = 0.0

    def step(self, cmd_v, cmd_w, dt):
        # first-order lag with acceleration limits, like geared servo wheels
        self.v += max(-0.6 * dt, min(0.6 * dt, cmd_v - self.v))
        self.w += max(-4.0 * dt, min(4.0 * dt, cmd_w - self.w))
        self.yaw = nav_core.wrap_angle(self.yaw + self.w * dt)
        dx = self.v * dt
        self.x += dx * math.cos(self.yaw)
        self.y += dx * math.sin(self.yaw)
        self.path += abs(dx)

    def wheels(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = {}
        for name, (f, l) in {"fl": (WHEEL_FWD, WHEEL_SIDE), "fr": (WHEEL_FWD, -WHEEL_SIDE),
                             "rl": (-WHEEL_FWD, WHEEL_SIDE), "rr": (-WHEEL_FWD, -WHEEL_SIDE)}.items():
            out[name] = (self.x + f * c - l * s, self.y + f * s + l * c)
        return out


def scan_rocks(arena, pose, yaw_offset, n=360, rng=None, lost=0.12, rmax=6.0):
    """LiDAR ranges (angle_min = -pi) of the rocks, 2-D ray / rotated-box intersection."""
    x, y, yaw = pose
    a = -math.pi + np.arange(n) * (2 * math.pi / n)
    ang = yaw + yaw_offset + a
    dx, dy = np.cos(ang), np.sin(ang)
    best = np.full(n, np.inf)
    for r in arena.rocks:
        c, s = math.cos(r.rot), math.sin(r.rot)
        ox, oy = (x - r.x) * c + (y - r.y) * s, -(x - r.x) * s + (y - r.y) * c
        ldx, ldy = dx * c + dy * s, -dx * s + dy * c
        tn, tf = np.full(n, -np.inf), np.full(n, np.inf)
        for o, d, h in ((ox, ldx, r.hw), (oy, ldy, r.hd)):
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (-h - o) / d, (h - o) / d
            lo, hi = np.minimum(t1, t2), np.maximum(t1, t2)
            par = np.abs(d) < 1e-12
            lo = np.where(par, np.where(abs(o) <= h, -np.inf, np.inf), lo)
            hi = np.where(par, np.where(abs(o) <= h, np.inf, -np.inf), hi)
            tn, tf = np.maximum(tn, lo), np.minimum(tf, hi)
        hit = (tn <= tf) & (tn > 0.05)
        best = np.where(hit & (tn < best), tn, best)
    best = np.where(best < rmax, best, np.inf)
    if rng is not None:
        best = np.where(rng.rand(n) < lost * (best < np.inf), np.inf, best)
    return best


def dist_to_rock(rock, x, y):
    lx, ly = rock.local(x, y)
    return math.hypot(max(abs(lx) - rock.hw, 0.0), max(abs(ly) - rock.hd, 0.0))


class Result(object):
    def __init__(self):
        self.reached = False
        self.status = ""
        self.sim_time = 0.0
        self.distance = 0.0
        self.replans = 0
        self.bumps = 0
        self.rock_touch = 0
        self.rim_contact = 0
        self.inside_crater = 0
        self.off_table = 0
        self.min_rock_gap = float("inf")
        self.min_crater_gap = float("inf")
        self.cam_frames = 0
        self.plan_ms = 0.0
        self.trace = []

    def summary(self):
        return ("reached=%s t=%.0fs dist=%.2fm replans=%d bumps=%d | rock-touch=%d rim-contact=%d "
                "inside-crater=%d off-table=%d | min gap rock=%.2f crater=%.2f" % (
                    self.reached, self.sim_time, self.distance, self.replans, self.bumps,
                    self.rock_touch, self.rim_contact, self.inside_crater, self.off_table,
                    self.min_rock_gap, self.min_crater_gap))

    @property
    def clean(self):
        return self.reached and not (self.rock_touch or self.rim_contact or self.inside_crater
                                     or self.off_table)


def run_mission(arena, algorithm, start, goals, use_camera=True, params=None, seed=0,
                max_time=240.0, cam_period=0.4, dt=0.1, drift=0.0, floor=None, verbose=False,
                occupancy=None, log_csv=None):
    """Drive from start (x, y, yaw) through `goals` (a goal, or a list = a route)."""
    rs = np.random.RandomState(seed)
    node, clock = make_node(arena, algorithm, params, log_csv)

    rover = Rover(*start)
    cam = aw.Camera(INTR, height=0.25, tilt_deg=30.0, forward=0.10)
    detector = None
    if use_camera:
        prm = tc.TerrainParams()
        u, v = cc.pixel_grid(INTR, step=prm.pixel_step, step_y=prm.pixel_step_y,
                             top_fraction=prm.top_fraction)
        if floor is None:
            flat = aw.Arena(8.0)
            floor = cc.fit_floor(cc.depth_to_points(aw.render(flat, cam, (3, 3, 0), u, v), INTR, u, v))
        detector = tc.TerrainDetector(floor, INTR, prm)

    msgs = rospy.msgs
    res = Result()
    est = [0.0, 0.0, 0.0]                         # pose error (drift) applied to the odometry

    def publish_pose():
        m = msgs["PoseWithCovarianceStamped"]()
        p = m.pose.pose
        p.position.x, p.position.y = rover.x + est[0], rover.y + est[1]
        qx, qy, qz, qw = quat(0.0, 0.0, rover.yaw + est[2])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        rospy.SUBSCRIBERS["robot_pose_ekf/odom_combined"](m)

    def publish_imu():
        w = rover.wheels()
        on = dict((k, arena.on_rim(pt[0], pt[1], WHEEL_REACH)) for k, pt in w.items())
        h = dict((k, RIM_BUMP if v else 0.0) for k, v in on.items())
        pitch = math.atan2((h["fl"] + h["fr"]) / 2 - (h["rl"] + h["rr"]) / 2, 2 * WHEEL_FWD)
        roll = math.atan2((h["fl"] + h["rl"]) / 2 - (h["fr"] + h["rr"]) / 2, 2 * WHEEL_SIDE)
        pitch += rs.normal(0, math.radians(0.15))
        roll += rs.normal(0, math.radians(0.15))
        m = msgs["Imu"]()
        o = m.orientation
        o.x, o.y, o.z, o.w = quat(roll, pitch, rover.yaw)
        rospy.SUBSCRIBERS["imu/data"](m)
        return any(on.values())

    def publish_scan():
        m = msgs["LaserScan"]()
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / 360
        m.range_min, m.range_max = 0.05, 6.0
        m.ranges = [float(r) for r in scan_rocks(arena, (rover.x, rover.y, rover.yaw),
                                                 math.pi, rng=rs)]
        rospy.SUBSCRIBERS["scan"](m)

    def publish_camera():
        pose = (rover.x + est[0], rover.y + est[1], rover.yaw + est[2])
        img = aw.render(arena, cam, (rover.x, rover.y, rover.yaw), u, v, seed=int(rs.randint(1 << 30)))
        dets, _ = detector.process(img, pose)
        res.cam_frames += 1
        if dets:   # delivered late: depth + processing latency (the node compensates 0.1 s)
            inbox.append((clock.t + CAM_LATENCY,
                          json.dumps({"detections": tc.detections_to_json_obj(dets)})))

    if occupancy is not None:                     # a known map (make_arena_map.py), like map_server
        w, h, mres, mx, my, data = occupancy
        m = msgs["OccupancyGrid"]()
        m.info.width, m.info.height, m.info.resolution = w, h, mres
        m.info.origin.position.x, m.info.origin.position.y = mx, my
        m.data = data
        rospy.SUBSCRIBERS["map"](m)

    # first poses so the node has something to plan from
    publish_pose()
    publish_imu()
    publish_scan()
    if len(goals) > 1 or isinstance(goals[0], (list, tuple)):
        pts = [tuple(g) for g in goals]
        route = msgs["Path"]()
        for gx, gy in pts:
            ps = msgs["PoseStamped"]()
            ps.pose.position.x, ps.pose.position.y = gx, gy
            route.poses.append(ps)
        rospy.SUBSCRIBERS["quantum_nav/route"](route) if "quantum_nav/route" in rospy.SUBSCRIBERS \
            else rospy.SUBSCRIBERS["route"](route)
    else:
        g = msgs["PoseStamped"]()
        g.pose.position.x, g.pose.position.y = goals[0]
        rospy.SUBSCRIBERS["move_base_simple/goal"](g)

    inbox = []
    t0, next_cam, last_status = clock.t, 0.0, ""
    last_cmd = (0.0, 0.0)
    while clock.t - t0 < max_time:
        clock.t += dt
        ros_stubs.set_clock(clock.t)
        if drift:
            est[0] += rs.normal(0, drift * 0.01)
            est[1] += rs.normal(0, drift * 0.01)
            est[2] += rs.normal(0, drift * 0.002)
        publish_pose()
        publish_scan()
        publish_imu()
        if detector is not None and clock.t - t0 >= next_cam:
            next_cam = clock.t - t0 + cam_period
            publish_camera()
        while inbox and inbox[0][0] <= clock.t:
            data = inbox.pop(0)[1]
            cb = rospy.SUBSCRIBERS.get("terrain/detections")      # absent when use_craters is false
            if cb is not None:
                cb(msgs["String"](data=data))
        node.step(dt, t0)
        cmd = rospy.PUBLISHED.get("cmd_vel", [None])[-1]
        last_cmd = (cmd.linear.x, cmd.angular.z) if cmd is not None else (0.0, 0.0)
        touched = False
        for _ in range(SUBSTEPS):
            rover.step(last_cmd[0], last_cmd[1], dt / SUBSTEPS)
            clock.t += 0.0
            if any(arena.on_rim(pt[0], pt[1], WHEEL_REACH) for pt in rover.wheels().values()):
                touched = True
            publish_imu()

        # judge the true position
        if touched or arena.on_rim(rover.x, rover.y, BODY_R * 0.5):
            res.rim_contact += 1
        for cr in arena.craters:
            if cr.contains(rover.x, rover.y):
                res.inside_crater += 1
            res.min_crater_gap = min(res.min_crater_gap, float(cr.dist(rover.x, rover.y)) - cr.apothem - BODY_R)
        for rk in arena.rocks:
            gap = dist_to_rock(rk, rover.x, rover.y) - BODY_R
            res.min_rock_gap = min(res.min_rock_gap, gap)
            if gap < 0:
                res.rock_touch += 1
        if not (BODY_R <= rover.x <= arena.size - BODY_R and BODY_R <= rover.y <= arena.size - BODY_R):
            res.off_table += 1
        res.trace.append((clock.t - t0, rover.x, rover.y, rover.yaw))

        status = node.status
        if status != last_status:
            if status.startswith("replanned") or status.startswith("crater"):
                res.replans += 1
            if "bump" in status:
                res.bumps += 1
            if verbose:
                print("%6.1fs %-45s (%.2f, %.2f)" % (clock.t - t0, status, rover.x, rover.y))
            last_status = status
        if status.startswith("goal reached"):
            res.reached = True
            break
        if status.startswith("ABORTED") or status.startswith("no path") or status.startswith("no path around"):
            break
    res.status = node.status
    res.sim_time = clock.t - t0
    res.distance = rover.path
    res.node = node
    res.log = list(rospy.LOG)
    return res


def main():
    arena = standard_arena()
    print("Arena %.2f m, %d craters, %d rocks. Start (0.25, 0.25) -> goal (1.95, 1.95)\n" % (
        arena.size, len(arena.craters), len(arena.rocks)))
    for algo in nav_core.ALGORITHMS:
        t = real_time.time()
        r = run_mission(arena, algo, (0.25, 0.25, 0.0), [(1.95, 1.95)])
        print("%-9s %s   [%.0fs wall]" % (algo, r.summary(), real_time.time() - t))


if __name__ == "__main__":
    main()
