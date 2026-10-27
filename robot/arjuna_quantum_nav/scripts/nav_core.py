#!/usr/bin/env python3
"""Planning + path-following core for the Arjuna Edu Kit (ROS Melodic, Python 3.6).

Pure Python (stdlib only, no rospy, no numpy) so it runs on the Jetson Nano's
Python 3.6.9 and can be unit-tested on any machine (tests/test_arjuna_nav_core.py).

It mirrors two parts of this repo so the robot behaves like the simulations:
  * web/js/sim.js                     — directional slope energy model and the
                                        Dijkstra / A* / QAOA planners being compared
  * navigation_pkg/path_following.py  — the waypoint-following control law

Python 3.6 rules for this file: no dataclasses, no `from __future__ import annotations`.
"""

import bisect
import functools
import heapq
import math
import random
import time

# ── directional slope energy model (same constants as web/js/sim.js) ──────────
# grade = rise / run along the direction of travel (signed).
#   uphill   : cost grows with grade (work against gravity)
#   downhill : gentle descents are gravity-assisted, steeper ones need braking
#   |slope| > max_slope_deg : impassable
UPHILL_K = 4.0
ASSIST_GRADE = 0.15
BRAKE_K = 3.0
MIN_GRADE_FACTOR = 1.0 - ASSIST_GRADE  # cheapest possible metre (keeps A* admissible)

ALGORITHMS = ("dijkstra", "astar", "qaoa")
ENERGY_TIE = 0.01


def grade_factor(grade):
    """Energy multiplier for driving one metre at the given signed grade."""
    if grade >= 0:
        return 1.0 + UPHILL_K * grade
    g = -grade
    f = 1.0 - min(g, ASSIST_GRADE)
    if g > ASSIST_GRADE:
        f += BRAKE_K * (g - ASSIST_GRADE)
    return f


# ── planning grid ─────────────────────────────────────────────────────────────
class PlanningGrid(object):
    """Inflated occupancy grid in world metres, plus an optional elevation model.

    `blocked` is row-major (index = iy * width + ix). `elevation` is an optional
    callable (x, y) -> height in metres; without it the floor is treated as flat.
    """

    def __init__(self, width, height, resolution, origin_x, origin_y, blocked,
                 elevation=None, max_slope_deg=15.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.blocked = list(blocked)
        self.elevation = elevation
        self.max_grade = math.tan(math.radians(max_slope_deg))

    # -- construction --------------------------------------------------------
    @classmethod
    def from_occupancy(cls, width, height, resolution, origin_x, origin_y, data,
                       plan_resolution=0.1, robot_radius=0.2, occupied_threshold=50,
                       unknown_is_blocked=True, elevation=None, max_slope_deg=15.0):
        """Build from nav_msgs/OccupancyGrid fields, downsampled to plan_resolution."""
        k = max(1, int(round(plan_resolution / resolution)))
        cw, ch = (width + k - 1) // k, (height + k - 1) // k
        coarse = [False] * (cw * ch)
        for iy in range(height):
            row = iy * width
            crow = (iy // k) * cw
            for ix in range(width):
                v = data[row + ix]
                if v >= occupied_threshold or (v < 0 and unknown_is_blocked):
                    coarse[crow + ix // k] = True
        grid = cls(cw, ch, resolution * k, origin_x, origin_y, coarse,
                   elevation=elevation, max_slope_deg=max_slope_deg)
        grid.inflate(robot_radius)
        return grid

    @classmethod
    def empty(cls, center_x, center_y, size=20.0, resolution=0.1, elevation=None,
              max_slope_deg=15.0):
        """Obstacle-free square grid centred on a point (used when no /map is available)."""
        n = int(math.ceil(size / resolution))
        return cls(n, n, resolution, center_x - size / 2.0, center_y - size / 2.0,
                   [False] * (n * n), elevation=elevation, max_slope_deg=max_slope_deg)

    def copy(self):
        return PlanningGrid(self.width, self.height, self.resolution, self.origin_x,
                            self.origin_y, self.blocked, self.elevation,
                            math.degrees(math.atan(self.max_grade)))

    def _disk(self, radius):
        r = int(math.ceil(radius / self.resolution))
        return [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                if (dx * dx + dy * dy) * self.resolution ** 2 <= radius * radius + 1e-9]

    def inflate(self, radius):
        """Grow every obstacle by `radius` metres (the robot's footprint)."""
        if radius <= 0:
            return
        disk = self._disk(radius)
        src = [i for i, b in enumerate(self.blocked) if b]
        for i in src:
            ix, iy = i % self.width, i // self.width
            for dx, dy in disk:
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.blocked[ny * self.width + nx] = True

    def add_obstacle_points(self, points, radius):
        """Mark world points (e.g. live LiDAR hits) as obstacles, inflated by radius."""
        disk = self._disk(radius)
        for x, y in points:
            ix, iy = self.world_to_cell(x, y)
            for dx, dy in disk:
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.blocked[ny * self.width + nx] = True

    # -- geometry ------------------------------------------------------------
    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def cell_to_world(self, ix, iy):
        return (self.origin_x + (ix + 0.5) * self.resolution,
                self.origin_y + (iy + 0.5) * self.resolution)

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def is_blocked(self, ix, iy):
        return (not self.in_bounds(ix, iy)) or self.blocked[iy * self.width + ix]

    def height_at(self, x, y):
        return self.elevation(x, y) if self.elevation else 0.0

    def step(self, a, b):
        """(distance m, signed grade, energy) for a move between neighbouring cells.

        energy is None when the slope is steeper than max_slope_deg.
        """
        ax, ay = self.cell_to_world(a[0], a[1])
        bx, by = self.cell_to_world(b[0], b[1])
        d = math.hypot(bx - ax, by - ay)
        grade = (self.height_at(bx, by) - self.height_at(ax, ay)) / d if d > 0 else 0.0
        if abs(grade) > self.max_grade:
            return d, grade, None
        return d, grade, d * grade_factor(grade)

    def nearest_free(self, cell, max_radius_cells=10):
        """The closest unblocked cell to `cell` (robot or goal inside an inflated wall)."""
        if not self.is_blocked(cell[0], cell[1]):
            return cell
        for r in range(1, max_radius_cells + 1):
            best = None
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    c = (cell[0] + dx, cell[1] + dy)
                    if not self.is_blocked(c[0], c[1]):
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, c)
            if best:
                return best[1]
        return None


def load_elevation_csv(path, resolution, origin_x, origin_y):
    """Height model from a CSV grid of heights in metres (row 0 = lowest y).

    Returns a callable (x, y) -> height with bilinear interpolation, clamped at the
    edges. Measure a ramp/slope in your arena and write it as a CSV to make the
    planners see uphill/downhill on the real robot.
    """
    import csv
    with open(path) as f:
        rows = [[float(v) for v in row if v.strip()] for row in csv.reader(f) if row]
    h, w = len(rows), len(rows[0])

    def at(x, y):
        fx = min(max((x - origin_x) / resolution, 0.0), w - 1.0)
        fy = min(max((y - origin_y) / resolution, 0.0), h - 1.0)
        x0, y0 = int(fx), int(fy)
        x1, y1 = min(x0 + 1, w - 1), min(y0 + 1, h - 1)
        tx, ty = fx - x0, fy - y0
        top = rows[y0][x0] * (1 - tx) + rows[y0][x1] * tx
        bot = rows[y1][x0] * (1 - tx) + rows[y1][x1] * tx
        return top * (1 - ty) + bot * ty
    return at


# ── grid search shared by all planners ────────────────────────────────────────
NEIGHBOURS = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def _search(grid, start, goal, edge_cost, heuristic, max_expansions=400000):
    """Best-first grid search. Returns (cells, expanded) or (None, expanded)."""
    g = {start: 0.0}
    parent = {start: None}
    closed = set()
    counter = 0
    heap = [(heuristic(start), counter, start)]
    expanded = 0
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == goal:
            cells = []
            while cur is not None:
                cells.append(cur)
                cur = parent[cur]
            cells.reverse()
            return cells, expanded
        if expanded > max_expansions:
            break
        for dx, dy in NEIGHBOURS:
            nb = (cur[0] + dx, cur[1] + dy)
            if nb in closed or grid.is_blocked(nb[0], nb[1]):
                continue
            # no corner cutting past blocked cells on diagonal moves
            if dx and dy and (grid.is_blocked(cur[0] + dx, cur[1]) or grid.is_blocked(cur[0], cur[1] + dy)):
                continue
            c = edge_cost(cur, nb)
            if c is None:
                continue
            ng = g[cur] + c
            if ng < g.get(nb, float("inf")):
                g[nb] = ng
                parent[nb] = cur
                counter += 1
                heapq.heappush(heap, (ng + heuristic(nb), counter, nb))
    return None, expanded


def _energy_cost(grid):
    def cost(a, b):
        return grid.step(a, b)[2]
    return cost


def _distance_cost(grid):
    # slope-blind: only refuses impassable slopes
    def cost(a, b):
        d, _, e = grid.step(a, b)
        return None if e is None else d
    return cost


def _euclid_heuristic(grid, goal, scale):
    gx, gy = goal

    def h(c):
        return math.hypot(c[0] - gx, c[1] - gy) * grid.resolution * scale
    return h


def _zero(_c):
    return 0.0


# ── planners ──────────────────────────────────────────────────────────────────
def dijkstra_cells(grid, start, goal):
    """Classic shortest-DISTANCE Dijkstra — the slope-blind baseline."""
    return _search(grid, start, goal, _distance_cost(grid), _zero)


def astar_cells(grid, start, goal):
    """Energy-aware A* with an admissible heuristic (lowest-energy path)."""
    return _search(grid, start, goal, _energy_cost(grid),
                   _euclid_heuristic(grid, goal, MIN_GRADE_FACTOR))


def _perturbed_astar_cells(grid, start, goal, seed):
    """A* with deterministic terrain-roughness noise — generates corridor alternatives."""
    base = _energy_cost(grid)

    def noise(c):
        v = math.sin(c[0] * 127.1 + c[1] * 311.7 + seed * 43758.5453) * 43758.5453
        return (v - math.floor(v)) * 0.3

    def cost(a, b):
        e = base(a, b)
        return None if e is None else e * (1.0 + noise(b))
    return _search(grid, start, goal, cost, _euclid_heuristic(grid, goal, MIN_GRADE_FACTOR))


def qaoa_cells(grid, start, goal, shots=2048, seed=12345, info=None):
    """QAOA-style corridor optimisation, matching the web sim's qaoaPath().

    1. A* backbone, split at anchors into segments.
    2. Each segment gets up to 3 alternative sub-paths (perturbed A*); one binary
       variable per (segment, choice) — the QUBO's edge-selection qubits.
    3. A Boltzmann distribution over feasible choices stands in for the trained
       QAOA state (sharper with more layers p); `shots` samples are drawn.
    4. Decode the MOST-SAMPLED bitstring, as a real QAOA run does — so with shallow
       p it can miss the optimum (no classical post-selection of the cheapest).
    """
    backbone, expanded = astar_cells(grid, start, goal)
    if backbone is None:
        return None, expanded
    n_anchors = min(6, max(3, len(backbone) // 8))
    idx = [int(round(a * (len(backbone) - 1) / (n_anchors - 1))) for a in range(n_anchors)]
    idx = sorted(set(idx))
    energy = _energy_cost(grid)

    def seg_energy(cells):
        return sum(energy(cells[i - 1], cells[i]) for i in range(1, len(cells)))

    choices_by_seg = []
    for si in range(len(idx) - 1):
        a, b = idx[si], idx[si + 1]
        options = [backbone[a:b + 1]]
        for alt in range(3):
            cells, n = _perturbed_astar_cells(grid, backbone[a], backbone[b], alt * 17 + si * 7 + 42)
            expanded += n
            if cells and len(cells) > 1 and cells not in options:
                options.append(cells)
        choices_by_seg.append([(c, seg_energy(c)) for c in options])

    n_qubits = sum(len(c) for c in choices_by_seg) + max(0, len(idx) - 2)
    reps = min(3, max(1, int(math.ceil(n_qubits / 6.0))))

    # enumerate feasible combinations (exactly one choice per segment)
    combos = [([], 0.0)]
    for seg in choices_by_seg:
        combos = [(path + [ci], e + seg[ci][1]) for path, e in combos for ci in range(len(seg))]
    energies = [e for _, e in combos]
    e_min, e_max = min(energies), max(energies)
    temperature = max((e_max - e_min) / (2.5 * reps), 1e-3)
    weights = [math.exp(-(e - e_min) / temperature) for e in energies]
    total = sum(weights)
    probs = [w / total for w in weights]

    cumulative, acc = [], 0.0
    for p in probs:
        acc += p
        cumulative.append(acc)
    rng = random.Random(seed)
    counts = [0] * len(combos)
    for _ in range(shots):
        counts[min(bisect.bisect_left(cumulative, rng.random()), len(combos) - 1)] += 1
    best = max(range(len(combos)), key=lambda i: (counts[i], -energies[i]))

    cells = []
    for si, ci in enumerate(combos[best][0]):
        seg = choices_by_seg[si][ci][0]
        cells.extend(seg if si == 0 else seg[1:])
    if info is not None:
        info.update({"qubits": n_qubits, "reps": reps, "shots": shots,
                     "expectation": sum(p * e for p, e in zip(probs, energies)),
                     "decoded_count": counts[best]})
    return cells, expanded


_PLANNERS = {"dijkstra": dijkstra_cells, "astar": astar_cells, "qaoa": qaoa_cells}


class PlanResult(object):
    """One planner's answer plus its metrics (energy, length, climb, ...)."""

    def __init__(self, algorithm, cells, points, energy, distance, climb, descent,
                 max_slope_deg, expanded, runtime_s, extra=None):
        self.algorithm = algorithm
        self.cells = cells
        self.points = points
        self.energy = energy
        self.distance = distance
        self.climb = climb
        self.descent = descent
        self.max_slope_deg = max_slope_deg
        self.expanded = expanded
        self.runtime_s = runtime_s
        self.extra = extra or {}

    def summary(self):
        d = {"algorithm": self.algorithm, "energy": round(self.energy, 3),
             "distance_m": round(self.distance, 3), "climb_m": round(self.climb, 3),
             "descent_m": round(self.descent, 3), "max_slope_deg": round(self.max_slope_deg, 2),
             "nodes": self.expanded, "runtime_ms": round(self.runtime_s * 1000.0, 2)}
        d.update(self.extra)
        return d


def evaluate_cells(grid, cells):
    """(energy, distance, climb, descent, max_slope_deg) of a cell path, using the real model."""
    energy = dist = climb = descent = max_grade = 0.0
    for i in range(1, len(cells)):
        d, grade, e = grid.step(cells[i - 1], cells[i])
        energy += e if e is not None else float("inf")
        dist += d
        dh = grade * d
        if dh > 0:
            climb += dh
        else:
            descent -= dh
        max_grade = max(max_grade, abs(grade))
    return energy, dist, climb, descent, math.degrees(math.atan(max_grade))


def plan(grid, algorithm, start_xy, goal_xy):
    """Run one planner from start to goal (world metres). Returns PlanResult or None."""
    if algorithm not in _PLANNERS:
        raise ValueError("unknown algorithm %r (use one of %s)" % (algorithm, ", ".join(ALGORITHMS)))
    start = grid.nearest_free(grid.world_to_cell(*start_xy))
    goal = grid.nearest_free(grid.world_to_cell(*goal_xy))
    if start is None or goal is None:
        return None
    extra = {}
    t0 = time.time()
    if algorithm == "qaoa":
        cells, expanded = qaoa_cells(grid, start, goal, info=extra)
    else:
        cells, expanded = _PLANNERS[algorithm](grid, start, goal)
    runtime = time.time() - t0
    if not cells:
        return None
    energy, dist, climb, descent, max_slope = evaluate_cells(grid, cells)
    points = [grid.cell_to_world(c[0], c[1]) for c in cells]
    return PlanResult(algorithm, cells, points, energy, dist, climb, descent, max_slope,
                      expanded, runtime, extra)


def rank(results):
    """Sort PlanResults best-first: lowest energy, ties (< 0.01) broken by runtime."""
    def cmp(a, b):
        de = a.energy - b.energy
        if abs(de) >= ENERGY_TIE:
            return -1 if de < 0 else 1
        return -1 if a.runtime_s < b.runtime_s else (1 if a.runtime_s > b.runtime_s else 0)
    return sorted([r for r in results if r is not None], key=functools.cmp_to_key(cmp))


def plan_all(grid, start_xy, goal_xy):
    """Run every planner; returns {algorithm: PlanResult or None}."""
    return dict((a, plan(grid, a, start_xy, goal_xy)) for a in ALGORITHMS)


def thin_path(points, spacing=0.3, turn_deg=20.0):
    """Drop grid-step waypoints: keep corners and roughly one point per `spacing` metres."""
    if len(points) <= 2:
        return list(points)
    out = [points[0]]
    acc = 0.0
    for i in range(1, len(points) - 1):
        px, py = points[i - 1]
        cx, cy = points[i]
        nx, ny = points[i + 1]
        acc += math.hypot(cx - px, cy - py)
        h1 = math.atan2(cy - py, cx - px)
        h2 = math.atan2(ny - cy, nx - cx)
        turn = abs(math.degrees(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))))
        if acc >= spacing or turn >= turn_deg:
            out.append(points[i])
            acc = 0.0
    out.append(points[-1])
    return out


# ── waypoint following (port of navigation_pkg/path_following.py) ────────────
ANGULAR_GAIN = 2.2           # rad/s of turn rate per rad of heading error
LINEAR_GAIN = 0.9            # m/s of forward speed per metre to the waypoint
SLOWDOWN_YAW_RAD = 1.2       # forward speed falls to zero at this heading error
TURN_IN_PLACE_YAW_RAD = 0.85  # beyond this heading error, creep forward only
TURN_IN_PLACE_LINEAR = 0.03


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quaternion(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(s)


def compute_command(x, y, yaw, goal_x, goal_y, max_linear, max_angular):
    """Proportional controller: (linear m/s, angular rad/s) towards one waypoint."""
    dx, dy = goal_x - x, goal_y - y
    distance = math.hypot(dx, dy)
    yaw_error = wrap_angle(math.atan2(dy, dx) - yaw)
    angular = max(-max_angular, min(max_angular, ANGULAR_GAIN * yaw_error))
    speed_scale = max(0.0, 1.0 - abs(yaw_error) / SLOWDOWN_YAW_RAD)
    linear = max(0.0, min(max_linear, LINEAR_GAIN * distance * speed_scale))
    if abs(yaw_error) > TURN_IN_PLACE_YAW_RAD:
        linear = min(TURN_IN_PLACE_LINEAR, max_linear)
    return linear, angular


class WaypointFollower(object):
    """Walks a list of (x, y) waypoints; call step() once per control tick.

    step() returns (linear, angular, finished).
    """

    def __init__(self, waypoints, goal_tolerance=0.15, waypoint_tolerance=0.2,
                 max_linear=0.15, max_angular=0.8):
        if not waypoints:
            raise ValueError("WaypointFollower needs at least one waypoint.")
        self.waypoints = [(float(px), float(py)) for px, py in waypoints]
        self.goal_tolerance = goal_tolerance
        self.waypoint_tolerance = waypoint_tolerance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.index = 0
        self.done = False

    def step(self, x, y, yaw, max_linear=None):
        if self.done:
            return 0.0, 0.0, True
        # skip every waypoint already within tolerance (dense paths)
        while True:
            gx, gy = self.waypoints[self.index]
            final = self.index == len(self.waypoints) - 1
            tol = self.goal_tolerance if final else self.waypoint_tolerance
            if math.hypot(gx - x, gy - y) > tol:
                break
            self.index += 1
            if self.index >= len(self.waypoints):
                self.done = True
                return 0.0, 0.0, True
        lim = self.max_linear if max_linear is None else min(self.max_linear, max_linear)
        lin, ang = compute_command(x, y, yaw, gx, gy, lim, self.max_angular)
        return lin, ang, False


def uphill_speed_cap(max_linear, pitch_rad):
    """Slow down when climbing, like the web sim (descending does not speed up)."""
    grade = math.tan(pitch_rad)
    return max_linear / grade_factor(grade) if grade > 0 else max_linear


# ── run energy meter (same power model as the web sim's energy sensor) ────────
class EnergyMeter(object):
    """P = P_idle + k_lin|v| + k_ang|w| + gravity term (with partial regen downhill)."""

    def __init__(self, mass=5.0, p_idle=3.0, k_linear=15.0, k_angular=4.0, regen=0.3,
                 gravity=9.81):
        self.mass, self.p_idle, self.k_linear, self.k_angular = mass, p_idle, k_linear, k_angular
        self.regen, self.gravity = regen, gravity
        self.reset()

    def reset(self):
        self.joules = 0.0
        self.distance = 0.0
        self.climb = 0.0
        self.descent = 0.0
        self.elapsed = 0.0

    def update(self, linear, angular, pitch_rad, dt):
        if dt <= 0:
            return
        pg = self.mass * self.gravity * linear * math.sin(pitch_rad)
        if pg < 0:
            pg *= self.regen
        power = max(0.5 * self.p_idle,
                    self.p_idle + self.k_linear * abs(linear) + self.k_angular * abs(angular) + pg)
        self.joules += power * dt
        self.distance += abs(linear) * dt
        dz = linear * dt * math.sin(pitch_rad)
        if dz > 0:
            self.climb += dz
        else:
            self.descent -= dz
        self.elapsed += dt
