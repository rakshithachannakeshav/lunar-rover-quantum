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
        self.soft = None    # optional extra search cost near obstacles, see add_soft_margin()

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

    @classmethod
    def arena(cls, x0, y0, width, height, resolution=0.05, pad=0.3, wall_margin=0.0,
              robot_radius=0.0, elevation=None, max_slope_deg=15.0):
        """Grid for a walled arena [x0, x0+width] x [y0, y0+height]: everything outside it
        (and a `wall_margin` strip just inside, the border wall) is blocked, then grown by
        robot_radius. Driving off the table edge is therefore never planned."""
        nx = int(math.ceil((width + 2 * pad) / resolution))
        ny = int(math.ceil((height + 2 * pad) / resolution))
        ox, oy = x0 - pad, y0 - pad
        blocked = [False] * (nx * ny)
        lo_x, hi_x = x0 + wall_margin, x0 + width - wall_margin
        lo_y, hi_y = y0 + wall_margin, y0 + height - wall_margin
        for iy in range(ny):
            cy = oy + (iy + 0.5) * resolution
            for ix in range(nx):
                cx = ox + (ix + 0.5) * resolution
                if not (lo_x <= cx <= hi_x and lo_y <= cy <= hi_y):
                    blocked[iy * nx + ix] = True
        grid = cls(nx, ny, resolution, ox, oy, blocked, elevation=elevation,
                   max_slope_deg=max_slope_deg)
        grid.inflate_edges(robot_radius)
        return grid

    def inflate_edges(self, radius):
        """Like inflate(), but only grows blocked cells that touch a free one (fast for a
        big blocked surround)."""
        if radius <= 0:
            return
        edge = []
        for i, b in enumerate(self.blocked):
            if not b:
                continue
            ix, iy = i % self.width, i // self.width
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and \
                        not self.blocked[ny * self.width + nx]:
                    edge.append(i)
                    break
        disk = self._disk(radius)
        for i in edge:
            ix, iy = i % self.width, i // self.width
            for dx, dy in disk:
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.blocked[ny * self.width + nx] = True

    def add_soft_margin(self, radius, weight=1.5):
        """Make cells within `radius` metres of a blocked cell more expensive to search through
        (extra cost up to `weight` x the move cost, fading to 0 at `radius`). Paths then keep
        some distance from obstacles where there is room, but can still squeeze through gaps.
        Only the search is affected; the reported energy / distance stay the real ones."""
        if radius <= 0 or weight <= 0:
            self.soft = None
            return
        soft = [0.0] * len(self.blocked)
        disk = [(dx, dy, math.hypot(dx, dy) * self.resolution) for dx, dy in self._disk(radius)]
        for i, b in enumerate(self.blocked):
            if not b:
                continue
            ix, iy = i % self.width, i // self.width
            for dx, dy, d in disk:
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    j = ny * self.width + nx
                    if not self.blocked[j]:
                        v = weight * (1.0 - d / radius)
                        if v > soft[j]:
                            soft[j] = v
        self.soft = soft

    def penalty(self, cell):
        return self.soft[cell[1] * self.width + cell[0]] if self.soft else 0.0

    def add_disc(self, x, y, radius):
        """Block every cell whose centre is within `radius` metres of the world point."""
        r_cells = int(math.ceil(radius / self.resolution)) + 1
        ix, iy = self.world_to_cell(x, y)
        r2 = radius * radius
        for cy in range(max(0, iy - r_cells), min(self.height, iy + r_cells + 1)):
            for cx in range(max(0, ix - r_cells), min(self.width, ix + r_cells + 1)):
                wx, wy = self.cell_to_world(cx, cy)
                if (wx - x) ** 2 + (wy - y) ** 2 <= r2:
                    self.blocked[cy * self.width + cx] = True

    def copy(self):
        g = PlanningGrid(self.width, self.height, self.resolution, self.origin_x,
                         self.origin_y, self.blocked, self.elevation,
                         math.degrees(math.atan(self.max_grade)))
        g.soft = list(self.soft) if self.soft else None
        return g

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
        e = grid.step(a, b)[2]
        return e if e is None or grid.soft is None else e * (1.0 + grid.penalty(b))
    return cost


def _distance_cost(grid):
    # slope-blind: only refuses impassable slopes
    def cost(a, b):
        d, _, e = grid.step(a, b)
        return None if e is None else d * (1.0 + grid.penalty(b))
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

    def retarget_nearest(self, x, y):
        """After a detour, aim at the nearest waypoint not yet reached (never backwards)."""
        if self.done:
            return
        self.index = min(range(self.index, len(self.waypoints)),
                         key=lambda i: math.hypot(self.waypoints[i][0] - x, self.waypoints[i][1] - y))

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


# ── lidar obstacle avoidance (port of navigation_pkg/path_following.py) ──────
# Distances are tuned for the Arjuna kit (~0.4 m footprint, 0.10-0.12 m/s).
FRONT_HALF_ANGLE = math.radians(30.0)   # front sector: +-30 deg
SIDE_MAX_ANGLE = math.radians(100.0)    # side sectors: 30..100 deg each way
REAR_HALF_ANGLE = math.radians(30.0)    # rear sector, checked before reversing
SELF_FILTER_RANGE = 0.12                # returns this close are the robot itself


class ScanSectors(object):
    """A scan reduced to front (nearest), left/right (mean clearance), rear (nearest),
    and left_near/right_near (nearest return on each flank, for side clearance)."""

    def __init__(self, front, left, right, rear=float("inf"),
                 left_near=float("inf"), right_near=float("inf")):
        self.front = front
        self.left = left
        self.right = right
        self.rear = rear
        self.left_near = left_near
        self.right_near = right_near


def scan_sectors(ranges, angle_min, angle_increment, range_max, range_min=0.0,
                 yaw_offset=0.0, self_filter=SELF_FILTER_RANGE):
    """Reduce LaserScan ranges to ScanSectors (angle 0 = straight ahead after yaw_offset).

    inf / nan / out-of-range returns count as range_max (nothing seen).
    """
    floor = max(range_min, self_filter)
    front = rear = range_max
    left, right = [], []
    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or r < floor:
            continue
        r = min(float(r), range_max)
        a = wrap_angle(angle_min + i * angle_increment + yaw_offset)
        if abs(a) <= FRONT_HALF_ANGLE:
            front = min(front, r)
        elif FRONT_HALF_ANGLE < a <= SIDE_MAX_ANGLE:
            left.append(r)
        elif -SIDE_MAX_ANGLE <= a < -FRONT_HALF_ANGLE:
            right.append(r)
        elif abs(a) >= math.pi - REAR_HALF_ANGLE:
            rear = min(rear, r)
    return ScanSectors(front,
                       sum(left) / len(left) if left else range_max,
                       sum(right) / len(right) if right else range_max,
                       rear,
                       min(left) if left else range_max,
                       min(right) if right else range_max)


class ObstacleAvoider(object):
    """Lidar reflex that takes over from path following while something is in the way.

    clear -> (front < stop) -> turning away towards the more open side
    turning -> (front > clear) -> bypassing: straight ahead for commit_distance
    bypassing -> clear (detour finished), or back to turning if blocked again.
    backing (reverse, only if the rear is clear) runs first when very close.
    Flank guard: whenever something is closer than side_clearance on one side,
    keep moving but veer away from it (the front cone alone misses the robot's
    sides clipping an obstacle while driving past it).

    The chosen side sticks until reset_side() (called when a waypoint is reached)
    so the robot does not dither left/right in front of one obstacle.
    update() returns ((linear, angular) or None to follow the path, detour_finished).
    """

    CLEAR, BACKING, TURNING, BYPASSING = "clear", "backing", "turning", "bypassing"

    def __init__(self, stop_distance=0.40, clear_distance=0.60, backup_distance=0.22,
                 commit_distance=0.40, rear_clearance=0.25, max_linear=0.12,
                 max_angular=0.6, backup_speed=0.05, side_clearance=0.32):
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.backup_distance = backup_distance
        self.commit_distance = commit_distance
        self.rear_clearance = rear_clearance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.backup_speed = backup_speed
        self.side_clearance = side_clearance
        self.state = self.CLEAR
        self.side = None          # +1 = turn left, -1 = turn right
        self._bypass_from = (0.0, 0.0)

    @property
    def active(self):
        return self.state != self.CLEAR

    def reset_side(self):
        if not self.active:
            self.side = None

    def _flank_guard(self, scan):
        """Veer away from an obstacle too close on one side, or None if both sides are clear."""
        if min(scan.left_near, scan.right_near) >= self.side_clearance:
            return None
        away = -1 if scan.left_near < scan.right_near else 1
        return (0.5 * self.max_linear, away * 0.7 * self.max_angular)

    def update(self, scan, x, y):
        if self.state == self.CLEAR:
            if scan.front >= self.stop_distance:
                guard = self._flank_guard(scan)
                return guard, False
            if self.side is None:
                self.side = 1 if scan.left >= scan.right else -1
            self.state = self.BACKING if scan.front < self.backup_distance else self.TURNING

        if self.state == self.BACKING:
            if scan.front < self.backup_distance + 0.08 and scan.rear > self.rear_clearance:
                return (-self.backup_speed, 0.0), False
            self.state = self.TURNING  # backed off enough, or the rear is blocked: turn in place

        if self.state == self.TURNING:
            if scan.front < self.clear_distance:
                return (0.0, self.side * self.max_angular), False
            self.state = self.BYPASSING
            self._bypass_from = (x, y)

        # bypassing
        if scan.front < self.stop_distance:
            self.state = self.BACKING if scan.front < self.backup_distance else self.TURNING
            return self.update(scan, x, y)
        if math.hypot(x - self._bypass_from[0], y - self._bypass_from[1]) < self.commit_distance:
            return self._flank_guard(scan) or (0.7 * self.max_linear, 0.0), False
        self.state = self.CLEAR
        return None, True


class ProgressWatchdog(object):
    """True once the robot has driven `travel` metres without getting `step` metres
    closer to its current target: circling an obstacle or sliding along a wall.
    The avoider's own timer misses this, because the robot keeps switching between
    following the path and veering away, so no single avoidance lasts long."""

    def __init__(self, travel=3.0, step=0.15):
        self.travel_limit = travel
        self.step = step
        self.reset()

    def reset(self):
        self.target = None
        self.best = float("inf")
        self.travel = 0.0
        self.last = None

    def update(self, x, y, target):
        if self.last is not None:
            self.travel += math.hypot(x - self.last[0], y - self.last[1])
        self.last = (x, y)
        d = math.hypot(target[0] - x, target[1] - y)
        if target != self.target:
            self.target, self.best, self.travel = target, d, 0.0
        elif d < self.best - self.step:
            self.best, self.travel = d, 0.0
        return self.travel > self.travel_limit


# ── craters (ground dips) ─────────────────────────────────────────────────────
def robot_to_world(pose, points):
    """Robot-frame (forward, left) points -> world (x, y), given pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + fx * c - ly * s, y + fx * s + ly * c) for fx, ly in points]


def point_segment_distance(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    seg2 = dx * dx + dy * dy
    t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def points_near_path(points, current_xy, waypoints, start_index, clearance):
    """True if any point lies within `clearance` of the path still to drive:
    the robot's position, then waypoints[start_index:]."""
    route = [tuple(current_xy)] + [tuple(w) for w in waypoints[start_index:]]
    if len(route) == 1:
        route.append(route[0])
    for p in points:
        for a, b in zip(route[:-1], route[1:]):
            if point_segment_distance(p, a, b) <= clearance:
                return True
    return False


def discs_near_path(discs, current_xy, waypoints, start_index, clearance):
    """True if any disc (x, y, r) touches the path still to drive, with `clearance` to spare."""
    route = [tuple(current_xy)] + [tuple(w) for w in waypoints[start_index:]]
    if len(route) == 1:
        route.append(route[0])
    for x, y, r in discs:
        for a, b in zip(route[:-1], route[1:]):
            if point_segment_distance((x, y), a, b) <= r + clearance:
                return True
    return False


class CraterMap(object):
    """World positions of detected craters, de-duplicated on a `cell` grid so the same
    crater seen in many camera frames is stored once. Points persist for the session."""

    def __init__(self, cell=0.05):
        self.cell = cell
        self._keys = set()
        self.points = []

    def add(self, points):
        """Add world (x, y) points; returns only the ones not already known."""
        new = []
        for x, y in points:
            key = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
            if key not in self._keys:
                self._keys.add(key)
                self.points.append((x, y))
                new.append((x, y))
        return new

    def clear(self):
        self._keys.clear()
        del self.points[:]


# ── terrain seen by the depth camera, kept in the world frame ─────────────────
class TerrainMap(object):
    """Craters, rocks, border wall and no-floor regions in the WORLD frame, for the whole run.

    Detections arrive in the robot frame (terrain_core.py). They are moved to the world frame
    with the pose they were seen from and merged:
      * craters are circles; the same crater seen again updates one circle (a view that shows
        more of the rim wins), so a first glimpse of half a rim does not leave a ghost behind;
      * rocks / border / no-floor are cells on a grid; a cell has to be seen `confirm` times.
    Nothing is removed, because the arena does not change; clear() forgets everything.
    """

    KINDS = ("rock", "border", "nofloor", "rim")

    def __init__(self, cell=0.04, confirm=None, bounds=None):
        self.cell = cell
        # (x0, y0, x1, y1) of the arena, if known: nothing outside the table matters, so
        # detections there (the far side of the border wall, the void) are dropped
        self.bounds = bounds
        self.confirm = {"crater": 2, "rock": 2, "border": 2, "nofloor": 3, "rim": 2}
        if confirm:
            self.confirm.update(confirm)
        self.craters = []                       # dicts: x, y, r, span, hits, src
        self.cells = dict((k, {}) for k in self.KINDS)   # kind -> {key: [x, y, hits]}
        self.version = 0                        # bumps whenever the confirmed map changes

    # -- feeding -------------------------------------------------------------
    def add(self, pose, detections):
        """Merge robot-frame detections seen from `pose` (x, y, yaw).
        Returns the confirmed changes: ("crater", x, y, r) and ("cells", kind, [(x, y), ...])."""
        changes = []
        for d in detections:
            kind = d.get("type")
            if kind == "crater" and not d.get("partial") and d.get("r", 0) > 0:
                c = self._add_crater(pose, d)
                if c:
                    changes.append(c)
                continue
            if kind == "crater":
                kind = "rim"
            if kind not in self.cells:
                continue
            world = robot_to_world(pose, [(float(x), float(y)) for x, y in d.get("cells", [])])
            if kind == "rock" and self.bounds is not None:
                # a short piece of the border wall can look like a rock: along the edge of the
                # table it is the wall
                world_rock = [w for w in world if not self._on_edge(*w)]
                world_wall = [w for w in world if self._on_edge(*w)]
                new = self._add_cells("rock", world_rock) if world_rock else []
                wall = self._add_cells("border", world_wall) if world_wall else []
                if wall:
                    changes.append(("cells", "border", wall))
                if new:
                    changes.append(("cells", kind, new))
                continue
            new = self._add_cells(kind, world)
            if new:
                changes.append(("cells", kind, new))
        if changes:
            self.version += 1
        return changes

    def _on_edge(self, x, y, band=0.08):
        b = self.bounds
        return min(x - b[0], b[2] - x, y - b[1], b[3] - y) < band

    def _outside(self, x, y, slack=0.0):
        b = self.bounds
        return b is not None and not (b[0] - slack <= x <= b[2] + slack and
                                      b[1] - slack <= y <= b[3] + slack)

    def _add_crater(self, pose, d):
        (x, y), = robot_to_world(pose, [(float(d["x"]), float(d["y"]))])
        if self._outside(x, y, slack=0.05):
            return None
        r, span = float(d.get("fill_r", d["r"])), float(d.get("span", 0.0))
        need = self.confirm["crater"]
        for c in self.craters:
            if c["src"] != "camera":
                continue
            if math.hypot(c["x"] - x, c["y"] - y) <= 0.6 * max(c["r"], r) + 0.05:
                c["hits"] += 1
                if span >= c["span"] + 0.5:         # a better view of the rim: take its circle
                    c["x"], c["y"], c["r"], c["span"] = x, y, r, span
                elif span >= c["span"] - 0.5:       # comparable view: average the two
                    c["x"], c["y"], c["r"] = (0.5 * (c["x"] + x), 0.5 * (c["y"] + y),
                                              0.5 * (c["r"] + r))
                    c["span"] = max(c["span"], span)
                if c["hits"] >= need:
                    return ("crater", c["x"], c["y"], c["r"])
                return None
        self.craters.append({"x": x, "y": y, "r": r, "span": span, "hits": 1, "src": "camera"})
        return ("crater", x, y, r) if need <= 1 else None

    def _add_cells(self, kind, world):
        store = self.cells[kind]
        need = self.confirm[kind]
        new = []
        for x, y in world:
            if self._outside(x, y):
                continue
            key = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
            rec = store.get(key)
            if rec is None:
                store[key] = rec = [x, y, 0]
            rec[2] += 1
            if rec[2] == need:
                new.append((rec[0], rec[1]))
        return new

    def add_mark(self, x, y, r, src="bump"):
        """A crater edge felt by the IMU: a small blocked disc, confirmed at once."""
        for c in self.craters:
            if c["src"] == src and math.hypot(c["x"] - x, c["y"] - y) < r:
                return None
        self.craters.append({"x": x, "y": y, "r": r, "span": 0.0, "hits": 99, "src": src})
        self.version += 1
        return ("crater", x, y, r)

    def clear(self):
        del self.craters[:]
        for k in self.cells:
            self.cells[k].clear()
        self.version += 1

    # -- reading -------------------------------------------------------------
    def discs(self):
        """Confirmed craters as (x, y, r)."""
        return [(c["x"], c["y"], c["r"]) for c in self.craters
                if c["hits"] >= self.confirm["crater"]]

    def points(self, kinds=None):
        """Confirmed cell centres (x, y) of the given kinds (default: all cell kinds)."""
        out = []
        for k in (kinds or self.KINDS):
            need = self.confirm[k]
            out.extend((r[0], r[1]) for r in self.cells[k].values() if r[2] >= need)
        return out

    def blocking(self, x, y, pad=0.0):
        """Name of the detected object that the point lies on (within `pad` of it), else None."""
        for cx, cy, r in self.discs():
            if math.hypot(x - cx, y - cy) <= r + pad:
                return "crater"
        half = self.cell * 0.75 + pad
        names = {"rim": "crater", "nofloor": "table edge", "border": "border", "rock": "rock"}
        for k in ("rock", "border", "nofloor", "rim"):
            need = self.confirm[k]
            for r in self.cells[k].values():
                if r[2] >= need and abs(r[0] - x) <= half and abs(r[1] - y) <= half:
                    return names[k]
        return None

    def snapshot(self, clearance=0.0, max_cells=700):
        """JSON-able summary for the web page: circles and cell lists, in the world frame."""
        out = {"clearance": round(clearance, 3), "version": self.version,
               "craters": [{"x": round(c["x"], 3), "y": round(c["y"], 3), "r": round(c["r"], 3),
                            "src": c["src"]} for c in self.craters
                           if c["hits"] >= self.confirm["crater"]]}
        budget = max_cells
        for k in ("rock", "border", "nofloor", "rim"):
            pts = [[round(x, 3), round(y, 3)] for x, y in self.points([k])]
            out[k] = pts[:budget]
            budget = max(0, budget - len(out[k]))
        return out


# ── IMU bump (a wheel climbing a 2 cm crater rim) ─────────────────────────────
class BumpDetector(object):
    """Spots the short pitch / roll jolt of a wheel climbing a raised crater rim.

    A slow average of pitch and roll is the baseline (ramps and a tilted table follow it);
    a deviation from it larger than the threshold, while the rover is driving, is a bump.
    update() returns None or {"kind": "nose_up" | "nose_down" | "roll", "deg": magnitude}."""

    def __init__(self, pitch_deg=2.5, roll_deg=3.0, tau=1.5, samples=2, holdoff=3.0):
        self.pitch_thr = math.radians(pitch_deg)
        self.roll_thr = math.radians(roll_deg)
        self.tau, self.samples, self.holdoff = tau, samples, holdoff
        self.enabled = pitch_deg > 0
        self.reset()

    def reset(self):
        self.base_p = self.base_r = None
        self.count = 0
        self.last_t = None
        self.quiet_until = 0.0

    def update(self, t, pitch, roll, driving):
        if not self.enabled:
            return None
        if self.base_p is None:
            self.base_p, self.base_r, self.last_t = pitch, roll, t
            return None
        dt = max(0.0, min(t - self.last_t, 1.0))
        self.last_t = t
        dp, dr = pitch - self.base_p, roll - self.base_r
        over_p, over_r = abs(dp) > self.pitch_thr, abs(dr) > self.roll_thr
        if not (over_p or over_r):
            alpha = 1.0 - math.exp(-dt / self.tau)
            self.base_p += alpha * dp
            self.base_r += alpha * dr
            self.count = 0
            return None
        if not driving or t < self.quiet_until:
            return None
        self.count += 1
        if self.count < self.samples:
            return None
        self.count = 0
        self.quiet_until = t + self.holdoff
        if over_p and (abs(dp) / self.pitch_thr >= abs(dr) / self.roll_thr):
            return {"kind": "nose_up" if dp > 0 else "nose_down", "deg": math.degrees(abs(dp))}
        return {"kind": "roll", "deg": math.degrees(abs(dr))}


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
