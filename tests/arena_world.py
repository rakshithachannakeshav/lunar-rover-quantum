"""Synthetic lunar arena + OAK-D-like depth renderer for tests (numpy only, no ROS).

World (metres, origin at one corner of the table, x/y along its edges):
  * flat floor inside [0, size]^2; beyond the table edge there is NO floor
  * low border wall just inside the edge
  * craters: octagonal raised rim (apothem a = R cos 22.5 deg), `rim_w` wide, `rim_h` high;
    the floor inside is flat (a crater is NOT a hole)
  * rocks: boxes (rotated), tall, with shiny plastic = depth dropouts and spikes

`render()` casts one ray per requested pixel analytically and returns a uint16 depth image in
millimetres (0 = invalid), like the OAK-D stereo depth output.
"""

import math

import numpy as np

SQ = math.cos(math.radians(22.5))


class Crater(object):
    def __init__(self, x, y, diameter, rim_h=0.02, rim_w=0.02, rot_deg=0.0):
        self.x, self.y = x, y
        self.R = diameter / 2.0                # circumradius: "across" the octagon
        self.rim_h, self.rim_w = rim_h, rim_w
        self.rot = math.radians(rot_deg)

    @property
    def apothem(self):
        return self.R * SQ

    def normals(self):
        return [(math.cos(self.rot + math.radians(22.5 + 45 * k)),
                 math.sin(self.rot + math.radians(22.5 + 45 * k))) for k in range(8)]

    def dist(self, px, py):
        """Octagon 'distance': max over faces of the signed distance along the face normal."""
        d = None
        for nx, ny in self.normals():
            v = (px - self.x) * nx + (py - self.y) * ny
            d = v if d is None else np.maximum(d, v)
        return d

    def contains(self, px, py, pad=0.0):
        return self.dist(px, py) <= self.apothem + pad


class Rock(object):
    def __init__(self, x, y, width, depth, height, rot_deg=0.0):
        self.x, self.y = x, y
        self.hw, self.hd, self.h = width / 2.0, depth / 2.0, height
        self.rot = math.radians(rot_deg)

    def local(self, px, py):
        c, s = math.cos(self.rot), math.sin(self.rot)
        dx, dy = px - self.x, py - self.y
        return dx * c + dy * s, -dx * s + dy * c

    def contains(self, px, py, pad=0.0):
        lx, ly = self.local(px, py)
        return (np.abs(lx) <= self.hw + pad) & (np.abs(ly) <= self.hd + pad)


class Arena(object):
    def __init__(self, size=2.21, craters=(), rocks=(), wall_h=0.05, wall_t=0.03):
        self.size = size
        self.craters, self.rocks = list(craters), list(rocks)
        self.wall_h, self.wall_t = wall_h, wall_t

    def inside(self, px, py, margin=0.0):
        return (px >= margin) & (px <= self.size - margin) & (py >= margin) & (py <= self.size - margin)

    def in_rock(self, x, y, pad=0.0):
        return any(bool(r.contains(x, y, pad)) for r in self.rocks)

    def in_crater(self, x, y, pad=0.0):
        return any(bool(c.contains(x, y, pad)) for c in self.craters)

    def on_rim(self, x, y, pad=0.0):
        """True if (x, y) is on a crater rim, or within `pad` of it (a wheel is not a point)."""
        for c in self.craters:
            d = c.dist(x, y)
            if c.apothem - c.rim_w - pad <= d <= c.apothem + pad:
                return True
        return False


def _slab(o, d, lo, hi):
    """Ray/slab interval along one axis. o, d arrays; lo, hi scalars."""
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - o) / d, (hi - o) / d
    tn, tf = np.minimum(t1, t2), np.maximum(t1, t2)
    parallel = np.abs(d) < 1e-12
    inside = (o >= lo) & (o <= hi)
    tn = np.where(parallel, np.where(inside, -np.inf, np.inf), tn)
    tf = np.where(parallel, np.where(inside, np.inf, -np.inf), tf)
    return tn, tf


def _box_hit(o, d, lo, hi):
    """Entry distance of rays (o, d: N x 3) into the axis-aligned box lo..hi (3-tuples);
    inf when missed. Returns (t, axis of the hit face)."""
    tn, tf = -np.inf * np.ones(len(o)), np.inf * np.ones(len(o))
    axis = np.zeros(len(o), dtype=np.int8)
    for k in range(3):
        a, b = _slab(o[:, k], d[:, k], lo[k], hi[k])
        better = a > tn
        axis = np.where(better, k, axis)
        tn = np.maximum(tn, a)
        tf = np.minimum(tf, b)
    hit = (tn <= tf) & (tf > 0) & (tn > 0)
    return np.where(hit, tn, np.inf), axis


class Camera(object):
    def __init__(self, intr, height=0.25, tilt_deg=30.0, forward=0.10):
        self.intr, self.height = intr, height
        self.tilt = math.radians(tilt_deg)
        self.forward = forward


def camera_rays(cam, pose, u, v):
    x, y, yaw = pose
    h = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    L = math.cos(cam.tilt) * h + np.array([0.0, 0.0, -math.sin(cam.tilt)])
    D = -math.sin(cam.tilt) * h + np.array([0.0, 0.0, -math.cos(cam.tilt)])
    R = np.cross(D, L)
    origin = np.array([x + cam.forward * math.cos(yaw), y + cam.forward * math.sin(yaw), cam.height])
    i = cam.intr
    a = ((u - i.cx) / i.fx)[:, None]
    b = ((v - i.cy) / i.fy)[:, None]
    dirs = a * R + b * D + L          # unit lens-axis component: t is the depth along the lens axis
    return np.tile(origin, (len(u), 1)), dirs


def render(arena, cam, pose, u, v, noise=True, seed=0, ground_below_edge=False,
           shiny_dropout=0.35, shiny_spike=0.04, noise_scale=1.0, dropout=0.004):
    """Depth image (uint16 mm, 0 = invalid) with values only at the sampled pixels (u, v)."""
    rs = np.random.RandomState(seed)
    o, d = camera_rays(cam, pose, u, v)
    n = len(u)
    best = np.full(n, np.inf)
    shiny = np.zeros(n, dtype=bool)

    # floor, only inside the table
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -o[:, 2] / d[:, 2]
    t = np.where((d[:, 2] < 0) & (t > 0), t, np.inf)
    px, py = o[:, 0] + t * d[:, 0], o[:, 1] + t * d[:, 1]
    on_table = arena.inside(px, py)
    if ground_below_edge:                      # floor 0.7 m lower beyond the edge, still measured
        with np.errstate(divide="ignore", invalid="ignore"):
            tg = (-0.7 - o[:, 2]) / d[:, 2]
        tg = np.where((d[:, 2] < 0) & (tg > 0), tg, np.inf)
        best = np.where(on_table, t, tg)
    else:
        best = np.where(on_table, t, np.inf)

    # crater rims (top faces)
    for c in arena.craters:
        with np.errstate(divide="ignore", invalid="ignore"):
            tr = (c.rim_h - o[:, 2]) / d[:, 2]
        tr = np.where((d[:, 2] < 0) & (tr > 0), tr, np.inf)
        rx, ry = o[:, 0] + tr * d[:, 0], o[:, 1] + tr * d[:, 1]
        dd = c.dist(rx, ry)
        hit = (dd >= c.apothem - c.rim_w) & (dd <= c.apothem) & (tr < best)
        best = np.where(hit, tr, best)

    # rocks (rotated boxes): rotate the rays into the box frame
    for r in arena.rocks:
        cs, sn = math.cos(r.rot), math.sin(r.rot)
        ox, oy = o[:, 0] - r.x, o[:, 1] - r.y
        lo_ = np.stack([ox * cs + oy * sn, -ox * sn + oy * cs, o[:, 2]], axis=1)
        ld = np.stack([d[:, 0] * cs + d[:, 1] * sn, -d[:, 0] * sn + d[:, 1] * cs, d[:, 2]], axis=1)
        tb, _ = _box_hit(lo_, ld, (-r.hw, -r.hd, 0.0), (r.hw, r.hd, r.h))
        hit = tb < best
        best = np.where(hit, tb, best)
        shiny |= hit

    # border wall: four boxes just inside the table edge
    s, w, wh = arena.size, arena.wall_t, arena.wall_h
    for lo, hi in (((0, 0, 0), (s, w, wh)), ((0, s - w, 0), (s, s, wh)),
                   ((0, 0, 0), (w, s, wh)), ((s - w, 0, 0), (s, s, wh))):
        tb, _ = _box_hit(o, d, lo, hi)
        hit = tb < best
        best = np.where(hit, tb, best)
        shiny &= ~hit

    z = best.copy()
    if noise:
        sigma = noise_scale * (0.0012 + 0.0045 * z ** 2)   # ~ OAK-D Lite stereo: ~6 mm at 1 m
        z = z + rs.normal(0, 1, n) * np.where(np.isfinite(sigma), sigma, 0)
        drop = rs.rand(n)
        z = np.where(shiny & (drop < shiny_dropout), np.inf, z)
        spike = shiny & (drop > 1 - shiny_spike)
        z = np.where(spike, z + rs.choice([-0.3, 0.3], n), z)
        z = np.where(rs.rand(n) < dropout, np.inf, z)   # random stereo dropouts
    img = np.zeros((cam.intr.height, cam.intr.width), dtype=np.uint16)
    ok = np.isfinite(z) & (z > 0.05) & (z < 65.0)
    img[v[ok], u[ok]] = np.clip(z[ok] * 1000.0, 0, 65535).astype(np.uint16)
    return img
