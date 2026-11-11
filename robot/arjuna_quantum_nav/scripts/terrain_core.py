"""Terrain detection from an OAK-D depth image: crater rims, rocks, border wall, no-floor.

numpy only, Python 3.6 (no scipy, no OpenCV). Unit-tested with synthetic depth images in
tests/test_arjuna_terrain_core.py. The floor plane comes from crater_core.py (calibration).

Pipeline (all in the robot frame: x forward, y left, metres)
  1. depth pixels -> 3-D points -> height above the calibrated floor plane.
  2. Points of the last few frames (moved into the current robot frame with the odometry pose)
     are binned into a bird's-eye grid; each cell keeps the MEDIAN height (kills stereo spikes).
  3. Cells are classified against the floor around them (a 15x15 local median, so a slightly
     wrong plane or a pitching rover does not shift the heights):
       TALL   clearly above the floor           -> rock (compact) or border wall (long, thin)
       RIM    a few cm above the floor, thin    -> ring fit; the whole circle inside is a crater
       FLOOR  valid and near floor level
       INVALID no depth at all (shiny plastic, black wall, past the table edge)
  4. Invalid depth is handled by looking along each camera ray (see _sweep):
       gap right in front of / inside a tall object  -> part of the rock (blocked)
       long gap between floor cells                    -> a rock that returned no depth (blocked)
       gap behind a tall object                        -> its shadow (ignored)
       gap that runs on to the end after solid floor   -> no floor = table edge (blocked)
       short scattered gaps                            -> noise (ignored)

Everything a detection needs for the planner is in a plain dict, see _detection().
"""

import math

import numpy as np

FLOOR, TALL, INVALID, BELOW = 1, 2, 3, 4


class TerrainParams(object):
    """Detector settings. The ones that depend on the real stereo noise are marked (measure)."""

    def __init__(self, **kw):
        self.cell = 0.02              # bird's-eye cell size, m
        self.x_min, self.x_max = 0.20, 1.20   # watched range ahead of the robot centre, m
        self.half_width = 0.60        # watched half width, m
        self.camera_forward = 0.10    # camera ahead of the robot centre, m
        self.min_cell_points = 2      # depth samples a cell needs to count as measured
        self.tall_min = 0.05          # (measure) height above the floor that is an obstacle, m
        self.rock_min = 0.12          # a long thin cluster lower than 1.5x this is the border wall; else a rock
        self.rim_min = 0.010          # (measure) lowest ridge that counts as a crater rim, m
        self.rim_range = 0.90         # (measure) rims are only searched this far ahead, m
        self.min_rim_cells = 10       # ridge cells needed (cell = 2 cm: 10 cells ~ 20 cm of rim)
        self.min_span = 1.0           # rad of arc a rim fit must cover
        self.crater_r_min, self.crater_r_max = 0.09, 0.27   # fitted rim radius limits, m
        self.fit_tol = 0.02           # rms distance of rim cells from the fitted circle, m
        self.crater_pad = 0.03        # added to the fitted radius when filling the crater, m
        self.min_tall_cells = 4       # smallest rock, cells
        self.border_min_len = 0.50    # a tall/low cluster this long and thin is the border wall
        self.hole_min_len = 0.08      # a gap this long between floor cells is an object, m
        self.hole_block_len = 0.30    # ... and this much of it is blocked, m
        self.hole_min_area = 0.012    # m^2
        self.nofloor_min_len = 0.10   # no floor over this length of a ray = table edge, m
        self.nofloor_range = 1.00     # stereo is unreliable beyond this: no edge calls, m
        self.nofloor_min_area = 0.02  # m^2
        self.min_solid_len = 0.02     # floor length needed right before a gap, m
        self.below_floor = 0.05       # valid points this far under the floor = no floor, m
        self.bg_half = 7              # local floor window: 2*7+1 = 15 cells
        self.out_spacing = 0.04       # spacing of the cells published per detection, m
        self.frames = 3               # frames fused (needs the pose to compensate motion)
        self.speckle_abs = 0.03       # depth pixels this far (m) ...
        self.speckle_rel = 0.03       # ... plus this fraction of the depth from their neighbours' median are spikes
        self.pixel_step = 3           # sample every Nth pixel across ...
        self.pixel_step_y = 2         # ... and every Nth row down
        self.top_fraction = 0.10      # image rows above this fraction are not sampled (horizon)
        self.rock_extrude = 0.15      # hidden floor behind a tall object that is part of it, m
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError("unknown terrain parameter %r" % k)
            setattr(self, k, v)


# ── small array helpers (no scipy) ───────────────────────────────────────────
def _shift(a, dx, dy, fill):
    """b[i, j] = a[i + dx, j + dy], `fill` outside."""
    out = np.full(a.shape, fill, dtype=a.dtype)
    nx, ny = a.shape
    xs, xe = max(0, -dx), min(nx, nx - dx)
    ys, ye = max(0, -dy), min(ny, ny - dy)
    if xs < xe and ys < ye:
        out[xs:xe, ys:ye] = a[xs + dx:xe + dx, ys + dy:ye + dy]
    return out


def dilate(mask, r):
    out = mask.copy()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx or dy:
                out |= _shift(mask, dx, dy, False)
    return out


def erode(mask, r):
    return ~dilate(~mask, r)


def close_mask(mask, r):
    return erode(dilate(mask, r), r)


def label(mask):
    """8-connected components. Returns (labels int array, count); 0 = background."""
    lab = np.zeros(mask.shape, dtype=np.int32)
    nx, ny = mask.shape
    count = 0
    for sx, sy in zip(*np.nonzero(mask)):
        if lab[sx, sy]:
            continue
        count += 1
        lab[sx, sy] = count
        stack = [(sx, sy)]
        while stack:
            x, y = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    a, b = x + dx, y + dy
                    if 0 <= a < nx and 0 <= b < ny and mask[a, b] and not lab[a, b]:
                        lab[a, b] = count
                        stack.append((a, b))
    return lab, count


def window_median(a, half, min_count):
    """Median of the finite values in a (2*half+1)^2 window around every cell (NaN if fewer
    than min_count values). Cells with NaN are ignored, not treated as zero."""
    stack = np.stack([_shift(a, dx, dy, np.nan)
                      for dx in range(-half, half + 1) for dy in range(-half, half + 1)])
    count = np.isfinite(stack).sum(axis=0)
    srt = np.sort(stack, axis=0)          # NaN sorts last
    idx = np.clip((count - 1) // 2, 0, srt.shape[0] - 1)
    ii, jj = np.indices(a.shape)
    med = srt[idx, ii, jj]
    med[count < min_count] = np.nan
    return med


def speckle_filter(d, tol_abs, tol_rel):
    """Set to NaN every depth sample that differs from the median of its 3x3 neighbourhood by
    more than tol_abs + tol_rel * depth, or that has fewer than 3 valid neighbours."""
    stack = np.stack([_shift(d, dx, dy, np.nan)
                      for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy])
    count = np.isfinite(stack).sum(axis=0)
    srt = np.sort(stack, axis=0)
    idx = np.clip((count - 1) // 2, 0, srt.shape[0] - 1)
    ii, jj = np.indices(d.shape)
    med = srt[idx, ii, jj]
    with np.errstate(invalid="ignore"):
        bad = np.abs(d - med) > tol_abs + tol_rel * d
    bad |= count < 3
    out = d.copy()
    out[bad] = np.nan
    return out


def fit_circle(pts):
    """Least-squares (Kasa) circle through points (N x 2): centre (2,), radius."""
    x, y = pts[:, 0], pts[:, 1]
    a = np.stack([2 * x, 2 * y, np.ones_like(x)], axis=1)
    sol = np.linalg.lstsq(a, x * x + y * y, rcond=-1)[0]
    c = sol[:2]
    return c, math.sqrt(max(sol[2] + c.dot(c), 1e-12))


def fit_center_fixed_radius(pts, r, c0, iters=20):
    """Gauss-Newton centre for a circle of known radius."""
    c = np.array(c0, dtype=np.float64)
    for _ in range(iters):
        d = pts - c
        dist = np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)
        jac = -d / dist[:, None]
        step = np.linalg.lstsq(jac, -(dist - r), rcond=-1)[0]
        c = c + step
        if np.hypot(step[0], step[1]) < 1e-5:
            break
    return c


def circle_rms(pts, c, r):
    return float(np.sqrt(np.mean((np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - r) ** 2)))


def arc_span(pts, c):
    """Angle (rad) around centre c that the points cover (2*pi minus the biggest gap)."""
    ang = np.sort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
    gaps = np.diff(np.concatenate([ang, [ang[0] + 2 * math.pi]]))
    return float(2 * math.pi - gaps.max())


def fit_rim(pts, p):
    """Fit a crater circle to rim cells. Returns (centre, radius, span, rms) or None.
    A near-complete rim is fitted freely; a partial arc gets its radius pinned to the best
    value inside the allowed range so a short arc cannot invent a huge or tiny crater."""
    c, r = fit_circle(pts)
    span = arc_span(pts, c)
    if p.crater_r_min <= r <= p.crater_r_max and span >= 2.2:
        return c, r, span, circle_rms(pts, c, r)
    best = None
    for rr in np.arange(p.crater_r_min, p.crater_r_max + 1e-9, 0.01):
        cc = fit_center_fixed_radius(pts, rr, c)
        rms = circle_rms(pts, cc, rr)
        # tie-break towards the larger radius: the safer (bigger) crater
        if best is None or rms < best[3] - 0.002 or (abs(rms - best[3]) <= 0.002 and rr > best[1]):
            best = (cc, rr, arc_span(pts, cc), rms)
    return best


def _detection(kind, xy, **extra):
    d = {"type": kind, "cells": xy}
    d.update(extra)
    return d


# ── view geometry (what the camera can see on the floor) ─────────────────────
class ViewGeometry(object):
    """Cell centres of the watched grid and which of them the camera looks at."""

    def __init__(self, floor, intr, p, top_fraction=0.25):
        self.p = p
        self.nx = int(round((p.x_max - p.x_min) / p.cell))
        self.ny = int(round(2 * p.half_width / p.cell))
        ix, iy = np.indices((self.nx, self.ny))
        self.cx = p.x_min + (ix + 0.5) * p.cell
        self.cy = -p.half_width + (iy + 0.5) * p.cell
        pts = floor.from_robot(self.cx.ravel(), self.cy.ravel(), p.camera_forward)
        z = pts[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = pts[:, 0] / z * intr.fx + intr.cx
            v = pts[:, 1] / z * intr.fy + intr.cy
        v0 = intr.height * top_fraction
        ok = (z > 0.15) & (z < 3.0) & (u >= 2) & (u <= intr.width - 3) & (v >= v0 + 2) & \
             (v <= intr.height - 3)
        self.in_view = ok.reshape(self.nx, self.ny)
        dx = self.cx - p.camera_forward
        self.rng = np.hypot(dx, self.cy)
        self.dist_ahead = self.cx
        # rays: for bearings every half degree, the watched cells the line of sight passes
        # through, nearest first (marched in steps smaller than a cell so none is skipped)
        flat_view = self.in_view.ravel()
        flat_rng = self.rng.ravel()
        step = 0.7 * p.cell
        r = np.arange(0.0, math.hypot(p.x_max - p.camera_forward, p.half_width) + step, step)
        self.rays = []
        for bearing in np.arange(-60.0, 60.01, 0.5):
            a = math.radians(bearing)
            x, y = p.camera_forward + r * math.cos(a), r * math.sin(a)
            ix = np.floor((x - p.x_min) / p.cell).astype(np.int64)
            iy = np.floor((y + p.half_width) / p.cell).astype(np.int64)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            flat = (ix * self.ny + iy)[ok]
            flat = flat[flat_view[flat]]
            if len(flat) == 0:
                continue
            first = np.concatenate([[True], flat[1:] != flat[:-1]])
            self.rays.append(flat[first])

    def index(self, xy):
        """Grid indices and an in-grid mask for robot-frame points (N x 2)."""
        p = self.p
        ix = np.floor((xy[:, 0] - p.x_min) / p.cell).astype(np.int64)
        iy = np.floor((xy[:, 1] + p.half_width) / p.cell).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok


# ── the detector ─────────────────────────────────────────────────────────────
class TerrainDetector(object):
    def __init__(self, floor, intr, params=None):
        self.floor, self.intr = floor, intr
        self.p = params or TerrainParams()
        from crater_core import pixel_grid
        self.u, self.v = pixel_grid(intr, step=self.p.pixel_step, step_y=self.p.pixel_step_y,
                                    top_fraction=self.p.top_fraction)
        self.us, self.vs = np.unique(self.u), np.unique(self.v)
        self.geom = ViewGeometry(floor, intr, self.p, self.p.top_fraction)
        self.frames = []          # [(xy world (N,2), h (N,), pose)]
        self.last = {}            # masks of the last analyse(), for debugging

    # -- input ---------------------------------------------------------------
    def frame_points(self, depth_mm):
        """(robot-frame xy (N,2), height above floor (N,)) of one depth image.
        Invalid pixels are dropped, and so are speckle spikes (shiny plastic, flying pixels):
        a pixel whose depth disagrees with the median of its neighbours."""
        p = self.p
        d = depth_mm[self.vs[:, None], self.us[None, :]].astype(np.float64) / 1000.0
        d[(d < 0.15) | (d > 3.0)] = np.nan
        d = speckle_filter(d, p.speckle_abs, p.speckle_rel)
        ok = np.isfinite(d)
        z = d[ok]
        uu = np.broadcast_to(self.us[None, :], d.shape)[ok]
        vv = np.broadcast_to(self.vs[:, None], d.shape)[ok]
        pts = np.stack([(uu - self.intr.cx) / self.intr.fx * z,
                        (vv - self.intr.cy) / self.intr.fy * z, z], axis=1)
        return self.floor.to_robot(pts, p.camera_forward), self.floor.heights(pts)

    def add_frame(self, depth_mm, pose=None):
        """Store a depth frame. `pose` (x, y, yaw) in any fixed frame lets frames taken while
        moving be fused; without it frames are assumed to be taken from the same spot."""
        xy, h = self.frame_points(depth_mm)
        pose = pose if pose is not None else (0.0, 0.0, 0.0)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = np.stack([pose[0] + xy[:, 0] * c - xy[:, 1] * s,
                          pose[1] + xy[:, 0] * s + xy[:, 1] * c], axis=1)
        self.frames.append((world, h, pose))
        del self.frames[:-self.p.frames]

    def reset(self):
        del self.frames[:]

    def _fused_points(self, pose):
        pose = pose if pose is not None else (0.0, 0.0, 0.0)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        xs, hs = [], []
        for world, h, _ in self.frames:
            dx, dy = world[:, 0] - pose[0], world[:, 1] - pose[1]
            xs.append(np.stack([dx * c + dy * s, -dx * s + dy * c], axis=1))
            hs.append(h)
        return np.concatenate(xs), np.concatenate(hs)

    # -- rasterise -----------------------------------------------------------
    def raster(self, pose=None):
        """(H, N, HQ): median height, sample count and upper-quartile height per cell of the
        bird's-eye grid. The median is robust to spikes; the upper quartile keeps a thin 2 cm
        rim that only covers part of a cell."""
        g = self.geom
        xy, h = self._fused_points(pose)
        ix, iy, ok = g.index(xy)
        ix, iy, h = ix[ok], iy[ok], h[ok]
        cell_id = ix * g.ny + iy
        order = np.lexsort((h, cell_id))
        cell_id, h = cell_id[order], h[order]
        ids, start, cnt = np.unique(cell_id, return_index=True, return_counts=True)
        H = np.full(g.nx * g.ny, np.nan)
        N = np.zeros(g.nx * g.ny, dtype=np.int32)
        H[ids] = h[start + cnt // 2]
        HQ = np.full(g.nx * g.ny, np.nan)
        HQ[ids] = h[start + np.where(cnt >= 4, (3 * cnt) // 4, cnt // 2)]
        N[ids] = cnt
        return H.reshape(g.nx, g.ny), N.reshape(g.nx, g.ny), HQ.reshape(g.nx, g.ny)

    def _fill_gaps(self, H, N):
        """Cells with no sample but most neighbours measured (pixel-sampling gaps) get the
        median of those neighbours. Real holes (shiny plastic) are bigger and stay empty."""
        have = N >= self.p.min_cell_points
        Hm = np.where(have, H, np.nan)
        nb = np.stack([_shift(Hm, dx, dy, np.nan)
                       for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy])
        cnt = np.isfinite(nb).sum(axis=0)
        med = np.nanmedian(np.where(np.isfinite(nb), nb, np.nan), axis=0) if nb.size else Hm
        fill = (~have) & (cnt >= 5)
        out = Hm.copy()
        out[fill] = med[fill]
        return out

    # -- main entry ----------------------------------------------------------
    def analyse(self, pose=None):
        """Classify the fused frames. Returns (detections, info)."""
        import warnings
        p, g = self.p, self.geom
        H, N, HQ = self.raster(pose)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            Hf = self._fill_gaps(H, N)
            HQf = self._fill_gaps(HQ, N)
            valid = np.isfinite(Hf)
            # local floor level: median of the low cells around (ignore rock / rim cells)
            low = np.where(valid & (Hf < p.tall_min * 0.8), Hf, np.nan)
            bg = window_median(low, p.bg_half, min_count=30)
        bg = np.where(np.isfinite(bg), bg, 0.0)
        hrel = Hf - bg
        watched = g.in_view

        qrel = HQf - bg
        tall = valid & watched & (qrel > p.tall_min)
        tall = _drop_small(tall, p.min_tall_cells)
        below = valid & watched & (Hf < -p.below_floor)
        dets = []

        # ---- walk every ray: dropouts in rocks, shadows, table edge ----
        state = np.zeros(H.shape, dtype=np.int8)
        state[watched & valid] = FLOOR
        state[watched & ~valid] = INVALID
        state[below] = BELOW
        state[tall] = TALL
        hole, tail, head, shadow = self._sweep(state, tall)

        # ---- tall objects: rocks and the border wall ----
        # A rock seen from a low camera shows only its front face; the floor hidden behind it
        # (up to rock_extrude) belongs to the rock, so the planner cannot route through it.
        tall_blocked = np.zeros_like(tall)
        lab, n = label(dilate(tall | shadow, 1))
        for k in range(1, n + 1):
            region = lab == k
            cells = region & tall
            if not cells.any():
                continue
            xy = np.stack([g.cx[cells], g.cy[cells]], axis=1)
            height = float(np.percentile(hrel[cells], 90))
            major, minor = _extent(xy)
            kind = "border" if (major >= p.border_min_len and minor <= 0.5 * major and
                                height < p.rock_min * 1.5) else "rock"
            if kind == "border":
                blocked = dilate(cells, 1)
            else:
                blocked = _fill_holes(close_mask(dilate(cells | (region & (shadow | hole)), 1), 3))
            tall_blocked |= blocked
            bxy = np.stack([g.cx[blocked], g.cy[blocked]], axis=1)
            cx, cy = bxy.mean(axis=0)
            rad = float(np.max(np.hypot(bxy[:, 0] - cx, bxy[:, 1] - cy))) + p.cell
            dets.append(_detection(kind, self._thin(blocked), x=float(cx), y=float(cy), r=rad,
                                   height=height, size=[major, minor]))

        # A no-depth region whose near end is narrow (a wedge opening up from a point) is an
        # object that returned nothing (shiny rock): block only its first hole_block_len.
        # One whose near end spans the view or reaches its side is the table edge: block it all.
        edge_cells = np.zeros_like(tall)
        obj_cells = hole & ~tall_blocked & ~shadow
        lab2, n2 = label(dilate(tail & ~tall_blocked & ~shadow, 1))
        side_touch = np.zeros_like(watched)       # cells within 3 cells (sideways) of the view's edge
        for dy in (-3, -2, -1, 1, 2, 3):
            side_touch |= _shift(~watched, 0, dy, True)
        for k in range(1, n2 + 1):
            comp = (lab2 == k) & tail & ~tall_blocked & ~shadow
            if not comp.any():
                continue
            front = comp & (g.rng <= g.rng[comp].min() + 0.10)
            on_side = bool((front & side_touch).any()) or                 float(g.cy[front].max() - g.cy[front].min()) >= 0.6
            if on_side:
                edge_cells |= comp
            else:
                obj_cells |= (comp & head)
        for kind, mask, min_area in (("rock", obj_cells, p.hole_min_area),
                                     ("nofloor", edge_cells, p.nofloor_min_area)):
            lab2, n2 = label(dilate(mask, 1))
            for k in range(1, n2 + 1):
                cells = (lab2 == k) & mask
                if cells.sum() * p.cell ** 2 < min_area:
                    continue
                xy = np.stack([g.cx[cells], g.cy[cells]], axis=1)
                cx, cy = xy.mean(axis=0)
                rad = float(np.max(np.hypot(xy[:, 0] - cx, xy[:, 1] - cy))) + p.cell
                dets.append(_detection(kind, self._thin(cells), x=float(cx), y=float(cy), r=rad,
                                       source="no-depth" if kind == "rock" else "edge"))

        # ---- crater rims ----
        ridge = (valid & watched & (qrel >= p.rim_min) & (qrel <= p.tall_min) &
                 (g.dist_ahead <= p.rim_range) & ~dilate(tall_blocked | tall, 2))
        ridge = ridge & (_neighbours(ridge) >= 1)      # drop isolated specks
        lab3, n3 = label(close_mask(ridge, 2))
        for k in range(1, n3 + 1):
            cells = (lab3 == k) & ridge
            if int(cells.sum()) < p.min_rim_cells:
                continue
            xy = np.stack([g.cx[cells], g.cy[cells]], axis=1)
            height = float(np.median(qrel[cells]))
            fit = fit_rim(xy, p)
            if fit is not None and fit[3] <= p.fit_tol and fit[2] >= p.min_span:
                c, r, span, rms = fit
                fr = r + p.crater_pad
                disk = (g.cx - c[0]) ** 2 + (g.cy - c[1]) ** 2 <= fr * fr
                fill_xy = self._disk_points(c, fr)
                dets.append(_detection("crater", fill_xy, x=float(c[0]), y=float(c[1]), r=float(r),
                                       fill_r=float(fr), span=float(span), height=height,
                                       rms=float(rms), rim_cells=int(cells.sum())))
            elif int(cells.sum()) >= 2 * p.min_rim_cells:
                major, minor = _extent(xy)
                line = major >= p.border_min_len and minor <= 0.5 * major
                # a long low straight wall is the border; any other raised line that is not a
                # circle is blocked as seen, as a partial crater rim
                dets.append(_detection("border" if line else "crater",
                                       self._thin(dilate(cells, 1)),
                                       x=float(xy[:, 0].mean()), y=float(xy[:, 1].mean()),
                                       r=0.0, span=0.0, height=height, partial=True,
                                       rim_cells=int(cells.sum())))
        self.last = {"state": state, "tall": tall, "hole": hole, "nofloor": edge_cells,
                     "shadow": shadow, "ridge": ridge, "hrel": hrel, "qrel": qrel}
        info = {"cells_valid": int(valid.sum()), "cells_watched": int(watched.sum()),
                "invalid_fraction": float((watched & ~valid).sum()) / max(1, int(watched.sum()))}
        return dets, info

    def process(self, depth_mm, pose=None):
        self.add_frame(depth_mm, pose)
        return self.analyse(pose)

    # -- helpers -------------------------------------------------------------
    def _thin(self, mask):
        """Cell centres of a mask, one per out_spacing grid square."""
        g = self.geom
        pts = np.stack([g.cx[mask], g.cy[mask]], axis=1)
        return _thin_points(pts, self.p.out_spacing)

    def _disk_points(self, c, radius):
        s = self.p.out_spacing
        n = int(math.ceil(radius / s))
        offs = np.arange(-n, n + 1) * s
        gx, gy = np.meshgrid(c[0] + offs, c[1] + offs)
        gx, gy = gx.ravel(), gy.ravel()
        keep = (gx - c[0]) ** 2 + (gy - c[1]) ** 2 <= radius * radius
        return np.stack([gx[keep], gy[keep]], axis=1)

    def _sweep(self, state, tall):
        """Walk every camera ray from near to far.
        Returns (hole, tail, head, shadow) masks:
          hole    no-depth gap between floor cells that is big enough to be an object
          tail    gap that runs on to the end of the ray after solid floor (edge or object)
          head    the first hole_block_len of such a tail
          shadow  floor hidden right behind a tall object"""
        p, g = self.p, self.geom
        flat = state.ravel()
        rng = g.rng.ravel()
        tall_near = dilate(tall, 2).ravel()      # a ray grazing a rock's corner is in its shadow
        hole = np.zeros(flat.shape, dtype=bool)
        tail = np.zeros(flat.shape, dtype=bool)
        head = np.zeros(flat.shape, dtype=bool)
        shadow = np.zeros(flat.shape, dtype=bool)
        keep = max(1, int(round(p.hole_block_len / p.cell)))
        for ray in g.rays:
            st = flat[ray]
            r = rng[ray]
            m = len(ray)
            last_solid, r_floor0, r_floor1, r_tall = 0, 0.0, 0.0, 0.0
            seen_tall = False       # once a ray met a tall object, later gaps are its shadow
            i = 0
            while i < m:
                s = st[i]
                if s == FLOOR:
                    if last_solid != FLOOR:
                        r_floor0 = r[i]
                    last_solid, r_floor1 = FLOOR, r[i]
                    i += 1
                    continue
                if s == TALL:
                    last_solid, r_tall, seen_tall = TALL, r[i], True
                    i += 1
                    continue
                j = i
                while j < m and st[j] in (INVALID, BELOW):
                    j += 1
                length = r[j - 1] - r[i] + p.cell
                if last_solid != TALL and (tall_near[ray[i]] or (i > 0 and tall_near[ray[i - 1]])):
                    last_solid, r_tall, seen_tall = TALL, r[i], True
                if last_solid == TALL:
                    near = r[i:j] - r_tall <= p.rock_extrude
                    shadow[ray[i:j][near]] = True
                elif seen_tall:
                    pass
                elif last_solid == FLOOR and r_floor1 - r_floor0 + p.cell >= p.min_solid_len:
                    if j == m:
                        if length >= p.nofloor_min_len and r[i] <= p.nofloor_range:
                            tail[ray[i:]] = True
                            head[ray[i:i + keep]] = True
                    elif st[j] == TALL:
                        if length <= p.hole_block_len:
                            hole[ray[i:j]] = True
                    elif length >= p.hole_min_len:
                        hole[ray[i:min(j, i + keep)]] = True
                last_solid = INVALID        # the next floor run starts fresh
                i = j
        shape = state.shape
        return (hole.reshape(shape), tail.reshape(shape), head.reshape(shape),
                shadow.reshape(shape))


def _drop_small(mask, min_cells):
    """Remove connected groups of fewer than min_cells cells (speckle)."""
    lab, n = label(dilate(mask, 1))
    out = np.zeros_like(mask)
    for k in range(1, n + 1):
        grp = (lab == k) & mask
        if int(grp.sum()) >= min_cells:
            out |= grp
    return out


def _neighbours(mask):
    return sum(_shift(mask.astype(np.int8), dx, dy, 0)
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy)


def _fill_holes(mask):
    """Cells of `mask`'s background that cannot reach the grid border (enclosed holes)."""
    outside = ~mask
    reach = np.zeros_like(mask)
    reach[0, :] |= outside[0, :]
    reach[-1, :] |= outside[-1, :]
    reach[:, 0] |= outside[:, 0]
    reach[:, -1] |= outside[:, -1]
    while True:
        grown = (dilate(reach, 1) & outside) | reach
        if (grown == reach).all():
            break
        reach = grown
    return mask | (outside & ~reach)


def _thin_points(pts, spacing):
    if len(pts) == 0:
        return pts
    key = np.floor(pts / spacing).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    return pts[np.sort(first)]


def _extent(xy):
    """(major, minor) extent in metres of a point cloud along its principal axes."""
    if len(xy) < 2:
        return 0.0, 0.0
    c = xy - xy.mean(axis=0)
    _, vec = np.linalg.eigh(np.cov(c.T))
    proj = c.dot(vec)
    ext = proj.max(axis=0) - proj.min(axis=0)
    return float(ext.max()), float(ext.min())


# ── measure mode: pick the thresholds from the real camera ───────────────────
def measure(detector, depth_frames, poses=None):
    """Noise and rim-height statistics from several depth frames of the SAME scene.

    Look at flat floor to get the noise, then at a crater (rim in view) for the rim height.
    Returns a dict with floor_noise_mm, rim_height_mm, a suggested rim_min / tall_min, and a
    message saying whether a 2 cm rim stands out of the noise enough to trust.
    """
    p = detector.p
    detector.reset()
    per_frame_noise = []
    for i, d in enumerate(depth_frames):
        pose = poses[i] if poses else None
        detector.add_frame(d, pose)
        xy, h = detector.frame_points(d)
        near = (xy[:, 0] > 0.3) & (xy[:, 0] < 0.9) & (np.abs(xy[:, 1]) < 0.3) & (np.abs(h) < 0.04)
        if near.sum() > 50:
            per_frame_noise.append(1.4826 * float(np.median(np.abs(h[near] - np.median(h[near])))))
    import warnings
    pose = poses[-1] if poses else None
    H, N, HQ = detector.raster(pose)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        Hf = detector._fill_gaps(H, N)
        HQf = detector._fill_gaps(HQ, N)
        valid = np.isfinite(Hf)
        low = np.where(valid & (Hf < p.tall_min * 0.8), Hf, np.nan)
        bg = window_median(low, p.bg_half, min_count=30)
    hrel = np.where(np.isfinite(bg), Hf - bg, Hf)
    qrel = np.where(np.isfinite(bg), HQf - bg, HQf)
    g = detector.geom
    ok = valid & g.in_view & (g.dist_ahead <= p.rim_range)
    sigma_cell = 1.4826 * float(np.nanmedian(np.abs(hrel[ok] - np.nanmedian(hrel[ok])))) if ok.any() else float("nan")
    # the ridge: cells clearly above the noise but below the tall threshold
    thr = max(3.0 * sigma_cell, 0.006) if np.isfinite(sigma_cell) else 0.01
    cand = ok & (qrel > thr) & (qrel < p.tall_min)
    cand &= _neighbours(cand) >= 2
    out = {"frames": len(depth_frames),
           "floor_noise_mm_single_frame": 1000 * float(np.median(per_frame_noise)) if per_frame_noise else float("nan"),
           "floor_noise_mm_fused_cell": 1000 * sigma_cell,
           "floor_bias_mm": 1000 * float(np.nanmedian(Hf[ok])) if ok.any() else float("nan"),
           "valid_fraction": float(valid[g.in_view].mean()) if g.in_view.any() else 0.0,
           "ridge_cells": int(cand.sum())}
    if cand.sum() >= p.min_rim_cells:
        v = qrel[cand]
        rim = float(np.median(v))
        out.update(rim_height_mm_median=1000 * rim, rim_height_mm_p90=1000 * float(np.percentile(v, 90)))
        out["suggested_rim_min_mm"] = 1000 * max(0.5 * rim, 3.0 * sigma_cell)
        out["rim_snr"] = rim / sigma_cell if sigma_cell > 0 else float("inf")
    else:
        out["rim_height_mm_median"] = None
    out["suggested_tall_min_mm"] = 1000 * max(0.05, 8.0 * (sigma_cell if np.isfinite(sigma_cell) else 0.005))
    dets, _ = detector.analyse(pose)
    out["detections"] = [(d["type"], round(d["x"], 2), round(d["y"], 2), round(d.get("r", 0.0), 2))
                         for d in dets]
    out["message"] = _verdict(out, p)
    return out


def _verdict(m, p):
    noise = m["floor_noise_mm_fused_cell"]
    if m.get("rim_height_mm_median") is None:
        return ("No rim-like ridge found. Floor noise %.1f mm per cell. Aim the camera at a crater "
                "from 0.4-0.8 m, then run again." % noise)
    snr = m["rim_snr"]
    if snr >= 4:
        return ("Rim %.1f mm vs noise %.1f mm (%.1fx): detection should be reliable. "
                "Use rim_min:=%.4f." % (m["rim_height_mm_median"], noise, snr,
                                          m["suggested_rim_min_mm"] / 1000.0))
    if snr >= 2.5:
        return ("Rim %.1f mm vs noise %.1f mm (%.1fx): marginal. Use rim_min:=%.4f, keep craters "
                "within 0.7 m, and rely on the bump sensor as backup." % (
                    m["rim_height_mm_median"], noise, snr, m["suggested_rim_min_mm"] / 1000.0))
    return ("Rim %.1f mm is only %.1fx the noise (%.1f mm): the camera cannot see this rim "
            "reliably. Rely on the IMU bump detector for craters, or move the camera closer / "
            "lower." % (m["rim_height_mm_median"], snr, noise))


def detections_to_json_obj(dets):
    out = []
    for d in dets:
        o = dict((k, v) for k, v in d.items() if k != "cells")
        o["cells"] = [[round(float(x), 3), round(float(y), 3)] for x, y in d["cells"]]
        for k, v in list(o.items()):
            if isinstance(v, float):
                o[k] = round(v, 3)
            elif isinstance(v, (np.floating, np.integer)):
                o[k] = float(v)
        out.append(o)
    return out
