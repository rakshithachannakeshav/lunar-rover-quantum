"""Floor calibration and depth geometry for the OAK-D Lite. numpy only, Python 3.6.

Used by terrain_core.py (the terrain detector) and crater_detector_node.py, and unit-tested in
tests/test_arjuna_crater_core.py with synthetic depth images (no camera needed).

Idea: with the rover on flat ground, fit a plane to the floor the camera sees (calibration).
After that, every depth pixel gives a 3-D point; its signed height above that floor plane says
whether it is floor (~0), a raised crater rim (~+2 cm), a rock or wall (well above), or ground
below the floor level. What to do with those heights lives in terrain_core.py.
Robot frame: x forward (from the robot centre), y left, in metres.

Camera frame (OAK-D / OpenCV convention): x right, y down, z forward along the lens axis.
"""

import json

import numpy as np


class Intrinsics(object):
    def __init__(self, fx, fy, cx, cy, width, height):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.width, self.height = int(width), int(height)


def pixel_grid(intr, step=6, top_fraction=0.35, step_y=None):
    """Sampled pixel coordinates (u, v): every `step` px across (`step_y` down, default the
    same), skipping the top part of the image (it looks at the horizon / far away, not at the
    ground just ahead)."""
    step_y = step_y or step
    v0 = int(intr.height * top_fraction)
    us = np.arange(step // 2, intr.width, step)
    vs = np.arange(max(v0, step_y // 2), intr.height, step_y)
    uu, vv = np.meshgrid(us, vs)
    return uu.ravel(), vv.ravel()


def depth_to_points(depth_mm, intr, u, v, min_z=0.15, max_z=3.0):
    """3-D camera-frame points (N x 3, metres) for the sampled pixels with valid depth."""
    z = depth_mm[v, u].astype(np.float64) / 1000.0
    ok = (z > min_z) & (z < max_z)
    z = z[ok]
    x = (u[ok] - intr.cx) / intr.fx * z
    y = (v[ok] - intr.cy) / intr.fy * z
    return np.stack([x, y, z], axis=1)


class FloorModel(object):
    """Floor plane as seen by the camera: unit normal `n` (pointing up, towards the
    camera) and a point `c` on it. Also gives the floor-aligned forward/left axes."""

    def __init__(self, n, c):
        n = np.asarray(n, dtype=np.float64)
        n = n / np.linalg.norm(n)
        c = np.asarray(c, dtype=np.float64)
        if np.dot(n, -c) < 0:          # camera (origin) must be on the +n side
            n = -n
        self.n, self.c = n, c
        lens = np.array([0.0, 0.0, 1.0])
        f = lens - np.dot(lens, n) * n  # lens axis projected onto the floor = forward
        self.f = f / np.linalg.norm(f)
        self.l = np.cross(n, self.f)    # up x forward = left

    @property
    def camera_height(self):
        return float(np.dot(self.n, -self.c))

    @property
    def tilt_deg(self):
        """How far the lens axis points down from horizontal."""
        return float(np.degrees(np.arcsin(np.clip(-self.n[2], -1.0, 1.0))))

    def heights(self, pts):
        return (pts - self.c).dot(self.n)

    def to_robot(self, pts, camera_forward=0.0):
        """Floor-plane coordinates (forward, left) of camera-frame points, measured
        from the robot centre (the camera sits `camera_forward` m ahead of it)."""
        return np.stack([camera_forward + pts.dot(self.f), pts.dot(self.l)], axis=1)

    def from_robot(self, fwd, left, camera_forward=0.0, height=0.0):
        """Inverse of to_robot: camera-frame points for floor-plane coordinates (forward,
        left) in the robot frame, `height` m above the floor. Arrays in, (N x 3) out."""
        a = np.asarray(fwd, dtype=np.float64) - camera_forward
        b = np.asarray(left, dtype=np.float64)
        foot = -self.camera_height * self.n
        return (foot + np.outer(a, self.f) + np.outer(b, self.l) +
                np.outer(np.full(a.shape, height), self.n))

    def to_json(self):
        return json.dumps({"n": self.n.tolist(), "c": self.c.tolist(),
                           "camera_height_m": self.camera_height, "tilt_deg": self.tilt_deg})

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        return cls(d["n"], d["c"])


def fit_floor(pts, iterations=3, inlier_m=0.02):
    """Least-squares plane through `pts`, refitted on inliers to ignore clutter."""
    if len(pts) < 50:
        raise ValueError("Too few depth points (%d) to fit the floor" % len(pts))
    keep = pts
    for _ in range(iterations):
        c = keep.mean(axis=0)
        _, _, vt = np.linalg.svd(keep - c, full_matrices=False)
        model = FloorModel(vt[2], c)
        res = np.abs(model.heights(pts))
        inliers = pts[res < inlier_m]
        if len(inliers) < 50:
            break
        keep = inliers
    return model
