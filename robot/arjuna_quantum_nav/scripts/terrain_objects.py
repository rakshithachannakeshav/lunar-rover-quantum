"""Terrain OBJECTS for the user: one entry per crater / rock / wall / table edge, with its size,
a stable id across frames, and a human-readable line. numpy only, Python 3.6 (Jetson Nano).

terrain_core.TerrainDetector.analyse() measures every detection in the ROBOT frame of one fused
frame (info["objects"]). This module
  * moves those measurements to the map frame with the rover pose,
  * matches them to the objects already known (same class, nearest centre within a gate that
    grows with the object's size), so the same rock keeps the same id while the rover drives,
  * smooths sizes over the last frames, preferring views where the whole object was seen
    (a low camera sees the back of a rock only when the rock is lower than the camera),
  * forgets an object that the camera is looking at but no longer sees,
  * reports each object as JSON (published on /terrain/objects) and as one short line
    ("rock #2: 31 x 27 cm, 24 cm tall, 0.82 m ahead", appended to /terrain/status).

Text is ASCII only: the Jetson's terminal may not print anything else.
Unit-tested in tests/test_arjuna_terrain_objects.py.
"""

import math

import numpy as np

CLASS_NAMES = {"crater": "crater", "rock": "rock", "wall": "wall", "table_edge": "table edge"}


def _median(vals):
    return float(np.median(vals)) if len(vals) else None


def _pct(vals, q):
    return float(np.percentile(vals, q)) if len(vals) else None


def rect_corners(x, y, length, width, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [[x + c * dl - s * dw, y + s * dl + c * dw]
            for dl, dw in ((0.5 * length, 0.5 * width), (-0.5 * length, 0.5 * width),
                           (-0.5 * length, -0.5 * width), (0.5 * length, -0.5 * width))]


def circle_points(x, y, r, n=24):
    return [[x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n)]
            for k in range(n)]


def distance_to_polygon(px, py, poly):
    """Distance from (px, py) to a closed polygon's boundary, 0 if the point is inside."""
    if not poly:
        return float("inf")
    inside = False
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        if (ay > py) != (by > py) and px < (bx - ax) * (py - ay) / (by - ay) + ax:
            inside = not inside
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(ax + t * dx - px, ay + t * dy - py))
    return 0.0 if inside else best


def direction_words(bearing_deg):
    """Bearing relative to the rover's heading (+ = left) as words."""
    b = (bearing_deg + 180.0) % 360.0 - 180.0
    a = abs(b)
    side = "left" if b > 0 else "right"
    if a <= 20:
        return "ahead"
    if a <= 70:
        return "ahead-" + side
    if a <= 110:
        return "to the " + side
    if a <= 160:
        return "behind-" + side
    return "behind"


def _cm(v):
    return "%d" % int(round(100.0 * v))


class _Track(object):
    def __init__(self, meas, stamp):
        self.id = None              # given when confirmed, so noise never uses up an id
        self.cls = meas["class"]
        self.hist = []
        self.frames_seen = 0
        self.misses = 0
        self.first_t = stamp
        self.last_t = stamp
        self.visible = False
        self.add(meas, stamp)

    def add(self, meas, stamp):
        self.hist.append(meas)
        self.frames_seen += 1
        self.misses = 0
        self.last_t = stamp
        self.visible = True

    def centre(self):
        """Median centre of the last views, of the complete ones if there are any: a partly
        seen object's centre is the centre of the part that was seen."""
        full = [m for m in self.hist if m.get("footprint_complete")]
        h = (full or self.hist)[-8:]
        return _median([m["x"] for m in h]), _median([m["y"] for m in h])

    def size(self):
        return max(m["length"] for m in self.hist[-8:])


class ObjectTracker(object):
    """Keeps the objects seen so far in the map frame; see the module docstring."""

    def __init__(self, gate_min=0.10, gate_max=0.40, history=15, miss_limit=6,
                 min_frames=2, tentative_s=1.5, max_objects=60):
        self.gate_min, self.gate_max = gate_min, gate_max
        self.history = history
        self.miss_limit = miss_limit
        self.min_frames = min_frames
        self.tentative_s = tentative_s
        self.max_objects = max_objects
        self.tracks = []
        self.next_id = 1

    def reset(self):
        del self.tracks[:]

    # -- input ---------------------------------------------------------------
    @staticmethod
    def to_world(meas, pose):
        """Copy of a robot-frame measurement in the map frame of `pose` (x, y, yaw)."""
        x0, y0, th = pose
        c, s = math.cos(th), math.sin(th)

        def tf(px, py):
            return [x0 + c * px - s * py, y0 + s * px + c * py]

        m = dict(meas)
        m["x"], m["y"] = tf(meas["x"], meas["y"])
        m["yaw"] = meas.get("yaw", 0.0) + th
        if meas.get("outline"):
            m["outline"] = [tf(px, py) for px, py in meas["outline"]]
        if meas.get("circle"):
            cx, cy, r = meas["circle"]
            m["circle"] = tf(cx, cy) + [r]
        return m

    def update(self, measurements, pose, sees=None, stamp=0.0):
        """Feed one frame's robot-frame measurements (terrain_core info["objects"]).
        `sees(x, y, kind)` (robot frame) says whether the camera is watching a point now; an
        object it is watching but no longer finds is forgotten after miss_limit frames.
        Returns report(pose)."""
        pose = tuple(pose) if pose is not None else (0.0, 0.0, 0.0)
        world = [self.to_world(m, pose) for m in measurements]
        for t in self.tracks:
            t.visible = False
        pairs = []
        for ti, t in enumerate(self.tracks):
            tx, ty = t.centre()
            for mi, m in enumerate(world):
                if m["class"] != t.cls:
                    continue
                d = math.hypot(m["x"] - tx, m["y"] - ty)
                gate = min(self.gate_max, max(self.gate_min, 0.5 * max(t.size(), m["length"])))
                if d <= gate:
                    pairs.append((d, ti, mi))
        pairs.sort()
        used_t, used_m = set(), set()
        for d, ti, mi in pairs:
            if ti in used_t or mi in used_m:
                continue
            used_t.add(ti)
            used_m.add(mi)
            t = self.tracks[ti]
            t.add(world[mi], stamp)
            del t.hist[:-self.history]
        for mi, m in enumerate(world):
            if mi not in used_m:
                self.tracks.append(_Track(m, stamp))
        # objects the camera looks at but did not find
        c, s = math.cos(pose[2]), math.sin(pose[2])
        keep = []
        for ti, t in enumerate(self.tracks):
            if ti not in used_t and t.hist and t.last_t != stamp:
                tx, ty = t.centre()
                dx, dy = tx - pose[0], ty - pose[1]
                rx, ry = c * dx + s * dy, -s * dx + c * dy
                if sees is not None and sees(rx, ry, t.hist[-1].get("type")):
                    t.misses += 1
            if t.misses >= self.miss_limit:
                continue
            if t.frames_seen < self.min_frames and stamp - t.last_t > self.tentative_s:
                continue
            keep.append(t)
        if len(keep) > self.max_objects:      # drop the ones seen longest ago
            keep.sort(key=lambda t: t.last_t)
            keep = keep[len(keep) - self.max_objects:]
            keep.sort(key=lambda t: (t.id is None, t.id or 0))
        self.tracks = keep
        # ids in the order objects are confirmed: 1, 2, 3 ... with no gaps
        for t in self.tracks:
            if t.id is None and t.frames_seen >= self.min_frames:
                t.id = self.next_id
                self.next_id += 1
        return self.report(pose)

    # -- output --------------------------------------------------------------
    def _estimate(self, t):
        hist = t.hist
        full = [m for m in hist if m.get("footprint_complete")]
        pool = full or hist
        complete = bool(full)
        latest = pool[-1]
        if complete:
            length = _median([m["length"] for m in pool])
            width = _median([m["width"] for m in pool])
        else:            # every view was cut: sizes are lower bounds, take a high one
            length = _pct([m["length"] for m in pool], 80)
            width = _pct([m["width"] for m in pool], 80)
        hs = [m for m in hist if m.get("height") is not None]
        exact = [m["height"] for m in hs if not m.get("height_is_min")]
        if exact:
            height, height_is_min = _median(exact), False
        elif hs:
            height, height_is_min = _pct([m["height"] for m in hs], 80), True
        else:
            height, height_is_min = None, False
        x, y = t.centre()
        est = {"x": x, "y": y, "yaw": latest.get("yaw", 0.0), "length": length, "width": width,
               "height": height, "height_is_min": height_is_min, "footprint_complete": complete,
               "quality": _median([m.get("quality", 0.5) for m in pool]),
               "source": latest.get("source", "depth"), "partial": bool(latest.get("partial")),
               "inner_diameter": None, "outer_diameter": None, "size_is_estimate": False}
        if t.cls == "crater":
            outs = [m["outer_diameter"] for m in pool if m.get("outer_diameter")]
            ins = [m["inner_diameter"] for m in pool if m.get("inner_diameter")]
            est["outer_diameter"] = _median(outs)
            est["inner_diameter"] = _median(ins)
            spans = [m.get("span", 0.0) for m in pool]
            # a short arc pins the radius: the diameter is then a guess
            est["size_is_estimate"] = est["partial"] or (not complete and max(spans) < 2.2)
        if t.cls == "crater" and est["outer_diameter"] and not complete:
            est["outline"] = circle_points(x, y, 0.5 * est["outer_diameter"])
        elif t.cls in ("table_edge",) or not latest.get("outline") or length is None:
            est["outline"] = latest.get("outline") or []
        else:
            est["outline"] = rect_corners(x, y, length, width, est["yaw"])
        return est

    def report(self, pose):
        """Objects seen at least min_frames times, as JSON-ready dicts (metres, map frame)."""
        pose = tuple(pose) if pose is not None else (0.0, 0.0, 0.0)
        out = []
        for t in self.tracks:
            if t.id is None:            # not confirmed yet (seen fewer than min_frames times)
                continue
            e = self._estimate(t)
            dx, dy = e["x"] - pose[0], e["y"] - pose[1]
            bearing = math.degrees(math.atan2(dy, dx) - pose[2])
            bearing = (bearing + 180.0) % 360.0 - 180.0
            seen = 1.0 - math.exp(-t.frames_seen / 3.0)
            missed = 1.0 - float(t.misses) / self.miss_limit
            o = {"id": t.id, "class": t.cls,
                 "x": round(e["x"], 3), "y": round(e["y"], 3), "yaw": round(e["yaw"], 3),
                 "length_m": _r(e["length"]), "width_m": _r(e["width"]),
                 "height_m": _r(e["height"]), "height_is_min": e["height_is_min"],
                 "footprint_complete": e["footprint_complete"],
                 "size_is_estimate": e["size_is_estimate"],
                 "inner_diameter_m": _r(e["inner_diameter"]),
                 "outer_diameter_m": _r(e["outer_diameter"]),
                 "distance_m": round(distance_to_polygon(pose[0], pose[1], e["outline"]), 3),
                 "centre_distance_m": round(math.hypot(dx, dy), 3),
                 "bearing_deg": round(bearing, 1),
                 "frames_seen": t.frames_seen, "visible": t.visible,
                 "confidence": round(max(0.0, min(1.0, e["quality"] * seen * missed)), 2),
                 "source": e["source"],
                 "outline": [[round(px, 3), round(py, 3)] for px, py in e["outline"]]}
            if not math.isfinite(o["distance_m"]):
                o["distance_m"] = o["centre_distance_m"]
            o["label"] = short_label(o)
            o["text"] = describe(o)
            out.append(o)
        return out


def _r(v):
    return None if v is None else round(float(v), 3)


def size_text(o):
    """'31 x 27 cm' style footprint; craters: '34 cm across (inner 29 cm)'."""
    if o["class"] == "crater":
        d = o["outer_diameter_m"] or o["length_m"] or 0.0
        if o["size_is_estimate"]:
            return "~%s cm across (part of rim seen)" % _cm(d)
        s = "%s cm across" % _cm(d)
        if o["inner_diameter_m"]:
            s += " (inner %s cm)" % _cm(o["inner_diameter_m"])
        return s
    if o["class"] == "table_edge":
        return "%s cm wide" % _cm(o["length_m"] or 0.0)
    s = "%s x %s cm" % (_cm(o["length_m"] or 0.0), _cm(o["width_m"] or 0.0))
    if o["source"] == "no-depth":
        return "~" + s + " (no depth)"
    if not o["footprint_complete"]:
        s += " (back not seen)"
    return s


def height_text(o):
    h = o["height_m"]
    if h is None:
        return ""
    if o["class"] == "crater":
        return "rim %.1f cm high" % (100.0 * h)
    return "%s%s cm tall" % (">=" if o["height_is_min"] else "", _cm(h))


def describe(o):
    """One human line: 'rock #2: 31 x 27 cm, 24 cm tall, 0.82 m ahead'."""
    parts = [size_text(o)]
    h = height_text(o)
    if h:
        parts.append(h)
    where = direction_words(o["bearing_deg"])
    parts.append("%.2f m %s" % (o["distance_m"], where))
    return "%s #%d: %s" % (CLASS_NAMES.get(o["class"], o["class"]), o["id"], ", ".join(parts))


def short_label(o):
    """Map label: 'rock #2 31x27x24 cm', 'crater #1 34 cm', 'wall #3 120 cm'."""
    name = "%s #%d" % (CLASS_NAMES.get(o["class"], o["class"]), o["id"])
    if o["class"] == "crater":
        d = o["outer_diameter_m"] or o["length_m"] or 0.0
        return "%s %s%s cm" % (name, "~" if o["size_is_estimate"] else "", _cm(d))
    if o["class"] == "table_edge":
        return name
    if o["class"] == "wall":
        return "%s %s cm" % (name, _cm(o["length_m"] or 0.0))
    s = "%s %sx%s" % (name, _cm(o["length_m"] or 0.0), _cm(o["width_m"] or 0.0))
    if o["height_m"] is not None:
        s += "x%s%s" % (">=" if o["height_is_min"] else "", _cm(o["height_m"]))
    return s + " cm"


def objects_message(objects, frame_id, stamp, pose):
    """Body of the /terrain/objects JSON message."""
    rover = None
    if pose is not None:
        rover = {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 3)}
    return {"frame_id": frame_id, "stamp": round(float(stamp), 3), "rover": rover,
            "objects": objects}


def status_lines(objects, visible_only=True):
    return [o["text"] for o in objects if o["visible"] or not visible_only]


# ── preview image overlay ────────────────────────────────────────────────────
def project_world_points(points, pose, floor, camera_forward, rot, trans, intr):
    """Pixel (u, v) in the colour image of map-frame points (N x 3: x, y, height above floor).

    map -> robot frame (pose) -> depth-camera frame (floor model) -> colour-camera frame
    (rotation `rot` 3x3, translation `trans` metres: p_rgb = rot.p_depth + trans) -> pixels.
    Returns (uv N x 2, ok N bool): ok is False for points behind the colour camera."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = pts[:, 0] - pose[0], pts[:, 1] - pose[1]
    fwd, left = c * dx + s * dy, -s * dx + c * dy
    cam = np.stack([floor.from_robot([f], [l], camera_forward, h)[0]
                    for f, l, h in zip(fwd, left, pts[:, 2])]) if len(pts) else np.zeros((0, 3))
    rgb = cam.dot(np.asarray(rot, dtype=np.float64).T) + np.asarray(trans, dtype=np.float64)
    z = rgb[:, 2]
    ok = z > 0.05
    zs = np.where(ok, z, 1.0)
    uv = np.stack([rgb[:, 0] / zs * intr.fx + intr.cx, rgb[:, 1] / zs * intr.fy + intr.cy], axis=1)
    return uv, ok


def image_overlays(objects, pose, floor, camera_forward, rot, trans, intr):
    """For each visible object, the polylines to draw on the colour image and where its label
    goes: [{"id", "class", "label", "lines": [[(u, v), ...], ...], "anchor": (u, v)}].
    Rocks / walls are drawn as boxes (footprint at the floor and at their top), craters as
    the rim circle. Nothing is returned for an object fully behind the camera."""
    out = []
    for o in objects:
        if not o.get("visible") or not o.get("outline"):
            continue
        h = o["height_m"] or 0.0
        base = [[x, y, 0.0] for x, y in o["outline"]]
        lines = [base + [base[0]]]
        if o["class"] in ("rock", "wall") and h > 0:
            top = [[x, y, h] for x, y in o["outline"]]
            lines.append(top + [top[0]])
            for b, t in zip(base, top):
                lines.append([b, t])
        if o["class"] == "crater" and h > 0:
            lines = [[[x, y, h] for x, y, _ in lines[0]]]
        drawn = []
        for line in lines:
            uv, ok = project_world_points(line, pose, floor, camera_forward, rot, trans, intr)
            if ok.all():
                drawn.append([(float(u), float(v)) for u, v in uv])
        if not drawn:
            continue
        allp = np.array([p for line in drawn for p in line])
        anchor = (float(allp[:, 0].min()), float(allp[:, 1].min()) - 4.0)
        out.append({"id": o["id"], "class": o["class"], "label": o["label"], "lines": drawn,
                    "anchor": anchor})
    return out
