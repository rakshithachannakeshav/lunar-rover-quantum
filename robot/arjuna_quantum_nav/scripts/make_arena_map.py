#!/usr/bin/env python3
"""Turn the arena size and a list of craters / rocks into a map_server map (PGM + YAML).

Python 3.6, standard library only; runs on the Jetson or any laptop.

    python3 make_arena_map.py layout.json --out ~/arena_map            # writes arena_map.pgm/.yaml
    python3 make_arena_map.py layout.json --out ~/arena_map --preview  # also prints it as text
    roslaunch arjuna_quantum_nav arena.launch use_map:=true map_file:=$HOME/arena_map.yaml

The layout is JSON. Positions are measured from the arena's ORIGIN CORNER along its two edges
(x along one edge, y along the other; the rover's start pose must be given in the same frame).
Sizes are across the widest point of the object.

    {"units": "in",                      // in | cm | m   (default m)
     "size": 87,                         // square arena; or "width" and "height"
     "border_width": 1.2,                // thickness of the low border wall, drawn as blocked
     "craters": [{"x": 25, "y": 30, "diameter": 14},          // octagon; "rotation_deg" optional
                 {"x": 60, "y": 45, "diameter": 12}],
     "rocks":   [{"x": 40, "y": 70, "width": 14, "depth": 12}]}   // "rotation_deg" optional

Craters are filled completely (rim and inside), rocks as rectangles, the border wall as a band
just inside the edge, and everything outside the arena is blocked, so the planners never drive
off the table. The planners add the rover's radius round every blocked cell themselves
(robot_radius), so draw the objects at their true size here.
"""

import argparse
import json
import math
import os
import sys

UNIT = {"m": 1.0, "cm": 0.01, "mm": 0.001, "in": 0.0254}
FREE, OCC = 254, 0


def load_layout(path):
    with open(os.path.expanduser(path)) as f:
        text = "\n".join(line.split("//")[0] for line in f)     # allow // comments
    return json.loads(text)


def normalise(layout):
    """Layout dict -> everything in metres."""
    k = UNIT[str(layout.get("units", "m"))]
    w = layout.get("width", layout.get("size"))
    h = layout.get("height", layout.get("size"))
    if w is None or h is None:
        raise ValueError('layout needs "size" (or "width" and "height")')
    out = {"width": w * k, "height": h * k,
           "border": float(layout.get("border_width", 0.0)) * k,
           "origin": [float(v) * k for v in layout.get("arena_origin", [0.0, 0.0])],
           "craters": [], "rocks": []}
    for c in layout.get("craters", []):
        out["craters"].append({"x": c["x"] * k, "y": c["y"] * k, "d": c["diameter"] * k,
                               "rot": math.radians(c.get("rotation_deg", 0.0))})
    for r in layout.get("rocks", []):
        out["rocks"].append({"x": r["x"] * k, "y": r["y"] * k, "w": r["width"] * k,
                             "d": r.get("depth", r["width"]) * k,
                             "rot": math.radians(r.get("rotation_deg", 0.0))})
    for o in out["craters"] + out["rocks"]:
        if not (0.0 <= o["x"] <= out["width"] and 0.0 <= o["y"] <= out["height"]):
            raise ValueError("object at (%.2f, %.2f) m is outside the %.2f x %.2f m arena"
                             % (o["x"], o["y"], out["width"], out["height"]))
    return out


def in_octagon(px, py, c):
    """Regular octagon with circumradius d/2 (corner to corner = d), rotated by rot."""
    apothem = c["d"] / 2.0 * math.cos(math.radians(22.5))
    dx, dy = px - c["x"], py - c["y"]
    for k in range(8):
        a = c["rot"] + math.radians(22.5 + 45 * k)
        if dx * math.cos(a) + dy * math.sin(a) > apothem:
            return False
    return True


def in_rect(px, py, r):
    dx, dy = px - r["x"], py - r["y"]
    c, s = math.cos(r["rot"]), math.sin(r["rot"])
    return abs(dx * c + dy * s) <= r["w"] / 2.0 and abs(-dx * s + dy * c) <= r["d"] / 2.0


def build_map(layout, resolution=0.01, margin=0.20):
    """Returns (width_px, height_px, rows, origin) where rows[0] is the TOP row (max y) and
    origin = (x, y) of the bottom-left pixel corner in the arena frame."""
    lay = normalise(layout)
    ox, oy = lay["origin"]
    x0, y0 = ox - margin, oy - margin
    wpx = int(math.ceil((lay["width"] + 2 * margin) / resolution))
    hpx = int(math.ceil((lay["height"] + 2 * margin) / resolution))
    rows = []
    for j in range(hpx):
        y = y0 + (hpx - 1 - j + 0.5) * resolution
        row = bytearray([FREE]) * wpx
        for i in range(wpx):
            x = x0 + (i + 0.5) * resolution
            ax, ay = x - ox, y - oy                      # position inside the arena
            blocked = not (0.0 <= ax <= lay["width"] and 0.0 <= ay <= lay["height"])
            if not blocked and lay["border"] > 0:
                b = lay["border"]
                blocked = (ax < b or ay < b or ax > lay["width"] - b or ay > lay["height"] - b)
            if not blocked:
                blocked = any(in_octagon(ax, ay, c) for c in lay["craters"]) or \
                    any(in_rect(ax, ay, r) for r in lay["rocks"])
            if blocked:
                row[i] = OCC
        rows.append(bytes(row))
    return wpx, hpx, rows, (x0, y0)


def write_map(layout, out, resolution=0.01, margin=0.20):
    """Write <out>.pgm and <out>.yaml. Returns (pgm_path, yaml_path)."""
    out = os.path.expanduser(out)
    wpx, hpx, rows, (x0, y0) = build_map(layout, resolution, margin)
    pgm, yml = out + ".pgm", out + ".yaml"
    with open(pgm, "wb") as f:
        f.write(("P5\n# arena map made by make_arena_map.py\n%d %d\n255\n" % (wpx, hpx)).encode())
        for r in rows:
            f.write(r)
    with open(yml, "w") as f:
        f.write("image: %s\nresolution: %g\norigin: [%.4f, %.4f, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (os.path.basename(pgm),
                                                               resolution, x0, y0))
    return pgm, yml


def read_map(yaml_path):
    """Read a PGM + YAML back as (width, height, resolution, origin_x, origin_y, data) in the
    nav_msgs/OccupancyGrid layout (row 0 = bottom, 0..100 / -1). Handles what write_map and
    map_server's simple YAML write."""
    yaml_path = os.path.expanduser(yaml_path)
    info = {}
    with open(yaml_path) as f:
        for line in f:
            if ":" in line and not line.strip().startswith("#"):
                k, v = line.split(":", 1)
                info[k.strip()] = v.strip()
    ox, oy = [float(v) for v in info["origin"].strip("[]").split(",")[:2]]
    res = float(info["resolution"])
    occ_t, free_t = float(info.get("occupied_thresh", 0.65)), float(info.get("free_thresh", 0.196))
    negate = int(info.get("negate", 0))
    with open(os.path.join(os.path.dirname(yaml_path), info["image"]), "rb") as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                              # P5, width, height, maxval
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            while raw[pos:pos + 1] != b"\n":
                pos += 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pix = raw[pos + 1:pos + 1 + w * h]
    data = [0] * (w * h)
    for j in range(h):
        for i in range(w):
            v = pix[j * w + i] / float(maxval)
            p = v if negate else 1.0 - v
            data[(h - 1 - j) * w + i] = 100 if p > occ_t else (0 if p < free_t else -1)
    return w, h, res, ox, oy, data


def preview(layout, resolution=0.05, margin=0.0):
    wpx, hpx, rows, _ = build_map(layout, resolution, margin)
    return "\n".join("".join("#" if v == OCC else "." for v in r) for r in rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("layout", help="layout JSON file")
    ap.add_argument("--out", default="~/arena_map", help="output path without extension")
    ap.add_argument("--resolution", type=float, default=0.01, help="metres per pixel")
    ap.add_argument("--margin", type=float, default=0.20, help="blocked metres drawn outside")
    ap.add_argument("--preview", action="store_true", help="print the map as text")
    args = ap.parse_args(argv)
    layout = load_layout(args.layout)
    pgm, yml = write_map(layout, args.out, args.resolution, args.margin)
    lay = normalise(layout)
    print("Wrote %s and %s" % (pgm, yml))
    print("Arena %.3f x %.3f m, %d craters, %d rocks" % (lay["width"], lay["height"],
                                                          len(lay["craters"]), len(lay["rocks"])))
    if args.preview:
        print(preview(layout, 0.05, 0.1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
