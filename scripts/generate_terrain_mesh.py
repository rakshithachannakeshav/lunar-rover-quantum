#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════╗
║  SMOOTH LUNAR TERRAIN GENERATOR  v2                                    ║
║                                                                        ║
║  Key design choices vs v1:                                             ║
║   • ONLY low-frequency noise (no high-freq spikes)                     ║
║   • scipy Gaussian smoothing applied twice (sigma 3 & 1.5)             ║
║   • Height range clamped to ±5 m → gently rolling landscape            ║
║   • Craters are wide + shallow (like real lunar craters)               ║
║   • 193×193 visual mesh (74 k triangles) for silky smooth surface      ║
║   • 65×65 collision mesh (8 k triangles) for fast physics              ║
║   • Proper smooth normals via central-difference gradient              ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

import numpy as np
from scipy.ndimage import gaussian_filter
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "worlds")
os.makedirs(OUT, exist_ok=True)

WORLD_SIZE  = 150.0   # metres across
MAX_HEIGHT  =   5.0   # maximum hill height above baseline (m)
CRATER_DEEP =   3.5   # maximum crater depth below rim (m)


# ─────────────────────────────────────────────────────────────────────────────
# Step 1 — Low-frequency base terrain
# ─────────────────────────────────────────────────────────────────────────────
def low_freq_terrain(grid_n: int, seed: int = 42) -> np.ndarray:
    """
    Build the base height field using only 3 low-frequency sine waves
    so there are NO high-frequency spikes.
    Each wave has a wavelength >> terrain grid spacing.
    """
    rng = np.random.default_rng(seed)
    xs  = np.linspace(0.0, 1.0, grid_n)
    X, Y = np.meshgrid(xs, xs)

    Z = np.zeros((grid_n, grid_n), dtype=np.float64)

    # Only 3 octaves, with large wavelengths and gentle amplitudes
    # freq=1 → 1 full cycle across 150 m (wavelength = 150 m)
    # freq=2 → 2 cycles, wavelength = 75 m
    # freq=3 → 3 cycles, wavelength = 50 m
    waves = [
        (1.0,  2.50, rng.uniform(0, 6.28), rng.uniform(0, 6.28)),
        (2.0,  1.50, rng.uniform(0, 6.28), rng.uniform(0, 6.28)),
        (3.0,  0.80, rng.uniform(0, 6.28), rng.uniform(0, 6.28)),
    ]
    for freq, amp, px, py in waves:
        Z += amp * np.sin(X * freq * 2 * np.pi + px) \
                 * np.cos(Y * freq * 2 * np.pi + py)

    return Z


# ─────────────────────────────────────────────────────────────────────────────
# Step 2 — Gaussian smoothing (the key fix)
# ─────────────────────────────────────────────────────────────────────────────
def smooth(Z: np.ndarray, sigma: float) -> np.ndarray:
    """Apply Gaussian blur — makes terrain silky smooth."""
    return gaussian_filter(Z, sigma=sigma, mode='reflect')


# ─────────────────────────────────────────────────────────────────────────────
# Step 3 — Add geological features (craters & hills) AFTER smoothing
# ─────────────────────────────────────────────────────────────────────────────
def add_craters(Z: np.ndarray, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
    """
    Add smooth Gaussian-bowl craters.
    Depth is subtle (≤ 3.5 m) and radii are large (8–25 m).
    Each crater also has a gentle raised rim.
    """
    # (cx_m, cy_m, radius_m, depth_m)
    craters = [
        ( 38,  20, 22, 3.2),   # large flat basin — far right
        (-30, -22, 26, 3.5),   # largest basin    — far left
        (  5, -20, 14, 2.5),   # medium crater
        (-12,  16, 10, 2.0),   # medium crater
        ( 22,   8,  8, 1.8),   # small crater
        (-40,   8, 16, 2.8),   # large far-left
        ( 15, -35, 12, 2.2),   # small far-bottom
    ]

    for cx, cy, cr, depth in craters:
        r = np.sqrt((X - cx)**2 + (Y - cy)**2)
        sigma_bowl = cr * 0.50   # wide, gentle bowl
        sigma_rim  = cr * 0.18   # narrow rim ring

        # Bowl depression
        Z -= depth * np.exp(-r**2 / (2 * sigma_bowl**2))

        # Raised rim (subtle — only 30 % of depth)
        rim_r = np.abs(r - cr)
        Z += depth * 0.30 * np.exp(-rim_r**2 / (2 * sigma_rim**2))

    return Z


def add_hills(Z: np.ndarray, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
    """
    Add smooth Gaussian hills.
    Heights ≤ 5 m, wide sigma for gentle slopes the rover can drive.
    """
    # (hx_m, hy_m, sigma_m, height_m)
    hills = [
        (-16,  28, 14, 4.5),
        ( 32, -28, 12, 4.0),
        (-32,  -8, 10, 3.5),
        (  4,  36, 13, 3.0),
        ( 22,  18,  9, 2.5),
        (-20,  10,  8, 2.0),
    ]

    for hx, hy, hr, ht in hills:
        r = np.sqrt((X - hx)**2 + (Y - hy)**2)
        Z += ht * np.exp(-r**2 / (2 * (hr * 0.60)**2))

    return Z


# ─────────────────────────────────────────────────────────────────────────────
# Step 4 — Flatten spawn zone so rover lands safely
# ─────────────────────────────────────────────────────────────────────────────
def flatten_spawn(Z: np.ndarray, X: np.ndarray, Y: np.ndarray,
                  radius: float = 6.0, target_z: float = -1.0) -> np.ndarray:
    """Blend terrain smoothly to target_z within radius of (0,0)."""
    r = np.sqrt(X**2 + Y**2)
    # Smooth cosine blend weight: 1 at centre, 0 at radius
    w = np.where(r < radius,
                 0.5 * (1.0 + np.cos(np.pi * r / radius)),
                 0.0)
    return Z * (1.0 - w) + target_z * w


# ─────────────────────────────────────────────────────────────────────────────
# Step 5 — Scale, clamp and re-centre
# ─────────────────────────────────────────────────────────────────────────────
def normalise(Z: np.ndarray, min_z: float = -6.0,
              max_z: float = 5.0) -> np.ndarray:
    """Hard-clamp to prevent extreme spikes."""
    return np.clip(Z, min_z, max_z)


# ─────────────────────────────────────────────────────────────────────────────
# Step 6 — Smooth vertex normals via central-difference gradient
# ─────────────────────────────────────────────────────────────────────────────
def compute_normals(X: np.ndarray, Y: np.ndarray,
                    Z: np.ndarray) -> tuple:
    dx = X[0, 2] - X[0, 0]
    dy = Y[2, 0] - Y[0, 0]

    Nz = np.ones_like(Z)
    Nx = np.zeros_like(Z)
    Ny = np.zeros_like(Z)

    # Interior — central differences
    Nx[1:-1, 1:-1] = -(Z[1:-1, 2:] - Z[1:-1, :-2]) / dx
    Ny[1:-1, 1:-1] = -(Z[2:, 1:-1] - Z[:-2, 1:-1]) / dy

    # Edges — replicate neighbours
    Nx[:,  0] = Nx[:,  1];  Nx[:, -1] = Nx[:, -2]
    Ny[0,  :] = Ny[1,  :];  Ny[-1, :] = Ny[-2, :]

    # Normalise
    L = np.sqrt(Nx**2 + Ny**2 + Nz**2)
    return Nx / L, Ny / L, Nz / L


# ─────────────────────────────────────────────────────────────────────────────
# Step 7 — Write OBJ
# ─────────────────────────────────────────────────────────────────────────────
def write_obj(path: str, X: np.ndarray, Y: np.ndarray,
              Z: np.ndarray, uv_tile: float = 10.0, label: str = "terrain"):
    n = Z.shape[0]
    Nx, Ny, Nz = compute_normals(X, Y, Z)
    n_verts = n * n
    n_tris  = 2 * (n - 1)**2

    print(f"  {label}")
    print(f"    Grid {n}×{n}  |  {n_verts:,} verts  |  {n_tris:,} tris")
    print(f"    Z: {Z.min():.2f} m → {Z.max():.2f} m  "
          f"(range {Z.max()-Z.min():.2f} m)")

    with open(path, "w") as f:
        f.write(f"# {label}\n# Auto-generated — do not edit\n\n")
        f.write("o terrain\n\n")

        # Vertices
        f.write("# Vertices\n")
        for j in range(n):
            for i in range(n):
                f.write(f"v {X[j,i]:.4f} {Y[j,i]:.4f} {Z[j,i]:.4f}\n")

        # UVs  (tiled so texture stays crisp on large terrain)
        f.write("\n# UV coordinates\n")
        for j in range(n):
            for i in range(n):
                u = (i / (n - 1)) * uv_tile
                v = (j / (n - 1)) * uv_tile
                f.write(f"vt {u:.4f} {v:.4f}\n")

        # Normals
        f.write("\n# Vertex normals\n")
        for j in range(n):
            for i in range(n):
                f.write(f"vn {Nx[j,i]:.4f} {Ny[j,i]:.4f} {Nz[j,i]:.4f}\n")

        # Faces — CCW triangles, two per quad
        f.write("\n# Faces\n")
        for j in range(n - 1):
            for i in range(n - 1):
                v00 = j * n + i + 1
                v10 = j * n + i + 2
                v01 = (j + 1) * n + i + 1
                v11 = (j + 1) * n + i + 2
                f.write(f"f {v00}/{v00}/{v00} {v11}/{v11}/{v11} "
                        f"{v10}/{v10}/{v10}\n")
                f.write(f"f {v00}/{v00}/{v00} {v01}/{v01}/{v01} "
                        f"{v11}/{v11}/{v11}\n")

    size_mb = os.path.getsize(path) / 1024 / 1024
    print(f"    → {path}  ({size_mb:.1f} MB)\n")


# ─────────────────────────────────────────────────────────────────────────────
# Pipeline
# ─────────────────────────────────────────────────────────────────────────────
def build_terrain(grid_n: int, seed: int = 42) -> tuple:
    xs = np.linspace(-WORLD_SIZE / 2, WORLD_SIZE / 2, grid_n)
    X, Y = np.meshgrid(xs, xs)

    # 1. Low-frequency base
    Z = low_freq_terrain(grid_n, seed)

    # 2. FIRST heavy smooth pass — kills any residual jaggedness
    Z = smooth(Z, sigma=grid_n * 0.06)   # ~6 % of grid width

    # 3. Add geological features
    Z = add_craters(Z, X, Y)
    Z = add_hills(Z, X, Y)

    # 4. SECOND lighter smooth pass — softens crater/hill edges
    Z = smooth(Z, sigma=grid_n * 0.025)

    # 5. Flatten spawn zone
    Z = flatten_spawn(Z, X, Y, radius=6.0, target_z=-1.0)

    # 6. Hard clamp — absolutely no spikes beyond ±6 m
    Z = normalise(Z, min_z=-CRATER_DEEP - 0.5, max_z=MAX_HEIGHT)

    return xs, X, Y, Z


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n══════════════════════════════════════════════════════")
    print("  Smooth Lunar Terrain Generator  v2")
    print("══════════════════════════════════════════════════════\n")

    # ── Visual mesh: 193×193 ──────────────────────────────────────────────────
    print("[1/2] Visual mesh (193×193) …")
    _, Xv, Yv, Zv = build_terrain(grid_n=193, seed=42)
    write_obj(
        os.path.join(OUT, "terrain_visual.obj"),
        Xv, Yv, Zv,
        uv_tile=10.0,
        label="Lunar terrain — visual (193×193)",
    )

    # ── Collision mesh: 65×65 ────────────────────────────────────────────────
    print("[2/2] Collision mesh (65×65) …")
    _, Xc, Yc, Zc = build_terrain(grid_n=65, seed=42)
    write_obj(
        os.path.join(OUT, "terrain_collision.obj"),
        Xc, Yc, Zc,
        uv_tile=4.0,
        label="Lunar terrain — collision (65×65)",
    )

    # Print spawn guidance
    ci = 193 // 2
    z0 = Zv[ci, ci]
    spawn_z = round(z0 + 0.55, 2)
    print(f"Terrain Z at origin : {z0:.3f} m")
    print(f"Recommended spawn Z : {spawn_z:.2f} m")
    print("\n══════════════════════════════════════════════════════")
    print("  DONE ✓")
    print("══════════════════════════════════════════════════════\n")
