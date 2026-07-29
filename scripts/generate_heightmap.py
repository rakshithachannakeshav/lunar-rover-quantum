#!/usr/bin/env python3
"""
FIXED Heightmap Generator v3
Root cause of previous bugs:
  1. H range was 0→0.5 only (missing upper half) because craters dominated
     and the clip to [0,1] after re-centering cut the top off.
  2. Rocks all had wrong Z — terrain was -12.5m at every rock position.

Fixes:
  • Build H in [-1, +1] normalised space, then map to [0.1, 0.9]
  • Use size=10m, pos=-5m → gentle ±5m terrain (no extreme spikes)
  • Compute actual terrain Z at every rock position and output it
  • Single final hard clamp with percentile to kill outliers
"""
import numpy as np
from scipy.ndimage import gaussian_filter
from PIL import Image
import os, struct, zlib

OUT   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "worlds")
os.makedirs(OUT, exist_ok=True)
SIZE  = 1025    # power-of-2 + 1
WORLD = 150.0   # world width/depth (m)
HRANGE = 10.0   # total height range in SDF (m)  — gentle ±5 m
HPOS   = -5.0   # pos z: base of heightmap so pixel=0.5 → z=0 m

# Pixel-to-world and world-to-pixel helpers
def world_to_px(wx, wy):
    px = int((wx + WORLD/2) / WORLD * (SIZE - 1))
    py = int((wy + WORLD/2) / WORLD * (SIZE - 1))
    return np.clip(px, 0, SIZE-1), np.clip(py, 0, SIZE-1)

def terrain_z(H, wx, wy):
    """Return world Z at (wx,wy) given current H field."""
    px, py = world_to_px(wx, wy)
    return HPOS + float(H[py, px]) * HRANGE


# ─────────────────────────────────────────────────────────────────────────────
# Build height field in proper [0,1] range with positive hills
# ─────────────────────────────────────────────────────────────────────────────
def build_H():
    xs = np.linspace(-WORLD/2, WORLD/2, SIZE)
    X, Y = np.meshgrid(xs, xs)
    rng  = np.random.default_rng(42)

    # ── 1. Low-frequency base noise (3 octaves only) ─────────────────────────
    H = np.zeros((SIZE, SIZE), dtype=np.float64)
    for freq, amp in [(1.0, 0.08), (2.0, 0.05), (3.0, 0.03)]:
        px = rng.uniform(0, 6.28)
        py = rng.uniform(0, 6.28)
        lx = np.linspace(0, freq * 6.28, SIZE)
        Xn, Yn = np.meshgrid(lx, lx)
        H += amp * np.sin(Xn + px) * np.cos(Yn + py)
    # H is now roughly in [-0.16, +0.16]

    # ── 2. Heavy Gaussian smooth — kills any high-freq artifacts ─────────────
    H = gaussian_filter(H, sigma=SIZE * 0.06)

    # ── 3. Craters (negative gaussian bowls) ─────────────────────────────────
    # Amplitudes in [0,1] space where full range = HRANGE = 10 m
    # depth 0.20 → 2 m deep;  depth 0.30 → 3 m deep
    craters = [
        ( 38,  20, 20, 0.28),
        (-28, -22, 24, 0.30),
        (  6, -18, 13, 0.20),
        (-12,  17,  9, 0.18),
        ( 20,   7,  7, 0.14),
        (-38,   8, 15, 0.24),
    ]
    for cx, cy, cr, depth in craters:
        r = np.sqrt((X - cx)**2 + (Y - cy)**2)
        s = cr * 0.45
        H -= depth * np.exp(-r**2 / (2*s**2))
        # Subtle rim (20 % of depth)
        rim = np.abs(r - cr)
        H += depth * 0.20 * np.exp(-rim**2 / (2*(cr*0.15)**2))

    # ── 4. Hills (positive gaussian bumps) ───────────────────────────────────
    # height 0.25 → 2.5 m tall;  height 0.35 → 3.5 m tall
    hills = [
        (-16,  28, 14, 0.35),
        ( 30, -28, 12, 0.30),
        (-30,  -8, 10, 0.28),
        (  5,  34, 13, 0.25),
        ( 22,  18,  9, 0.22),
        (-20,  10,  8, 0.20),
        (  0, -30, 11, 0.18),
    ]
    for hx, hy, hr, ht in hills:
        r = np.sqrt((X - hx)**2 + (Y - hy)**2)
        s = hr * 0.55
        H += ht * np.exp(-r**2 / (2*s**2))

    # ── 5. Second Gaussian smooth — softens edges of craters/hills ───────────
    H = gaussian_filter(H, sigma=SIZE * 0.020)

    # ── 6. Flatten 6 m spawn zone at origin ──────────────────────────────────
    flat_r = 6.0 / WORLD * (SIZE - 1)
    pi = np.arange(SIZE)
    PX, PY = np.meshgrid(pi, pi)
    d = np.sqrt((PX - SIZE//2)**2 + (PY - SIZE//2)**2)
    w = np.where(d < flat_r, 0.5*(1+np.cos(np.pi*d/flat_r)), 0.0)
    H = H*(1-w) + 0.0*w  # blend to 0 (midpoint before mapping)

    # ── 7. Kill outliers: clip to [2nd, 98th] percentile ────────────────────
    lo, hi = np.percentile(H, 2), np.percentile(H, 98)
    H = np.clip(H, lo, hi)
    print(f"  After percentile clip: [{lo:.4f}, {hi:.4f}]")

    # ── 8. One final gentle smooth ────────────────────────────────────────────
    H = gaussian_filter(H, sigma=SIZE * 0.010)

    # ── 9. Map to [0.10, 0.90] so midpoint=0 stays at 0.50 ─────────────────
    # First normalise so current range spans [-1, +1]
    h_abs_max = max(abs(H.min()), abs(H.max()))
    if h_abs_max > 1e-6:
        H = H / h_abs_max          # now in [-1, +1]
    H = H * 0.40 + 0.50           # map [-1,+1] → [0.10, 0.90]

    # Verify centre pixel is 0.50 (because spawn zone is flat at H=0→midpoint)
    H_c = H[SIZE//2, SIZE//2]
    print(f"  Centre pixel H = {H_c:.4f}  (want 0.50, diff={H_c-0.5:+.4f})")

    H = np.clip(H, 0.0, 1.0)

    # Print final Z stats
    zmin = HPOS + H.min() * HRANGE
    zmax = HPOS + H.max() * HRANGE
    zc   = HPOS + H[SIZE//2, SIZE//2] * HRANGE
    print(f"  Final Z range: {zmin:.2f} m  →  {zmax:.2f} m")
    print(f"  Z at origin:   {zc:.3f} m")
    return H


# ─────────────────────────────────────────────────────────────────────────────
# Write 16-bit PNG (manual to avoid PIL 16-bit issues)
# ─────────────────────────────────────────────────────────────────────────────
def write_png16(path, H):
    h16 = (H * 65535).clip(0, 65535).astype(np.uint16)
    n   = h16.shape[0]

    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)

    sig  = b'\x89PNG\r\n\x1a\n'
    ihdr = chunk(b'IHDR', struct.pack('>IIBBBBB', n, n, 16, 0, 0, 0, 0))
    rows = b''.join(b'\x00' + row.astype('>u2').tobytes() for row in h16)
    idat = chunk(b'IDAT', zlib.compress(rows, 6))
    iend = chunk(b'IEND', b'')

    with open(path, 'wb') as f:
        f.write(sig + ihdr + idat + iend)
    kb = os.path.getsize(path) / 1024
    print(f"  → {path}  ({kb:.0f} KB,  {n}×{n}, 16-bit)")


# ─────────────────────────────────────────────────────────────────────────────
# Compute correct rock Z positions from heightmap
# ─────────────────────────────────────────────────────────────────────────────
def compute_rock_positions(H):
    """
    Each entry: (name, wx, wy, half_height_of_rock)
    Returns dict name → (wx, wy, wz_on_surface)
    """
    rocks = [
        ('boulder_a',  14,  -8,  1.1),
        ('boulder_b', -12,  10,  1.0),
        ('boulder_c',   8,  20,  0.8),
        ('rock_d',     -5, -14,  0.4),
        ('rock_e',     22,   5,  0.5),
        ('boulder_f', -22,  -8,  1.4),
        ('rock_g',      3,  -5,  0.25),
        ('boulder_h',  -8, -20,  0.7),
        ('boulder_i', -35,  14,  0.9),
        ('rock_j',     30, -12,  0.6),
    ]
    print("\n  Rock surface placement:")
    positions = {}
    for name, wx, wy, half_h in rocks:
        tz = terrain_z(H, wx, wy)
        # rock centre Z = terrain_z + half_height_of_rock
        rz = tz + half_h + 0.05   # tiny margin so it rests on surface
        positions[name] = (wx, wy, rz, tz)
        print(f"    {name:12s} ({wx:4},{wy:4})  terrain={tz:+.2f}m  rock_z={rz:+.2f}m")
    return positions


# ─────────────────────────────────────────────────────────────────────────────
def write_collision_obj(path, H_full):
    # Downsample 1025x1025 to 129x129 for fast and accurate physics
    H_coll = H_full[::8, ::8]
    n = H_coll.shape[0]
    
    xs = np.linspace(-WORLD/2, WORLD/2, n)
    X, Y = np.meshgrid(xs, xs)
    Z = H_coll * HRANGE
    
    # Compute central difference normals
    dx = xs[2] - xs[0]
    dy = xs[2] - xs[0]
    Nz = np.ones_like(Z)
    Nx = np.zeros_like(Z)
    Ny = np.zeros_like(Z)
    
    # Interior central difference
    Nx[1:-1, 1:-1] = -(Z[1:-1, 2:] - Z[1:-1, :-2]) / dx
    Ny[1:-1, 1:-1] = -(Z[2:, 1:-1] - Z[:-2, 1:-1]) / dy
    
    # Border normal replication
    Nx[:, 0] = Nx[:, 1]; Nx[:, -1] = Nx[:, -2]
    Ny[0, :] = Ny[1, :]; Ny[-1, :] = Ny[-2, :]
    
    # Normalization
    L = np.sqrt(Nx**2 + Ny**2 + Nz**2)
    Nx, Ny, Nz = Nx / L, Ny / L, Nz / L
    
    with open(path, 'w') as f:
        f.write("# Terrain Collision Mesh\n")
        f.write("o terrain_collision\n")
        
        # Vertices
        for j in range(n):
            for i in range(n):
                f.write(f"v {X[j,i]:.4f} {Y[j,i]:.4f} {Z[j,i]:.4f}\n")
                
        # Normals
        for j in range(n):
            for i in range(n):
                f.write(f"vn {Nx[j,i]:.4f} {Ny[j,i]:.4f} {Nz[j,i]:.4f}\n")
                
        # Faces referencing both vertices and normals
        for j in range(n - 1):
            for i in range(n - 1):
                v00 = j * n + i + 1
                v10 = j * n + i + 2
                v01 = (j + 1) * n + i + 1
                v11 = (j + 1) * n + i + 2
                f.write(f"f {v00}//{v00} {v11}//{v11} {v10}//{v10}\n")
                f.write(f"f {v00}//{v00} {v01}//{v01} {v11}//{v11}\n")
                
    kb = os.path.getsize(path) / 1024
    print(f"  → {path}  ({kb:.0f} KB,  {n}×{n} collision mesh with normals)")


# ─────────────────────────────────────────────────────────────────────────────
# Also generate solid-colour diffuse + flat normal map textures
# ─────────────────────────────────────────────────────────────────────────────
def write_textures():
    # 128×128 lunar grey diffuse with subtle grain
    rng = np.random.default_rng(7)
    base = np.array([162, 148, 128], dtype=np.float32)
    noise = rng.normal(0, 5, (128, 128, 3)).astype(np.float32)
    diff = np.clip(base + noise, 0, 255).astype(np.uint8)
    Image.fromarray(diff, 'RGB').save(os.path.join(OUT, 'terrain_tex.png'))

    # 128×128 flat normal map
    nm = np.zeros((128, 128, 3), dtype=np.uint8)
    nm[:,:,0] = 128; nm[:,:,1] = 128; nm[:,:,2] = 255
    Image.fromarray(nm, 'RGB').save(os.path.join(OUT, 'terrain_normal.png'))
    print("  → terrain_tex.png + terrain_normal.png  (128×128)")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n══════════════════════════════════════════════")
    print("  Fixed Heightmap Generator  v3")
    print(f"  SIZE={SIZE}  WORLD={WORLD}m  HRANGE={HRANGE}m  HPOS={HPOS}m")
    print("══════════════════════════════════════════════\n")

    print("[1/3] Building heightmap and collision mesh …")
    H = build_H()
    write_png16(os.path.join(OUT, 'heightmap.png'), H)
    write_collision_obj(os.path.join(OUT, 'terrain_collision.obj'), H)

    print("\n[2/3] Computing rock positions …")
    rock_pos = compute_rock_positions(H)

    print("\n[3/3] Writing textures …")
    write_textures()

    print(f"\n  Spawn Z recommendation: +2.0 m  (terrain at origin = {HPOS + H[SIZE//2,SIZE//2]*HRANGE:.2f} m)")
    print("\n══════════════════════════════════════════════")
    print("  DONE ✓")
    print("══════════════════════════════════════════════\n")
