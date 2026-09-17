#!/usr/bin/env python3
"""
Generate procedural terrain assets for the Lunar Rover Gazebo simulation.
Outputs:
  - heightmap.png      : 513x513 greyscale, multi-octave terrain relief
  - terrain_diffuse.png: 1024x1024 RGB muddy grey-brown regolith texture
  - terrain_normal.png : 1024x1024 RGB surface normal map for depth
  - rock_diffuse.png   : 256x256  RGB dark basalt rock texture
"""

import os
import numpy as np
from PIL import Image

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "worlds")
os.makedirs(OUT, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# Utility: band-limited noise by summing trig octaves
# ─────────────────────────────────────────────────────────────────────────────
def trig_noise(size, octaves, seed=0):
    rng = np.random.default_rng(seed)
    X, Y = np.meshgrid(np.linspace(0, 1, size), np.linspace(0, 1, size))
    h = np.zeros((size, size), dtype=np.float64)
    amp = 1.0
    total = 0.0
    for i in range(octaves):
        freq = 2 ** (i + 1)
        px = rng.uniform(0, 2 * np.pi)
        py = rng.uniform(0, 2 * np.pi)
        h += amp * (
            np.sin(X * freq * 2 * np.pi + px) *
            np.cos(Y * freq * 2 * np.pi + py)
        )
        total += amp
        amp *= 0.55   # each octave is quieter
    return h / total   # normalised to [-1, 1] roughly


# ─────────────────────────────────────────────────────────────────────────────
# 1. HEIGHTMAP  — 513×513 greyscale (Gazebo needs power-of-2 + 1)
#    Multiple octaves → gentle rolling hills + fine surface texture
# ─────────────────────────────────────────────────────────────────────────────
print("Generating heightmap.png …")
SIZE_HM = 513

# Large-scale rolling terrain
low  = trig_noise(SIZE_HM, octaves=3, seed=42)
# Medium lumps (crater rims etc.)
mid  = trig_noise(SIZE_HM, octaves=4, seed=7)
# Fine surface grain
fine = trig_noise(SIZE_HM, octaves=5, seed=99)

h = low * 0.55 + mid * 0.30 + fine * 0.15

# Carve 3 soft crater depressions with gaussian bowls
for (cx, cy, cr, depth) in [(0.35, 0.60, 0.10, 0.5),
                              (0.70, 0.25, 0.07, 0.4),
                              (0.55, 0.80, 0.05, 0.3)]:
    gx = np.linspace(0, 1, SIZE_HM)
    gy = np.linspace(0, 1, SIZE_HM)
    GX, GY = np.meshgrid(gx, gy)
    dist = np.sqrt((GX - cx)**2 + (GY - cy)**2)
    crater = -depth * np.exp(-(dist**2) / (2 * (cr * 0.5)**2))
    h += crater

# Normalise to 0–255
h = (h - h.min()) / (h.max() - h.min())
hm_img = Image.fromarray((h * 255).astype(np.uint8), mode='L')
hm_img.save(os.path.join(OUT, "heightmap.png"))
print(f"  → {OUT}/heightmap.png  ({SIZE_HM}×{SIZE_HM})")


# ─────────────────────────────────────────────────────────────────────────────
# 2. TERRAIN DIFFUSE TEXTURE — 1024×1024 muddy grey-brown regolith
#    Realistic mix of: dark basalt dust, lighter sand patches, mud tones
# ─────────────────────────────────────────────────────────────────────────────
print("Generating terrain_diffuse.png …")
SIZE_TX = 1024

# Three noise layers at different scales for colour variation
c0 = trig_noise(SIZE_TX, octaves=3, seed=11)   # large blotches
c1 = trig_noise(SIZE_TX, octaves=5, seed=22)   # medium grain
c2 = trig_noise(SIZE_TX, octaves=7, seed=33)   # fine speckle

# Combine
cn = c0 * 0.50 + c1 * 0.30 + c2 * 0.20
cn = (cn - cn.min()) / (cn.max() - cn.min())   # 0→1

# Base colour palette (lunar mud: warm grey-brown)
# dark basalt    → (80,  75,  65)   when cn low
# mid regolith   → (115, 108, 96)   when cn mid
# lighter dust   → (145, 138, 124)  when cn high
r = (80 + cn * 65).astype(np.uint8)
g = (72 + cn * 66).astype(np.uint8)
b = (60 + cn * 64).astype(np.uint8)

# Scatter a few darker mud patches
mud = trig_noise(SIZE_TX, octaves=6, seed=55)
mud = (mud - mud.min()) / (mud.max() - mud.min())
mask = (mud > 0.72).astype(np.float32)   # ~28 % coverage

r = np.clip(r.astype(np.float32) - mask * 28, 0, 255).astype(np.uint8)
g = np.clip(g.astype(np.float32) - mask * 24, 0, 255).astype(np.uint8)
b = np.clip(b.astype(np.float32) - mask * 18, 0, 255).astype(np.uint8)

diffuse_arr = np.stack([r, g, b], axis=-1)
Image.fromarray(diffuse_arr, mode='RGB').save(
    os.path.join(OUT, "terrain_diffuse.png"))
print(f"  → {OUT}/terrain_diffuse.png  ({SIZE_TX}×{SIZE_TX})")


# ─────────────────────────────────────────────────────────────────────────────
# 3. TERRAIN NORMAL MAP — 1024×1024 blue-ish surface normals
#    Adds micro-relief depth so lighting shows surface texture
# ─────────────────────────────────────────────────────────────────────────────
print("Generating terrain_normal.png …")
n0 = trig_noise(SIZE_TX, octaves=6, seed=77)
n1 = trig_noise(SIZE_TX, octaves=8, seed=88)
nf = n0 * 0.6 + n1 * 0.4
nf = (nf - nf.min()) / (nf.max() - nf.min())   # 0→1

# Compute dx/dy gradients to get surface normals
dx = np.gradient(nf, axis=1) * 4.0   # strength multiplier
dy = np.gradient(nf, axis=0) * 4.0
# Build normal vector (Nx, Ny, Nz), normalise
length = np.sqrt(dx**2 + dy**2 + 1.0)
nx = -dx / length
ny = -dy / length
nz =  1.0 / length

# Encode into RGB: 0 = -1, 128 = 0, 255 = +1
nr = ((nx + 1.0) * 0.5 * 255).astype(np.uint8)
ng = ((ny + 1.0) * 0.5 * 255).astype(np.uint8)
nb = ((nz + 1.0) * 0.5 * 255).astype(np.uint8)

normal_arr = np.stack([nr, ng, nb], axis=-1)
Image.fromarray(normal_arr, mode='RGB').save(
    os.path.join(OUT, "terrain_normal.png"))
print(f"  → {OUT}/terrain_normal.png  ({SIZE_TX}×{SIZE_TX})")


# ─────────────────────────────────────────────────────────────────────────────
# 4. ROCK DIFFUSE — 256×256 dark basalt rock texture
#    Coarser grain, darker, slight grey sparkle (feldspar)
# ─────────────────────────────────────────────────────────────────────────────
print("Generating rock_diffuse.png …")
SIZE_RK = 256

r0 = trig_noise(SIZE_RK, octaves=5, seed=13)
r1 = trig_noise(SIZE_RK, octaves=7, seed=17)
rf = r0 * 0.6 + r1 * 0.4
rf = (rf - rf.min()) / (rf.max() - rf.min())

rr = (52 + rf * 40).astype(np.uint8)
rg = (50 + rf * 38).astype(np.uint8)
rb = (46 + rf * 36).astype(np.uint8)

# White feldspar sparkles
sparkle = trig_noise(SIZE_RK, octaves=9, seed=31)
sparkle = (sparkle - sparkle.min()) / (sparkle.max() - sparkle.min())
sp_mask = (sparkle > 0.90).astype(np.float32)
rr = np.clip(rr.astype(np.float32) + sp_mask * 60, 0, 255).astype(np.uint8)
rg = np.clip(rg.astype(np.float32) + sp_mask * 60, 0, 255).astype(np.uint8)
rb = np.clip(rb.astype(np.float32) + sp_mask * 60, 0, 255).astype(np.uint8)

rock_arr = np.stack([rr, rg, rb], axis=-1)
Image.fromarray(rock_arr, mode='RGB').save(
    os.path.join(OUT, "rock_diffuse.png"))
print(f"  → {OUT}/rock_diffuse.png  ({SIZE_RK}×{SIZE_RK})")

print("\nAll terrain textures generated successfully!")
