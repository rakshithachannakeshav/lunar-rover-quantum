import os
import numpy as np
from PIL import Image

img = Image.open(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'rover_simulation', 'worlds', 'heightmap.png')))
arr = np.array(img).astype(np.float32) / 65535.0
print('PNG shape:', arr.shape, '| mode:', img.mode)
print('H range:  %.4f -> %.4f' % (arr.min(), arr.max()))
print('Z range:  %.2f m -> %.2f m' % ((arr.min()-0.5)*25, (arr.max()-0.5)*25))
print('Z at origin (512,512): %.3f m' % ((arr[512,512]-0.5)*25))
p01 = np.percentile(arr, 1)
p99 = np.percentile(arr, 99)
print('1pct Z: %.2f m | 99pct Z: %.2f m' % ((p01-0.5)*25, (p99-0.5)*25))

WORLD = 150.0
SIZE  = 1025
rocks = [
  ('boulder_a', 14, -8),  ('boulder_b', -12, 10), ('boulder_c', 8,  20),
  ('rock_d',    -5, -14), ('rock_e',    22,   5),  ('boulder_f', -22, -8),
  ('rock_g',    3,  -5),  ('boulder_h', -8, -20),  ('boulder_i', -35, 14),
  ('rock_j',    30, -12),
]
print()
print('Rock terrain heights:')
for name, wx, wy in rocks:
    px = int((wx + WORLD/2) / WORLD * (SIZE-1))
    py = int((wy + WORLD/2) / WORLD * (SIZE-1))
    px = max(0, min(SIZE-1, px))
    py = max(0, min(SIZE-1, py))
    h = float(arr[py, px])
    z = (h - 0.5) * 25
    print('  %-12s (%4d,%4d) terrain_z = %+.2f m' % (name, wx, wy, z))
