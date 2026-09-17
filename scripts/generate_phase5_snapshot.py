import os
import json
from datetime import datetime

import cv2
import numpy as np


HEIGHTMAP = "src/rover_simulation/worlds/heightmap.png"
OUTPUT_DIR = "results/terrain_maps"

RESOLUTION = 0.05
ORIGIN_X = -17.0
ORIGIN_Y = -15.0


def main():
    if not os.path.exists(HEIGHTMAP):
        raise FileNotFoundError(
            f"Heightmap not found: {HEIGHTMAP}"
        )

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    image = cv2.imread(
        HEIGHTMAP,
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f"Could not read {HEIGHTMAP}"
        )

    height_values = image.astype(np.float32)

    minimum = float(height_values.min())
    maximum = float(height_values.max())

    if maximum > minimum:
        normalized = (
            (height_values - minimum)
            / (maximum - minimum)
        )
    else:
        normalized = np.zeros_like(height_values)

    # Terrain encoding used by the project's
    # TerrainStorage / graph model:
    #
    # -1  = unknown
    # 10  = flat
    # 50  = rocky
    # 90  = crater
    # 100 = obstacle

    terrain = np.full(
        image.shape,
        10,
        dtype=np.int8
    )

    # Estimate terrain roughness.
    gx = cv2.Sobel(
        normalized,
        cv2.CV_32F,
        1,
        0,
        ksize=3
    )

    gy = cv2.Sobel(
        normalized,
        cv2.CV_32F,
        0,
        1,
        ksize=3
    )

    roughness = cv2.magnitude(gx, gy)

    terrain[roughness > 0.12] = 50

    # Very high regions are treated as obstacles.
    terrain[normalized > 0.92] = 100

    height, width = terrain.shape
    timestamp = datetime.now().isoformat()

    np.savez_compressed(
        os.path.join(
            OUTPUT_DIR,
            "latest.npz"
        ),
        grid=terrain,
        resolution=RESOLUTION,
        origin_x=ORIGIN_X,
        origin_y=ORIGIN_Y,
        width=width,
        height=height,
        timestamp=timestamp,
    )

    values, counts = np.unique(
        terrain,
        return_counts=True
    )

    terrain_counts = {
        int(value): int(count)
        for value, count in zip(values, counts)
    }

    metadata = {
        "timestamp": timestamp,
        "timestamp_epoch": datetime.now().timestamp(),
        "resolution": RESOLUTION,
        "origin_x": ORIGIN_X,
        "origin_y": ORIGIN_Y,
        "width": width,
        "height": height,
        "cell_counts": {
            "unknown_count": terrain_counts.get(-1, 0),
            "flat_count": terrain_counts.get(10, 0),
            "rocky_count": terrain_counts.get(50, 0),
            "crater_count": terrain_counts.get(90, 0),
            "obstacle_count": terrain_counts.get(100, 0),
            "total_cells": width * height,
        },
        "encoding": {
            "-1": "unknown",
            "10": "flat",
            "50": "rocky",
            "90": "crater_interior",
            "100": "obstacle",
        },
        "source": "offline_heightmap_phase5",
    }

    with open(
        os.path.join(
            OUTPUT_DIR,
            "latest_meta.json"
        ),
        "w",
        encoding="utf-8"
    ) as file:
        json.dump(
            metadata,
            file,
            indent=2
        )

    print(
        "Created:",
        os.path.join(
            OUTPUT_DIR,
            "latest.npz"
        )
    )

    print(
        "Created:",
        os.path.join(
            OUTPUT_DIR,
            "latest_meta.json"
        )
    )

    print(
        f"Grid: {width} x {height}"
    )

    print(
        f"Terrain counts: {terrain_counts}"
    )


if __name__ == "__main__":
    main()