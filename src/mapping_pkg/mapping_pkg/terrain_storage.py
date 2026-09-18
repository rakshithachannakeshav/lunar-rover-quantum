#!/usr/bin/env python3

# Copyright 2026 Rakshitha Channakeshav
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Handles periodic persistence of classified terrain grids to disk."""

from datetime import datetime
import json
import os
from typing import Any, Dict, Optional

import numpy as np


class TerrainStorage:
    """Manages serialization of classified terrain maps to the results/ directory."""

    def __init__(
        self,
        output_dir: str = '',
        save_interval_sec: float = 10.0,
        keep_history: bool = False
    ):
        """Initialize TerrainStorage.

        Args:
            output_dir: Destination folder. If empty, automatically resolves to
                        <repo_root>/results/terrain_maps.
            save_interval_sec: Minimum duration in seconds between disk writes.
            keep_history: If True, writes timestamped archives in addition to latest.npz.

        """
        self.save_interval_sec = save_interval_sec
        self.keep_history = keep_history
        self.last_save_time: Optional[float] = None

        if not output_dir:
            cwd = os.getcwd()
            has_results = os.path.isdir(os.path.join(cwd, 'results'))
            has_src = os.path.isdir(os.path.join(cwd, 'src'))
            if has_results and has_src:
                repo_root = cwd
            else:
                candidate = os.path.dirname(os.path.abspath(__file__))
                repo_root = candidate
                for _ in range(8):
                    if (os.path.isdir(os.path.join(candidate, 'results')) and
                            os.path.isdir(os.path.join(candidate, 'src'))):
                        repo_root = candidate
                        break
                    parent = os.path.dirname(candidate)
                    if parent == candidate:
                        break
                    candidate = parent
            self.output_dir = os.path.join(repo_root, 'results', 'terrain_maps')
        else:
            self.output_dir = os.path.abspath(output_dir)

        os.makedirs(self.output_dir, exist_ok=True)

    def should_save(self, current_time_sec: float) -> bool:
        """Check if save_interval_sec has elapsed since last save."""
        if self.last_save_time is None:
            return True
        return (current_time_sec - self.last_save_time) >= self.save_interval_sec

    def save(
        self,
        terrain_grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        width: int,
        height: int,
        timestamp_sec: Optional[float] = None
    ) -> Dict[str, str]:
        """Save terrain map grid and metadata to disk.

        Args:
            terrain_grid: 2D numpy array (height, width) with encoded terrain values.
            resolution: Grid resolution in meters/cell.
            origin_x: World X coordinate of grid origin in meters.
            origin_y: World Y coordinate of grid origin in meters.
            width: Number of columns.
            height: Number of rows.
            timestamp_sec: Current UNIX timestamp in seconds.

        Returns:
            Dictionary with paths of saved files.

        """
        now = datetime.now()
        iso_time = now.isoformat()
        current_sec = timestamp_sec if timestamp_sec is not None else now.timestamp()

        unique, counts = np.unique(terrain_grid, return_counts=True)
        counts_dict = {int(k): int(v) for k, v in zip(unique, counts)}

        class_summary = {
            'unknown_count': counts_dict.get(-1, 0),
            'flat_count': counts_dict.get(10, 0),
            'rocky_count': counts_dict.get(50, 0),
            'crater_count': counts_dict.get(90, 0),
            'obstacle_count': counts_dict.get(100, 0),
            'total_cells': int(width * height)
        }

        metadata: Dict[str, Any] = {
            'timestamp': iso_time,
            'timestamp_epoch': float(current_sec),
            'resolution': float(resolution),
            'origin_x': float(origin_x),
            'origin_y': float(origin_y),
            'width': int(width),
            'height': int(height),
            'cell_counts': class_summary,
            'encoding': {
                '-1': 'unknown',
                '10': 'flat',
                '50': 'rocky',
                '90': 'crater_interior',
                '100': 'obstacle'
            }
        }

        latest_npz_path = os.path.join(self.output_dir, 'latest.npz')
        np.savez_compressed(
            latest_npz_path,
            grid=terrain_grid.astype(np.int8),
            resolution=float(resolution),
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            width=int(width),
            height=int(height),
            timestamp=iso_time
        )

        latest_json_path = os.path.join(self.output_dir, 'latest_meta.json')
        with open(latest_json_path, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2)

        saved_files = {
            'latest_npz': latest_npz_path,
            'latest_json': latest_json_path
        }

        if self.keep_history:
            stamp_str = now.strftime('%Y%m%d_%H%M%S')
            history_npz_path = os.path.join(self.output_dir, f'terrain_{stamp_str}.npz')
            np.savez_compressed(
                history_npz_path,
                grid=terrain_grid.astype(np.int8),
                resolution=float(resolution),
                origin_x=float(origin_x),
                origin_y=float(origin_y),
                width=int(width),
                height=int(height),
                timestamp=iso_time
            )
            saved_files['history_npz'] = history_npz_path

        self.last_save_time = current_sec
        return saved_files
