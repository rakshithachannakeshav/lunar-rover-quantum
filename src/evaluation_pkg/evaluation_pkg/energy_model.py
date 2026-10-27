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

"""
energy_model.py
---------------
Phase 10: a simple battery/energy model and an odometry distance tracker.
Pure Python - no rclpy - so it is unit-tested on any machine.

The model is an explicit *assumption*, not a hardware calibration:

    P = P_idle + k_linear * |v| + k_angular * |omega|      [W]
    E += P * dt                                            [J]

The defaults are placeholders for a small lunar-rover-class vehicle and are
meant to be replaced with measured values when the real robot (Phase 11) is
available.
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class BatteryParams:
    """Tunable parameters of the battery model."""

    capacity_wh: float = 40.0
    voltage_v: float = 24.0
    idle_power_w: float = 3.0
    k_linear: float = 15.0    # W per (m/s) of commanded forward speed
    k_angular: float = 4.0    # W per (rad/s) of commanded turn rate


class BatteryModel:
    """Integrates commanded motion into consumed energy and state of charge."""

    def __init__(self, params: Optional[BatteryParams] = None):
        self.params = params or BatteryParams()
        if self.params.capacity_wh <= 0.0:
            raise ValueError('capacity_wh must be positive.')
        if self.params.voltage_v <= 0.0:
            raise ValueError('voltage_v must be positive.')

        self.consumed_j = 0.0
        self.distance_m = 0.0
        self.elapsed_s = 0.0
        self.power_w = 0.0

    @property
    def capacity_j(self) -> float:
        return self.params.capacity_wh * 3600.0

    @property
    def soc(self) -> float:
        """State of charge in [0, 1]."""
        return max(0.0, 1.0 - self.consumed_j / self.capacity_j)

    def step(self, linear_v: float, angular_w: float, dt: float) -> float:
        """Advance the model by dt seconds at the given commanded velocities.

        Returns the energy (J) consumed during this step. Non-positive dt is
        ignored (returns 0.0).
        """
        if dt <= 0.0:
            return 0.0

        p = self.params
        self.power_w = (
            p.idle_power_w
            + p.k_linear * abs(linear_v)
            + p.k_angular * abs(angular_w)
        )
        energy = self.power_w * dt
        self.consumed_j += energy
        self.elapsed_s += dt
        return energy

    def add_distance(self, delta_m: float) -> None:
        """Accumulate distance travelled (from odometry)."""
        if delta_m > 0.0:
            self.distance_m += delta_m

    def status(self) -> Dict[str, Any]:
        """Snapshot in units convenient for sensor_msgs/BatteryState."""
        p = self.params
        capacity_ah = p.capacity_wh / p.voltage_v
        return {
            'percentage': self.soc,
            'voltage': p.voltage_v,
            'current': self.power_w / p.voltage_v,
            'capacity_ah': capacity_ah,
            'charge_ah': self.soc * capacity_ah,
            'consumed_j': self.consumed_j,
            'power_w': self.power_w,
            'distance_m': self.distance_m,
            'elapsed_s': self.elapsed_s,
        }


def consumed_joules(capacity_ah: float, charge_ah: float, voltage_v: float) -> float:
    """Energy drawn so far, recovered from a BatteryState's charge/capacity/voltage."""
    return max(0.0, capacity_ah - charge_ah) * voltage_v * 3600.0


class DistanceTracker:
    """Accumulates path length from a stream of (x, y) odometry positions."""

    def __init__(self):
        self.total_m = 0.0
        self._last = None

    def update(self, x: float, y: float) -> float:
        """Add a position sample and return the cumulative distance (m)."""
        if self._last is not None:
            self.total_m += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        return self.total_m

    def reset(self) -> None:
        self.total_m = 0.0
        self._last = None
