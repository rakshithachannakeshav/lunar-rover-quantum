#!/usr/bin/env python3
"""
diff_drive_odometry.py
----------------------
Pure (no ROS) differential-drive dead-reckoning helpers, shared by
``encoder_processor.py`` and exercised directly by unit tests.

Given the arc length each wheel-set has travelled since the last update
(in metres), integrate the rover pose with a midpoint (2nd-order) update:

    d_center = (d_left + d_right) / 2
    d_theta  = (d_right - d_left) / wheel_separation
    theta_m  = theta + d_theta / 2
    x += d_center * cos(theta_m)
    y += d_center * sin(theta_m)
    theta = wrap(theta + d_theta)

Wheel *joint* angles are in radians; multiply a joint-angle delta by the
wheel radius to get the arc length this module expects.
"""

import math
from typing import Tuple


def wrap_angle(theta: float) -> float:
    """Wrap an angle to the half-open interval (-pi, pi]."""
    wrapped = math.atan2(math.sin(theta), math.cos(theta))
    # atan2 returns [-pi, pi]; normalise the -pi endpoint to +pi so the
    # interval is (-pi, pi] and round-trips are stable.
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def integrate_pose(
    x: float,
    y: float,
    theta: float,
    d_left_m: float,
    d_right_m: float,
    wheel_separation: float,
) -> Tuple[float, float, float]:
    """Advance pose (x, y, theta) by one differential-drive step.

    Args:
        x: Current world X (m).
        y: Current world Y (m).
        theta: Current heading (rad).
        d_left_m: Distance the left wheel-set travelled this step (m).
        d_right_m: Distance the right wheel-set travelled this step (m).
        wheel_separation: Track width between left and right wheels (m).

    Returns:
        Updated (x, y, theta) with theta wrapped to (-pi, pi].
    """
    if wheel_separation <= 0.0:
        raise ValueError('wheel_separation must be positive')

    d_center = 0.5 * (d_left_m + d_right_m)
    d_theta = (d_right_m - d_left_m) / wheel_separation
    theta_mid = theta + 0.5 * d_theta

    x_new = x + d_center * math.cos(theta_mid)
    y_new = y + d_center * math.sin(theta_mid)
    theta_new = wrap_angle(theta + d_theta)
    return x_new, y_new, theta_new


def yaw_to_quaternion(theta: float) -> Tuple[float, float, float, float]:
    """Convert a planar yaw angle (rad) to a quaternion (x, y, z, w)."""
    half = 0.5 * theta
    return 0.0, 0.0, math.sin(half), math.cos(half)
