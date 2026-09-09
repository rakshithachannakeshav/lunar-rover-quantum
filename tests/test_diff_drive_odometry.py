#!/usr/bin/env python3
"""
tests/test_diff_drive_odometry.py
---------------------------------
Pure-Python unit tests for sensor_pkg differential-drive dead reckoning.
Executed by pytest in CI (no ROS2 / Gazebo dependency).
"""

import math
import os
import sys

# encoder_processor.py imports this module by bare name (flat ament_cmake
# scripts), so mirror that here: put its directory on the path and import flat.
PKG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'sensor_pkg', 'sensor_pkg')
)
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from diff_drive_odometry import (  # noqa: E402
    integrate_pose,
    wrap_angle,
    yaw_to_quaternion,
)


def test_straight_line_advances_x_only():
    """Equal wheel travel => move along heading, no rotation."""
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 1.0, 1.0, 0.5)
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(theta, 0.0, abs_tol=1e-9)


def test_in_place_spin_changes_theta_only():
    """Opposite, equal wheel travel => rotate in place."""
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, -0.25, 0.25, 0.5)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    # d_theta = (0.25 - (-0.25)) / 0.5 = 1.0 rad
    assert math.isclose(theta, 1.0, abs_tol=1e-9)


def test_single_midpoint_step_matches_hand_computation():
    """One asymmetric step lands on the closed-form midpoint result."""
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 1.0, 2.0, 1.0)
    # d_center = 1.5, d_theta = 1.0, theta_mid = 0.5
    assert math.isclose(x, 1.5 * math.cos(0.5), rel_tol=1e-12)
    assert math.isclose(y, 1.5 * math.sin(0.5), rel_tol=1e-12)
    assert math.isclose(theta, 1.0, abs_tol=1e-12)


def test_theta_wraps_into_half_open_interval():
    """Accumulated heading stays in (-pi, pi]."""
    _, _, theta = integrate_pose(0.0, 0.0, 3.0, -0.1, 0.1, 0.1)
    # raw theta = 3.0 + 2.0 = 5.0 -> 5.0 - 2*pi
    assert math.isclose(theta, 5.0 - 2.0 * math.pi, abs_tol=1e-9)
    assert -math.pi < theta <= math.pi


def test_quarter_circle_accumulation():
    """Many small steps around a radius-R left turn end near (R, R, pi/2)."""
    radius = 2.0
    wheel_sep = 0.84
    steps = 2000
    d_theta_step = (math.pi / 2.0) / steps
    d_center_step = radius * d_theta_step
    d_left = d_center_step - 0.5 * wheel_sep * d_theta_step
    d_right = d_center_step + 0.5 * wheel_sep * d_theta_step

    x, y, theta = 0.0, 0.0, 0.0
    for _ in range(steps):
        x, y, theta = integrate_pose(x, y, theta, d_left, d_right, wheel_sep)

    assert math.isclose(x, radius, abs_tol=1e-2)
    assert math.isclose(y, radius, abs_tol=1e-2)
    assert math.isclose(theta, math.pi / 2.0, abs_tol=1e-6)


def test_wrap_angle_endpoints():
    assert math.isclose(wrap_angle(0.0), 0.0, abs_tol=1e-12)
    assert math.isclose(wrap_angle(math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(wrap_angle(-math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-12)


def test_yaw_to_quaternion_known_values():
    qx, qy, qz, qw = yaw_to_quaternion(0.0)
    assert (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 1.0)

    qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2.0)
    assert math.isclose(qz, math.sin(math.pi / 4.0), abs_tol=1e-12)
    assert math.isclose(qw, math.cos(math.pi / 4.0), abs_tol=1e-12)

    qx, qy, qz, qw = yaw_to_quaternion(math.pi)
    assert math.isclose(qz, 1.0, abs_tol=1e-12)
    assert math.isclose(qw, 0.0, abs_tol=1e-12)
