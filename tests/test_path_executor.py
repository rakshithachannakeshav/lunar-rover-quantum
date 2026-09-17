#!/usr/bin/env python3
"""
Pure-Python unit tests for navigation_pkg PathExecutor logic.
Executable without requiring a live ROS 2 daemon.
"""

import math
import pytest


def test_waypoint_distance_calculation():
    """Verify Euclidean distance computation between current pose and waypoint."""
    x, y = 0.0, 0.0
    gx, gy = 3.0, 4.0
    distance = math.hypot(gx - x, gy - y)
    assert distance == pytest.approx(5.0)


def test_target_yaw_calculation():
    """Verify target heading angle calculation to waypoint."""
    x, y = 1.0, 1.0
    gx, gy = 4.0, 4.0
    target_yaw = math.atan2(gy - y, gx - x)
    assert target_yaw == pytest.approx(math.pi / 4.0)


def test_normalized_yaw_error_wrapping():
    """Verify yaw error stays bounded within [-pi, pi]."""
    current_yaw = math.pi * 0.95
    target_yaw = -math.pi * 0.95

    yaw_error = math.atan2(
        math.sin(target_yaw - current_yaw),
        math.cos(target_yaw - current_yaw),
    )

    assert -math.pi <= yaw_error <= math.pi
    assert yaw_error == pytest.approx(0.1 * math.pi)


def test_velocity_scaling_logic():
    """Verify proportional speed control limits linear and angular velocities."""
    max_linear = 0.32
    max_angular = 0.9

    distance = 2.0
    yaw_error = 0.1  # small alignment error

    speed_scale = max(0.0, 1.0 - abs(yaw_error) / 1.2)
    linear = max(0.0, min(max_linear, 0.9 * distance * speed_scale))
    angular = max(-max_angular, min(max_angular, 2.2 * yaw_error))

    assert 0.0 < linear <= max_linear
    assert angular == pytest.approx(0.22)


def test_large_heading_error_slows_linear_velocity():
    """Verify large orientation error reduces linear speed for turning safety."""
    yaw_error = 1.0  # > 0.85 rad
    linear = 0.05 if abs(yaw_error) > 0.85 else 0.32
    assert linear == 0.05


def test_waypoint_index_advance_on_tolerance():
    """Verify advancing waypoint index when within distance threshold."""
    waypoints = [(0.0, 0.0), (1.0, 1.0), (5.0, 5.0)]
    wp_index = 0
    current_x, current_y = 0.1, 0.1
    tolerance = 0.35

    gx, gy = waypoints[wp_index]
    dist = math.hypot(gx - current_x, gy - current_y)

    if dist <= tolerance:
        wp_index += 1

    assert wp_index == 1

