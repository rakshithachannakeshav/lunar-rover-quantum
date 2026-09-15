import math

import pytest


pytestmark = pytest.mark.ros2


def test_waypoint_distance():
    x, y = 0.0, 0.0
    gx, gy = 3.0, 4.0

    distance = math.hypot(gx - x, gy - y)

    assert distance == pytest.approx(5.0)


def test_target_yaw():
    x, y = 0.0, 0.0
    gx, gy = 1.0, 1.0

    target_yaw = math.atan2(gy - y, gx - x)

    assert target_yaw == pytest.approx(math.pi / 4)


def test_normalized_yaw_error():
    current_yaw = math.pi * 1.9
    target_yaw = -math.pi * 1.9

    yaw_error = math.atan2(
        math.sin(target_yaw - current_yaw),
        math.cos(target_yaw - current_yaw),
    )

    assert -math.pi <= yaw_error <= math.pi


def test_waypoint_tolerance():
    distance = math.hypot(0.2, 0.2)
    tolerance = 0.35

    assert distance < tolerance


def test_max_velocity_limits():
    max_linear = 0.32
    max_angular = 0.9

    requested_linear = 2.0
    requested_angular = 5.0

    linear = max(0.0, min(max_linear, requested_linear))
    angular = max(-max_angular, min(max_angular, requested_angular))

    assert linear == pytest.approx(0.32)
    assert angular == pytest.approx(0.9)