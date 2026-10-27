#!/usr/bin/env python3
"""
tests/test_path_following.py
----------------------------
Pure-Python unit tests for navigation_pkg.path_following (Phase 9 control law).
These import the real module, so a change to the controller is actually caught.
No ROS 2 / Gazebo dependency.
"""

import math
import os
import sys

import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'navigation_pkg'))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from navigation_pkg.path_following import (  # noqa: E402
    DEFAULT_MAX_ANGULAR,
    DEFAULT_MAX_LINEAR,
    WaypointFollower,
    compute_command,
    wrap_angle,
    yaw_from_quaternion,
)


def test_wrap_angle_stays_in_range():
    assert wrap_angle(0.1 * math.pi - (-0.1 * math.pi)) == pytest.approx(0.2 * math.pi)
    assert wrap_angle(3.0 * math.pi / 2.0) == pytest.approx(-math.pi / 2.0)
    assert wrap_angle(-3.0 * math.pi / 2.0) == pytest.approx(math.pi / 2.0)


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    s = math.sin(math.pi / 4.0)
    assert yaw_from_quaternion(0.0, 0.0, s, s) == pytest.approx(math.pi / 2.0)
    assert yaw_from_quaternion(0.0, 0.0, 1.0, 0.0) == pytest.approx(math.pi)


def test_facing_the_goal_drives_forward_at_the_speed_limit():
    linear, angular = compute_command(0.0, 0.0, 0.0, 2.0, 0.0)
    assert linear == pytest.approx(DEFAULT_MAX_LINEAR)
    assert angular == pytest.approx(0.0)


def test_small_heading_error_gives_proportional_turn():
    # goal is 0.1 rad to the left of the current heading
    linear, angular = compute_command(0.0, 0.0, 0.0, 2.0 * math.cos(0.1), 2.0 * math.sin(0.1))
    assert angular == pytest.approx(2.2 * 0.1)
    assert 0.0 < linear <= DEFAULT_MAX_LINEAR


def test_speed_scales_down_with_distance_and_heading_error():
    # 0.6 rad of error -> speed_scale 0.5; 0.2 m away -> 0.9 * 0.2 * 0.5
    linear, _ = compute_command(0.0, 0.0, 0.0, 0.2 * math.cos(0.6), 0.2 * math.sin(0.6))
    assert linear == pytest.approx(0.9 * 0.2 * 0.5)


def test_large_heading_error_turns_in_place_at_the_angular_limit():
    linear, angular = compute_command(0.0, 0.0, 0.0, -2.0, 0.1)  # goal almost directly behind
    assert linear == pytest.approx(0.05)
    assert abs(angular) == pytest.approx(DEFAULT_MAX_ANGULAR)


def test_heading_error_wraps_the_short_way_round():
    # yaw just under +pi, goal direction just over -pi: the short turn is small and positive
    yaw = math.pi * 0.95
    goal = (math.cos(-math.pi * 0.95), math.sin(-math.pi * 0.95))
    _, angular = compute_command(0.0, 0.0, yaw, goal[0], goal[1])
    assert angular == pytest.approx(2.2 * 0.1 * math.pi)


def test_custom_speed_limits_are_respected():
    linear, angular = compute_command(0.0, 0.0, 0.0, 10.0, 0.0, max_linear=0.1, max_angular=0.3)
    assert linear == pytest.approx(0.1)
    linear, angular = compute_command(0.0, 0.0, 0.0, -1.0, 1.0, max_linear=0.1, max_angular=0.3)
    assert abs(angular) == pytest.approx(0.3)


def test_follower_rejects_an_empty_path():
    with pytest.raises(ValueError):
        WaypointFollower([])


def test_follower_advances_when_within_the_waypoint_tolerance():
    follower = WaypointFollower([(1.0, 0.0), (2.0, 0.0)], waypoint_tolerance=0.35)
    far = follower.step(0.0, 0.0, 0.0)
    assert far.reached_index is None and far.linear > 0.0

    hit = follower.step(0.9, 0.0, 0.0)
    assert hit.reached_index == 0 and not hit.finished
    assert (hit.linear, hit.angular) == (0.0, 0.0)
    assert follower.index == 1


def test_final_waypoint_uses_the_goal_tolerance():
    # 0.4 m away: outside the 0.35 waypoint tolerance, inside the 0.5 goal tolerance
    middle = WaypointFollower([(0.4, 0.0), (5.0, 0.0)], goal_tolerance=0.5, waypoint_tolerance=0.35)
    assert middle.step(0.0, 0.0, 0.0).reached_index is None

    final = WaypointFollower([(0.4, 0.0)], goal_tolerance=0.5, waypoint_tolerance=0.35)
    result = final.step(0.0, 0.0, 0.0)
    assert result.reached_index == 0 and result.finished and final.done


def test_follower_stops_after_the_final_waypoint():
    follower = WaypointFollower([(0.1, 0.0)])
    assert follower.step(0.0, 0.0, 0.0).finished
    after = follower.step(5.0, 5.0, 1.0)  # far away, but the run is over
    assert (after.linear, after.angular) == (0.0, 0.0)
    assert after.reached_index is None and not after.finished


def test_closed_loop_drive_reaches_every_waypoint():
    """Drive a unicycle with the controller and check it visits the whole path."""
    waypoints = [(1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0)]
    follower = WaypointFollower(waypoints)
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05

    reached = []
    for _ in range(6000):  # 300 s of simulated time
        result = follower.step(x, y, yaw)
        if result.reached_index is not None:
            reached.append(result.reached_index)
        x += result.linear * math.cos(yaw) * dt
        y += result.linear * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + result.angular * dt)
        if follower.done:
            break

    assert follower.done
    assert reached == [0, 1, 2, 3]
    assert math.hypot(x - waypoints[-1][0], y - waypoints[-1][1]) <= 0.5
