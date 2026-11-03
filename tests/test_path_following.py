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
    DEFAULT_GOAL_TOLERANCE,
    DEFAULT_MAX_ANGULAR,
    DEFAULT_MAX_LINEAR,
    ObstacleAvoider,
    ScanSectors,
    WaypointFollower,
    compute_command,
    scan_sectors,
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


# --- obstacle avoidance -----------------------------------------------------

N_BEAMS = 360
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_BEAMS
RANGE_MAX = 12.0


def _scan_with(obstacles):
    """Ranges for a 360-beam lidar: obstacles is a list of (angle, range) to place."""
    ranges = [float('inf')] * N_BEAMS
    for angle, r in obstacles:
        i = int(round((angle - ANGLE_MIN) / ANGLE_INC)) % N_BEAMS
        ranges[i] = r
    return ranges


def test_default_goal_tolerance_is_tight():
    # 0.5 m used to stop the rover half a metre short of the goal.
    assert DEFAULT_GOAL_TOLERANCE <= 0.2


def test_scan_sectors_reads_front_and_sides():
    ranges = _scan_with([(0.0, 0.6), (1.0, 0.8), (-1.0, 3.0)])
    s = scan_sectors(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MAX)
    assert s.front == pytest.approx(0.6)
    assert s.left < s.right  # left side has the close return


def test_scan_sectors_ignores_inf_nan_and_self_hits():
    ranges = _scan_with([(0.0, 0.1), (0.1, float('nan'))])
    s = scan_sectors(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MAX, range_min=0.12)
    assert s.front == pytest.approx(RANGE_MAX)


def test_avoider_ignores_distant_obstacles():
    avoider = ObstacleAvoider()
    command, done = avoider.update(ScanSectors(front=3.0, left=5.0, right=5.0), 0.0, 0.0)
    assert command is None and not done and not avoider.active


def test_avoider_turns_towards_the_more_open_side():
    left_open = ObstacleAvoider()
    (lin, ang), _ = left_open.update(ScanSectors(front=0.5, left=4.0, right=1.0), 0.0, 0.0)
    assert lin == pytest.approx(0.0) and ang > 0.0

    right_open = ObstacleAvoider()
    (lin, ang), _ = right_open.update(ScanSectors(front=0.5, left=1.0, right=4.0), 0.0, 0.0)
    assert lin == pytest.approx(0.0) and ang < 0.0


def test_avoider_backs_up_when_very_close():
    avoider = ObstacleAvoider()
    (lin, ang), _ = avoider.update(ScanSectors(front=0.25, left=2.0, right=2.0), 0.0, 0.0)
    assert lin < 0.0 and ang == pytest.approx(0.0)


def test_avoider_bypasses_then_hands_back():
    avoider = ObstacleAvoider(commit_distance=0.5)
    avoider.update(ScanSectors(front=0.5, left=4.0, right=1.0), 0.0, 0.0)
    (lin, ang), _ = avoider.update(ScanSectors(front=5.0, left=4.0, right=1.0), 0.0, 0.0)
    assert lin > 0.0 and ang == pytest.approx(0.0)  # driving straight past it
    command, done = avoider.update(ScanSectors(front=5.0, left=4.0, right=1.0), 0.6, 0.0)
    assert command is None and done and not avoider.active


def test_follower_without_scan_behaves_as_before():
    follower = WaypointFollower([(2.0, 0.0)])
    result = follower.step(0.0, 0.0, 0.0)
    assert not result.avoiding and result.linear > 0.0


def _simulated_scan(x, y, yaw, rocks):
    """Ray-cast a 360-beam lidar against circular rocks [(cx, cy, radius)]."""
    ranges = []
    for i in range(N_BEAMS):
        a = yaw + ANGLE_MIN + i * ANGLE_INC
        dx, dy = math.cos(a), math.sin(a)
        best = float('inf')
        for cx, cy, rad in rocks:
            fx, fy = x - cx, y - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - rad * rad
            disc = b * b - c
            if disc >= 0.0:
                t = -b - math.sqrt(disc)
                if t > 0.0:
                    best = min(best, t)
        ranges.append(best)
    return scan_sectors(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MAX)


def test_closed_loop_drive_goes_round_a_rock_to_the_exact_goal():
    """A rock sits on the straight line to the goal: the rover must not hit it,
    must go round, and must finish within the goal tolerance of the endpoint."""
    rock = (2.0, 0.05, 0.4)
    goal = (4.0, 0.0)
    follower = WaypointFollower([(1.0, 0.0), (2.0, 0.0), (3.0, 0.0), goal])
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05
    closest = float('inf')
    avoided = False

    for _ in range(12000):  # 600 s of simulated time
        scan = _simulated_scan(x, y, yaw, [rock])
        result = follower.step(x, y, yaw, scan)
        avoided |= result.avoiding
        x += result.linear * math.cos(yaw) * dt
        y += result.linear * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + result.angular * dt)
        closest = min(closest, math.hypot(x - rock[0], y - rock[1]) - rock[2])
        if follower.done:
            break

    assert follower.done
    assert avoided
    assert closest > 0.45  # rover half-width (panel 0.9 m): never touched the rock
    assert math.hypot(x - goal[0], y - goal[1]) <= DEFAULT_GOAL_TOLERANCE


def _drive(rocks, waypoints, seconds=800.0):
    follower = WaypointFollower(waypoints)
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05
    closest, skipped = float('inf'), []
    for _ in range(int(seconds / dt)):
        result = follower.step(x, y, yaw, _simulated_scan(x, y, yaw, rocks))
        if result.skipped_index is not None:
            skipped.append(result.skipped_index)
        x += result.linear * math.cos(yaw) * dt
        y += result.linear * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + result.angular * dt)
        closest = min([closest] + [math.hypot(x - cx, y - cy) - r for cx, cy, r in rocks])
        if follower.done:
            break
    return follower, (x, y), closest, skipped


def test_flank_guard_veers_away_from_a_close_side():
    avoider = ObstacleAvoider()
    command, _ = avoider.update(ScanSectors(front=5.0, left=3.0, right=3.0, left_near=0.4), 0.0, 0.0)
    assert command[0] > 0.0 and command[1] < 0.0   # keep going, veer right
    assert not avoider.active                       # a veer, not a full detour


def test_avoider_does_not_reverse_into_something_behind():
    avoider = ObstacleAvoider()
    (lin, ang), _ = avoider.update(ScanSectors(front=0.25, left=2.0, right=2.0, rear=0.5), 0.0, 0.0)
    assert lin == pytest.approx(0.0) and ang != 0.0  # turns in place instead


def test_scan_sectors_reports_rear_and_nearest_flank():
    ranges = _scan_with([(math.pi, 0.9), (1.0, 0.7), (1.2, 2.0)])
    s = scan_sectors(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MAX)
    assert s.rear == pytest.approx(0.9)
    assert s.left_near == pytest.approx(0.7)


def test_waypoint_behind_a_wall_is_skipped_and_the_goal_reached():
    wall = [(3.0, k / 10.0, 0.1) for k in range(-8, 9, 2)]   # 1.7 m wall over waypoint (3, 0)
    follower, (x, y), closest, skipped = _drive(wall, [(1.0 * i, 0.0) for i in range(1, 7)])
    assert follower.done and not follower.stuck
    assert skipped == [2]
    assert math.hypot(x - 6.0, y) <= DEFAULT_GOAL_TOLERANCE
    assert closest > 0.45


def test_goal_inside_an_obstacle_gives_up_instead_of_circling():
    rock = [(6.0, 0.0, 0.5)]
    follower, _, closest, _ = _drive(rock, [(1.0 * i, 0.0) for i in range(1, 7)])
    assert follower.done and follower.stuck
    assert closest > 0.45
