#!/usr/bin/env python3
"""
Phase 9: waypoint-following control law (pure Python, no rclpy).

`PathExecutorNode` is a thin ROS wrapper around this module, so the control law
can be unit-tested on any machine (see tests/test_path_following.py).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

DEFAULT_GOAL_TOLERANCE = 0.5
DEFAULT_WAYPOINT_TOLERANCE = 0.35
DEFAULT_MAX_LINEAR = 0.32
DEFAULT_MAX_ANGULAR = 0.9

ANGULAR_GAIN = 2.2           # rad/s of turn rate per rad of heading error
LINEAR_GAIN = 0.9            # m/s of forward speed per metre to the waypoint
SLOWDOWN_YAW_RAD = 1.2       # forward speed falls to zero at this heading error
TURN_IN_PLACE_YAW_RAD = 0.85  # beyond this heading error, creep forward only
TURN_IN_PLACE_LINEAR = 0.05


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about z) of a quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def compute_command(
    x: float,
    y: float,
    yaw: float,
    goal_x: float,
    goal_y: float,
    max_linear: float = DEFAULT_MAX_LINEAR,
    max_angular: float = DEFAULT_MAX_ANGULAR,
) -> Tuple[float, float]:
    """Proportional controller: (linear m/s, angular rad/s) towards one waypoint."""
    dx = goal_x - x
    dy = goal_y - y
    distance = math.hypot(dx, dy)
    yaw_error = wrap_angle(math.atan2(dy, dx) - yaw)

    angular = max(-max_angular, min(max_angular, ANGULAR_GAIN * yaw_error))

    speed_scale = max(0.0, 1.0 - abs(yaw_error) / SLOWDOWN_YAW_RAD)
    linear = max(0.0, min(max_linear, LINEAR_GAIN * distance * speed_scale))
    if abs(yaw_error) > TURN_IN_PLACE_YAW_RAD:
        linear = TURN_IN_PLACE_LINEAR

    return linear, angular


@dataclass
class StepResult:
    """One control tick: the command, plus what happened to the waypoint list."""

    linear: float = 0.0
    angular: float = 0.0
    reached_index: Optional[int] = None  # waypoint reached on this tick, if any
    finished: bool = False               # True on the tick the final waypoint is reached


class WaypointFollower:
    """Walks a list of (x, y) waypoints, one `step()` per control tick."""

    def __init__(
        self,
        waypoints: Sequence[Tuple[float, float]],
        goal_tolerance: float = DEFAULT_GOAL_TOLERANCE,
        waypoint_tolerance: float = DEFAULT_WAYPOINT_TOLERANCE,
        max_linear: float = DEFAULT_MAX_LINEAR,
        max_angular: float = DEFAULT_MAX_ANGULAR,
    ):
        if not waypoints:
            raise ValueError("WaypointFollower needs at least one waypoint.")
        self.waypoints: List[Tuple[float, float]] = [(float(px), float(py)) for px, py in waypoints]
        self.goal_tolerance = goal_tolerance
        self.waypoint_tolerance = waypoint_tolerance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.index = 0
        self.done = False

    def step(self, x: float, y: float, yaw: float) -> StepResult:
        """Advance one control tick from the rover's current pose."""
        if self.done:
            return StepResult()

        goal_x, goal_y = self.waypoints[self.index]
        is_final = self.index == len(self.waypoints) - 1
        tolerance = self.goal_tolerance if is_final else self.waypoint_tolerance

        if math.hypot(goal_x - x, goal_y - y) <= tolerance:
            reached = self.index
            self.index += 1
            self.done = self.index >= len(self.waypoints)
            return StepResult(reached_index=reached, finished=self.done)

        linear, angular = compute_command(
            x, y, yaw, goal_x, goal_y, self.max_linear, self.max_angular
        )
        return StepResult(linear=linear, angular=angular)
