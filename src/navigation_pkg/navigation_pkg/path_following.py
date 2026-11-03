#!/usr/bin/env python3
"""
Phase 9: waypoint-following control law (pure Python, no rclpy).

`PathExecutorNode` is a thin ROS wrapper around this module, so the control law
can be unit-tested on any machine (see tests/test_path_following.py).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

DEFAULT_GOAL_TOLERANCE = 0.15
DEFAULT_WAYPOINT_TOLERANCE = 0.35
DEFAULT_MAX_LINEAR = 0.32
DEFAULT_MAX_ANGULAR = 0.9

# Obstacle avoidance (lidar). Angles are in the lidar frame: 0 = straight ahead.
DEFAULT_STOP_DISTANCE = 0.7     # obstacle closer than this in front -> start avoiding
DEFAULT_CLEAR_DISTANCE = 1.1    # turn until the front is at least this clear
DEFAULT_BACKUP_DISTANCE = 0.35  # closer than this -> reverse first
DEFAULT_COMMIT_DISTANCE = 0.8   # drive this far past the obstacle before re-aiming
FRONT_HALF_ANGLE = 0.5          # rad either side of straight ahead
SIDE_MAX_ANGLE = 1.6            # side sectors span FRONT_HALF_ANGLE..SIDE_MAX_ANGLE
REAR_HALF_ANGLE = 0.5           # rear sector, checked before reversing
SELF_FILTER_RANGE = 0.2         # returns closer than this are the rover itself
BACKUP_LINEAR = -0.08
# Footprint from rover.urdf.xacro: chassis 1.0 x 0.7 m, panel 0.9 m wide, lidar 0.2 m
# ahead of centre. Measured from the lidar, the body reaches ~0.45 m sideways and
# ~0.7 m backwards.
DEFAULT_SIDE_CLEARANCE = 0.6    # veer away from anything closer than this on a flank
DEFAULT_REAR_CLEARANCE = 1.0    # only reverse if the rear is at least this clear
MAX_DETOURS_PER_WAYPOINT = 2    # then skip that waypoint: it is probably behind/inside an obstacle
MAX_DETOURS_TO_GOAL = 6         # then give up: the goal itself is unreachable
# Progress watchdog: driving this far without getting PROGRESS_STEP closer to the
# target means the rover is circling or sliding along an obstacle around it.
NO_PROGRESS_TRAVEL = 4.0
PROGRESS_STEP = 0.25

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
class ScanSectors:
    """A lidar scan reduced to what the avoider needs.

    `front` is the nearest return straight ahead; `left` / `right` are the mean
    clearance on each side, used to pick the more open way round an obstacle;
    `left_near` / `right_near` are the nearest return on each flank (side
    clearance) and `rear` the nearest return behind (checked before reversing).
    """

    front: float
    left: float
    right: float
    rear: float = math.inf
    left_near: float = math.inf
    right_near: float = math.inf


def scan_sectors(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_max: float,
    range_min: float = 0.0,
) -> ScanSectors:
    """Reduce a LaserScan's ranges to front / left / right sectors.

    inf, nan and out-of-range returns count as `range_max` (nothing seen);
    returns inside SELF_FILTER_RANGE are the rover's own body and are dropped.
    """
    floor = max(range_min, SELF_FILTER_RANGE)
    front = rear = range_max
    left: List[float] = []
    right: List[float] = []
    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or r < floor:
            continue
        r = min(float(r), range_max)
        angle = wrap_angle(angle_min + i * angle_increment)
        if abs(angle) <= FRONT_HALF_ANGLE:
            front = min(front, r)
        elif FRONT_HALF_ANGLE < angle <= SIDE_MAX_ANGLE:
            left.append(r)
        elif -SIDE_MAX_ANGLE <= angle < -FRONT_HALF_ANGLE:
            right.append(r)
        elif abs(angle) >= math.pi - REAR_HALF_ANGLE:
            rear = min(rear, r)
    return ScanSectors(
        front=front,
        left=sum(left) / len(left) if left else range_max,
        right=sum(right) / len(right) if right else range_max,
        rear=rear,
        left_near=min(left) if left else range_max,
        right_near=min(right) if right else range_max,
    )


class ObstacleAvoider:
    """Lidar reflex that overrides path following while something is in the way.

    CLEAR -> (front < stop) -> TURNING away towards the more open side
    TURNING -> (front > clear) -> BYPASSING straight ahead for commit_distance
    BYPASSING -> CLEAR, or back to TURNING if blocked again.
    BACKING (reverse) runs first when the obstacle is very close, but only while
    the rear is clear; otherwise the rover turns in place.
    Flank guard: whenever something is closer than side_clearance on one side,
    keep moving but veer away from it (the front cone alone misses the rover's
    sides clipping an obstacle while driving past it).

    The chosen side is kept until `reset_side()` (called when a waypoint is
    reached), so the rover does not dither left/right around one rock.
    """

    CLEAR, BACKING, TURNING, BYPASSING = "clear", "backing", "turning", "bypassing"

    def __init__(
        self,
        stop_distance: float = DEFAULT_STOP_DISTANCE,
        clear_distance: float = DEFAULT_CLEAR_DISTANCE,
        backup_distance: float = DEFAULT_BACKUP_DISTANCE,
        commit_distance: float = DEFAULT_COMMIT_DISTANCE,
        max_linear: float = DEFAULT_MAX_LINEAR,
        max_angular: float = DEFAULT_MAX_ANGULAR,
        side_clearance: float = DEFAULT_SIDE_CLEARANCE,
        rear_clearance: float = DEFAULT_REAR_CLEARANCE,
    ):
        self.side_clearance = side_clearance
        self.rear_clearance = rear_clearance
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.backup_distance = backup_distance
        self.commit_distance = commit_distance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.state = self.CLEAR
        self.side: Optional[int] = None  # +1 = turn left, -1 = turn right
        self._bypass_from: Tuple[float, float] = (0.0, 0.0)

    @property
    def active(self) -> bool:
        return self.state != self.CLEAR

    def reset_side(self) -> None:
        if not self.active:
            self.side = None

    def _flank_guard(self, scan: ScanSectors) -> Optional[Tuple[float, float]]:
        """Veer away from an obstacle too close on one side; None if both sides are clear."""
        if min(scan.left_near, scan.right_near) >= self.side_clearance:
            return None
        away = -1 if scan.left_near < scan.right_near else 1
        return (0.5 * self.max_linear, away * 0.7 * self.max_angular)

    def update(self, scan: ScanSectors, x: float, y: float) -> Tuple[Optional[Tuple[float, float]], bool]:
        """Returns (command or None to follow the path, detour_finished)."""
        if self.state == self.CLEAR:
            if scan.front >= self.stop_distance:
                return self._flank_guard(scan), False
            if self.side is None:
                self.side = 1 if scan.left >= scan.right else -1
            self.state = self.BACKING if scan.front < self.backup_distance else self.TURNING

        if self.state == self.BACKING:
            if scan.front < self.backup_distance + 0.1 and scan.rear > self.rear_clearance:
                return (BACKUP_LINEAR, 0.0), False
            self.state = self.TURNING  # backed off enough, or the rear is blocked: turn in place

        if self.state == self.TURNING:
            if scan.front < self.clear_distance:
                return (0.0, self.side * self.max_angular), False
            self.state = self.BYPASSING
            self._bypass_from = (x, y)

        # BYPASSING
        if scan.front < self.stop_distance:
            self.state = self.BACKING if scan.front < self.backup_distance else self.TURNING
            return self.update(scan, x, y)
        if math.hypot(x - self._bypass_from[0], y - self._bypass_from[1]) < self.commit_distance:
            return self._flank_guard(scan) or (0.6 * self.max_linear, 0.0), False
        self.state = self.CLEAR
        return None, True


@dataclass
class StepResult:
    """One control tick: the command, plus what happened to the waypoint list."""

    linear: float = 0.0
    angular: float = 0.0
    reached_index: Optional[int] = None  # waypoint reached on this tick, if any
    finished: bool = False               # True on the tick the final waypoint is reached
    avoiding: bool = False               # True while the obstacle avoider is in control
    detouring: bool = False              # True during a full detour (not just a flank veer)
    skipped_index: Optional[int] = None  # waypoint given up on after repeated detours
    stuck: bool = False                  # True on the tick the follower gives up on the goal


class WaypointFollower:
    """Walks a list of (x, y) waypoints, one `step()` per control tick."""

    def __init__(
        self,
        waypoints: Sequence[Tuple[float, float]],
        goal_tolerance: float = DEFAULT_GOAL_TOLERANCE,
        waypoint_tolerance: float = DEFAULT_WAYPOINT_TOLERANCE,
        max_linear: float = DEFAULT_MAX_LINEAR,
        max_angular: float = DEFAULT_MAX_ANGULAR,
        avoider: Optional[ObstacleAvoider] = None,
    ):
        if not waypoints:
            raise ValueError("WaypointFollower needs at least one waypoint.")
        self.waypoints: List[Tuple[float, float]] = [(float(px), float(py)) for px, py in waypoints]
        self.goal_tolerance = goal_tolerance
        self.waypoint_tolerance = waypoint_tolerance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.avoider = avoider if avoider is not None else ObstacleAvoider(
            max_linear=max_linear, max_angular=max_angular
        )
        self.index = 0
        self.done = False
        self.stuck = False
        self.detours = 0          # detours since the current target waypoint was set
        self._watch_index = -1
        self._best_dist = math.inf
        self._travel = 0.0
        self._last_xy: Optional[Tuple[float, float]] = None

    def _no_progress(self, x: float, y: float) -> bool:
        """Update the progress watchdog; True once the rover is going nowhere."""
        if self._last_xy is not None:
            self._travel += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        gx, gy = self.waypoints[self.index]
        d = math.hypot(gx - x, gy - y)
        if self.index != self._watch_index:
            self._watch_index, self._best_dist, self._travel = self.index, d, 0.0
        elif d < self._best_dist - PROGRESS_STEP:
            self._best_dist, self._travel = d, 0.0
        return self._travel > NO_PROGRESS_TRAVEL

    def _give_up_on_target(self) -> StepResult:
        """Skip an unreachable waypoint, or stop if it is the goal."""
        if self.index == len(self.waypoints) - 1:
            self.done = self.stuck = True
            return StepResult(stuck=True)
        skipped = self.index
        self.index += 1
        self.detours = 0
        self.avoider.reset_side()
        return StepResult(skipped_index=skipped)

    def _retarget_after_detour(self, x: float, y: float) -> None:
        """After going round an obstacle, aim at the nearest waypoint not yet reached.

        Waypoints the detour carried the rover past are skipped; the index never
        goes backwards, and the final waypoint is never skipped.
        """
        remaining = range(self.index, len(self.waypoints))
        self.index = min(
            remaining,
            key=lambda i: math.hypot(self.waypoints[i][0] - x, self.waypoints[i][1] - y),
        )

    def step(self, x: float, y: float, yaw: float, scan: Optional[ScanSectors] = None) -> StepResult:
        """Advance one control tick from the rover's current pose.

        With `scan`, the obstacle avoider can take over the command until the
        rover is clear; without it, this is plain waypoint following.
        """
        if self.done:
            return StepResult()

        if self._no_progress(x, y):
            return self._give_up_on_target()

        if scan is not None:
            command, detour_finished = self.avoider.update(scan, x, y)
            if command is not None:
                return StepResult(linear=command[0], angular=command[1], avoiding=True,
                                  detouring=self.avoider.active)
            if detour_finished:
                before = self.index
                self._retarget_after_detour(x, y)
                self.detours = 1 if self.index != before else self.detours + 1
                is_final = self.index == len(self.waypoints) - 1
                if self.detours >= (MAX_DETOURS_TO_GOAL if is_final else MAX_DETOURS_PER_WAYPOINT):
                    return self._give_up_on_target()

        goal_x, goal_y = self.waypoints[self.index]
        is_final = self.index == len(self.waypoints) - 1
        tolerance = self.goal_tolerance if is_final else self.waypoint_tolerance

        if math.hypot(goal_x - x, goal_y - y) <= tolerance:
            reached = self.index
            self.index += 1
            self.done = self.index >= len(self.waypoints)
            self.detours = 0
            self.avoider.reset_side()
            return StepResult(reached_index=reached, finished=self.done)

        linear, angular = compute_command(
            x, y, yaw, goal_x, goal_y, self.max_linear, self.max_angular
        )
        return StepResult(linear=linear, angular=angular)
