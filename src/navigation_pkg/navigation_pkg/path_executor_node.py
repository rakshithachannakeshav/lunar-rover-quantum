#!/usr/bin/env python3
"""
Phase 9: Path Executor Node.

Subscribes to a path topic (/path/quantum or /path/classical), odometry (/odom)
and the lidar (/scan), and publishes velocity commands (/cmd_vel) to drive the lunar
rover along the sequence of waypoints using a smooth proportional waypoint-following
controller. When the lidar sees an obstacle ahead the rover turns away towards the
more open side, drives past it, then re-aims at the path.

The control law lives in navigation_pkg.path_following (pure Python, unit-tested);
this node only wires it to ROS.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from navigation_pkg.path_following import (
    DEFAULT_GOAL_TOLERANCE,
    DEFAULT_MAX_ANGULAR,
    DEFAULT_MAX_LINEAR,
    DEFAULT_BACKUP_DISTANCE,
    DEFAULT_CLEAR_DISTANCE,
    DEFAULT_COMMIT_DISTANCE,
    DEFAULT_REAR_CLEARANCE,
    DEFAULT_SIDE_CLEARANCE,
    DEFAULT_STOP_DISTANCE,
    DEFAULT_WAYPOINT_TOLERANCE,
    ObstacleAvoider,
    WaypointFollower,
    scan_sectors,
    yaw_from_quaternion,
)

# The planners publish the path once, RELIABLE + TRANSIENT_LOCAL, depth 1. The
# subscription must match (a volatile subscriber would miss a path published
# before this node came up).
PATH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class PathExecutorNode(Node):
    def __init__(self):
        super().__init__("path_executor")

        self.declare_parameter("path_topic", "/path/quantum")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_tolerance", DEFAULT_GOAL_TOLERANCE)
        self.declare_parameter("waypoint_tolerance", DEFAULT_WAYPOINT_TOLERANCE)
        self.declare_parameter("max_linear", DEFAULT_MAX_LINEAR)
        self.declare_parameter("max_angular", DEFAULT_MAX_ANGULAR)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("avoid_obstacles", True)
        self.declare_parameter("stop_distance", DEFAULT_STOP_DISTANCE)
        self.declare_parameter("clear_distance", DEFAULT_CLEAR_DISTANCE)
        self.declare_parameter("backup_distance", DEFAULT_BACKUP_DISTANCE)
        self.declare_parameter("commit_distance", DEFAULT_COMMIT_DISTANCE)
        self.declare_parameter("side_clearance", DEFAULT_SIDE_CLEARANCE)
        self.declare_parameter("rear_clearance", DEFAULT_REAR_CLEARANCE)
        # Give up (stop) after avoiding continuously for this long: boxed in.
        self.declare_parameter("avoid_timeout", 30.0)
        # A scan older than this is ignored (lidar dropped out).
        self.declare_parameter("scan_timeout", 0.5)

        self.path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        self.avoid_obstacles = bool(self.get_parameter("avoid_obstacles").value)

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self._on_path,
            PATH_QOS,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        self.timer = self.create_timer(0.05, self._control_loop)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.have_odom = False
        self.follower = None
        self.scan = None
        self.scan_time = None
        self.was_avoiding = False
        self.was_detouring = False
        self.avoiding_since = None

        self.get_logger().info(
            f"Path executor started. Listening on {self.path_topic}, {odom_topic} "
            f"and {scan_topic} (obstacle avoidance {'on' if self.avoid_obstacles else 'off'})..."
        )

    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received an empty path message.")
            return

        waypoints = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.follower = WaypointFollower(
            waypoints,
            goal_tolerance=float(self.get_parameter("goal_tolerance").value),
            waypoint_tolerance=float(self.get_parameter("waypoint_tolerance").value),
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            avoider=ObstacleAvoider(
                stop_distance=float(self.get_parameter("stop_distance").value),
                clear_distance=float(self.get_parameter("clear_distance").value),
                backup_distance=float(self.get_parameter("backup_distance").value),
                commit_distance=float(self.get_parameter("commit_distance").value),
                side_clearance=float(self.get_parameter("side_clearance").value),
                rear_clearance=float(self.get_parameter("rear_clearance").value),
                max_linear=float(self.get_parameter("max_linear").value),
                max_angular=float(self.get_parameter("max_angular").value),
            ),
        )

        self.avoiding_since = None
        self.was_detouring = False
        self.get_logger().info(
            f"Received new path on {self.path_topic} with {len(waypoints)} waypoints."
        )

    def _on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info(
                f"/odom received: rover initial pose = ({self.x:.2f}, {self.y:.2f})"
            )

    def _on_scan(self, msg: LaserScan):
        self.scan = scan_sectors(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max, msg.range_min
        )
        self.scan_time = self.get_clock().now()

    def _fresh_scan(self):
        if not self.avoid_obstacles or self.scan is None:
            return None
        age = (self.get_clock().now() - self.scan_time).nanoseconds * 1e-9
        if age > float(self.get_parameter("scan_timeout").value):
            return None
        return self.scan

    def _control_loop(self):
        twist = Twist()

        if not self.have_odom or self.follower is None or self.follower.done:
            self.cmd_pub.publish(twist)
            return

        scan = self._fresh_scan()
        result = self.follower.step(self.x, self.y, self.yaw, scan)

        if result.detouring and not self.was_detouring:
            side = "left" if self.follower.avoider.side == 1 else "right"
            self.get_logger().warn(
                f"Obstacle {scan.front:.2f} m ahead at ({self.x:.2f}, {self.y:.2f}); "
                f"turning {side} to go round it."
            )
        elif self.was_detouring and not result.detouring:
            self.get_logger().info(
                f"Clear of obstacle; resuming path at waypoint "
                f"{self.follower.index + 1}/{len(self.follower.waypoints)}."
            )
        self.was_detouring = result.detouring

        now = self.get_clock().now()
        if result.avoiding:
            if self.avoiding_since is None:
                self.avoiding_since = now
            stuck_for = (now - self.avoiding_since).nanoseconds * 1e-9
            if stuck_for > float(self.get_parameter("avoid_timeout").value):
                self.get_logger().error(
                    f"Could not get round the obstacle in {stuck_for:.0f} s at "
                    f"({self.x:.2f}, {self.y:.2f}); stopping. Send a new path to retry."
                )
                self.follower = None
                self.avoiding_since = None
                self.cmd_pub.publish(Twist())
                return
        else:
            self.avoiding_since = None
        self.was_avoiding = result.avoiding

        if result.skipped_index is not None:
            gx, gy = self.follower.waypoints[result.skipped_index]
            self.get_logger().warn(
                f"Waypoint {result.skipped_index + 1}/{len(self.follower.waypoints)} at "
                f"({gx:.2f}, {gy:.2f}) looks blocked after repeated detours; skipping it."
            )
        if result.stuck:
            self.get_logger().error(
                f"Goal unreachable after repeated detours at ({self.x:.2f}, {self.y:.2f}); "
                f"stopping. Send a new path to retry."
            )
            self.cmd_pub.publish(Twist())
            return

        if result.reached_index is not None:
            gx, gy = self.follower.waypoints[result.reached_index]
            self.get_logger().info(
                f"Waypoint {result.reached_index + 1}/{len(self.follower.waypoints)} "
                f"reached at ({gx:.2f}, {gy:.2f})"
            )
            if result.finished:
                self.get_logger().info("Final waypoint reached. Rover stopped.")

        twist.linear.x = result.linear
        twist.angular.z = result.angular
        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
