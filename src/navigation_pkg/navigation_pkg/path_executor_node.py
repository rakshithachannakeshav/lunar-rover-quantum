#!/usr/bin/env python3
"""
Phase 9: Path Executor Node.

Subscribes to a path topic (/path/quantum or /path/classical) and odometry (/odom),
and publishes velocity commands (/cmd_vel) to drive the lunar rover along the sequence
of waypoints using a smooth proportional waypoint-following controller.

The control law lives in navigation_pkg.path_following (pure Python, unit-tested);
this node only wires it to ROS.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
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
    DEFAULT_WAYPOINT_TOLERANCE,
    WaypointFollower,
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

        self.path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

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

        self.get_logger().info(
            f"Path executor started. Listening on {self.path_topic} and {odom_topic}..."
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
        )

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

    def _control_loop(self):
        twist = Twist()

        if not self.have_odom or self.follower is None or self.follower.done:
            self.cmd_pub.publish(twist)
            return

        result = self.follower.step(self.x, self.y, self.yaw)

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
