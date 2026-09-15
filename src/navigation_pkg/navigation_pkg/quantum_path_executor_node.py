#!/usr/bin/env python3
"""Follow the Phase-8 QAOA path using rover odometry."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


GOAL_TOLERANCE = 0.5
WAYPOINT_TOLERANCE = 0.35
MAX_LINEAR = 0.32
MAX_ANGULAR = 0.9


class QuantumPathExecutor(Node):
    def __init__(self):
        super().__init__("quantum_path_executor")

        self.declare_parameter("path_topic", "/path/quantum")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_tolerance", GOAL_TOLERANCE)
        self.declare_parameter("waypoint_tolerance", WAYPOINT_TOLERANCE)
        self.declare_parameter("max_linear", MAX_LINEAR)
        self.declare_parameter("max_angular", MAX_ANGULAR)

        path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self._on_path,
            10,
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
        self.path_received = False
        self.waypoints = []
        self.wp_index = 0
        self.done = False

        self.get_logger().info(
            "Quantum path executor started. "
            f"Waiting for {path_topic}."
        )

    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received an empty quantum path.")
            return

        self.waypoints = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.wp_index = 0
        self.done = False
        self.path_received = True

        self.get_logger().info(
            f"Received quantum path with {len(self.waypoints)} waypoints."
        )

    def _on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(siny, cosy)

        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info(
                f"/odom received: pose=({self.x:.2f}, {self.y:.2f})"
            )

    def _control_loop(self):
        twist = Twist()

        if not self.have_odom:
            self.cmd_pub.publish(twist)
            return

        if not self.path_received:
            self.cmd_pub.publish(twist)
            return

        if self.done:
            self.cmd_pub.publish(twist)
            return

        if self.wp_index >= len(self.waypoints):
            self.done = True
            self.get_logger().info("Quantum path complete. Rover stopped.")
            self.cmd_pub.publish(twist)
            return

        gx, gy = self.waypoints[self.wp_index]

        dx = gx - self.x
        dy = gy - self.y
        distance = math.hypot(dx, dy)

        tolerance = float(
            self.get_parameter("goal_tolerance").value
            if self.wp_index == len(self.waypoints) - 1
            else self.get_parameter("waypoint_tolerance").value
        )

        if distance <= tolerance:
            self.get_logger().info(
                f"Waypoint {self.wp_index + 1}/{len(self.waypoints)} reached "
                f"at ({gx:.2f}, {gy:.2f})"
            )

            self.wp_index += 1

            if self.wp_index >= len(self.waypoints):
                self.done = True
                self.get_logger().info(
                    "Final quantum waypoint reached. Rover stopped."
                )

            self.cmd_pub.publish(twist)
            return

        target_yaw = math.atan2(dy, dx)

        yaw_error = math.atan2(
            math.sin(target_yaw - self.yaw),
            math.cos(target_yaw - self.yaw),
        )

        max_angular = float(
            self.get_parameter("max_angular").value
        )
        max_linear = float(
            self.get_parameter("max_linear").value
        )

        twist.angular.z = max(
            -max_angular,
            min(max_angular, 2.2 * yaw_error),
        )

        speed_scale = max(
            0.0,
            1.0 - abs(yaw_error) / 1.2,
        )

        twist.linear.x = max(
            0.0,
            min(max_linear, 0.9 * distance * speed_scale),
        )

        if abs(yaw_error) > 0.85:
            twist.linear.x = 0.05

        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = QuantumPathExecutor()

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