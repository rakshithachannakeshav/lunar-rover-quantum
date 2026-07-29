#!/usr/bin/env python3
"""Autonomous patrol along the lunar trail toward the goal marker."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

WAYPOINTS = [
    (-2.0, 0.0), (-1.0, 0.0), (0.0, 0.0), (1.0, 0.1), (2.0, 0.3),
    (3.0, 0.5), (4.0, 0.7), (5.0, 0.8), (6.0, 0.7), (7.0, 0.4), (8.0, 0.0),
]

GOAL_TOLERANCE = 0.5
MAX_LINEAR = 0.32
MAX_ANGULAR = 0.9
BOOTSTRAP_SEC = 4.0


class RoverPatrol(Node):
    def __init__(self):
        super().__init__('rover_patrol')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self._control_loop)
        self.start_time = self.get_clock().now()

        self.x = -2.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.wp_index = 0
        self.done = False
        self._logged_odom = False
        self._logged_fallback = False

        self.get_logger().info(
            'Patrol started — creeping forward until /odom, then following trail to (8,0).'
        )

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        if not self.have_odom:
            self.have_odom = True
            if not self._logged_odom:
                self.get_logger().info(
                    f'/odom received — pose ({self.x:.2f}, {self.y:.2f})'
                )
                self._logged_odom = True

    def _open_loop_twist(self) -> Twist:
        """Timed drive along trail when /odom is not available yet."""
        t = self._elapsed()
        twist = Twist()
        # Segment: creep forward, gentle right arc toward +X
        if t < 8.0:
            twist.linear.x = 0.22
            twist.angular.z = -0.08
        elif t < 16.0:
            twist.linear.x = 0.25
            twist.angular.z = 0.05
        elif t < 24.0:
            twist.linear.x = 0.2
            twist.angular.z = -0.12
        else:
            twist.linear.x = 0.15
        return twist

    def _closed_loop_twist(self) -> Twist:
        twist = Twist()
        if self.wp_index >= len(WAYPOINTS):
            self.done = True
            self.get_logger().info('Goal reached — patrol complete.')
            return twist

        gx, gy = WAYPOINTS[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)

        if dist < GOAL_TOLERANCE:
            self.wp_index += 1
            self.get_logger().info(
                f'Waypoint {self.wp_index}/{len(WAYPOINTS)} reached'
            )
            return twist

        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(
            math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))

        twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 2.2 * yaw_err))
        speed_scale = max(0.0, 1.0 - abs(yaw_err) / 1.2)
        twist.linear.x = max(0.0, min(MAX_LINEAR, 0.9 * dist * speed_scale))
        if abs(yaw_err) > 0.85:
            twist.linear.x = 0.05
        return twist

    def _control_loop(self):
        twist = Twist()

        if self.done:
            self.pub.publish(twist)
            return

        if self.have_odom:
            twist = self._closed_loop_twist()
        elif self._elapsed() < BOOTSTRAP_SEC:
            twist.linear.x = 0.18
            twist.angular.z = 0.0
        else:
            if not self._logged_fallback:
                self.get_logger().warn(
                    'No /odom yet — open-loop drive (check: ros2 topic echo /odom)'
                )
                self._logged_fallback = True
            twist = self._open_loop_twist()

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = RoverPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
