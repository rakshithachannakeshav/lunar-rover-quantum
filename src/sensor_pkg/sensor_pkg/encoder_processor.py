#!/usr/bin/env python3
"""
encoder_processor.py
--------------------
Wheel odometry for the lunar rover.

Gazebo Harmonic does not publish encoder ticks, so this node integrates
the wheel *joint angles* from ``/joint_states`` into a full 2D pose
(position + heading) using differential-drive kinematics and publishes
``nav_msgs/Odometry`` on ``/odom``.

Kinematics live in ``diff_drive_odometry`` and are unit-tested in
``tests/test_diff_drive_odometry.py``.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# sensor_pkg installs its nodes as flat scripts (ament_cmake), so this sibling
# module is imported by bare name — it lands in the same lib/sensor_pkg/ dir.
from diff_drive_odometry import integrate_pose, yaw_to_quaternion


class EncoderProcessor(Node):

    def __init__(self):
        super().__init__('encoder_processor')

        # Rover geometry (matches the DiffDrive plugin in lunar_terrain.world).
        self.declare_parameter('wheel_radius', 0.18)
        self.declare_parameter('wheel_separation', 0.84)
        self.declare_parameter(
            'left_joint_names',
            ['front_left_joint', 'mid_left_joint', 'rear_left_joint'],
        )
        self.declare_parameter(
            'right_joint_names',
            ['front_right_joint', 'mid_right_joint', 'rear_right_joint'],
        )

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.left_joint_names = list(self.get_parameter('left_joint_names').value)
        self.right_joint_names = list(self.get_parameter('right_joint_names').value)

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = None
        self.prev_right = None
        self.prev_stamp = None

        self.get_logger().info(
            '✅ Encoder Processor started — integrating /joint_states into /odom'
        )

    def _mean_position(self, msg, names):
        """Mean joint position (rad) over whichever of *names* are present."""
        vals = [msg.position[msg.name.index(n)] for n in names if n in msg.name]
        if not vals:
            return None
        return sum(vals) / len(vals)

    def joint_callback(self, msg):
        left_pos = self._mean_position(msg, self.left_joint_names)
        right_pos = self._mean_position(msg, self.right_joint_names)
        if left_pos is None or right_pos is None:
            return

        stamp = self.get_clock().now()

        if self.prev_left is None:
            self.prev_left = left_pos
            self.prev_right = right_pos
            self.prev_stamp = stamp
            return

        # Wheel-angle deltas (rad) -> arc length (m).
        d_left = (left_pos - self.prev_left) * self.wheel_radius
        d_right = (right_pos - self.prev_right) * self.wheel_radius
        self.prev_left = left_pos
        self.prev_right = right_pos

        self.x, self.y, self.theta = integrate_pose(
            self.x, self.y, self.theta, d_left, d_right, self.wheel_separation
        )

        dt = (stamp - self.prev_stamp).nanoseconds * 1e-9
        self.prev_stamp = stamp
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_separation

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        if dt > 0.0:
            odom.twist.twist.linear.x = d_center / dt
            odom.twist.twist.angular.z = d_theta / dt

        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = EncoderProcessor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
