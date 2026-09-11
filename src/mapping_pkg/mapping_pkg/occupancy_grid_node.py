#!/usr/bin/env python3

# Copyright 2026 Rakshitha Channakeshav
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Probabilistic 2D occupancy grid mapping node using log-odds updates."""

import math

from geometry_msgs.msg import TransformStamped
from mapping_pkg.grid_utils import (
    bresenham,
    is_in_bounds,
    log_odds_to_occupancy_grid,
    world_to_grid,
)
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


class OccupancyGridNode(Node):
    """ROS2 node that builds a 2D probabilistic occupancy grid map."""

    def __init__(self):
        super().__init__('occupancy_grid_node')

        # ── Declare Parameters ────────────────────────────────────────────────
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 30.0)
        self.declare_parameter('height_m', 30.0)
        self.declare_parameter('origin_x', -17.0)
        self.declare_parameter('origin_y', -15.0)
        self.declare_parameter('l_occ', 0.85)
        self.declare_parameter('l_free', -0.4)
        self.declare_parameter('l_min', -2.0)
        self.declare_parameter('l_max', 3.5)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('broadcast_static_tf', True)

        # ── Retrieve Parameters ───────────────────────────────────────────────
        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.l_occ = float(self.get_parameter('l_occ').value)
        self.l_free = float(self.get_parameter('l_free').value)
        self.l_min = float(self.get_parameter('l_min').value)
        self.l_max = float(self.get_parameter('l_max').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.broadcast_static_tf = bool(self.get_parameter('broadcast_static_tf').value)

        # Grid dimensions in cells
        self.width_cells = int(round(self.width_m / self.resolution))
        self.height_cells = int(round(self.height_m / self.resolution))

        # 2D Grid arrays: log_odds initialized to 0.0, observed_mask to False
        self.log_odds = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        self.observed_mask = np.zeros((self.height_cells, self.width_cells), dtype=bool)

        self.get_logger().info(
            f'Occupancy Grid initialized: {self.width_cells}x{self.height_cells} cells '
            f'({self.width_m}mx{self.height_m}m @ {self.resolution}m/cell), '
            f'origin=({self.origin_x}, {self.origin_y})'
        )

        # ── Static TF Broadcaster (map -> odom) ──────────────────────────────
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        if self.broadcast_static_tf:
            self._broadcast_map_to_odom_tf()

        # ── TF Buffer & Listener ──────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions & Publishers ────────────────────────────────────────
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        # ── Decoupled Timer for Publishing Grid ───────────────────────────────
        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.publish_timer = self.create_timer(timer_period, self.publish_map_callback)

        self.get_logger().info('✅ OccupancyGridNode ready — listening to /scan, publishing /map')

    def _broadcast_map_to_odom_tf(self):
        """Broadcast static identity transform from map to odom."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Broadcasting static identity TF: {self.map_frame} -> {self.odom_frame}'
        )

    def scan_callback(self, msg: LaserScan):
        """Process incoming LaserScan message and update log-odds grid."""
        source_frame = msg.header.frame_id if msg.header.frame_id else 'lidar_link'

        try:
            timeout_dur = Duration(seconds=0.05)
            if self.tf_buffer.can_transform(self.map_frame, source_frame,
                                            msg.header.stamp, timeout=timeout_dur):
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, msg.header.stamp
                )
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, rclpy.time.Time()
                )
        except Exception as ex:
            self.get_logger().debug(f'TF lookup {self.map_frame} -> {source_frame} failed: {ex}')
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)

        c0, r0 = world_to_grid(tx, ty, self.origin_x, self.origin_y, self.resolution)

        angle = msg.angle_min
        for distance in msg.ranges:
            # NaN / Inf / at-or-beyond range_max = the beam hit nothing. Still
            # trace it as a clearing ray to range_max (free space), just don't
            # place an obstacle at the end. Only a genuine below-min reading is
            # dropped as unreliable.
            no_return = (math.isnan(distance) or math.isinf(distance)
                         or distance >= msg.range_max)
            if not no_return and distance < msg.range_min:
                angle += msg.angle_increment
                continue

            is_max = no_return
            effective_dist = msg.range_max if is_max else distance

            px = effective_dist * math.cos(angle)
            py = effective_dist * math.sin(angle)

            ex = tx + r00 * px + r01 * py
            ey = ty + r10 * px + r11 * py

            c1, r1 = world_to_grid(ex, ey, self.origin_x, self.origin_y, self.resolution)
            cells = bresenham(c0, r0, c1, r1)

            for c, r in cells[:-1]:
                if is_in_bounds(c, r, self.width_cells, self.height_cells):
                    val = self.log_odds[r, c] + self.l_free
                    self.log_odds[r, c] = min(max(val, self.l_min), self.l_max)
                    self.observed_mask[r, c] = True

            if not is_max and len(cells) > 0:
                tc, tr = cells[-1]
                if is_in_bounds(tc, tr, self.width_cells, self.height_cells):
                    val = self.log_odds[tr, tc] + self.l_occ
                    self.log_odds[tr, tc] = min(max(val, self.l_min), self.l_max)
                    self.observed_mask[tr, tc] = True

            angle += msg.angle_increment

    def publish_map_callback(self):
        """Publish the current occupancy grid to /map."""
        now = self.get_clock().now().to_msg()

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = now
        grid_msg.header.frame_id = self.map_frame

        grid_msg.info.map_load_time = now
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width_cells
        grid_msg.info.height = self.height_cells
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_data = log_odds_to_occupancy_grid(self.log_odds, self.observed_mask)
        grid_msg.data = grid_data.flatten().tolist()

        self.map_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
