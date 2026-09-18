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


"""Terrain classification node for rover navigation cost mapping."""

import math
from typing import Optional

import cv2
from geometry_msgs.msg import Point
from mapping_pkg.grid_utils import (
    grid_to_world,
    is_in_bounds,
    world_to_grid,
)
from mapping_pkg.terrain_storage import TerrainStorage
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class TerrainClassifierNode(Node):
    """ROS2 node that classifies 2D grid cells into terrain types."""

    def __init__(self):
        super().__init__('terrain_classifier_node')

        # ── Declare Parameters ────────────────────────────────────────────────
        self.declare_parameter('roughness_window', 5)
        self.declare_parameter('roughness_threshold', 0.02)
        self.declare_parameter('crater_min_radius_px', 8)
        self.declare_parameter('crater_max_radius_px', 80)
        self.declare_parameter('hough_dp', 1.2)
        self.declare_parameter('hough_param1', 50.0)
        self.declare_parameter('hough_param2', 20.0)
        self.declare_parameter('hough_min_dist', 15.0)
        self.declare_parameter('viz_downsample', 3)
        self.declare_parameter('classification_rate', 1.0)
        self.declare_parameter('save_interval_sec', 10.0)
        self.declare_parameter('keep_history', False)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('map_frame', 'map')

        # ── Retrieve Parameters ───────────────────────────────────────────────
        self.roughness_window = int(self.get_parameter('roughness_window').value)
        self.roughness_threshold = float(self.get_parameter('roughness_threshold').value)
        self.crater_min_radius_px = int(self.get_parameter('crater_min_radius_px').value)
        self.crater_max_radius_px = int(self.get_parameter('crater_max_radius_px').value)
        self.hough_dp = float(self.get_parameter('hough_dp').value)
        self.hough_param1 = float(self.get_parameter('hough_param1').value)
        self.hough_param2 = float(self.get_parameter('hough_param2').value)
        self.hough_min_dist = float(self.get_parameter('hough_min_dist').value)
        self.viz_downsample = max(1, int(self.get_parameter('viz_downsample').value))
        self.classification_rate = float(self.get_parameter('classification_rate').value)
        self.save_interval_sec = float(self.get_parameter('save_interval_sec').value)
        self.keep_history = bool(self.get_parameter('keep_history').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.map_frame = str(self.get_parameter('map_frame').value)

        # ── Cached Inputs ─────────────────────────────────────────────────────
        self.latest_map_msg: Optional[OccupancyGrid] = None
        self.latest_scan_msg: Optional[LaserScan] = None

        # ── TF Buffer & Listener ──────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Storage Persistence Handler ───────────────────────────────────────
        self.storage = TerrainStorage(
            output_dir=self.output_dir,
            save_interval_sec=self.save_interval_sec,
            keep_history=self.keep_history
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

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

        # ── Publishers ────────────────────────────────────────────────────────
        terrain_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.terrain_map_pub = self.create_publisher(
            OccupancyGrid,
            '/terrain_map',
            terrain_qos
        )

        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/terrain_map_markers',
            10
        )

        # ── Periodic Execution Timer ──────────────────────────────────────────
        timer_period = 1.0 / max(self.classification_rate, 0.1)
        self.classify_timer = self.create_timer(timer_period, self.process_and_publish)

        self.get_logger().info('✅ TerrainClassifierNode initialized.')

    def map_callback(self, msg: OccupancyGrid):
        """Cache latest occupancy grid."""
        self.latest_map_msg = msg

    def scan_callback(self, msg: LaserScan):
        """Cache latest LiDAR scan."""
        self.latest_scan_msg = msg

    def process_and_publish(self):
        """Execute periodic terrain classification and publish results."""
        if self.latest_map_msg is None:
            return

        map_msg = self.latest_map_msg
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        if width == 0 or height == 0:
            return

        raw_map = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # ── Step 1: Base classification ───────────────────────────────────────
        terrain_grid = np.full((height, width), -1, dtype=np.int8)

        observed_free = (raw_map >= 0) & (raw_map < 50)
        terrain_grid[observed_free] = 10

        occupied = raw_map >= 50
        terrain_grid[occupied] = 100

        # ── Step 2: Roughness Analysis (Rocky Detection) ───────────────────────
        if self.latest_scan_msg is not None:
            self._apply_roughness_classification(
                terrain_grid, self.latest_scan_msg, origin_x, origin_y,
                resolution, width, height
            )

        # ── Step 3: Hough Circle Crater Detection ─────────────────────────────
        self._apply_crater_classification(terrain_grid, raw_map, width, height)

        # ── Step 4: Publish /terrain_map OccupancyGrid ─────────────────────────
        now = self.get_clock().now().to_msg()
        terrain_msg = OccupancyGrid()
        terrain_msg.header.stamp = now
        target_frame = map_msg.header.frame_id if map_msg.header.frame_id else self.map_frame
        terrain_msg.header.frame_id = target_frame
        terrain_msg.info = map_msg.info
        terrain_msg.data = terrain_grid.flatten().tolist()
        self.terrain_map_pub.publish(terrain_msg)

        # ── Step 5: Publish /terrain_map_markers MarkerArray ───────────────────
        self._publish_markers(
            terrain_grid, origin_x, origin_y, resolution, width, height,
            terrain_msg.header.frame_id
        )

        # ── Step 6: Periodic Persistence to Disk ──────────────────────────────
        current_time_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.storage.should_save(current_time_sec):
            saved = self.storage.save(
                terrain_grid=terrain_grid,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
                width=width,
                height=height,
                timestamp_sec=current_time_sec
            )
            self.get_logger().info(f'Persisted terrain map to {saved["latest_npz"]}')

    def _apply_roughness_classification(
        self,
        terrain_grid: np.ndarray,
        scan_msg: LaserScan,
        origin_x: float,
        origin_y: float,
        resolution: float,
        width: int,
        height: int
    ):
        """Compute rolling range variance and mark rocky cells."""
        source_frame = scan_msg.header.frame_id if scan_msg.header.frame_id else 'lidar_link'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, rclpy.time.Time()
            )
        except Exception:
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

        ranges = scan_msg.ranges
        num_ranges = len(ranges)
        half_w = self.roughness_window // 2

        for i in range(num_ranges):
            dist = ranges[i]
            if (math.isnan(dist) or math.isinf(dist) or
                    dist < scan_msg.range_min or dist >= scan_msg.range_max):
                continue

            window = [
                ranges[k] for k in range(max(0, i - half_w), min(num_ranges, i + half_w + 1))
                if not (math.isnan(ranges[k]) or math.isinf(ranges[k]) or
                        ranges[k] < scan_msg.range_min or ranges[k] >= scan_msg.range_max)
            ]

            if len(window) >= 3:
                var = float(np.var(window))
                if var >= self.roughness_threshold:
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    px = dist * math.cos(angle)
                    py = dist * math.sin(angle)

                    ex = tx + r00 * px + r01 * py
                    ey = ty + r10 * px + r11 * py

                    c, r = world_to_grid(ex, ey, origin_x, origin_y, resolution)
                    for dc, dr in [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nc, nr = c + dc, r + dr
                        if is_in_bounds(nc, nr, width, height):
                            if terrain_grid[nr, nc] == 10:
                                terrain_grid[nr, nc] = 50

    def _apply_crater_classification(
        self,
        terrain_grid: np.ndarray,
        raw_map: np.ndarray,
        width: int,
        height: int
    ):
        """Detect circular crater rims via Hough Circle Transform and mark interior."""
        obstacle_mask = np.zeros((height, width), dtype=np.uint8)
        obstacle_mask[raw_map >= 50] = 255

        blurred = cv2.GaussianBlur(obstacle_mask, (5, 5), 1.5)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.crater_min_radius_px,
            maxRadius=self.crater_max_radius_px
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])

                r_inner = int(r * 0.85)
                min_c = max(0, cx - r_inner)
                max_c = min(width - 1, cx + r_inner)
                min_r = max(0, cy - r_inner)
                max_r = min(height - 1, cy + r_inner)

                for row in range(min_r, max_r + 1):
                    for col in range(min_c, max_c + 1):
                        dx = col - cx
                        dy = row - cy
                        if dx * dx + dy * dy <= r_inner * r_inner:
                            if terrain_grid[row, col] in (10, 50):
                                terrain_grid[row, col] = 90

    def _publish_markers(
        self,
        terrain_grid: np.ndarray,
        origin_x: float,
        origin_y: float,
        resolution: float,
        width: int,
        height: int,
        frame_id: str
    ):
        """Publish downsampled MarkerArray for color-coded RViz visualization."""
        marker_array = MarkerArray()
        step = self.viz_downsample
        cell_size = resolution * float(step)

        color_flat = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.6)
        color_rocky = ColorRGBA(r=0.9, g=0.6, b=0.1, a=0.7)
        color_crater = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.8)

        marker_flat = Marker()
        marker_flat.header.frame_id = frame_id
        marker_flat.header.stamp = self.get_clock().now().to_msg()
        marker_flat.ns = 'terrain_flat'
        marker_flat.id = 1
        marker_flat.type = Marker.CUBE_LIST
        marker_flat.action = Marker.ADD
        marker_flat.scale.x = cell_size
        marker_flat.scale.y = cell_size
        marker_flat.scale.z = 0.02
        marker_flat.color = color_flat

        marker_rocky = Marker()
        marker_rocky.header.frame_id = frame_id
        marker_rocky.header.stamp = self.get_clock().now().to_msg()
        marker_rocky.ns = 'terrain_rocky'
        marker_rocky.id = 2
        marker_rocky.type = Marker.CUBE_LIST
        marker_rocky.action = Marker.ADD
        marker_rocky.scale.x = cell_size
        marker_rocky.scale.y = cell_size
        marker_rocky.scale.z = 0.04
        marker_rocky.color = color_rocky

        marker_crater = Marker()
        marker_crater.header.frame_id = frame_id
        marker_crater.header.stamp = self.get_clock().now().to_msg()
        marker_crater.ns = 'terrain_crater'
        marker_crater.id = 3
        marker_crater.type = Marker.CUBE_LIST
        marker_crater.action = Marker.ADD
        marker_crater.scale.x = cell_size
        marker_crater.scale.y = cell_size
        marker_crater.scale.z = 0.06
        marker_crater.color = color_crater

        for row in range(0, height, step):
            for col in range(0, width, step):
                val = terrain_grid[row, col]
                if val in (10, 50, 90):
                    wx, wy = grid_to_world(col, row, origin_x, origin_y, resolution)
                    pt = Point(x=wx, y=wy, z=0.02)
                    if val == 10:
                        marker_flat.points.append(pt)
                    elif val == 50:
                        marker_rocky.points.append(pt)
                    elif val == 90:
                        marker_crater.points.append(pt)

        marker_array.markers.extend([marker_flat, marker_rocky, marker_crater])
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
