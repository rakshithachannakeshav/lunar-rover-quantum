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

"""
classical_planner_node.py
-------------------------
ROS 2 wrapper node for classical path planning (Dijkstra & A*).
Consumes results/graphs/latest.graphml and publishes:
  - /path/classical (nav_msgs/msg/Path): Canonical A* path for Phase 9 path_executor
  - /path/classical/dijkstra (nav_msgs/msg/Path): Debug topic for side-by-side RViz inspection
"""

import math
import os
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from planning_pkg.classical_planning import (
    PlanResult,
    compare,
    load_weighted_graph,
    resolve_node,
    run_astar,
    run_dijkstra,
    save_plan_results,
)


class ClassicalPlannerNode(Node):
    """ROS 2 node that loads the energy-weighted graph and publishes classical paths."""

    def __init__(self):
        super().__init__('classical_planner')

        # ── Declare Parameters ────────────────────────────────────────────────
        self.declare_parameter('graph_path', 'results/graphs/latest.graphml')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('out_dir', 'results/paths')
        self.declare_parameter('save_results', True)

        # ── Retrieve Parameters ───────────────────────────────────────────────
        self.graph_path = str(self.get_parameter('graph_path').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.out_dir = str(self.get_parameter('out_dir').value)
        self.save_results = bool(self.get_parameter('save_results').value)

        # ── QoS Configuration ─────────────────────────────────────────────────
        # Transient local durability ensures late subscribers (like RViz or path_executor)
        # receive the path message immediately upon connection.
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        # Canonical topic for Phase 9 path execution (publishes A* result)
        self.path_pub = self.create_publisher(Path, '/path/classical', path_qos)
        # Debug topic for side-by-side visualization in RViz2
        self.dijkstra_debug_pub = self.create_publisher(Path, '/path/classical/dijkstra', path_qos)

        self.get_logger().info("Classical planner node initialized.")

        # Plan immediately on startup
        # TODO(Phase 9): trigger on demand via service / action / goal pose topic
        self._plan_and_publish()

    def _plan_and_publish(self):
        """Execute Dijkstra & A* planning, log comparison, and publish paths."""
        # 1. Validate goal parameters
        if math.isnan(self.goal_x) or math.isnan(self.goal_y):
            self.get_logger().error(
                "Goal coordinates not set! Please provide 'goal_x' and 'goal_y' parameters. "
                "Planning aborted."
            )
            return

        # 2. Check graph file existence
        if not os.path.exists(self.graph_path):
            self.get_logger().error(
                f"Graph file '{self.graph_path}' does not exist! "
                "Make sure Phase 6 graph_model has generated the graphml file."
            )
            return

        # 3. Load graph
        try:
            graph = load_weighted_graph(self.graph_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load graph from {self.graph_path}: {e}")
            return

        # 4. Resolve start and goal coordinates to graph nodes
        try:
            start_node = resolve_node(graph, self.start_x, self.start_y)
            goal_node = resolve_node(graph, self.goal_x, self.goal_y)
        except Exception as e:
            self.get_logger().error(f"Failed to resolve start/goal coordinates: {e}")
            return

        self.get_logger().info(
            f"Planning from ({self.start_x:.2f}, {self.start_y:.2f}) [node {start_node}] "
            f"to ({self.goal_x:.2f}, {self.goal_y:.2f}) [node {goal_node}]"
        )

        # 5. Run both algorithms
        try:
            dijkstra_res = run_dijkstra(graph, start_node, goal_node)
            astar_res = run_astar(graph, start_node, goal_node)
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            return

        # 6. Compare and log one-line human-readable summary
        comp = compare(dijkstra_res, astar_res)
        self.get_logger().info(
            f"Dijkstra: {dijkstra_res.total_energy:.2f} energy / {dijkstra_res.num_waypoints} waypoints / {dijkstra_res.runtime_ms:.2f}ms | "
            f"A*: {astar_res.total_energy:.2f} energy / {astar_res.num_waypoints} waypoints / {astar_res.runtime_ms:.2f}ms"
        )

        # 7. Persist results for Phase 10 evaluation
        if self.save_results:
            try:
                saved = save_plan_results(dijkstra_res, astar_res, comp, out_dir=self.out_dir)
                self.get_logger().info(f"Saved classical plan results to {saved['latest']}")
            except Exception as e:
                self.get_logger().warn(f"Could not persist plan results: {e}")

        # 8. Build and publish nav_msgs/msg/Path messages
        # A* is the canonical/cheaper choice for execution
        astar_msg = self._build_path_msg(astar_res)
        dijkstra_msg = self._build_path_msg(dijkstra_res)

        self.path_pub.publish(astar_msg)
        self.dijkstra_debug_pub.publish(dijkstra_msg)

        self.get_logger().info("Published canonical path on /path/classical and debug on /path/classical/dijkstra")

    def _build_path_msg(self, plan: PlanResult) -> Path:
        """Construct nav_msgs/msg/Path from a PlanResult."""
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in plan.path_coords:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orientation: Identity quaternion (w=1.0)
            # Note: Heading calculation between consecutive waypoints is deferred to
            # the Phase 9 path_executor controller.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = ClassicalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
