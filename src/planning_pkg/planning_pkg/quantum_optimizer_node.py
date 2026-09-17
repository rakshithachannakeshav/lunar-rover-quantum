#!/usr/bin/env python3
"""ROS 2 wrapper for the Phase-8 quantum optimizer."""

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from planning_pkg.classical_planning import (
    load_weighted_graph,
    resolve_node,
    run_astar,
)
from planning_pkg.quantum_optimizer import (
    compare_quantum_classical,
    run_qaoa,
    save_quantum_results,
)


class QuantumOptimizerNode(Node):
    def __init__(self):
        super().__init__("quantum_optimizer")

        self.declare_parameter(
            "graph_path",
            "results/graphs/latest.graphml",
        )
        self.declare_parameter("start_x", float("nan"))
        self.declare_parameter("start_y", float("nan"))
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("out_dir", "results/paths")
        self.declare_parameter("max_edges", 10)
        self.declare_parameter("reps", 1)
        self.declare_parameter("shots", 2048)
        self.declare_parameter("restarts", 4)
        self.declare_parameter("seed", 7)
        self.declare_parameter("save_results", True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.path_pub = self.create_publisher(
            Path,
            "/path/quantum",
            qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            10,
        )

        self.start_x = float("nan")
        self.start_y = float("nan")
        self.planning_started = False

        self.get_logger().info(
            "Quantum optimizer started. Waiting for /odom..."
        )

    def _on_odom(self, msg: Odometry):
        if self.planning_started:
            return

        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y

        self.get_logger().info(
            f"Received rover position from /odom: "
            f"({self.start_x:.3f}, {self.start_y:.3f})"
        )

        self.planning_started = True
        self._run()

    def _run(self):
        graph_path = str(
            self.get_parameter("graph_path").value
        )

        gx = float(
            self.get_parameter("goal_x").value
        )
        gy = float(
            self.get_parameter("goal_y").value
        )

        frame_id = str(
            self.get_parameter("frame_id").value
        )

        out_dir = str(
            self.get_parameter("out_dir").value
        )

        if not os.path.exists(graph_path):
            self.get_logger().error(
                f"Graph file does not exist: {graph_path}"
            )
            return

        if math.isnan(gx) or math.isnan(gy):
            self.get_logger().error(
                "goal_x and goal_y must be provided."
            )
            return

        try:
            graph = load_weighted_graph(graph_path)

            start = resolve_node(
                graph,
                self.start_x,
                self.start_y,
            )

            goal = resolve_node(
                graph,
                gx,
                gy,
            )

            self.get_logger().info(
                f"Planning from ({self.start_x:.2f}, "
                f"{self.start_y:.2f}) to ({gx:.2f}, {gy:.2f})"
            )

            classical = run_astar(
                graph,
                start,
                goal,
            )

            self.get_logger().info(
                f"A*: {classical.total_energy:.4f} energy / "
                f"{len(classical.path_nodes)} waypoints"
            )

            quantum = run_qaoa(
                graph,
                start,
                goal,
                classical.path_nodes,
                max_edges=int(
                    self.get_parameter("max_edges").value
                ),
                reps=int(
                    self.get_parameter("reps").value
                ),
                shots=int(
                    self.get_parameter("shots").value
                ),
                restarts=int(
                    self.get_parameter("restarts").value
                ),
                seed=int(
                    self.get_parameter("seed").value
                ),
            )

            comparison = compare_quantum_classical(
                classical,
                quantum,
            )

            if bool(
                self.get_parameter("save_results").value
            ):
                saved = save_quantum_results(
                    quantum,
                    comparison,
                    out_dir,
                )

                self.get_logger().info(
                    f"Saved quantum results to "
                    f"{saved['latest']}"
                )

            self.path_pub.publish(
                self._build_path(
                    quantum.path_coords,
                    frame_id,
                )
            )

            self.get_logger().info(
                f"QAOA energy={quantum.total_energy:.3f} | "
                f"classical A*={classical.total_energy:.3f} | "
                f"qubits={quantum.qubits} | "
                f"valid_samples={quantum.valid_samples} | "
                f"fallback={quantum.fallback_to_classical}"
            )

            self.get_logger().info(
                "Published quantum path on /path/quantum"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Quantum optimization failed: {exc}"
            )

    def _build_path(self, coords, frame_id):
        msg = Path()

        msg.header.frame_id = frame_id
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for x, y in coords:
            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = QuantumOptimizerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()