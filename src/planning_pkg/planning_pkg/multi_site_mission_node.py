#!/usr/bin/env python3
"""ROS 2 wrapper for the hybrid multi-site mission planner.

Takes the depot from the first /odom message, plans the visiting order of the
`sites` parameter (QAOA by default) with A* legs on the slope/shadow cost map,
and publishes the stitched route once as nav_msgs/Path (RELIABLE +
TRANSIENT_LOCAL, matching navigation_pkg.path_executor_node).
"""

import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from planning_pkg.cost_map import CostMapConfig, CostWeights
from planning_pkg.mission_planner import load_cost_map, plan_mission, save_mission_plan
from planning_pkg.tsp_qaoa import QaoaConfig


class MultiSiteMissionNode(Node):
    def __init__(self):
        super().__init__("multi_site_mission")

        self.declare_parameter("heightmap_path", "src/rover_simulation/worlds/heightmap.png")
        # Flat list [x1, y1, x2, y2, ...] of science sites in the map frame.
        # Default: 12 sites spread over the 150 m map (~1 km closed tour).
        self.declare_parameter("sites", [35.0, 30.0, -30.0, 40.0, 45.0, -25.0, -40.0, -35.0,
                                         10.0, 55.0, -55.0, 5.0, 55.0, 10.0, 0.0, -50.0,
                                         20.0, -15.0, -15.0, 20.0, -20.0, -15.0, 25.0, 25.0])
        self.declare_parameter("solver", "qaoa")
        self.declare_parameter("return_to_depot", True)
        self.declare_parameter("resolution", 1.0)
        self.declare_parameter("w_dist", 1.0)
        self.declare_parameter("w_slope", 2.0)
        self.declare_parameter("w_shadow", 1.5)
        self.declare_parameter("w_uphill", 2.0)
        self.declare_parameter("reps", 1)
        self.declare_parameter("optimizer", "COBYLA")
        self.declare_parameter("shots", 4096)
        self.declare_parameter("max_qubits", 20)
        self.declare_parameter("window", 3)
        self.declare_parameter("seed", 7)
        self.declare_parameter("path_topic", "/path/mission")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("out_dir", "results/missions")
        self.declare_parameter("save_results", True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        path_topic = str(self.get_parameter("path_topic").value)
        self.path_pub = self.create_publisher(Path, path_topic, qos)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.planning_started = False

        self.get_logger().info(
            f"Multi-site mission planner started (publishing {path_topic}). Waiting for /odom..."
        )

    def _param(self, name):
        return self.get_parameter(name).value

    def _on_odom(self, msg: Odometry):
        if self.planning_started:
            return
        self.planning_started = True
        depot = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Depot from /odom: ({depot[0]:.2f}, {depot[1]:.2f})")
        self._run(depot)

    def _run(self, depot):
        heightmap = str(self._param("heightmap_path"))
        flat = [float(v) for v in self._param("sites")]
        if not os.path.exists(heightmap):
            self.get_logger().error(f"Heightmap does not exist: {heightmap}")
            return
        if len(flat) < 2 or len(flat) % 2:
            self.get_logger().error("'sites' must be a flat [x1, y1, x2, y2, ...] list.")
            return
        sites = list(zip(flat[0::2], flat[1::2]))

        config = CostMapConfig(
            resolution_m=float(self._param("resolution")),
            weights=CostWeights(float(self._param("w_dist")),
                                float(self._param("w_slope")),
                                float(self._param("w_shadow")),
                                float(self._param("w_uphill"))),
        )
        qaoa = QaoaConfig(
            reps=int(self._param("reps")),
            optimizer=str(self._param("optimizer")),
            shots=int(self._param("shots")),
            max_qubits=int(self._param("max_qubits")),
            seed=int(self._param("seed")),
        )
        try:
            cost_map, graph = load_cost_map(heightmap, config=config)
            plan = plan_mission(
                graph, cost_map, depot, sites,
                solver=str(self._param("solver")),
                closed=bool(self._param("return_to_depot")),
                qaoa=qaoa,
                window=int(self._param("window")),
            )
        except Exception as exc:
            self.get_logger().error(f"Mission planning failed: {exc}")
            return

        if bool(self._param("save_results")):
            saved = save_mission_plan(plan, str(self._param("out_dir")))
            self.get_logger().info(f"Saved mission plan to {saved['latest']}")

        self.path_pub.publish(self._build_path(plan.path_coords, str(self._param("frame_id"))))
        info = plan.solver_info
        self.get_logger().info(
            f"{plan.solver} ({info.get('mode', '-')}) order={plan.order} "
            f"cost={plan.tour_cost:.2f} waypoints={len(plan.path_coords)} | "
            f"qubits={info.get('qubits', '-')} p_feasible={info.get('p_feasible', '-')} "
            f"repaired={info.get('repaired', info.get('repaired_windows', '-'))}"
        )

    def _build_path(self, coords, frame_id):
        msg = Path()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in coords:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MultiSiteMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
