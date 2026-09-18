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
evaluator_node.py
-----------------
Phase 10: compares what the rover *planned* with what it actually spent driving.

Loads the planned path of one planner from the comparison file written by
`python -m evaluation_pkg.metrics`, then follows odometry and the simulated
battery until the rover reaches that path's final waypoint.

Subscribes:
  - /odom (nav_msgs/msg/Odometry)
  - /battery/status (sensor_msgs/msg/BatteryState)

Publishes:
  - /metrics (std_msgs/msg/String): JSON execution report, 1 Hz

On arrival it writes results/energy_comparison/execution_<planner>_<timestamp>.json
and latest_execution.json (once per run).

Parameters: planner (dijkstra | astar | qaoa), comparison_path, out_dir,
goal_tolerance, odom_topic, battery_topic, metrics_topic.
"""

import json
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from evaluation_pkg.energy_model import consumed_joules
from evaluation_pkg.metrics import ExecutionMonitor, PlannerMetrics


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator')

        self.declare_parameter('planner', 'astar')
        self.declare_parameter('comparison_path', 'results/energy_comparison/latest_comparison.json')
        self.declare_parameter('out_dir', 'results/energy_comparison')
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('battery_topic', '/battery/status')
        self.declare_parameter('metrics_topic', '/metrics')

        def param(name):
            return self.get_parameter(name).value

        planner = str(param('planner'))
        comparison_path = str(param('comparison_path'))
        self.out_dir = str(param('out_dir'))

        self.monitor = self._load_monitor(planner, comparison_path, float(param('goal_tolerance')))
        self.written = False

        self.create_subscription(
            Odometry, str(param('odom_topic')), self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(BatteryState, str(param('battery_topic')), self._on_battery, 10)
        self.metrics_pub = self.create_publisher(String, str(param('metrics_topic')), 10)
        self.create_timer(1.0, self._publish_metrics)

        self.get_logger().info(
            f"Evaluating '{planner}' path from {comparison_path}: goal "
            f"({self.monitor.goal[0]:.2f}, {self.monitor.goal[1]:.2f}), "
            f"planned energy {self.monitor.planned.energy:.3f}."
        )

    def _load_monitor(self, planner, comparison_path, goal_tolerance):
        """Fail loudly at start-up: an evaluator with nothing to compare against is useless."""
        if not os.path.exists(comparison_path):
            raise FileNotFoundError(
                f"Comparison file '{comparison_path}' not found. Run "
                "`python -m evaluation_pkg.metrics` first (see README, Phase 10)."
            )
        with open(comparison_path, 'r', encoding='utf-8') as f:
            comparison = json.load(f)

        planners = comparison.get('planners', {})
        if planner not in planners:
            raise KeyError(
                f"Planner '{planner}' not in {comparison_path} (available: {sorted(planners)})."
            )
        return ExecutionMonitor(PlannerMetrics(**planners[planner]), goal_tolerance)

    def _on_odom(self, msg: Odometry):
        arrived = self.monitor.update_position(
            msg.pose.pose.position.x, msg.pose.pose.position.y
        )
        if arrived and not self.written:
            self._write_result()

    def _on_battery(self, msg: BatteryState):
        self.monitor.update_battery(
            consumed_joules(msg.capacity, msg.charge, msg.voltage), msg.percentage
        )

    def _publish_metrics(self):
        msg = String()
        msg.data = json.dumps(self.monitor.report())
        self.metrics_pub.publish(msg)

    def _write_result(self):
        report = self.monitor.report()
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.out_dir, f"execution_{report['planner']}_{stamp}.json")
        for target in (path, os.path.join(self.out_dir, 'latest_execution.json')):
            with open(target, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2)
        self.written = True
        self.get_logger().info(
            f"Goal reached: drove {report['actual_distance_m']:.2f} m "
            f"(planned {report['planned_distance_m']:.2f} m), "
            f"used {report['consumed_j']:.1f} J. Report saved to {path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
