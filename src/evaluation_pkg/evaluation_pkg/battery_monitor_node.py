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
battery_monitor_node.py
-----------------------
Phase 10: a *simulated* battery. Gazebo has no battery, so this node integrates
the commanded velocities through evaluation_pkg.energy_model.BatteryModel and
publishes the result.

Subscribes:
  - /cmd_vel (geometry_msgs/msg/Twist): commanded motion driving the power model
  - /odom (nav_msgs/msg/Odometry): distance travelled

Publishes:
  - /battery/status (sensor_msgs/msg/BatteryState): percentage in [0, 1], voltage,
    current (negative = discharging), charge/capacity in Ah

Parameters: capacity_wh, voltage_v, idle_power_w, k_linear, k_angular,
publish_hz, cmd_timeout_s, cmd_vel_topic, odom_topic, status_topic.

The power model is an assumption, not a measurement - see energy_model.py.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from evaluation_pkg.energy_model import BatteryModel, BatteryParams, DistanceTracker


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        defaults = BatteryParams()
        self.declare_parameter('capacity_wh', defaults.capacity_wh)
        self.declare_parameter('voltage_v', defaults.voltage_v)
        self.declare_parameter('idle_power_w', defaults.idle_power_w)
        self.declare_parameter('k_linear', defaults.k_linear)
        self.declare_parameter('k_angular', defaults.k_angular)
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('cmd_timeout_s', 1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('status_topic', '/battery/status')

        def param(name):
            return self.get_parameter(name).value

        self.model = BatteryModel(BatteryParams(
            capacity_wh=float(param('capacity_wh')),
            voltage_v=float(param('voltage_v')),
            idle_power_w=float(param('idle_power_w')),
            k_linear=float(param('k_linear')),
            k_angular=float(param('k_angular')),
        ))
        self.cmd_timeout_s = float(param('cmd_timeout_s'))
        self.tracker = DistanceTracker()

        self.linear_v = 0.0
        self.angular_w = 0.0
        self.last_cmd_time = None
        self.last_step_time = self.get_clock().now()

        self.create_subscription(Twist, str(param('cmd_vel_topic')), self._on_cmd, 10)
        self.create_subscription(
            Odometry, str(param('odom_topic')), self._on_odom, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(BatteryState, str(param('status_topic')), 10)
        self.create_timer(1.0 / float(param('publish_hz')), self._tick)

        self.get_logger().info(
            f"Battery monitor started: {self.model.params.capacity_wh:.0f} Wh, "
            f"{self.model.params.voltage_v:.0f} V (simulated model). "
            f"Publishing {param('status_topic')}."
        )

    def _on_cmd(self, msg: Twist):
        self.linear_v = msg.linear.x
        self.angular_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _on_odom(self, msg: Odometry):
        previous = self.tracker.total_m
        total = self.tracker.update(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.model.add_distance(total - previous)

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_step_time).nanoseconds * 1e-9
        self.last_step_time = now

        # A stale command means the executor stopped publishing: count it as idle.
        stale = (
            self.last_cmd_time is None
            or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout_s
        )
        linear_v, angular_w = (0.0, 0.0) if stale else (self.linear_v, self.angular_w)
        self.model.step(linear_v, angular_w, dt)

        self.status_pub.publish(self._to_message(now))

    def _to_message(self, now) -> BatteryState:
        status = self.model.status()
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = float(status['voltage'])
        msg.current = -float(status['current'])  # negative = discharging
        msg.charge = float(status['charge_ah'])
        msg.capacity = float(status['capacity_ah'])  # no ageing modelled: full == design
        msg.design_capacity = float(status['capacity_ah'])
        msg.percentage = float(status['percentage'])
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
