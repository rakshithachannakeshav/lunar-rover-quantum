#!/usr/bin/env python3
"""Always publishes forward velocity — use to verify diff-drive works."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class RoverCreep(Node):
    def __init__(self):
        super().__init__('rover_creep')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._ticks = 0
        self._settle_ticks = 50  # 5 s at 10 Hz — let rover rest on pad
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('Creep: 5 s settle, then linear.x=0.15')

    def _tick(self):
        self._ticks += 1
        twist = Twist()
        if self._ticks > self._settle_ticks:
            twist.linear.x = 0.15
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = RoverCreep()
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
