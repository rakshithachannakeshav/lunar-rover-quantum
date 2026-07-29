#!/usr/bin/env python3
"""Keyboard teleop for the lunar rover — publishes geometry_msgs/Twist on /cmd_vel.

Run in a real terminal (not embedded in launch without TTY):
  ros2 run rover_simulation rover_teleop.py
"""

import os
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
Lunar Rover Teleop
------------------
   u    i    o      forward / spin
   j    k    l      turn left / stop / turn right
   m    ,    .      backward

  w/s   increase / decrease linear speed
  a/d   increase / decrease angular speed
  q     quit
"""


class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear = 0.35
        self.angular = 0.9
        self.twist = Twist()
        self.get_logger().info('Rover teleop ready — focus this terminal and drive.')

    def publish(self):
        self.pub.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publish()


_tty_fd = None


def _open_tty_fd():
    """Return a file descriptor suitable for raw keyboard input."""
    global _tty_fd
    if sys.stdin.isatty():
        return sys.stdin.fileno()
    try:
        _tty_fd = os.open('/dev/tty', os.O_RDONLY)
        return _tty_fd
    except OSError:
        return None


def get_key():
    fd = _open_tty_fd()
    if fd is None:
        raise RuntimeError('no_tty')
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = os.read(fd, 1).decode('utf-8', errors='ignore')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    if _open_tty_fd() is None:
        print(
            '\nERROR: Keyboard teleop needs an interactive terminal.\n'
            'Do NOT start teleop from the launch file. Use a second terminal:\n\n'
            '  source ~/lunar-rover-quantum/install/setup.bash\n'
            '  ros2 run rover_simulation rover_teleop.py\n\n'
            'Or use autonomous patrol:\n'
            '  ros2 launch rover_simulation demo.launch.py mode:=patrol\n',
            file=sys.stderr,
        )
        sys.exit(1)

    rclpy.init()
    node = RoverTeleop()
    print(HELP)

    bindings = {
        'i': (1, 0),
        'o': (1, -1),
        'u': (1, 1),
        ',': (-1, 0),
        'm': (-1, -1),
        '.': (-1, 1),
        'j': (0, 1),
        'l': (0, -1),
        'k': (0, 0),
    }

    try:
        while rclpy.ok():
            try:
                key = get_key()
            except RuntimeError:
                node.get_logger().error('Lost keyboard TTY — exiting.')
                break
            if key == '\x03' or key == 'q':
                break
            if key == 'w':
                node.linear = min(node.linear + 0.05, 1.0)
                print(f'linear speed: {node.linear:.2f} m/s')
                continue
            if key == 's':
                node.linear = max(node.linear - 0.05, 0.05)
                print(f'linear speed: {node.linear:.2f} m/s')
                continue
            if key == 'a':
                node.angular = min(node.angular + 0.1, 2.0)
                print(f'angular speed: {node.angular:.2f} rad/s')
                continue
            if key == 'd':
                node.angular = max(node.angular - 0.1, 0.1)
                print(f'angular speed: {node.angular:.2f} rad/s')
                continue

            if key not in bindings:
                continue

            lin_sign, ang_sign = bindings[key]
            node.twist.linear.x = node.linear * lin_sign
            node.twist.angular.z = node.angular * ang_sign
            node.publish()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        if _tty_fd is not None:
            os.close(_tty_fd)


if __name__ == '__main__':
    main()
