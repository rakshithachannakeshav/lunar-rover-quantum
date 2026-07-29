#!/usr/bin/env python3
"""Hold W/A/S/D to drive — uses /dev/tty (no extra packages)."""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
Lunar Rover Keyboard
--------------------
  Hold W = forward   S = back   A = left   D = right
  Space = stop       Q = quit
"""


class RoverKeyboard(Node):
    def __init__(self):
        super().__init__('rover_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear = 0.35
        self.angular = 0.9
        self.held = set()
        self._last_key = {}
        self.running = True
        self.timer = self.create_timer(0.05, self._publish)
        self.get_logger().info('Keyboard ready — hold W/A/S/D in THIS terminal.')

    def _publish(self):
        now = time.time()
        for k in list(self.held):
            if now - self._last_key.get(k, 0) > 0.2:
                self.held.discard(k)
        twist = Twist()
        if 'w' in self.held:
            twist.linear.x = self.linear
        elif 's' in self.held:
            twist.linear.x = -self.linear
        if 'a' in self.held:
            twist.angular.z = self.angular
        elif 'd' in self.held:
            twist.angular.z = -self.angular
        self.pub.publish(twist)


def _open_tty():
    if sys.stdin.isatty():
        return sys.stdin.fileno(), None
    try:
        fd = os.open('/dev/tty', os.O_RDONLY)
        return fd, fd
    except OSError:
        return None, None


def _reader(node: RoverKeyboard, fd: int, extra_fd):
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while node.running and rclpy.ok():
            ready, _, _ = select.select([fd], [], [], 0.05)
            if not ready:
                continue
            ch = os.read(fd, 1).decode('utf-8', errors='ignore').lower()
            if ch == '\x03' or ch == 'q':
                node.running = False
                break
            if ch == ' ':
                node.held.clear()
                continue
            if ch in 'wasd':
                node.held.add(ch)
                node._last_key[ch] = time.time()
                if ch == 'w':
                    node.held.discard('s')
                elif ch == 's':
                    node.held.discard('w')
                elif ch == 'a':
                    node.held.discard('d')
                elif ch == 'd':
                    node.held.discard('a')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if extra_fd is not None:
            os.close(extra_fd)


def main():
    fd, extra = _open_tty()
    if fd is None:
        print(
            '\nERROR: No terminal for keyboard input.\n'
            'Run in a WSL terminal:\n'
            '  ros2 run rover_simulation rover_keyboard.py\n',
            file=sys.stderr,
        )
        sys.exit(1)

    print(HELP)
    rclpy.init()
    node = RoverKeyboard()
    thread = threading.Thread(target=_reader, args=(node, fd, extra), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        thread.join(timeout=1.0)
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
