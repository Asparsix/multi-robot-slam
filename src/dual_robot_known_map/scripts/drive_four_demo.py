#!/usr/bin/env python3
"""Drive four house robots on short, diverging open-loop paths.

Spawn (from four_robots_collab_slam.launch.py) — one per room:
  robot1: (-3, -3), yaw = 0     (+x toward doorway)
  robot2: ( 3, -3), yaw = π     (-x toward doorway)
  robot3: (-3,  3), yaw = 0     (+x toward doorway)
  robot4: ( 3,  3), yaw = π     (-x toward doorway)

Short travel (~0.4 m) keeps them inside their rooms / near doorways.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


ROBOTS = ('robot1', 'robot2', 'robot3', 'robot4')


class FourDrive(Node):
    def __init__(self) -> None:
        super().__init__('four_drive_demo')
        self._pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in ROBOTS
        }

    def _stop(self) -> None:
        stop = Twist()
        for _ in range(10):
            for p in self._pubs.values():
                p.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def drive(self, duration_s: float = 4.0) -> None:
        t_warm = time.time()
        while rclpy.ok() and (time.time() - t_warm) < 0.8:
            for p in self._pubs.values():
                p.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

        speed = 0.10
        self.get_logger().info(
            f'Four-robot house drive @ {speed:.2f} m/s for {duration_s:.1f}s'
        )
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < duration_s:
            msg = Twist()
            msg.linear.x = speed
            for p in self._pubs.values():
                p.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        self._stop()
        self.get_logger().info('Four-robot demo drive finished (stopped).')


def main() -> None:
    rclpy.init()
    node = FourDrive()
    try:
        node.drive(4.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
