#!/usr/bin/env python3
"""Drive four warehouse robots on short, diverging open-loop paths.

Spawn (from four_robots_collab_slam.launch.py):
  robot1: (-3, 0), yaw = 0     (+x)
  robot2: ( 3, 0), yaw = π     (-x)
  robot3: (-3, 5), yaw = 0     (+x)
  robot4: ( 3, 5), yaw = π     (-x)

They start ~6 m apart in x and move away from the center aisle,
so they do not meet. Slow speed keeps them clear of nearby shelves.
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

    def drive(self, duration_s: float = 9.0) -> None:
        t_warm = time.time()
        while rclpy.ok() and (time.time() - t_warm) < 0.8:
            for p in self._pubs.values():
                p.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

        speed = 0.12
        self.get_logger().info(
            f'Four-robot diverging drive @ {speed:.2f} m/s for {duration_s:.1f}s'
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
        node.drive(9.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
