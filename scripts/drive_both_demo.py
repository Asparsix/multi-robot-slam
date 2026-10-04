#!/usr/bin/env python3
"""Drive both robots on short, collision-free open-loop paths.

Spawn (from launch):
  robot1: (0.5, 0.5), yaw = 0     (+x / east)
  robot2: (-0.5, -0.5), yaw = π/2 (+y / north)

In tb3_sandbox the lane at y≈0.5 is clear along +x, and x≈-0.5 is clear
along +y. Robots start ~1.4 m apart and move away from each other.

Travel budget: 0.10 m/s × 7 s ≈ 0.7 m — stays between the near pillars.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DualDrive(Node):
    def __init__(self) -> None:
        super().__init__('dual_drive_demo')
        self._p1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self._p2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

    def _stop(self) -> None:
        stop = Twist()
        for _ in range(10):
            self._p1.publish(stop)
            self._p2.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def drive(self, duration_s: float = 9.0) -> None:
        # Let pubs connect
        t_warm = time.time()
        while rclpy.ok() and (time.time() - t_warm) < 0.8:
            self._p1.publish(Twist())
            self._p2.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

        # Wall-clock duration; with ~0.5–1.0 RTF this stays ~0.7–1.3 m.
        speed = 0.14
        self.get_logger().info(
            f'Diverging safe paths: r1→+x, r2→+y @ {speed:.2f} m/s for {duration_s:.1f}s'
        )
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < duration_s:
            m1 = Twist()
            m2 = Twist()
            m1.linear.x = speed
            m2.linear.x = speed
            self._p1.publish(m1)
            self._p2.publish(m2)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        self._stop()
        self.get_logger().info('Demo drive finished (stopped).')


def main() -> None:
    rclpy.init()
    node = DualDrive()
    try:
        node.drive(9.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
