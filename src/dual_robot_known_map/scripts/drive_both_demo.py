#!/usr/bin/env python3
"""Drive both robots on short, collision-free open-loop paths.

Spawn (from launch):
  robot1: (0.5, 0.5), yaw = 0     (+x / east)
  robot2: (-0.5, -0.5), yaw = π/2 (+y / north)

tb3_sandbox free lanes near spawn:
  - y ≈ 0.5 is clear along +x  → robot1 drives east
  - x ≈ -0.5 is clear along +y → robot2 drives north

They start ~1.4 m apart and move away from each other, so they do not
meet. Speeds stay low (~0.12 m/s) for ~9 s ≈ 1.1 m travel — well short
of the outer walls / pillar clusters.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DualDrive(Node):
    def __init__(self) -> None:
        super().__init__('dual_drive_demo')
        self._p1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self._p2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self._min1 = float('inf')
        self._min2 = float('inf')
        self.create_subscription(LaserScan, '/robot1/scan', self._scan1, 10)
        self.create_subscription(LaserScan, '/robot2/scan', self._scan2, 10)

    def _front_min(self, msg: LaserScan, half_width_rad: float = 0.45) -> float:
        """Minimum valid range in a forward cone."""
        n = len(msg.ranges)
        if n == 0:
            return float('inf')
        # indices covering [-half_width, +half_width] about angle 0
        mins = []
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if abs(ang) <= half_width_rad and math.isfinite(r) and r > 0.05:
                mins.append(r)
        return min(mins) if mins else float('inf')

    def _scan1(self, msg: LaserScan) -> None:
        self._min1 = self._front_min(msg)

    def _scan2(self, msg: LaserScan) -> None:
        self._min2 = self._front_min(msg)

    def _stop(self) -> None:
        stop = Twist()
        for _ in range(8):
            self._p1.publish(stop)
            self._p2.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def drive(self, duration_s: float = 9.0) -> None:
        # Warm up subscriptions / sim clock
        t_warm = time.time()
        while rclpy.ok() and (time.time() - t_warm) < 1.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        speed = 0.12  # m/s — ~1.1 m in 9 s
        clear_stop = 0.35  # emergency stop if obstacle ahead

        self.get_logger().info(
            f'Diverging paths: r1→+x, r2→+y at {speed:.2f} m/s for {duration_s:.1f}s'
        )
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < duration_s:
            rclpy.spin_once(self, timeout_sec=0.0)
            m1 = Twist()
            m2 = Twist()
            # Straight body-frame forward = east for r1, north for r2
            if self._min1 > clear_stop:
                m1.linear.x = speed
            else:
                self.get_logger().warn(f'robot1 front clear={self._min1:.2f}m — hold')
            if self._min2 > clear_stop:
                m2.linear.x = speed
            else:
                self.get_logger().warn(f'robot2 front clear={self._min2:.2f}m — hold')
            self._p1.publish(m1)
            self._p2.publish(m2)
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
