#!/usr/bin/env python3
"""Publish a crossing/swap goal set for the 4-robot MAPF demo."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


GOALS = {
    'robot1': (2.5, 2.5, math.pi / 4),
    'robot2': (-2.5, 2.5, 3 * math.pi / 4),
    'robot3': (2.5, -2.5, -math.pi / 4),
    'robot4': (-2.5, -2.5, -3 * math.pi / 4),
}


class SwarmDemoGoals(Node):
    def __init__(self):
        super().__init__('swarm_demo_goals')
        self.declare_parameter('delay_sec', 35.0)
        self.delay = float(self.get_parameter('delay_sec').value)
        self._sent = False
        self._wall_t0 = time.monotonic()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pubs = {
            name: self.create_publisher(PoseStamped, f'/swarm/{name}/goal', latched)
            for name in GOALS
        }
        self._trigger = self.create_publisher(Bool, '/swarm/demo_goals_sent', 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Will publish swap goals after {self.delay:.1f}s (wall time)')

    def _tick(self):
        if self._sent:
            return
        if time.monotonic() - self._wall_t0 < self.delay:
            return
        stamp = self.get_clock().now().to_msg()
        for name, (x, y, yaw) in GOALS.items():
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            self._pubs[name].publish(msg)
            self.get_logger().info(f'Published goal {name} -> ({x}, {y})')
        flag = Bool()
        flag.data = True
        self._trigger.publish(flag)
        self._sent = True


def main():
    rclpy.init()
    node = SwarmDemoGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
