#!/usr/bin/env python3
"""Publish crossing/swap goal sets for the 4-robot MAPF demo.

Sends an initial swap after delay_sec (wall time). If repeat=True, toggles
between swap goals and return-to-spawn goals each time /swarm/makespan_complete
is received so the demo keeps moving.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


# Corner swap — inset from walls so goals sit in free (non-inflated) space
GOALS_SWAP = {
    'robot1': (2.0, 2.0, math.pi / 4),
    'robot2': (-2.0, 2.0, 3 * math.pi / 4),
    'robot3': (2.0, -2.0, -math.pi / 4),
    'robot4': (-2.0, -2.0, -3 * math.pi / 4),
}

# Near spawn corners (return leg)
GOALS_HOME = {
    'robot1': (-2.0, -2.0, 0.0),
    'robot2': (2.0, -2.0, math.pi),
    'robot3': (-2.0, 2.0, 0.0),
    'robot4': (2.0, 2.0, math.pi),
}


class SwarmDemoGoals(Node):
    def __init__(self):
        super().__init__('swarm_demo_goals')
        self.declare_parameter('delay_sec', 35.0)
        self.declare_parameter('repeat', True)
        self.declare_parameter('replan_pause_sec', 2.0)
        self.delay = float(self.get_parameter('delay_sec').value)
        self.repeat = bool(self.get_parameter('repeat').value)
        self.replan_pause = float(self.get_parameter('replan_pause_sec').value)

        self._sent_first = False
        self._round = 0  # even = swap, odd = home
        self._wall_t0 = time.monotonic()
        self._next_ok_wall = 0.0

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pubs = {
            name: self.create_publisher(PoseStamped, f'/swarm/{name}/goal', latched)
            for name in GOALS_SWAP
        }
        self._trigger = self.create_publisher(Bool, '/swarm/demo_goals_sent', 10)
        self.create_subscription(Bool, '/swarm/makespan_complete', self._on_done, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Will publish swap goals after {self.delay:.1f}s wall '
            f'(repeat={self.repeat})'
        )

    def _publish_set(self, goals: dict, label: str):
        stamp = self.get_clock().now().to_msg()
        for name, (x, y, yaw) in goals.items():
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            self._pubs[name].publish(msg)
            self.get_logger().info(f'[{label}] {name} -> ({x}, {y})')
        flag = Bool()
        flag.data = True
        self._trigger.publish(flag)

    def _tick(self):
        if self._sent_first:
            return
        if time.monotonic() - self._wall_t0 < self.delay:
            return
        self._publish_set(GOALS_SWAP, 'swap')
        self._sent_first = True
        self._round = 0
        self._next_ok_wall = time.monotonic() + self.replan_pause

    def _on_done(self, msg: Bool):
        if not msg.data or not self.repeat:
            return
        if not self._sent_first:
            return
        if time.monotonic() < self._next_ok_wall:
            return
        self._round += 1
        if self._round % 2 == 1:
            self._publish_set(GOALS_HOME, 'home')
        else:
            self._publish_set(GOALS_SWAP, 'swap')
        self._next_ok_wall = time.monotonic() + self.replan_pause


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
