#!/usr/bin/env python3
"""Publish odom->base_footprint on the namespaced TF tree from odometry.

AMCL owns map->odom; this node only owns odom->base_footprint.

Important: TF stamps must stay consistent with sensor (scan) stamps. Republishing
with clock.now() pushes TF into the future and makes AMCL/costmaps drop scans
("timestamp earlier than all the data in the transform cache"). So we always
stamp with the odometry header time.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class PrefixOdomTf(Node):
    def __init__(self) -> None:
        super().__init__('prefix_odom_tf')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('prefix', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Keepalive rate using the *last odom stamp* (not wall/sim now).
        self.declare_parameter('republish_hz', 20.0)

        self._prefix = self.get_parameter('prefix').value
        if self._prefix and not self._prefix.endswith('/'):
            self._prefix += '/'
        self._odom_frame = self._prefix + self.get_parameter('odom_frame').value
        self._base_frame = self._prefix + self.get_parameter('base_frame').value

        self._br = TransformBroadcaster(self)
        self._last: TransformStamped | None = None

        topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, topic, self._cb, 50)

        hz = float(self.get_parameter('republish_hz').value)
        if hz > 0.0:
            self.create_timer(1.0 / hz, self._republish)

        self.get_logger().info(
            f'{topic} -> TF {self._odom_frame} -> {self._base_frame} '
            f'(republish {hz:.1f} Hz, odom stamps)'
        )

    def _make_tf(self, msg: Odometry) -> TransformStamped:
        t = TransformStamped()
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        return t

    def _cb(self, msg: Odometry) -> None:
        t = self._make_tf(msg)
        self._last = t
        self._br.sendTransform(t)

    def _republish(self) -> None:
        """Re-send last odom TF with the same stamp (keepalive for late joiners)."""
        if self._last is None:
            return
        self._br.sendTransform(self._last)


def main() -> None:
    rclpy.init()
    node = PrefixOdomTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
