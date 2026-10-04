#!/usr/bin/env python3
"""Publish odom->base TF on the global /tf tree with a frame prefix."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class PrefixOdomTf(Node):
    def __init__(self) -> None:
        super().__init__('prefix_odom_tf')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('prefix', 'robot1/')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._prefix = self.get_parameter('prefix').value
        if self._prefix and not self._prefix.endswith('/'):
            self._prefix += '/'
        self._odom_frame = self._prefix + self.get_parameter('odom_frame').value
        self._base_frame = self._prefix + self.get_parameter('base_frame').value

        self._br = TransformBroadcaster(self)
        topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, topic, self._cb, 50)
        self.get_logger().info(
            f'{topic} -> TF {self._odom_frame} -> {self._base_frame}'
        )

    def _cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


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
