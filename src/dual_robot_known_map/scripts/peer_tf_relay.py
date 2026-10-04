#!/usr/bin/env python3
"""Relay peer robot TF into the host namespace tree for one-RViz viewing.

Publishes under viz_prefix (default robot2/) onto host /robot1/tf(+_static),
keeping global_odom shared so both robots sit in the same tree as collaborative
slam's map -> global_odom.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

STATIC_QOS = QoSProfile(
    depth=100,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class PeerTfRelay(Node):
    def __init__(self) -> None:
        super().__init__('peer_tf_relay')
        self.declare_parameter('peer_tf_topic', '/robot2/tf')
        self.declare_parameter('peer_tf_static_topic', '/robot2/tf_static')
        self.declare_parameter('host_tf_topic', '/robot1/tf')
        self.declare_parameter('host_tf_static_topic', '/robot1/tf_static')
        self.declare_parameter('viz_prefix', 'robot2')
        self.declare_parameter('shared_frames', ['global_odom'])

        self._prefix = self.get_parameter('viz_prefix').value
        shared = self.get_parameter('shared_frames').get_parameter_value().string_array_value
        self._shared = set(shared or ['global_odom'])

        peer_tf = self.get_parameter('peer_tf_topic').value
        peer_static = self.get_parameter('peer_tf_static_topic').value
        host_tf = self.get_parameter('host_tf_topic').value
        host_static = self.get_parameter('host_tf_static_topic').value

        self._tf_pub = self.create_publisher(TFMessage, host_tf, 100)
        self._static_pub = self.create_publisher(TFMessage, host_static, STATIC_QOS)
        self.create_subscription(TFMessage, peer_tf, self._on_tf, 100)
        self.create_subscription(TFMessage, peer_static, self._on_static, STATIC_QOS)
        self.get_logger().info(
            f'Relay {peer_tf} -> {host_tf} prefix={self._prefix}/ shared={self._shared}'
        )

    def _rewrite(self, frame: str) -> str:
        frame = frame.lstrip('/')
        if not frame or frame in self._shared:
            return frame
        if frame.startswith(self._prefix + '/'):
            return frame
        return f'{self._prefix}/{frame}'

    def _convert(self, msg: TFMessage) -> TFMessage:
        out = TFMessage()
        for t in msg.transforms:
            # Skip peer map->... (host slam owns map); keep global_odom and body
            parent = t.header.frame_id.lstrip('/')
            if parent == 'map' or parent.endswith('/map'):
                continue
            nt = type(t)()
            nt.header = t.header
            nt.header.frame_id = self._rewrite(t.header.frame_id)
            nt.child_frame_id = self._rewrite(t.child_frame_id)
            nt.transform = t.transform
            out.transforms.append(nt)
        return out

    def _on_tf(self, msg: TFMessage) -> None:
        out = self._convert(msg)
        if out.transforms:
            self._tf_pub.publish(out)

    def _on_static(self, msg: TFMessage) -> None:
        out = self._convert(msg)
        if out.transforms:
            self._static_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PeerTfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
