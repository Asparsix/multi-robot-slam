#!/usr/bin/env python3
"""Republish LaserScan with a prefixed frame_id for the global TF tree."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PrefixScan(Node):
    def __init__(self) -> None:
        super().__init__('prefix_scan')
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_viz')
        self.declare_parameter('frame_id', 'robot1/base_scan')

        self._frame = self.get_parameter('frame_id').value
        out = self.get_parameter('output_topic').value
        inp = self.get_parameter('input_topic').value
        self._pub = self.create_publisher(LaserScan, out, 10)
        self.create_subscription(LaserScan, inp, self._cb, 10)
        self.get_logger().info(f'{inp} -> {out} (frame={self._frame})')

    def _cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self._frame
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PrefixScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
