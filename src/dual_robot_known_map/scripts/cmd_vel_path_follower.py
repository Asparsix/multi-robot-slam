#!/usr/bin/env python3
"""Lightweight MAPF path tracker — publishes cmd_vel (no Nav2).

Follows /swarm/robotN/path after /swarm/paths_ready.
Pose source: odom + known spawn (map←odom static), with TF as optional assist.
Independent per-robot follow (no all-four TF gate).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


DEFAULT_ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4']
DEFAULT_SPAWNS = [
    -3.0, -3.0, 0.0,
    3.0, -3.0, math.pi,
    -3.0, 3.0, 0.0,
    3.0, 3.0, math.pi,
]


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _odom_to_map(
    ox: float, oy: float, oyaw: float, sx: float, sy: float, syaw: float
) -> Tuple[float, float, float]:
    """Compose static map→odom (spawn) with odom→base."""
    c, s = math.cos(syaw), math.sin(syaw)
    mx = sx + c * ox - s * oy
    my = sy + s * ox + c * oy
    return mx, my, _wrap(syaw + oyaw)


class CmdVelPathFollower(Node):
    def __init__(self):
        super().__init__('cmd_vel_path_follower')
        self.declare_parameter('robot_names', DEFAULT_ROBOTS)
        self.declare_parameter('spawns_xy_yaw', DEFAULT_SPAWNS)
        self.declare_parameter('linear_speed', 0.28)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('waypoint_tol', 0.22)
        self.declare_parameter('final_tol', 0.18)
        self.declare_parameter('pose_stride', 2)
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('goal_yaw_tol', 0.45)

        self._robots: List[str] = [str(r) for r in self.get_parameter('robot_names').value]
        raw = [float(x) for x in self.get_parameter('spawns_xy_yaw').value]
        self._spawns: Dict[str, Tuple[float, float, float]] = {}
        for i, name in enumerate(self._robots):
            base = 3 * i
            if base + 2 < len(raw):
                self._spawns[name] = (raw[base], raw[base + 1], raw[base + 2])
            else:
                self._spawns[name] = (0.0, 0.0, 0.0)

        self._v = float(self.get_parameter('linear_speed').value)
        self._w = float(self.get_parameter('angular_speed').value)
        self._tol = float(self.get_parameter('waypoint_tol').value)
        self._final_tol = float(self.get_parameter('final_tol').value)
        self._stride = max(1, int(self.get_parameter('pose_stride').value))
        self._yaw_tol = float(self.get_parameter('goal_yaw_tol').value)

        self._paths: Dict[str, List[PoseStamped]] = {}
        self._idx: Dict[str, int] = {n: 0 for n in self._robots}
        self._active = False
        self._cur: Dict[str, PoseStamped] = {}
        self._odom: Dict[str, Odometry] = {}

        self._cmd_pubs = {
            n: self.create_publisher(Twist, f'/{n}/cmd_vel', 10) for n in self._robots
        }
        for n in self._robots:
            self.create_subscription(
                Path, f'/swarm/{n}/path', lambda msg, name=n: self._on_path(name, msg), 10
            )
            self.create_subscription(
                Odometry, f'/{n}/odom', lambda msg, name=n: self._on_odom(name, msg), 20
            )
        self.create_subscription(Bool, '/swarm/paths_ready', self._on_ready, 10)
        self._done_pub = self.create_publisher(Bool, '/swarm/makespan_complete', 10)

        # Optional TF (not required to drive)
        self._tf: Dict[str, Buffer] = {}
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        for n in self._robots:
            buf = Buffer(cache_time=Duration(seconds=30.0))
            self._tf[n] = buf
            self.create_subscription(
                TFMessage, f'/{n}/tf',
                lambda msg, b=buf: self._feed(b, msg, False), 100,
            )
            self.create_subscription(
                TFMessage, f'/{n}/tf_static',
                lambda msg, b=buf: self._feed(b, msg, True), static_qos,
            )

        hz = float(self.get_parameter('tick_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'cmd_vel follower ready v={self._v:.2f} (odom+spawn pose) robots={self._robots}'
        )

    @staticmethod
    def _feed(buf: Buffer, msg: TFMessage, is_static: bool):
        for tf in msg.transforms:
            if is_static:
                buf.set_transform_static(tf, 'cmd_vel_follower')
            else:
                buf.set_transform(tf, 'cmd_vel_follower')

    def _on_odom(self, name: str, msg: Odometry):
        self._odom[name] = msg

    def _on_path(self, name: str, msg: Path):
        if not msg.poses:
            return
        full = list(msg.poses)
        sampled = full[:: self._stride]
        if sampled[-1] is not full[-1]:
            sampled.append(full[-1])
        self._paths[name] = sampled

    def _on_ready(self, msg: Bool):
        if not msg.data:
            self._active = False
            self._stop_all()
            return
        for n in self._robots:
            if n not in self._paths or not self._paths[n]:
                self.get_logger().error(f'{n}: no path on paths_ready')
                return
            self._idx[n] = 0
        self._active = True
        self.get_logger().info(
            'Tracking new MAPF paths: '
            + ', '.join(f'{n}={len(self._paths[n])}' for n in self._robots)
        )

    def _update_pose(self, name: str) -> bool:
        # Prefer odom + spawn (reliable under load)
        if name in self._odom:
            o = self._odom[name]
            sx, sy, syaw = self._spawns[name]
            mx, my, myaw = _odom_to_map(
                o.pose.pose.position.x,
                o.pose.pose.position.y,
                _yaw(o.pose.pose.orientation),
                sx, sy, syaw,
            )
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = mx
            ps.pose.position.y = my
            ps.pose.orientation.z = math.sin(myaw / 2.0)
            ps.pose.orientation.w = math.cos(myaw / 2.0)
            self._cur[name] = ps
            return True
        # Fallback TF
        try:
            t = self._tf[name].lookup_transform('map', 'base_footprint', rclpy.time.Time())
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.orientation = t.transform.rotation
            self._cur[name] = ps
            return True
        except Exception:
            return False

    def _stop_all(self):
        stop = Twist()
        for p in self._cmd_pubs.values():
            p.publish(stop)

    def _control(self, name: str) -> bool:
        """Drive toward current waypoint. Return True if this robot finished its path."""
        poses = self._paths.get(name, [])
        i = self._idx[name]
        if i >= len(poses):
            self._cmd_pubs[name].publish(Twist())
            return True

        cur = self._cur.get(name)
        if cur is None:
            return False

        goal = poses[i]
        dx = goal.pose.position.x - cur.pose.position.x
        dy = goal.pose.position.y - cur.pose.position.y
        dist = math.hypot(dx, dy)
        is_final = i >= len(poses) - 1
        tol = self._final_tol if is_final else self._tol

        if dist < 0.05 and not is_final:
            self._idx[name] = i + 1
            self._cmd_pubs[name].publish(Twist())
            return False

        if dist < tol:
            if is_final:
                gy = _yaw(goal.pose.orientation)
                cy = _yaw(cur.pose.orientation)
                ey = _wrap(gy - cy)
                if abs(ey) > self._yaw_tol:
                    cmd = Twist()
                    cmd.angular.z = self._w if ey > 0 else -self._w
                    self._cmd_pubs[name].publish(cmd)
                    return False
                self._cmd_pubs[name].publish(Twist())
                self._idx[name] = i + 1
                return True
            self._idx[name] = i + 1
            return False

        target_yaw = math.atan2(dy, dx)
        cy = _yaw(cur.pose.orientation)
        ey = _wrap(target_yaw - cy)

        cmd = Twist()
        if abs(ey) > 0.6:
            cmd.angular.z = max(-self._w, min(self._w, 2.0 * ey))
            cmd.linear.x = 0.05
        else:
            cmd.angular.z = max(-self._w, min(self._w, 1.5 * ey))
            cmd.linear.x = self._v * max(0.35, 1.0 - abs(ey) / 1.2)
        self._cmd_pubs[name].publish(cmd)
        return False

    def _tick(self):
        if not self._active:
            return

        done = True
        for n in self._robots:
            if not self._update_pose(n):
                self.get_logger().warn(f'{n}: waiting for odom…', throttle_duration_sec=3.0)
                done = False
                continue
            if not self._control(n):
                done = False

        if done:
            self.get_logger().info('Makespan complete (cmd_vel follower)')
            self._active = False
            self._stop_all()
            msg = Bool()
            msg.data = True
            self._done_pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
