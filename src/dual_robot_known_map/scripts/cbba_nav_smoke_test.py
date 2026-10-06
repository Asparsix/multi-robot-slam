#!/usr/bin/env python3
"""Nav2 smoke tests for CBBA goals: sequential or parallel (no MAPF).

Modes (parameter ``mode``):
  sequential     — robot1 T02→T01, then robot2 T06→T07
  parallel_all   — all four to inner quadrant goals at once
  parallel_pairs — only 2 move at a time: r1↔r2 opposite corners, then r3↔r4
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

Point = Tuple[float, float]

TASKS: Dict[str, Point] = {
    'T01': (-1.5, -1.5),
    'T02': (-1.0, -2.5),
    'T06': (2.0, -2.0),
    'T07': (2.5, -1.0),
    'T08': (-2.0, 2.0),
    'T09': (1.5, 2.0),
}

# All four to opposite-side cells (used only by parallel_all).
PARALLEL_ALL: Dict[str, Point] = {
    'robot1': (2.0, -2.0),   # SW -> SE (robot2's side)
    'robot2': (-1.5, -1.5),  # SE -> SW (robot1's side)
    'robot3': (1.5, 2.0),    # NW -> NE (robot4's side)
    'robot4': (-2.0, 2.0),   # NE -> NW (robot3's side)
}

# Only TWO robots move at a time: swap into the partner's corner/side.
# Goals are offset from exact spawn so they don't sit on each other.
PARALLEL_PAIRS: List[Tuple[Dict[str, Point], str]] = [
    (
        {
            'robot1': (2.0, -2.5),   # toward robot2 SE corner
            'robot2': (-1.5, -2.5),  # toward robot1 SW corner
        },
        'south swap: robot1↔robot2 opposite corners',
    ),
    (
        {
            'robot3': (1.5, 2.5),    # toward robot4 NE corner
            'robot4': (-2.0, 2.5),   # toward robot3 NW corner
        },
        'north swap: robot3↔robot4 opposite corners',
    ),
]

SEQUENTIAL_PLAN: List[Tuple[str, List[str]]] = [
    ('robot1', ['T02', 'T01']),
    ('robot2', ['T06', 'T07']),
]


class NavSmokeTest(Node):
    def __init__(self):
        super().__init__('cbba_nav_smoke_test')
        self.declare_parameter('mode', 'parallel_pairs')
        self.declare_parameter('goal_tolerance_m', 0.40)
        self.declare_parameter('goal_timeout_sec', 240.0)

        self._tol = float(self.get_parameter('goal_tolerance_m').value)
        self._timeout = float(self.get_parameter('goal_timeout_sec').value)
        self._mode = str(self.get_parameter('mode').value)

        self._nav_clients: Dict[str, ActionClient] = {}
        self._tf_bufs: Dict[str, Buffer] = {}
        static_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for name in ('robot1', 'robot2', 'robot3', 'robot4'):
            self._nav_clients[name] = ActionClient(
                self, NavigateToPose, f'/{name}/navigate_to_pose'
            )
            buf = Buffer(cache_time=Duration(seconds=60.0))
            self._tf_bufs[name] = buf
            self.create_subscription(
                TFMessage, f'/{name}/tf',
                lambda m, b=buf: self._feed_tf(b, m, False), 200,
            )
            self.create_subscription(
                TFMessage, f'/{name}/tf_static',
                lambda m, b=buf: self._feed_tf(b, m, True), static_qos,
            )

    @staticmethod
    def _feed_tf(buf: Buffer, msg: TFMessage, is_static: bool) -> None:
        for t in msg.transforms:
            if is_static:
                buf.set_transform_static(t, 'cbba_nav_smoke_test')
            else:
                buf.set_transform(t, 'cbba_nav_smoke_test')

    def pose(self, name: str) -> Optional[Point]:
        try:
            t = self._tf_bufs[name].lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            p = t.transform.translation
            return p.x, p.y
        except Exception:
            return None

    def _wait_tf(self, names: List[str], wall_sec: float = 10.0) -> None:
        t_end = time.time() + wall_sec
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.pose(n) is not None for n in names):
                return

    def _wait_server(self, name: str) -> bool:
        ok = self._nav_clients[name].wait_for_server(timeout_sec=60.0)
        if not ok:
            self.get_logger().error(f'{name}: navigate_to_pose not available')
        return ok

    def _go_one(self, name: str, xy: Point) -> Tuple[bool, str, Optional[Point], Optional[Point]]:
        if not self._wait_server(name):
            return False, 'no server', None, None
        start = self.pose(name)
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(xy[0])
        ps.pose.position.y = float(xy[1])
        ps.pose.orientation.w = 1.0
        goal.pose = ps

        self.get_logger().info(f'{name} -> {xy} start={start}')
        send = self._nav_clients[name].send_goal_async(goal)
        t0 = time.time()
        while not send.done() and time.time() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send.done():
            return False, 'send timeout', start, self.pose(name)
        gh = send.result()
        if gh is None or not gh.accepted:
            return False, 'rejected', start, self.pose(name)

        result_fut = gh.get_result_async()
        while not result_fut.done() and time.time() - t0 < self._timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            p = self.pose(name)
            if p is not None and math.hypot(p[0] - xy[0], p[1] - xy[1]) <= self._tol:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return True, f'reached dist={math.hypot(p[0]-xy[0], p[1]-xy[1]):.3f}', start, p

        end = self.pose(name)
        if result_fut.done():
            st = result_fut.result().status
            d = None if end is None else math.hypot(end[0] - xy[0], end[1] - xy[1])
            ok = st == GoalStatus.STATUS_SUCCEEDED or (d is not None and d <= self._tol)
            return ok, f'status={st} dist={d}', start, end
        d = None if end is None else math.hypot(end[0] - xy[0], end[1] - xy[1])
        ok = d is not None and d <= self._tol
        return ok, f'timeout dist={d}', start, end

    def _go_parallel(self, assignments: Dict[str, Point]) -> bool:
        names = list(assignments.keys())
        self._wait_tf(names)
        for n in names:
            if not self._wait_server(n):
                return False

        handles: Dict[str, dict] = {}
        stamp = self.get_clock().now().to_msg()
        for name, xy in assignments.items():
            goal = NavigateToPose.Goal()
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = stamp
            ps.pose.position.x = float(xy[0])
            ps.pose.position.y = float(xy[1])
            ps.pose.orientation.w = 1.0
            goal.pose = ps
            start = self.pose(name)
            send = self._nav_clients[name].send_goal_async(goal)
            handles[name] = {
                'xy': xy, 'start': start, 'send': send,
                'gh': None, 'result': None, 'done': False, 'ok': False, 'msg': '',
            }
            self.get_logger().info(f'parallel send {name} -> {xy} start={start}')

        t0 = time.time()
        while time.time() - t0 < self._timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            for name, h in handles.items():
                if h['done']:
                    continue
                if h['gh'] is None and h['send'].done():
                    gh = h['send'].result()
                    h['gh'] = gh
                    if gh and gh.accepted:
                        h['result'] = gh.get_result_async()
                    else:
                        h['done'] = True
                        h['msg'] = 'rejected'
                xy = h['xy']
                p = self.pose(name)
                if p is not None and math.hypot(p[0] - xy[0], p[1] - xy[1]) <= self._tol:
                    h['done'] = True
                    h['ok'] = True
                    h['msg'] = f'reached dist={math.hypot(p[0]-xy[0], p[1]-xy[1]):.3f}'
                    if h['gh']:
                        try:
                            h['gh'].cancel_goal_async()
                        except Exception:
                            pass
                    continue
                res = h.get('result')
                if res and res.done():
                    st = res.result().status
                    p = self.pose(name)
                    d = None if p is None else math.hypot(p[0] - xy[0], p[1] - xy[1])
                    h['done'] = True
                    h['ok'] = st == GoalStatus.STATUS_SUCCEEDED or (
                        d is not None and d <= self._tol
                    )
                    h['msg'] = f'status={st} dist={d}'

            if all(h['done'] for h in handles.values()):
                break

        all_ok = True
        for name, h in handles.items():
            end = self.pose(name)
            self.get_logger().info(
                f'RESULT {name} ok={h["ok"]} {h["msg"]} end={end}'
            )
            all_ok = all_ok and h['ok']
        return all_ok

    def run(self) -> int:
        self.get_logger().info(f'mode={self._mode}')
        if self._mode == 'sequential':
            self._wait_tf(['robot1', 'robot2'])
            for robot, tids in SEQUENTIAL_PLAN:
                for tid in tids:
                    ok, msg, start, end = self._go_one(robot, TASKS[tid])
                    self.get_logger().info(f'{robot} {tid} ok={ok} {msg} end={end}')
                    if not ok:
                        return 1
            return 0

        if self._mode == 'parallel_all':
            return 0 if self._go_parallel(PARALLEL_ALL) else 1

        if self._mode == 'parallel_pairs':
            for assignments, label in PARALLEL_PAIRS:
                self.get_logger().info(f'=== {label} (only 2 robots) ===')
                if not self._go_parallel(assignments):
                    return 1
            return 0

        self.get_logger().error(
            f'Unknown mode {self._mode!r}; use sequential, parallel_all, parallel_pairs'
        )
        return 1


def main():
    rclpy.init()
    node = NavSmokeTest()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
