"""Nav2 ComputePathToPose path length for CBBA costs."""

from __future__ import annotations

import math
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


Point = Tuple[float, float]


class Nav2PathCost:
    """Query planner path length via Nav2 (same map for all robots)."""

    def __init__(
        self,
        node: Node,
        action_name: str = '/robot1/compute_path_to_pose',
        planner_id: str = 'GridBased',
        server_timeout: float = 30.0,
        call_timeout: float = 20.0,
    ):
        self._node = node
        self._client = ActionClient(node, ComputePathToPose, action_name)
        self._planner_id = planner_id
        self._server_timeout = server_timeout
        self._call_timeout = call_timeout
        self._cache: Dict[Tuple[float, float, float, float], float] = {}
        self._logger = node.get_logger()

    def wait_for_server(self) -> bool:
        ok = self._client.wait_for_server(timeout_sec=self._server_timeout)
        if not ok:
            self._logger.error(
                f'Nav2 planner action not available: {self._client._action_name}'
            )
        return ok

    @staticmethod
    def _path_length_m(path) -> float:
        if path is None or len(path.poses) < 2:
            return 0.0
        total = 0.0
        prev = path.poses[0].pose.position
        for ps in path.poses[1:]:
            p = ps.pose.position
            total += math.hypot(p.x - prev.x, p.y - prev.y)
            prev = p
        return total

    def path_length(self, start: Point, goal: Point) -> float:
        key = (
            round(start[0], 3),
            round(start[1], 3),
            round(goal[0], 3),
            round(goal[1], 3),
        )
        if key in self._cache:
            return self._cache[key]

        last_err: Exception | None = None
        for attempt in range(1, 3):
            try:
                length = self._path_length_uncached(start, goal)
                self._cache[key] = length
                self._logger.info(
                    f'Nav2 path {start} -> {goal}: {length:.2f} m',
                    throttle_duration_sec=2.0,
                )
                return length
            except Exception as e:
                last_err = e
                self._logger.warn(
                    f'Nav2 path attempt {attempt}/2 failed {start}->{goal}: {e}'
                )
                import time as _time
                t_end = _time.monotonic() + 1.5
                while _time.monotonic() < t_end:
                    rclpy.spin_once(self._node, timeout_sec=0.1)

        # Fallback so CBBA can still assign under planner load; execution still uses MAPF+Nav2.
        eucl = math.hypot(goal[0] - start[0], goal[1] - start[1]) * 1.25
        self._logger.warn(
            f'Using Euclidean fallback {start}->{goal}: {eucl:.2f} m '
            f'(last Nav2 error: {last_err})'
        )
        self._cache[key] = eucl
        return eucl

    def _path_length_uncached(self, start: Point, goal: Point) -> float:
        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.planner_id = self._planner_id
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.start.pose.position.x = start[0]
        goal_msg.start.pose.position.y = start[1]
        goal_msg.start.pose.orientation.w = 1.0

        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = goal_msg.start.header.stamp
        goal_msg.goal.pose.position.x = goal[0]
        goal_msg.goal.pose.position.y = goal[1]
        goal_msg.goal.pose.orientation.w = 1.0

        send = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send, timeout_sec=self._call_timeout)
        if not send.done():
            raise RuntimeError(f'ComputePathToPose timed out {start} -> {goal}')
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f'ComputePathToPose rejected {start} -> {goal}')

        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, result_fut, timeout_sec=self._call_timeout
        )
        if not result_fut.done():
            raise RuntimeError(f'ComputePathToPose result timed out {start} -> {goal}')

        res = result_fut.result().result
        return self._path_length_m(res.path)

    def bundle_path_length(self, start: Point, task_points: list[Point]) -> float:
        if not task_points:
            return 0.0
        total = 0.0
        cur = start
        for pt in task_points:
            total += self.path_length(cur, pt)
            cur = pt
        return total
