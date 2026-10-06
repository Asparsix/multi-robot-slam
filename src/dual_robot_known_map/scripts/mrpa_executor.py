#!/usr/bin/env python3
"""Timestep MAPF executor (mapf_ros plan_executor style).

Walks each robot's MAPF path one time-step at a time:
  for t in 0..makespan-1:
    send NavigateToPose(pose[t]) to every robot that has step t
    wait until all of those robots are near their step goal
    then advance

Keeps space-time sync so MAPF collision guarantees are preserved,
unlike dumping the whole path into NavigateThroughPoses.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


DEFAULT_ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4']


def _yaw_of(pose) -> float:
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _near(
    cur: PoseStamped,
    goal: PoseStamped,
    xy_tol: float,
    yaw_tol: float = 2.0 * math.pi,
) -> bool:
    dx = cur.pose.position.x - goal.pose.position.x
    dy = cur.pose.position.y - goal.pose.position.y
    dist2 = dx * dx + dy * dy
    if dist2 < 1e-12:
        # already on cell (wait step) — treat as reached
        return True
    if dist2 > xy_tol * xy_tol:
        return False
    dyaw = abs(_yaw_of(cur.pose) - _yaw_of(goal.pose))
    while dyaw > math.pi:
        dyaw -= 2.0 * math.pi
    return abs(dyaw) <= yaw_tol


class TimestepExecutor(Node):
    def __init__(self):
        super().__init__('mrpa_executor')
        self.declare_parameter('robot_names', DEFAULT_ROBOTS)
        self.declare_parameter('xy_goal_tolerance', 0.25)
        self.declare_parameter('yaw_goal_tolerance', 0.35)
        self.declare_parameter('mid_xy_tolerance', 0.40)
        self.declare_parameter('pose_stride', 1)  # execute every Nth MAPF pose
        self.declare_parameter('step_timeout_sec', 45.0)
        self.declare_parameter('tick_hz', 10.0)

        self._robot_names: List[str] = [
            str(r) for r in self.get_parameter('robot_names').value
        ]
        self._xy_tol = float(self.get_parameter('xy_goal_tolerance').value)
        self._yaw_tol = float(self.get_parameter('yaw_goal_tolerance').value)
        self._mid_tol = float(self.get_parameter('mid_xy_tolerance').value)
        self._pose_stride = max(1, int(self.get_parameter('pose_stride').value))
        self._step_timeout = float(self.get_parameter('step_timeout_sec').value)

        self._paths: Dict[str, Path] = {}
        self._poses: Dict[str, List[PoseStamped]] = {}
        self._makespan = 0
        self._step = -1
        self._active = False
        self._waiting = False
        self._step_goals: Dict[str, PoseStamped] = {}
        self._step_t0: Optional[float] = None
        self._goal_handles: Dict[str, object] = {}
        self._nav_clients: Dict[str, ActionClient] = {}
        self._cur_poses: Dict[str, PoseStamped] = {}
        self._tf_ready = False

        for name in self._robot_names:
            self.create_subscription(
                Path,
                f'/swarm/{name}/path',
                lambda msg, n=name: self._store_path(n, msg),
                10,
            )
            self._nav_clients[name] = ActionClient(
                self,
                NavigateToPose,
                f'/{name}/navigate_to_pose',
            )

        self.create_subscription(Bool, '/swarm/paths_ready', self._on_ready, 10)

        # Namespaced TF (same pattern as prioritized_mapf)
        self._tf_buffers: Dict[str, Buffer] = {}
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        for name in self._robot_names:
            buf = Buffer(cache_time=Duration(seconds=30.0))
            self._tf_buffers[name] = buf
            self.create_subscription(
                TFMessage,
                f'/{name}/tf',
                lambda msg, b=buf: self._feed_tf(b, msg, False),
                100,
            )
            self.create_subscription(
                TFMessage,
                f'/{name}/tf_static',
                lambda msg, b=buf: self._feed_tf(b, msg, True),
                static_qos,
            )

        hz = float(self.get_parameter('tick_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'Timestep executor ready for {self._robot_names} '
            f'(stride={self._pose_stride}, mid_tol={self._mid_tol})'
        )

    @staticmethod
    def _feed_tf(buf: Buffer, msg: TFMessage, is_static: bool):
        for tf in msg.transforms:
            if is_static:
                buf.set_transform_static(tf, 'mrpa_executor')
            else:
                buf.set_transform(tf, 'mrpa_executor')

    def _store_path(self, name: str, msg: Path):
        self._paths[name] = msg

    def _update_poses_from_tf(self):
        ok = True
        for name in self._robot_names:
            buf = self._tf_buffers[name]
            try:
                t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = t.transform.translation.x
                ps.pose.position.y = t.transform.translation.y
                ps.pose.position.z = t.transform.translation.z
                ps.pose.orientation = t.transform.rotation
                self._cur_poses[name] = ps
            except Exception:
                ok = False
        self._tf_ready = ok and len(self._cur_poses) == len(self._robot_names)

    def _on_ready(self, msg: Bool):
        if not msg.data:
            self.get_logger().warn('paths_ready=False — cancelling active plan')
            self._cancel_all()
            self._active = False
            return

        poses: Dict[str, List[PoseStamped]] = {}
        for name in self._robot_names:
            path = self._paths.get(name)
            if path is None or not path.poses:
                self.get_logger().error(f'{name}: missing path on paths_ready')
                return
            # Subsample while keeping shared time index alignment
            full = list(path.poses)
            sampled = full[:: self._pose_stride]
            if sampled[-1] is not full[-1]:
                sampled.append(full[-1])
            poses[name] = sampled

        self._poses = poses
        self._makespan = max(len(p) for p in poses.values())
        self._step = -1
        self._waiting = False
        self._step_goals.clear()
        self._step_t0 = None
        self._active = True
        self.get_logger().info(
            f'New plan: makespan={self._makespan} '
            f'lens={[len(self._poses[n]) for n in self._robot_names]}'
        )

    def _get_pose_at(self, name: str, step: int) -> Optional[PoseStamped]:
        seq = self._poses.get(name, [])
        if not seq:
            return None
        if step < len(seq):
            return seq[step]
        # Hold final pose past end of this agent's path
        return seq[-1]

    def _tick(self):
        self._update_poses_from_tf()
        if not self._active:
            return

        if not self._waiting:
            # Need TF only to start; once waiting, wall-clock timeout can advance
            if not self._tf_ready:
                self.get_logger().warn(
                    'Waiting for TF map->base_footprint…',
                    throttle_duration_sec=5.0,
                )
                return
            nxt = self._step + 1
            if nxt >= self._makespan:
                self.get_logger().info('Makespan complete — all step goals done')
                self._active = False
                return
            self._dispatch_step(nxt)
            return

        # Waiting for current step arrivals (wall clock — sim clock can stall)
        assert self._step_t0 is not None
        timed_out = (time.monotonic() - self._step_t0) > self._step_timeout

        all_near = True
        missing_tf = []
        for name, goal in self._step_goals.items():
            cur = self._cur_poses.get(name)
            if cur is None:
                missing_tf.append(name)
                all_near = False
                continue
            is_final = self._step >= self._makespan - 1
            if is_final:
                ok = _near(cur, goal, self._xy_tol, self._yaw_tol)
            else:
                ok = _near(cur, goal, self._mid_tol)
            if not ok:
                all_near = False

        if missing_tf:
            self.get_logger().warn(
                f'Step {self._step}: missing TF for {missing_tf}',
                throttle_duration_sec=5.0,
            )

        if all_near and not missing_tf:
            self.get_logger().info(
                f'Step {self._step}/{self._makespan - 1} reached by all agents'
            )
            self._waiting = False
            return

        if timed_out:
            self.get_logger().error(
                f'Step {self._step} timed out after {self._step_timeout:.0f}s wall — advancing anyway'
            )
            self._waiting = False

    def _dispatch_step(self, step: int):
        self._step = step
        self._step_goals.clear()
        stamp = self.get_clock().now().to_msg()
        is_final = step >= self._makespan - 1

        for name in self._robot_names:
            pose = self._get_pose_at(name, step)
            if pose is None:
                continue
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = stamp
            goal_pose.pose = pose.pose
            self._step_goals[name] = goal_pose

            client = self._nav_clients[name]
            if not client.server_is_ready():
                if not client.wait_for_server(timeout_sec=0.05):
                    self.get_logger().warn(
                        f'{name}: navigate_to_pose not ready',
                        throttle_duration_sec=2.0,
                    )
                    continue

            goal = NavigateToPose.Goal()
            goal.pose = goal_pose
            self.get_logger().info(
                f'{name}: step {step}/{self._makespan - 1} -> '
                f'({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
                + (' [FINAL]' if is_final else '')
            )
            fut = client.send_goal_async(goal)
            fut.add_done_callback(lambda f, n=name: self._on_goal_response(n, f))

        self._waiting = True
        self._step_t0 = time.monotonic()

    def _on_goal_response(self, name: str, fut):
        try:
            gh = fut.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(f'{name}: NavigateToPose REJECTED at step {self._step}')
                return
            self._goal_handles[name] = gh
        except Exception as e:
            self.get_logger().error(f'{name}: send error {e}')

    def _cancel_all(self):
        for name, gh in list(self._goal_handles.items()):
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
        self._goal_handles.clear()
        self._waiting = False


def main():
    rclpy.init()
    node = TimestepExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
