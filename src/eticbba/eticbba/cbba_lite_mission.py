#!/usr/bin/env python3
"""Fast demo mission: Euclidean CBBA → PIBT/ADG with independent task queues.

No Nav2 planner dependency. Each robot advances through its CBBA bundle on its
own completion; when any robot finishes a task, goals are updated and MAPF is
replanned for everyone from live poses (no global leg barrier).
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from eticbba.cbba_solver import run_cbba


DEFAULT_ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4']
CurrentGoal = Tuple[str, float, float]  # task_id, x, y


def _euclid_bundle_length(start: Tuple[float, float], points) -> float:
    if not points:
        return 0.0
    total = 0.0
    cur = start
    for p in points:
        total += math.hypot(p[0] - cur[0], p[1] - cur[1])
        cur = p
    return total


class CbbaLiteMission(Node):
    def __init__(self):
        super().__init__('cbba_lite_mission')
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('output_file', '/tmp/house_20_cbba_assignment.yaml')
        self.declare_parameter('result_file', '/tmp/cbba_pibt_mission_result.yaml')
        self.declare_parameter('goal_tolerance_xy', 0.50)
        self.declare_parameter('paths_ready_timeout_sec', 90.0)
        self.declare_parameter('leg_timeout_sec', 300.0)  # alias / fallback
        self.declare_parameter('task_timeout_sec', 0.0)  # 0 → use leg_timeout_sec
        self.declare_parameter('mission_timeout_sec', 0.0)  # 0 → auto from bundles
        self.declare_parameter('replan_retries', 4)
        self.declare_parameter('settle_sec', 1.0)
        self.declare_parameter('tf_wait_sec', 45.0)

        tasks_file = str(self.get_parameter('tasks_file').value)
        if not tasks_file:
            share = get_package_share_directory('eticbba')
            tasks_file = str(Path(share) / 'tasks' / 'house_10_tasks.yaml')
        with open(tasks_file, 'r', encoding='utf-8') as f:
            self._cfg = yaml.safe_load(f)

        self._robots: List[str] = list(self._cfg['robots']['names'])
        self._tasks = {t['id']: t for t in self._cfg['tasks']}
        self._tol = float(self.get_parameter('goal_tolerance_xy').value)
        self._frame = str(self._cfg.get('frame_id', 'map'))

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._goal_pubs = {
            name: self.create_publisher(PoseStamped, f'/swarm/{name}/goal', latched)
            for name in self._robots
        }

        self._paths_ready = False
        self._makespan_done = False
        self.create_subscription(Bool, '/swarm/paths_ready', self._on_paths_ready, 10)
        self.create_subscription(
            Bool, '/swarm/makespan_complete', self._on_makespan, 10
        )

        self._tf_buffers: Dict[str, Buffer] = {}
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        for name in self._robots:
            buf = Buffer(cache_time=Duration(seconds=30.0))
            self._tf_buffers[name] = buf
            self.create_subscription(
                TFMessage, f'/{name}/tf',
                lambda msg, b=buf: self._feed_tf(b, msg, False), 100,
            )
            self.create_subscription(
                TFMessage, f'/{name}/tf_static',
                lambda msg, b=buf: self._feed_tf(b, msg, True), static_qos,
            )

        self._completed: Dict[str, List[str]] = {n: [] for n in self._robots}

    @staticmethod
    def _feed_tf(buf: Buffer, msg: TFMessage, is_static: bool):
        for tf in msg.transforms:
            if is_static:
                buf.set_transform_static(tf, 'cbba_lite_mission')
            else:
                buf.set_transform(tf, 'cbba_lite_mission')

    def _on_paths_ready(self, msg: Bool):
        self._paths_ready = bool(msg.data)

    def _on_makespan(self, msg: Bool):
        if msg.data:
            self._makespan_done = True

    def _spin_until(self, predicate, timeout_sec: float, label: str) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.get_logger().error(f'Timeout waiting for {label} ({timeout_sec:.0f}s)')
        return False

    def _lookup_xy(self, name: str) -> Optional[Tuple[float, float]]:
        try:
            t = self._tf_buffers[name].lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def _wait_tf(self, timeout_sec: float) -> bool:
        def ready():
            return all(self._lookup_xy(n) is not None for n in self._robots)

        return self._spin_until(ready, timeout_sec, 'TF map->base for all robots')

    def _publish_goal(self, name: str, x: float, y: float):
        msg = PoseStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self._goal_pubs[name].publish(msg)

    def _task_xy(self, tid: str) -> CurrentGoal:
        t = self._tasks[tid]
        return tid, float(t['x']), float(t['y'])

    def _publish_goal_set(
        self, current: Dict[str, Optional[CurrentGoal]]
    ) -> bool:
        """Publish active goals; finished robots hold at live pose."""
        self._paths_ready = False
        self._makespan_done = False
        for name in self._robots:
            g = current[name]
            if g is not None:
                tid, x, y = g
                self._publish_goal(name, x, y)
                self.get_logger().info(f'{name}: goal {tid} -> ({x:.2f},{y:.2f})')
            else:
                xy = self._lookup_xy(name)
                if xy is None:
                    self.get_logger().error(f'{name}: no TF for hold goal')
                    return False
                self._publish_goal(name, xy[0], xy[1])
                self.get_logger().info(
                    f'{name}: hold at ({xy[0]:.2f},{xy[1]:.2f})'
                )
        return True

    def _wait_paths_ready(self, timeout_sec: float, label: str) -> bool:
        """Wait for paths_ready; republish is caller's job via retries."""
        # Let all latched goals arrive before PIBT's 0.5s tick plans.
        t_settle = time.monotonic() + 0.6
        while time.monotonic() < t_settle and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._spin_until(
            lambda: self._paths_ready, timeout_sec, label
        )

    def _run_cbba(self) -> dict:
        robot_names = list(self._cfg['robots']['names'])
        task_ids = [t['id'] for t in self._cfg['tasks']]
        task_xy = np.array(
            [[t['x'], t['y']] for t in self._cfg['tasks']], dtype=np.float64
        )
        spawns = {
            name: (
                float(self._cfg['robots']['spawns'][name]['x']),
                float(self._cfg['robots']['spawns'][name]['y']),
            )
            for name in robot_names
        }
        # Prefer live TF starts when available
        starts = []
        for name in robot_names:
            xy = self._lookup_xy(name)
            starts.append(xy if xy is not None else spawns[name])

        k = int(self._cfg['robots'].get('bundle_capacity_k', 2))
        score_offset = 1.0e4
        task_pts = [tuple(task_xy[j]) for j in range(len(task_ids))]

        def score_fn(agent_id: int, path_indices):
            if not path_indices:
                return 0.0
            points = [task_pts[j] for j in path_indices]
            length = _euclid_bundle_length(starts[agent_id], points)
            return score_offset * len(path_indices) - length

        self.get_logger().info(
            f'Running CBBA (Euclidean): {len(robot_names)} robots, '
            f'{len(task_ids)} tasks, K={k}'
        )
        agents = run_cbba(starts, task_xy, k, score_fn)

        assignment = {}
        assigned_set = set()
        for i, name in enumerate(robot_names):
            bundle_idx = agents[i].p
            bundle_ids = [task_ids[j] for j in bundle_idx]
            pts = [tuple(task_xy[j]) for j in bundle_idx] if bundle_idx else []
            plen = _euclid_bundle_length(starts[i], pts)
            assignment[name] = {
                'bundle_indices': bundle_idx,
                'bundle_task_ids': bundle_ids,
                'path_length_m': float(plen),
            }
            assigned_set.update(bundle_idx)

        pool = [task_ids[j] for j in range(len(task_ids)) if j not in assigned_set]
        result = {
            'algorithm': 'CBBA',
            'cost_model': 'euclidean',
            'bundle_capacity_k': k,
            'robots': assignment,
            'unassigned_task_ids': pool,
        }
        text = yaml.dump(result, sort_keys=False)
        self.get_logger().info(f'CBBA result:\n{text}')
        out = str(self.get_parameter('output_file').value)
        if out:
            Path(out).write_text(text, encoding='utf-8')
            self.get_logger().info(f'Wrote {out}')
        return result

    def _near(self, name: str, x: float, y: float) -> bool:
        xy = self._lookup_xy(name)
        if xy is None:
            return False
        return math.hypot(xy[0] - x, xy[1] - y) <= self._tol

    def run(self) -> int:
        tf_wait = float(self.get_parameter('tf_wait_sec').value)
        self.get_logger().info('Waiting for TF before CBBA…')
        if not self._wait_tf(tf_wait):
            return 2

        assignment = self._run_cbba()
        robots = assignment['robots']

        leftovers = list(assignment.get('unassigned_task_ids') or [])
        if leftovers:
            self.get_logger().info(f'Assigning leftovers: {leftovers}')
            for tid in leftovers:
                t = self._tasks[tid]
                tx, ty = float(t['x']), float(t['y'])
                best = None
                best_key = None
                for name in self._robots:
                    bundle = robots[name]['bundle_task_ids']
                    xy = self._lookup_xy(name) or (
                        float(self._cfg['robots']['spawns'][name]['x']),
                        float(self._cfg['robots']['spawns'][name]['y']),
                    )
                    key = (len(bundle), math.hypot(xy[0] - tx, xy[1] - ty))
                    if best_key is None or key < best_key:
                        best_key = key
                        best = name
                robots[best]['bundle_task_ids'].append(tid)
                self.get_logger().info(f'Leftover {tid} -> {best}')
            assignment['unassigned_task_ids'] = []
            out = str(self.get_parameter('output_file').value)
            if out:
                Path(out).write_text(
                    yaml.dump(assignment, sort_keys=False), encoding='utf-8'
                )

        bundles = {n: list(robots[n]['bundle_task_ids']) for n in self._robots}
        total_assigned = sum(len(b) for b in bundles.values())
        if total_assigned == 0:
            self.get_logger().error('CBBA assigned no tasks')
            return 3

        cursor = {n: 0 for n in self._robots}
        current: Dict[str, Optional[CurrentGoal]] = {}
        for name in self._robots:
            if bundles[name]:
                current[name] = self._task_xy(bundles[name][0])
            else:
                current[name] = None

        self.get_logger().info(
            f'Mission: independent queues ({total_assigned} tasks); bundles='
            + str(bundles)
        )

        paths_timeout = float(self.get_parameter('paths_ready_timeout_sec').value)
        leg_timeout = float(self.get_parameter('leg_timeout_sec').value)
        task_timeout = float(self.get_parameter('task_timeout_sec').value)
        if task_timeout <= 0.0:
            task_timeout = leg_timeout
        mission_timeout = float(self.get_parameter('mission_timeout_sec').value)
        max_bundle = max((len(b) for b in bundles.values()), default=1)
        if mission_timeout <= 0.0:
            mission_timeout = task_timeout * max_bundle + 60.0
        retries = max(1, int(self.get_parameter('replan_retries').value))
        settle = float(self.get_parameter('settle_sec').value)

        def plan_round(label: str) -> int:
            """Publish goals and wait for MAPF; return 0 ok, else error code."""
            for attempt in range(retries):
                if not self._publish_goal_set(current):
                    return 4
                tag = f'{label} try {attempt + 1}/{retries}'
                if self._wait_paths_ready(paths_timeout, f'paths_ready {tag}'):
                    self.get_logger().info(f'{label}: MAPF paths_ready — moving')
                    return 0
                self.get_logger().warn(f'{tag}: paths_ready missed — republish')
            return 5

        code = plan_round('initial plan')
        if code != 0:
            return code

        task_deadline = {
            n: time.monotonic() + task_timeout
            for n in self._robots
            if current[n] is not None
        }
        mission_deadline = time.monotonic() + mission_timeout
        replan_epoch = 0

        while rclpy.ok():
            if time.monotonic() > mission_deadline:
                self.get_logger().error(
                    f'Mission timeout ({mission_timeout:.0f}s); '
                    f'completed={self._completed}'
                )
                return 6

            finished: List[str] = []
            for name in self._robots:
                g = current[name]
                if g is None:
                    continue
                tid, x, y = g
                if self._near(name, x, y):
                    finished.append(name)
                elif time.monotonic() > task_deadline.get(name, float('inf')):
                    self.get_logger().error(
                        f'{name}: task {tid} timeout ({task_timeout:.0f}s) '
                        f'at goal ({x:.2f},{y:.2f})'
                    )
                    return 6

            if finished:
                t_settle = time.monotonic() + settle
                while time.monotonic() < t_settle and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.05)

                advanced = False
                for name in self._robots:
                    g = current[name]
                    if g is None:
                        continue
                    tid, x, y = g
                    if not self._near(name, x, y):
                        continue
                    self._completed[name].append(tid)
                    self.get_logger().info(f'COMPLETED {name} {tid}')
                    cursor[name] += 1
                    if cursor[name] < len(bundles[name]):
                        nxt = bundles[name][cursor[name]]
                        current[name] = self._task_xy(nxt)
                        task_deadline[name] = time.monotonic() + task_timeout
                        self.get_logger().info(
                            f'{name}: advance -> {nxt} '
                            f'({cursor[name] + 1}/{len(bundles[name])})'
                        )
                    else:
                        current[name] = None
                        task_deadline.pop(name, None)
                        self.get_logger().info(f'{name}: bundle done — holding')
                    advanced = True

                if all(current[n] is None for n in self._robots):
                    self.get_logger().info('All robot queues empty')
                    break

                if advanced:
                    replan_epoch += 1
                    code = plan_round(f'replan #{replan_epoch}')
                    if code != 0:
                        return code
                continue

            if all(current[n] is None for n in self._robots):
                break

            rclpy.spin_once(self, timeout_sec=0.1)

        expected = bundles
        ok = self._completed == expected
        all_task_ids = sorted(self._tasks.keys())
        done_ids = sorted({t for tasks in self._completed.values() for t in tasks})
        all_tasks_done = done_ids == all_task_ids
        result = {
            'success': ok and all_tasks_done,
            'all_tasks_done': all_tasks_done,
            'num_tasks': len(all_task_ids),
            'expected': expected,
            'completed': self._completed,
            'mode': 'lite_pibt_adg_independent',
            'replans': replan_epoch,
        }
        out = str(self.get_parameter('result_file').value)
        Path(out).write_text(yaml.dump(result, sort_keys=False), encoding='utf-8')
        self.get_logger().info(f'Wrote {out}\n{yaml.dump(result, sort_keys=False)}')
        if ok and all_tasks_done:
            self.get_logger().info(f'ALL_{len(all_task_ids)}_TASKS_OK')
            return 0
        if ok:
            self.get_logger().info('ALL_ASSIGNED_TASKS_OK')
            return 0
        self.get_logger().error('MISSION_INCOMPLETE')
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = CbbaLiteMission()
    code = 1
    try:
        code = node.run()
    except Exception as e:
        node.get_logger().error(f'Mission failed: {e}')
        code = 9
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
