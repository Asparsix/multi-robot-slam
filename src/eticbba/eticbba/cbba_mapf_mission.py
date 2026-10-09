#!/usr/bin/env python3
"""Full mission: CBBA assign → MAPF each bundle leg → verify every assigned task.

For leg k = 0 .. K-1:
  - publish /swarm/robotN/goal for robots that still have a k-th task
    (robots without a task hold at current TF pose so MAPF still plans all 4)
  - wait for /swarm/paths_ready
  - wait for /swarm/makespan_complete (or TF near goals)
  - confirm each active robot is within goal_tolerance of its task

Exit 0 only if every assigned task was completed.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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
from eticbba.nav2_path_cost import Nav2PathCost


DEFAULT_ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4']


class CbbaMapfMission(Node):
    def __init__(self):
        super().__init__('cbba_mapf_mission')
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('output_file', '/tmp/house_10_cbba_assignment.yaml')
        self.declare_parameter('result_file', '/tmp/cbba_mapf_mission_result.yaml')
        self.declare_parameter('goal_tolerance_xy', 0.40)
        self.declare_parameter('paths_ready_timeout_sec', 180.0)
        self.declare_parameter('leg_timeout_sec', 1200.0)
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('planner_wait_sec', 120.0)

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

        self._completed: Dict[str, List[str]] = {n: [] for n in self._robots}

    @staticmethod
    def _feed_tf(buf: Buffer, msg: TFMessage, is_static: bool):
        for tf in msg.transforms:
            if is_static:
                buf.set_transform_static(tf, 'cbba_mapf_mission')
            else:
                buf.set_transform(tf, 'cbba_mapf_mission')

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

    def _wait_tf(self, timeout_sec: float = 60.0) -> bool:
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

    def _run_cbba(self) -> dict:
        """Run CBBA on this node once Nav2 planner answers reliably."""
        import numpy as np

        path_cost = Nav2PathCost(
            self,
            action_name='/robot1/compute_path_to_pose',
            server_timeout=90.0,
            call_timeout=20.0,
        )
        planner_wait = float(self.get_parameter('planner_wait_sec').value)
        self.get_logger().info('Waiting for Nav2 ComputePathToPose…')
        t0 = time.monotonic()
        ready = False
        while rclpy.ok() and (time.monotonic() - t0) < planner_wait:
            if path_cost.wait_for_server():
                ready = True
                break
            rclpy.spin_once(self, timeout_sec=0.5)
        if not ready:
            raise RuntimeError('Nav2 planner action server not available')
        # Warm-up (may use Euclidean fallback if planner is slow)
        _ = path_cost.path_length((-3.0, -3.0), (-1.5, -1.5))
        self.get_logger().info('Planner ready (or Euclidean fallback active)')

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
        k = int(self._cfg['robots'].get('bundle_capacity_k', 3))
        score_offset = 1.0e4
        task_pts = [tuple(task_xy[j]) for j in range(len(task_ids))]

        def score_fn(agent_id: int, path_indices):
            if not path_indices:
                return 0.0
            points = [task_pts[j] for j in path_indices]
            spawn = spawns[robot_names[agent_id]]
            length = path_cost.bundle_path_length(spawn, points)
            return score_offset * len(path_indices) - length

        self.get_logger().info(
            f'Running CBBA: {len(robot_names)} robots, {len(task_ids)} tasks, K={k}'
        )
        agents = run_cbba(
            [spawns[n] for n in robot_names], task_xy, k, score_fn
        )

        assignment = {}
        assigned_set = set()
        for i, name in enumerate(robot_names):
            bundle_idx = agents[i].p
            bundle_ids = [task_ids[j] for j in bundle_idx]
            if bundle_idx:
                pts = [tuple(task_xy[j]) for j in bundle_idx]
                plen = path_cost.bundle_path_length(spawns[name], pts)
            else:
                plen = 0.0
            assignment[name] = {
                'bundle_indices': bundle_idx,
                'bundle_task_ids': bundle_ids,
                'path_length_m': float(plen),
            }
            assigned_set.update(bundle_idx)

        pool = [task_ids[j] for j in range(len(task_ids)) if j not in assigned_set]
        result = {
            'algorithm': 'CBBA',
            'cost_model': 'nav2_compute_path_to_pose_length',
            'bundle_capacity_k': k,
            'robots': assignment,
            'unassigned_task_ids': pool,
            'nav2_cache_queries': len(path_cost._cache),
        }
        text = yaml.dump(result, sort_keys=False)
        self.get_logger().info(f'CBBA result:\n{text}')
        out = str(self.get_parameter('output_file').value)
        if out:
            Path(out).write_text(text, encoding='utf-8')
            self.get_logger().info(f'Wrote {out}')
        if pool:
            self.get_logger().warn(f'Unassigned after CBBA: {pool}')
        else:
            self.get_logger().info('All 10 tasks assigned by CBBA')
        return result

    def _near(self, name: str, x: float, y: float) -> bool:
        xy = self._lookup_xy(name)
        if xy is None:
            return False
        return math.hypot(xy[0] - x, xy[1] - y) <= self._tol

    def run(self) -> int:
        self.get_logger().info('Waiting for TF before CBBA…')
        if not self._wait_tf(90.0):
            return 2

        self.get_logger().info('Running CBBA assignment…')
        assignment = self._run_cbba()
        robots = assignment['robots']

        # Ensure leftover tasks (e.g. T03/T04) also get a robot when K was tight
        leftovers = list(assignment.get('unassigned_task_ids') or [])
        if leftovers:
            self.get_logger().info(f'Assigning leftovers to robots: {leftovers}')
            for tid in leftovers:
                t = self._tasks[tid]
                tx, ty = float(t['x']), float(t['y'])
                best = None
                best_key = None
                for name in self._robots:
                    bundle = robots[name]['bundle_task_ids']
                    xy = self._lookup_xy(name)
                    if xy is None:
                        sx = float(self._cfg['robots']['spawns'][name]['x'])
                        sy = float(self._cfg['robots']['spawns'][name]['y'])
                        xy = (sx, sy)
                    # Prefer fewer tasks, then nearer robot
                    key = (len(bundle), math.hypot(xy[0] - tx, xy[1] - ty))
                    if best_key is None or key < best_key:
                        best_key = key
                        best = name
                assert best is not None
                robots[best]['bundle_task_ids'].append(tid)
                self.get_logger().info(f'Leftover {tid} -> {best}')
            assignment['unassigned_task_ids'] = []
            assignment['robots'] = robots
            out = str(self.get_parameter('output_file').value)
            if out:
                Path(out).write_text(
                    yaml.dump(assignment, sort_keys=False), encoding='utf-8'
                )

        max_legs = max(
            (len(robots[n]['bundle_task_ids']) for n in self._robots),
            default=0,
        )
        if max_legs == 0:
            self.get_logger().error('CBBA assigned no tasks')
            return 3

        self.get_logger().info(
            f'Mission: {max_legs} MAPF leg(s); bundles='
            + str({n: robots[n]['bundle_task_ids'] for n in self._robots})
        )

        paths_timeout = float(self.get_parameter('paths_ready_timeout_sec').value)
        leg_timeout = float(self.get_parameter('leg_timeout_sec').value)
        settle = float(self.get_parameter('settle_sec').value)

        for leg in range(max_legs):
            active: Dict[str, Tuple[str, float, float]] = {}
            for name in self._robots:
                bundle = robots[name]['bundle_task_ids']
                if leg < len(bundle):
                    tid = bundle[leg]
                    t = self._tasks[tid]
                    active[name] = (tid, float(t['x']), float(t['y']))

            self.get_logger().info(
                f'===== LEG {leg}/{max_legs - 1} active={list(active.keys())} ====='
            )

            self._paths_ready = False
            self._makespan_done = False

            # Publish goals for everyone (holders stay put for MAPF completeness)
            for name in self._robots:
                if name in active:
                    tid, x, y = active[name]
                    self._publish_goal(name, x, y)
                    self.get_logger().info(f'{name}: goal {tid} -> ({x:.2f},{y:.2f})')
                else:
                    xy = self._lookup_xy(name)
                    if xy is None:
                        self.get_logger().error(f'{name}: no TF for hold goal')
                        return 4
                    self._publish_goal(name, xy[0], xy[1])
                    self.get_logger().info(
                        f'{name}: hold at ({xy[0]:.2f},{xy[1]:.2f})'
                    )

            if not self._spin_until(
                lambda: self._paths_ready, paths_timeout, f'paths_ready leg {leg}'
            ):
                return 5
            self.get_logger().info(f'Leg {leg}: MAPF paths_ready')

            # Wait for executor makespan OR all active robots near goals
            def leg_done_ok():
                if self._makespan_done:
                    return True
                if not active:
                    return True
                return all(
                    self._near(n, x, y) for n, (_tid, x, y) in active.items()
                )

            if not self._spin_until(
                leg_done_ok, leg_timeout, f'makespan/arrival leg {leg}'
            ):
                for n, (tid, x, y) in active.items():
                    xy = self._lookup_xy(n)
                    if xy is None:
                        self.get_logger().error(f'{n} {tid}: no TF')
                    else:
                        d = math.hypot(xy[0] - x, xy[1] - y)
                        self.get_logger().error(
                            f'{n} {tid}: pos=({xy[0]:.2f},{xy[1]:.2f}) '
                            f'target=({x:.2f},{y:.2f}) d={d:.2f}'
                        )
                return 6

            # Settle and verify active tasks (even if makespan timed out steps)
            t_settle = time.monotonic() + settle
            while time.monotonic() < t_settle and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

            # Extra wait for TF near if makespan finished early via timeouts
            def all_near():
                return all(
                    self._near(n, x, y) for n, (_tid, x, y) in active.items()
                )

            if active and not all_near():
                if not self._spin_until(all_near, 120.0, f'TF near goals leg {leg}'):
                    for n, (tid, x, y) in active.items():
                        xy = self._lookup_xy(n)
                        d = (
                            math.hypot(xy[0] - x, xy[1] - y)
                            if xy is not None
                            else float('inf')
                        )
                        self.get_logger().error(
                            f'LEG {leg} FAIL: {n} did not reach {tid} (d={d:.2f})'
                        )
                    return 7

            for name, (tid, x, y) in active.items():
                self._completed[name].append(tid)
                self.get_logger().info(f'COMPLETED {name} {tid}')

        # Final report
        expected = {n: list(robots[n]['bundle_task_ids']) for n in self._robots}
        ok = self._completed == expected
        all_task_ids = sorted(self._tasks.keys())
        done_ids = sorted({t for tasks in self._completed.values() for t in tasks})
        all_ten = done_ids == all_task_ids
        result = {
            'success': ok and all_ten,
            'all_ten_tasks_done': all_ten,
            'expected': expected,
            'completed': self._completed,
            'unassigned_task_ids': assignment.get('unassigned_task_ids', []),
        }
        out = str(self.get_parameter('result_file').value)
        Path(out).write_text(yaml.dump(result, sort_keys=False), encoding='utf-8')
        self.get_logger().info(f'Wrote {out}\n{yaml.dump(result, sort_keys=False)}')
        if ok and all_ten:
            self.get_logger().info('ALL_TEN_TASKS_OK')
            return 0
        if ok:
            self.get_logger().info('ALL_ASSIGNED_TASKS_OK')
            return 0
        self.get_logger().error('MISSION_INCOMPLETE')
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = CbbaMapfMission()
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
