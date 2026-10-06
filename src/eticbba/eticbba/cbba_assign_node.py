#!/usr/bin/env python3
"""Run CBBA on house_10_tasks using Nav2 path length as cost."""

from __future__ import annotations

from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from eticbba.cbba_solver import run_cbba
from eticbba.nav2_path_cost import Nav2PathCost


class CbbaAssignNode(Node):
    def __init__(self):
        super().__init__('cbba_house_assign')
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('planner_action', '/robot1/compute_path_to_pose')
        self.declare_parameter('publish_goals', True)
        self.declare_parameter('output_file', '')

        tasks_file = self.get_parameter('tasks_file').value
        if not tasks_file:
            share = get_package_share_directory('eticbba')
            tasks_file = str(Path(share) / 'tasks' / 'house_10_tasks.yaml')
        self._cfg = self._load_tasks(tasks_file)

        self._robot_names: List[str] = list(self._cfg['robots']['names'])
        self._task_ids: List[str] = [t['id'] for t in self._cfg['tasks']]
        self._task_xy = np.array(
            [[t['x'], t['y']] for t in self._cfg['tasks']], dtype=np.float64
        )
        self._spawns = {
            name: (
                float(self._cfg['robots']['spawns'][name]['x']),
                float(self._cfg['robots']['spawns'][name]['y']),
            )
            for name in self._robot_names
        }
        self._k = int(self._cfg['robots'].get('bundle_capacity_k', 2))

        self._path_cost = Nav2PathCost(
            self,
            action_name=str(self.get_parameter('planner_action').value),
        )

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._goal_pubs = {}
        if self.get_parameter('publish_goals').value:
            for name in self._robot_names:
                self._goal_pubs[name] = self.create_publisher(
                    PoseStamped, f'/swarm/{name}/goal', latched
                )

    @staticmethod
    def _load_tasks(path: str) -> dict:
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def _make_score_fn(self, nav2: Nav2PathCost):
        spawns = [
            self._spawns[self._robot_names[i]] for i in range(len(self._robot_names))
        ]
        task_pts = [tuple(self._task_xy[j]) for j in range(len(self._task_ids))]

        # CBBA maximizes score and compares bids against y=0, so scores must be
        # positive. Use a large offset minus Nav2 path length (shorter => better).
        score_offset = 1.0e4

        def score_fn(agent_id: int, path_indices: Sequence[int]) -> float:
            if not path_indices:
                return 0.0
            points = [task_pts[j] for j in path_indices]
            length = nav2.bundle_path_length(spawns[agent_id], points)
            return score_offset * len(path_indices) - length

        return score_fn

    def run_assignment(self):
        if not self._path_cost.wait_for_server():
            raise RuntimeError('Nav2 compute_path_to_pose not available')

        self.get_logger().info(
            f'Running CBBA: {len(self._robot_names)} robots, '
            f'{len(self._task_ids)} tasks, K={self._k}, Nav2 path cost'
        )

        agent_states = [self._spawns[n] for n in self._robot_names]
        score_fn = self._make_score_fn(self._path_cost)
        agents = run_cbba(agent_states, self._task_xy, self._k, score_fn)

        assignment = {}
        assigned_set = set()
        for i, name in enumerate(self._robot_names):
            bundle_idx = agents[i].p
            bundle_ids = [self._task_ids[j] for j in bundle_idx]
            if bundle_idx:
                pts = [tuple(self._task_xy[j]) for j in bundle_idx]
                plen = self._path_cost.bundle_path_length(self._spawns[name], pts)
            else:
                plen = 0.0
            assignment[name] = {
                'bundle_indices': bundle_idx,
                'bundle_task_ids': bundle_ids,
                'path_length_m': float(plen),
            }
            assigned_set.update(bundle_idx)

        pool = [self._task_ids[j] for j in range(len(self._task_ids)) if j not in assigned_set]

        result = {
            'algorithm': 'CBBA',
            'cost_model': 'nav2_compute_path_to_pose_length',
            'bundle_capacity_k': self._k,
            'robots': assignment,
            'unassigned_task_ids': pool,
            'nav2_cache_queries': len(self._path_cost._cache),
        }

        text = yaml.dump(result, sort_keys=False)
        self.get_logger().info(f'CBBA result:\n{text}')

        out = self.get_parameter('output_file').value
        if out:
            Path(out).write_text(text, encoding='utf-8')
            self.get_logger().info(f'Wrote {out}')

        if self._goal_pubs:
            stamp = self.get_clock().now().to_msg()
            for name in self._robot_names:
                bundle = assignment[name]['bundle_task_ids']
                if not bundle:
                    continue
                tid = bundle[0]
                task = next(t for t in self._cfg['tasks'] if t['id'] == tid)
                msg = PoseStamped()
                msg.header.frame_id = self._cfg.get('frame_id', 'map')
                msg.header.stamp = stamp
                msg.pose.position.x = float(task['x'])
                msg.pose.position.y = float(task['y'])
                msg.pose.orientation.w = 1.0
                self._goal_pubs[name].publish(msg)
                self.get_logger().info(f'Published first goal {tid} for {name}')

        return result


def main():
    rclpy.init()
    node = CbbaAssignNode()
    try:
        node.run_assignment()
    except Exception as e:
        node.get_logger().error(f'CBBA assignment failed: {e}')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
