#!/usr/bin/env python3
"""SMART ADG execution monitor + cmd_vel (no Nav2).

Ports the release logic from smart-mapf/smart ``ADG`` / ``ADG_server``:
  - Build Type-2 dependency edges from MAPF timed paths
  - Release next action(s) only when dependencies are finished
  - Drive released goals with lightweight cmd_vel (odom + spawn pose)

Refs:
  https://github.com/smart-mapf/smart/blob/master/server/src/ADG.cpp
  https://github.com/smart-mapf/smart/blob/master/server/src/ADG_server.cpp
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path as FsPath
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray

# Allow importing sibling module when run as installed script
_SCRIPTS = FsPath(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from smart_adg import SmartADG, XY, paths_to_actions  # noqa: E402
from mapf_grid_params import compute_mapf_grid  # noqa: E402


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
    c, s = math.cos(syaw), math.sin(syaw)
    return sx + c * ox - s * oy, sy + s * ox + c * oy, _wrap(syaw + oyaw)


class AdgExecutor(Node):
    def __init__(self):
        super().__init__('adg_executor')
        self.declare_parameter('robot_names', DEFAULT_ROBOTS)
        self.declare_parameter('spawns_xy_yaw', DEFAULT_SPAWNS)
        self.declare_parameter('linear_speed', 0.22)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('goal_tol', 0.12)
        self.declare_parameter('pose_stride', 1)
        self.declare_parameter('tick_hz', 20.0)
        # cell_quant <= 0 and neighbor_buffer < 0 → take from /swarm/mapf_grid (planner)
        self.declare_parameter('cell_quant', 0.0)
        self.declare_parameter('neighbor_buffer', -1)
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('planning_margin', 0.08)
        self.declare_parameter('extra_sep_cells', 1)
        self.declare_parameter('lookahead_actions', 1)
        self.declare_parameter('hold_sec', 0.35)
        self.declare_parameter('clear_start_frac', 0.55)

        self._robots: List[str] = [str(r) for r in self.get_parameter('robot_names').value]
        raw = [float(x) for x in self.get_parameter('spawns_xy_yaw').value]
        self._spawns: Dict[str, Tuple[float, float, float]] = {}
        for i, name in enumerate(self._robots):
            b = 3 * i
            self._spawns[name] = (
                (raw[b], raw[b + 1], raw[b + 2]) if b + 2 < len(raw) else (0.0, 0.0, 0.0)
            )

        self._v = float(self.get_parameter('linear_speed').value)
        self._w = float(self.get_parameter('angular_speed').value)
        self._tol = float(self.get_parameter('goal_tol').value)
        self._stride = max(1, int(self.get_parameter('pose_stride').value))
        self._quant_param = float(self.get_parameter('cell_quant').value)
        self._neighbor_param = int(self.get_parameter('neighbor_buffer').value)
        self._robot_radius = float(self.get_parameter('robot_radius').value)
        self._planning_margin = float(self.get_parameter('planning_margin').value)
        self._extra_sep = max(0, int(self.get_parameter('extra_sep_cells').value))
        # Fallback until /swarm/mapf_grid arrives (assume common 0.05m maps)
        fb = compute_mapf_grid(
            0.05,
            self._robot_radius,
            planning_margin=self._planning_margin,
            extra_sep_cells=self._extra_sep,
        )
        self._quant = self._quant_param if self._quant_param > 0.0 else fb.coarse_res
        self._neighbor = (
            self._neighbor_param if self._neighbor_param >= 0 else fb.neighbor_buffer
        )
        self._grid_from_planner = False
        self._lookahead = max(1, int(self.get_parameter('lookahead_actions').value))
        self._hold_sec = float(self.get_parameter('hold_sec').value)
        self._clear_frac = float(self.get_parameter('clear_start_frac').value)

        self._paths_xy: Dict[str, List[XY]] = {}
        self._odom: Dict[str, Odometry] = {}
        self._cur: Dict[str, Tuple[float, float, float]] = {}
        self._adg: Optional[SmartADG] = None
        self._active = False
        # Per robot: currently executing action node id (goal of that action)
        self._active_node: Dict[str, Optional[int]] = {n: None for n in self._robots}
        self._at_goal_since: Dict[str, Optional[float]] = {n: None for n in self._robots}

        self._cmd = {
            n: self.create_publisher(Twist, f'/{n}/cmd_vel', 10) for n in self._robots
        }
        for n in self._robots:
            self.create_subscription(
                Path, f'/swarm/{n}/path', lambda m, name=n: self._on_path(name, m), 10
            )
            self.create_subscription(
                Odometry, f'/{n}/odom', lambda m, name=n: self._on_odom(name, m), 20
            )
        self.create_subscription(Bool, '/swarm/paths_ready', self._on_ready, 10)
        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Float64MultiArray, '/swarm/mapf_grid', self._on_mapf_grid, grid_qos
        )
        self._done_pub = self.create_publisher(Bool, '/swarm/makespan_complete', 10)

        hz = float(self.get_parameter('tick_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'SMART ADG executor ready robots={self._robots} '
            f'(Type-2 release + cmd_vel; quant auto from /swarm/mapf_grid)'
        )

    def _on_mapf_grid(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            return
        coarse_res = float(msg.data[0])
        neighbor = int(msg.data[3])
        if self._quant_param <= 0.0:
            self._quant = coarse_res
        if self._neighbor_param < 0:
            self._neighbor = max(0, neighbor)
        self._grid_from_planner = True
        self.get_logger().info(
            f'MAPF grid from planner: cell={self._quant:.3f}m '
            f'neighbor_buffer={self._neighbor}'
        )

    def _on_odom(self, name: str, msg: Odometry):
        self._odom[name] = msg

    def _on_path(self, name: str, msg: Path):
        if not msg.poses:
            return
        full = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        sampled = full[:: self._stride]
        if sampled[-1] != full[-1]:
            sampled.append(full[-1])
        self._paths_xy[name] = sampled

    def _on_ready(self, msg: Bool):
        if not msg.data:
            self._active = False
            self._adg = None
            self._stop_all()
            return
        for n in self._robots:
            if n not in self._paths_xy or not self._paths_xy[n]:
                self.get_logger().error(f'{n}: no path on paths_ready')
                return
        plans = paths_to_actions(self._paths_xy, self._robots)
        self._adg = SmartADG(
            plans, quant=self._quant, neighbor_buffer=self._neighbor
        )
        self._active_node = {n: None for n in self._robots}
        self._at_goal_since = {n: None for n in self._robots}
        # Initial release (SMART init → getPlan), lookahead-limited
        for rid, name in enumerate(self._robots):
            released = self._adg.release_for_agent(rid, max_n=self._lookahead)
            if released:
                self._active_node[name] = released[0]
        self._active = True
        self.get_logger().info(
            f'ADG built: nodes={self._adg.total_nodes} '
            f'type1={self._adg.type1_edges} type2={self._adg.type2_edges} '
            f'quant={self._quant:.2f} neighbor={self._neighbor} '
            f'paths=' + ', '.join(f'{n}={len(self._paths_xy[n])}' for n in self._robots)
        )

    def _update_pose(self, name: str) -> bool:
        if name not in self._odom:
            return False
        o = self._odom[name]
        sx, sy, syaw = self._spawns[name]
        self._cur[name] = _odom_to_map(
            o.pose.pose.position.x,
            o.pose.pose.position.y,
            _yaw(o.pose.pose.orientation),
            sx, sy, syaw,
        )
        return True

    def _stop_all(self):
        stop = Twist()
        for p in self._cmd.values():
            p.publish(stop)

    def _action_of(self, name: str, node_id: int):
        assert self._adg is not None
        rid = self._robots.index(name)
        nodes = self._adg.graph[rid]
        if node_id < 0 or node_id >= len(nodes):
            return None
        return nodes[node_id].action

    def _near_goal(self, name: str, goal: XY) -> bool:
        cur = self._cur.get(name)
        if cur is None:
            return False
        return math.hypot(goal[0] - cur[0], goal[1] - cur[1]) < self._tol

    def _cleared_start(self, name: str, start: XY, goal: XY) -> bool:
        """Require leaving start cell before releasing Type-2 dependents."""
        cur = self._cur.get(name)
        if cur is None:
            return False
        # Wait / zero move: start≈goal — nothing to clear
        if math.hypot(start[0] - goal[0], start[1] - goal[1]) < 0.05 * self._quant:
            return True
        d = math.hypot(cur[0] - start[0], cur[1] - start[1])
        return d >= self._clear_frac * self._quant

    def _drive_to(self, name: str, goal: XY) -> None:
        cur = self._cur.get(name)
        if cur is None:
            return
        x, y, cyaw = cur
        dx = goal[0] - x
        dy = goal[1] - y
        dist = math.hypot(dx, dy)
        if dist < self._tol or dist < 0.04:
            self._cmd[name].publish(Twist())
            return
        tyaw = math.atan2(dy, dx)
        ey = _wrap(tyaw - cyaw)
        cmd = Twist()
        if abs(ey) > 0.6:
            cmd.angular.z = max(-self._w, min(self._w, 2.0 * ey))
            cmd.linear.x = 0.05
        else:
            cmd.angular.z = max(-self._w, min(self._w, 1.5 * ey))
            cmd.linear.x = self._v * max(0.35, 1.0 - abs(ey) / 1.2)
        self._cmd[name].publish(cmd)

    def _action_done(self, name: str, node_id: int) -> bool:
        """Strict finish: near goal for hold_sec; clear-start except final action."""
        act = self._action_of(name, node_id)
        if act is None:
            return False
        if not self._near_goal(name, act.goal):
            self._at_goal_since[name] = None
            return False
        self._cmd[name].publish(Twist())
        now = time.monotonic()
        if self._at_goal_since[name] is None:
            self._at_goal_since[name] = now
            return False
        if now - self._at_goal_since[name] < self._hold_sec:
            return False
        # Last action: already at terminal pose — do not demand leaving start
        rid = self._robots.index(name)
        n_nodes = len(self._adg.graph[rid]) if self._adg is not None else 0
        if node_id < n_nodes - 1 and not self._cleared_start(
            name, act.start, act.goal
        ):
            return False
        return True

    def _next_action(self, rid: int, name: str) -> Optional[int]:
        """Pick next action: already-enqueued (SMART queue) or newly released."""
        assert self._adg is not None
        enq = self._adg.enqueue_nodes_idx[rid]
        if enq:
            return enq[0]
        released = self._adg.release_for_agent(rid, max_n=self._lookahead)
        if released:
            return released[0]
        return None

    def _tick(self):
        if not self._active or self._adg is None:
            return

        for rid, name in enumerate(self._robots):
            if not self._update_pose(name):
                self.get_logger().warn(
                    f'{name}: waiting for odom…', throttle_duration_sec=3.0
                )
                continue

            if self._adg.is_agent_finished(rid):
                self._cmd[name].publish(Twist())
                continue

            node_id = self._active_node[name]
            if node_id is None:
                node_id = self._next_action(rid, name)
                self._active_node[name] = node_id
                self._at_goal_since[name] = None
                if node_id is None:
                    self._cmd[name].publish(Twist())  # blocked on Type-2
                    continue

            act = self._action_of(name, node_id)
            if act is None:
                continue
            self._drive_to(name, act.goal)
            if self._action_done(name, node_id):
                self._adg.update_finished_node(rid, node_id)
                self._at_goal_since[name] = None
                self._active_node[name] = self._next_action(rid, name)

        if self._adg.all_finished():
            self.get_logger().info('Makespan complete (SMART ADG executor)')
            self._active = False
            self._stop_all()
            msg = Bool()
            msg.data = True
            self._done_pub.publish(msg)


def main():
    rclpy.init()
    node = AdgExecutor()
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
