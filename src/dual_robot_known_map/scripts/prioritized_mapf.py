#!/usr/bin/env python3
"""Prioritized multi-agent path finding for 4 namespaced robots.

Plans robots in fixed priority (robot1 > robot2 > robot3 > robot4).
Higher-priority paths are reserved in space-time; later robots A* around them.
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


Cell = Tuple[int, int]
STCell = Tuple[int, int, int]

ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4']


class PrioritizedMapf(Node):
    def __init__(self):
        super().__init__('prioritized_mapf')
        self.declare_parameter('robots', ROBOTS)
        self.declare_parameter('map_topic', '/robot1/map')
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('grid_stride', 2)  # plan on every Nth cell (~0.1 m)
        self.declare_parameter('max_time', 600)
        # Fallback spawn poses if TF is not ready yet
        self.declare_parameter('fallback_starts', [
            -3.0, -3.0, 3.0, -3.0, -3.0, 3.0, 3.0, 3.0,
        ])

        self.robots: List[str] = [str(r) for r in self.get_parameter('robots').value]
        fb = [float(x) for x in self.get_parameter('fallback_starts').value]
        self._fallback_starts = {
            self.robots[i]: (fb[2 * i], fb[2 * i + 1])
            for i in range(len(self.robots))
            if 2 * i + 1 < len(fb)
        }
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.stride = max(1, int(self.get_parameter('grid_stride').value))
        self.max_time = int(self.get_parameter('max_time').value)

        self._map: Optional[OccupancyGrid] = None
        self._grid: Optional[np.ndarray] = None  # True = blocked
        self._goals: Dict[str, PoseStamped] = {}
        self._goal_dirty = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._on_map,
            map_qos,
        )

        for name in self.robots:
            self.create_subscription(
                PoseStamped,
                f'/swarm/{name}/goal',
                lambda msg, n=name: self._on_goal(n, msg),
                10,
            )

        self._path_pubs = {
            name: self.create_publisher(Path, f'/swarm/{name}/path', 10)
            for name in self.robots
        }
        self._ready_pub = self.create_publisher(Bool, '/swarm/paths_ready', 10)

        self._tf_buffers: Dict[str, Buffer] = {}
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for name in self.robots:
            buf = Buffer(cache_time=Duration(seconds=30.0))
            self._tf_buffers[name] = buf
            self.create_subscription(
                TFMessage, f'/{name}/tf',
                lambda msg, b=buf: self._feed_tf(b, msg, False), 100)
            self.create_subscription(
                TFMessage, f'/{name}/tf_static',
                lambda msg, b=buf: self._feed_tf(b, msg, True), static_qos)

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Prioritized MAPF ready for {self.robots} stride={self.stride}'
        )

    @staticmethod
    def _feed_tf(buf: Buffer, msg: TFMessage, is_static: bool):
        for tf in msg.transforms:
            if is_static:
                buf.set_transform_static(tf, 'prioritized_mapf')
            else:
                buf.set_transform(tf, 'prioritized_mapf')

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg
        self._grid = self._inflate(msg)
        self.get_logger().info(
            f'Map {msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f}'
        )

    def _on_goal(self, name: str, msg: PoseStamped):
        self._goals[name] = msg
        self._goal_dirty = True
        self.get_logger().info(
            f'Goal {name}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def _inflate(self, msg: OccupancyGrid) -> np.ndarray:
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))
        # Occupied only (allow unknown as free for house SLAM maps)
        blocked = data >= int(100 * 0.65)
        rad_cells = max(1, int(math.ceil(self.robot_radius / msg.info.resolution)))
        pad = np.pad(blocked.astype(np.uint8), rad_cells, mode='constant', constant_values=1)
        out = np.zeros_like(blocked, dtype=bool)
        for dy in range(-rad_cells, rad_cells + 1):
            for dx in range(-rad_cells, rad_cells + 1):
                if dx * dx + dy * dy > rad_cells * rad_cells:
                    continue
                out |= pad[
                    rad_cells + dy: rad_cells + dy + h,
                    rad_cells + dx: rad_cells + dx + w,
                ].astype(bool)
        return out

    def _world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        assert self._map is not None
        info = self._map.info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        if c < 0 or r < 0 or c >= info.width or r >= info.height:
            return None
        return c, r

    def _cell_to_world(self, c: int, r: int) -> Tuple[float, float]:
        assert self._map is not None
        info = self._map.info
        x = info.origin.position.x + (c + 0.5) * info.resolution
        y = info.origin.position.y + (r + 0.5) * info.resolution
        return x, y

    def _lookup_start(self, name: str) -> Optional[Tuple[float, float]]:
        buf = self._tf_buffers[name]
        try:
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            fb = self._fallback_starts.get(name)
            if fb is not None:
                self.get_logger().warn(
                    f'TF start fail {name}, using spawn fallback {fb}: {e}',
                    throttle_duration_sec=5.0,
                )
                return fb
            self.get_logger().warn(f'TF start fail {name}: {e}', throttle_duration_sec=2.0)
            return None

    def _tick(self):
        if not self._goal_dirty:
            return
        if self._map is None or self._grid is None:
            return
        if len(self._goals) < len(self.robots):
            return
        starts = {}
        for name in self.robots:
            s = self._lookup_start(name)
            if s is None:
                return
            starts[name] = s
        self._goal_dirty = False
        self._plan_all(starts)

    def _plan_all(self, starts: Dict[str, Tuple[float, float]]):
        reservations: Set[STCell] = set()
        edge_res: Set[Tuple[int, int, int, int, int]] = set()
        ok_count = 0

        for name in self.robots:
            sx, sy = starts[name]
            g = self._goals[name]
            gx, gy = g.pose.position.x, g.pose.position.y
            sc = self._world_to_cell(sx, sy)
            gc = self._world_to_cell(gx, gy)
            if sc is None or gc is None:
                self.get_logger().error(f'{name}: start/goal off map')
                continue
            sc = self._snap_stride(self._nearest_free(sc))
            gc = self._snap_stride(self._nearest_free(gc))
            if sc is None or gc is None:
                self.get_logger().error(f'{name}: no free cell near start/goal')
                continue

            if not reservations:
                cells = self._spatial_astar(sc, gc)
            else:
                cells = self._spacetime_astar(sc, gc, reservations, edge_res)

            if cells is None:
                self.get_logger().error(f'{name}: MAPF failed')
                empty = Path()
                empty.header.frame_id = 'map'
                empty.header.stamp = self.get_clock().now().to_msg()
                self._path_pubs[name].publish(empty)
                continue

            for t, (c, r) in enumerate(cells):
                reservations.add((c, r, t))
                if t == len(cells) - 1:
                    for hold in range(1, 10):
                        reservations.add((c, r, t + hold))
                if t > 0:
                    pc, pr = cells[t - 1]
                    edge_res.add((pc, pr, c, r, t - 1))
                    edge_res.add((c, r, pc, pr, t - 1))

            path = self._cells_to_path(cells, g)
            self._path_pubs[name].publish(path)
            ok_count += 1
            self.get_logger().info(f'{name}: planned {len(cells)} cells / {len(path.poses)} poses')

        ready = Bool()
        ready.data = ok_count == len(self.robots)
        self._ready_pub.publish(ready)
        self.get_logger().info(f'paths_ready={ready.data} ({ok_count}/{len(self.robots)})')

    @staticmethod
    def _cells_from_res(reservations: Set[STCell]) -> Set[Cell]:
        return {(c, r) for c, r, _t in reservations}

    def _snap_stride(self, cell: Optional[Cell]) -> Optional[Cell]:
        if cell is None:
            return None
        c, r = cell
        c = (c // self.stride) * self.stride
        r = (r // self.stride) * self.stride
        return self._nearest_free((c, r))

    def _nearest_free(self, cell: Cell, max_r: int = 20) -> Optional[Cell]:
        c0, r0 = cell
        if self._is_free(c0, r0):
            return cell
        for rad in range(1, max_r + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    c, r = c0 + dc, r0 + dr
                    if self._is_free(c, r):
                        return c, r
        return None

    def _is_free(self, c: int, r: int) -> bool:
        assert self._grid is not None and self._map is not None
        if c < 0 or r < 0 or c >= self._map.info.width or r >= self._map.info.height:
            return False
        return not bool(self._grid[r, c])

    def _spatial_astar(
        self,
        start: Cell,
        goal: Cell,
        static_extra: Optional[Set[Cell]] = None,
    ) -> Optional[List[Cell]]:
        static_extra = static_extra or set()
        step = self.stride

        def h(c, r):
            return (abs(c - goal[0]) + abs(r - goal[1])) // step

        open_heap = []
        heapq.heappush(open_heap, (h(*start), 0, start))
        came: Dict[Cell, Optional[Cell]] = {start: None}
        gscore = {start: 0}
        nbrs = [(step, 0), (-step, 0), (0, step), (0, -step)]

        while open_heap:
            _, g, (c, r) = heapq.heappop(open_heap)
            if abs(c - goal[0]) < step and abs(r - goal[1]) < step:
                # finish to exact goal if free
                path = []
                cur: Optional[Cell] = (c, r)
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                path.reverse()
                if path[-1] != goal and self._is_free(*goal):
                    path.append(goal)
                return path

            for dc, dr in nbrs:
                nc, nr = c + dc, r + dr
                if not self._is_free(nc, nr):
                    continue
                if (nc, nr) in static_extra and (nc, nr) != goal:
                    continue
                nxt = (nc, nr)
                ng = g + 1
                if ng < gscore.get(nxt, 1e18):
                    gscore[nxt] = ng
                    came[nxt] = (c, r)
                    heapq.heappush(open_heap, (ng + h(nc, nr), ng, nxt))
        return None

    def _spacetime_astar(
        self,
        start: Cell,
        goal: Cell,
        reservations: Set[STCell],
        edge_res: Set[Tuple[int, int, int, int, int]],
    ) -> Optional[List[Cell]]:
        step = self.stride

        def h(c, r):
            return (abs(c - goal[0]) + abs(r - goal[1])) // step

        open_heap = []
        start_state: STCell = (start[0], start[1], 0)
        heapq.heappush(open_heap, (h(*start), 0, start_state))
        came: Dict[STCell, Optional[STCell]] = {start_state: None}
        gscore = {start_state: 0}
        # moves first, wait last
        neighbors = [(step, 0), (-step, 0), (0, step), (0, -step), (0, 0)]
        expansions = 0
        max_exp = 250000

        while open_heap and expansions < max_exp:
            expansions += 1
            _, g, (c, r, t) = heapq.heappop(open_heap)
            if abs(c - goal[0]) < step and abs(r - goal[1]) < step:
                path = []
                cur: Optional[STCell] = (c, r, t)
                while cur is not None:
                    path.append((cur[0], cur[1]))
                    cur = came[cur]
                path.reverse()
                if path[-1] != goal and self._is_free(*goal):
                    path.append(goal)
                return path

            if t >= self.max_time:
                continue

            for dc, dr in neighbors:
                nc, nr, nt = c + dc, r + dr, t + 1
                if not self._is_free(nc, nr):
                    continue
                if (nc, nr, nt) in reservations:
                    continue
                if (c, r, nc, nr, t) in edge_res:
                    continue
                nxt = (nc, nr, nt)
                ng = g + 1
                if ng < gscore.get(nxt, 1e18):
                    gscore[nxt] = ng
                    came[nxt] = (c, r, t)
                    f = ng + h(nc, nr)
                    # slight penalty for waiting so we prefer moving
                    if dc == 0 and dr == 0:
                        f += 0.5
                    heapq.heappush(open_heap, (f, ng, nxt))
        return None

    def _cells_to_path(self, cells: List[Cell], goal_pose: PoseStamped) -> Path:
        """One pose per MAPF timestep (including waits) for timestep execution.

        Wait actions keep the same (x,y) across consecutive poses so the
        executor can stay space-time aligned with reservations.
        """
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (c, r) in enumerate(cells):
            x, y = self._cell_to_world(c, r)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            # Point yaw along travel when moving; keep previous on waits
            if i + 1 < len(cells) and cells[i + 1] != (c, r):
                nx, ny = self._cell_to_world(*cells[i + 1])
                yaw = math.atan2(ny - y, nx - x)
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
            elif path.poses:
                ps.pose.orientation = path.poses[-1].pose.orientation
            else:
                ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if path.poses:
            path.poses[-1].pose.position.x = goal_pose.pose.position.x
            path.poses[-1].pose.position.y = goal_pose.pose.position.y
            path.poses[-1].pose.orientation = goal_pose.pose.orientation
        return path


def main():
    rclpy.init()
    node = PrioritizedMapf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
